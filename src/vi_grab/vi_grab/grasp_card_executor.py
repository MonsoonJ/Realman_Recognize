#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import time
import os
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import ( # pyright: ignore[reportMissingImports]
    Movejp, 
    Movel, 
    ObjectInfo, 
    Gripperpick, 
    Gripperset
)
from ultralytics import YOLO

class VisualServoYoloGrasp(Node):
    def __init__(self):
        super().__init__('visual_servo_save_photo')

        # ==================== 0. 初始化图片保存路径 ====================
        self.photo_dir = os.path.join(os.getcwd(), 'img_card')
        if not os.path.exists(self.photo_dir):
            os.makedirs(self.photo_dir)
            self.get_logger().info(f"已创建图片保存目录: {self.photo_dir}")
        else:
            self.get_logger().info(f"图片保存目录已存在: {self.photo_dir}")

        # ==================== 1. 手眼标定参数 ====================
        self.ROT_CAM_TO_EE = np.array([
            [ 0.99984546, -0.00464906, -0.01695408],
            [ 0.00448498,  0.99994286, -0.00970348],
            [ 0.01699823,  0.00962594,  0.99980918]
        ])
        self.TRANS_CAM_TO_EE = np.array([
             0.01213023, -0.06167299, 0.026354
        ])

        # ==================== 2. RealSense 相机初始化 ====================
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        
        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info("RealSense 启动成功，预热中...")
            time.sleep(2.0)
            self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        except Exception as e:
            self.get_logger().error(f"相机启动失败: {e}")
            return

        # ==================== 3. ROS & YOLO 初始化 ====================
        self.declare_parameter('model_path', 'models/card.pt')
        self.declare_parameter('target_class', 'card')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_class_name = self.get_parameter('target_class').get_parameter_value().string_value

        try:
            self.model = YOLO(model_path)
        except:
            self.get_logger().error(f'YOLO 加载失败')

        self.create_subscription(Pose, '/rm_driver/udp_arm_position', self.arm_pose_callback, 10)
        self.current_arm_pose = None
        
        self.movejp_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.movel_pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.gripper_pick_pub = self.create_publisher(Gripperpick, '/rm_driver/set_gripper_pick_cmd', 10)
        self.gripper_set_pub = self.create_publisher(Gripperset, '/rm_driver/set_gripper_position_cmd', 10)

        self.home_x, self.home_y, self.home_z = -0.4, 0.0, 0.4
        self.home_quat = None
        self.locks = set() 
        self.target_grasp_base = None
        self.target_grasp_quat = None
        
        self.timer = None
        self.timer = self.create_timer(1.0, self.check_ready_and_start)

    def destroy_node(self):
        if self.timer: self.timer.cancel()
        self.pipeline.stop()
        super().destroy_node()

    # ==================== 夹爪控制 ====================
    def open_gripper(self):
        cmd = Gripperset()
        cmd.position = 850 
        self.gripper_set_pub.publish(cmd)
        self.get_logger().info(">>> 夹爪: 张开")

    def close_gripper(self):
        cmd = Gripperpick()
        cmd.speed = 500
        cmd.force = 100
        self.gripper_pick_pub.publish(cmd)
        self.get_logger().info(">>> 夹爪: 闭合")

    # ==================== 辅助逻辑 ====================
    def switch_step(self, delay, callback):
        if self.timer: self.timer.cancel()
        self.timer = self.create_timer(delay, callback)

    def arm_pose_callback(self, msg: Pose):
        self.current_arm_pose = msg

    def check_ready_and_start(self):
        if self.current_arm_pose is not None and 'start' not in self.locks:
            self.locks.add('start')
            self.home_quat = self.get_vertical_down_quat()
            self.get_logger().info("系统就绪，开始任务...")
            self.step1_move_to_home()

    def get_vertical_down_quat(self):
        z_axis = np.array([0.0, 0.0, -1.0])
        y_axis = np.array([1.0, 0.0, 0.0])
        x_axis = np.cross(y_axis, z_axis)
        mat = np.column_stack((x_axis, y_axis, z_axis))
        return R.from_matrix(mat).as_quat()

    def get_latest_frames(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            aligned_frames = self.align.process(frames)
            c = aligned_frames.get_color_frame()
            d = aligned_frames.get_depth_frame()
            if not c or not d: return None, None
            return np.asanyarray(c.get_data()), d
        except: return None, None

    # ==================== 修改 1：增加 cam_dy 参数，默认 0.0 ====================
    def transform_pixel_to_base(self, u, v, depth, cam_dy=0.0):
        if self.current_arm_pose is None: return None
        p_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)
        
        # 在这里将偏移量加到相机的 Y 轴上 (p_cam[1])
        # 注意：相机坐标系 Y 轴向下，所以 cam_dy 为负数时，实际点会“向上”移
        p_cam_h = np.array([p_cam[0], p_cam[1] + cam_dy, p_cam[2], 1.0])
        
        T_ee_cam = np.eye(4)
        T_ee_cam[:3, :3] = self.ROT_CAM_TO_EE
        T_ee_cam[:3, 3] = self.TRANS_CAM_TO_EE
        
        q = self.current_arm_pose.orientation
        p = self.current_arm_pose.position
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_base_ee[:3, 3] = [p.x, p.y, p.z]
        
        p_base_h = T_base_ee @ (T_ee_cam @ p_cam_h)
        return p_base_h[:3]

    def get_robust_depth(self, depth_frame, u, v):
        depths = []
        for i in range(-2, 3):
            for j in range(-2, 3):
                if 0 <= u+i < 640 and 0 <= v+j < 480:
                    d = depth_frame.get_distance(u+i, v+j)
                    if 0.0 < d < 2.0: depths.append(d)
        return np.median(depths) if depths else 0.0

    # ==================== Step 1: 归位 ====================
    def step1_move_to_home(self):
        if 'step1' in self.locks: return
        self.locks.add('step1')
        self.open_gripper()
        self.get_logger().info("Step 1: 归位 (Home)...")
        self.send_movej_p(self.home_x, self.home_y, self.home_z, self.home_quat, speed=40)
        self.switch_step(5.0, self.step2_align_and_descend)

    # ==================== Step 2: 对齐并下降 (拍照1：矩形检测) ====================
    def step2_align_and_descend(self):
        if 'step2' in self.locks: return
        self.locks.add('step2')
        self.get_logger().info("Step 2: 视觉对齐 & 下降 (Align & Descend)...")

        img, d_frame = self.get_latest_frames()
        if img is None: 
            self.switch_step(0.5, self.step2_align_and_descend)
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 90, 40]), np.array([30, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.get_logger().warn("无轮廓，直接下探...")
            target_z = self.current_arm_pose.position.z - 0.10
            self.send_movel(self.current_arm_pose.position.x, 
                            self.current_arm_pose.position.y, 
                            target_z, 
                            self.home_quat)
            self.switch_step(3.0, self.step4_yolo)
            return

        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        cx, cy = int(rect[0][0]), int(rect[0][1])
        
        # 1. 计算对齐的 XY 坐标 (Step 2 这里不传 cam_dy，保持默认0，不偏移)
        real_depth = self.get_robust_depth(d_frame, cx, cy) or 0.4
        p_base = self.transform_pixel_to_base(cx, cy, real_depth)
        
        # 2. 计算对齐的旋转角度
        box = np.int32(cv2.boxPoints(rect))
        p0, p1, p2 = box[0], box[1], box[2]
        d01, d12 = np.linalg.norm(p0-p1), np.linalg.norm(p1-p2)
        long_vec = (p1-p0) if d01 > d12 else (p2-p1)
        angle = math.atan2(long_vec[1], long_vec[0])

        # ================== 保存检测结果图 1 (矩形) ==================
        try:
            debug_img = img.copy()
            cv2.drawContours(debug_img, [box], 0, (0, 255, 0), 1)
            cv2.circle(debug_img, (cx, cy), 2, (0, 0, 255), -1)
            
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.photo_dir, f"step2_rect_{timestamp}.jpg")
            cv2.imwrite(filename, debug_img)
            self.get_logger().info(f"Step 2 检测图已保存: {filename}")
        except Exception as e:
            self.get_logger().error(f"保存 Step 2 图片失败: {e}")
        # ==========================================================

        q_curr = [self.current_arm_pose.orientation.x, self.current_arm_pose.orientation.y, 
                  self.current_arm_pose.orientation.z, self.current_arm_pose.orientation.w]
        r_final = R.from_quat(q_curr) * R.from_euler('z', angle, degrees=False)
        self.target_grasp_quat = r_final.as_quat()

        target_z = self.current_arm_pose.position.z - 0.12
        if p_base is not None:
            self.send_movel(p_base[0], p_base[1], target_z, self.target_grasp_quat,speed=40)
        
        self.switch_step(4.0, self.step4_yolo)

    # ==================== Step 4: YOLO 识别 (拍照2：卡片检测) ====================
    def step4_yolo(self):
        if 'step4' in self.locks: return
        self.locks.add('step4')
        self.get_logger().info("Step 4: YOLO 最终精确定位...")
        
        img, d_frame = self.get_latest_frames()
        if img is None: 
            self.switch_step(0.1, self.step4_yolo)
            return

        # 运行 YOLO
        results_list = self.model(img, conf=0.5, verbose=False)
        results = results_list[0]
        
        candidates = []
        if results.boxes:
            for box in results.boxes:
                name = self.model.names[int(box.cls[0])]
                if self.target_class_name and name != self.target_class_name: continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                d = self.get_robust_depth(d_frame, cx, cy)
                if d > 0: candidates.append({'cx':cx, 'cy':cy, 'd':d, 'name':name})
        
        if not candidates:
            self.get_logger().warn("YOLO 未找到目标，重试...")
            self.switch_step(1.0, self.step4_yolo)
            return
        
        # 移除之前的 cy + 0.003 手动修正，使用新的坐标转换逻辑
        candidates.sort(key=lambda x: x['cx'] + x['cy'])
        target = candidates[0]

        # ================== 保存检测结果图 2 (YOLO) ==================
        try:
            annotated_frame = results.plot(labels=False, conf=False, line_width=1)
            
            cv2.drawMarker(annotated_frame, (target['cx'], target['cy']), 
                           (0, 255, 255), cv2.MARKER_CROSS, 10, 1)

            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.photo_dir, f"step4_yolo_{timestamp}.jpg")
            cv2.imwrite(filename, annotated_frame)
            self.get_logger().info(f"Step 4 检测图已保存: {filename}")
        except Exception as e:
            self.get_logger().error(f"保存 Step 4 图片失败: {e}")
        # ==========================================================

        # ==================== 修改 2：这里传入偏移量 -0.05 ====================
        # 在相机画面中，Y轴向下为正，向上为负。
        # 传入 -0.05 表示目标点在相机视野中向上移动 5 厘米
        p_base = self.transform_pixel_to_base(target['cx'], target['cy'], target['d'], cam_dy=-0.005)
        self.target_grasp_base = p_base
        
        if self.target_grasp_quat is None:
             q = self.current_arm_pose.orientation
             self.target_grasp_quat = [q.x, q.y, q.z, q.w]
             
        self.get_logger().info(f"锁定坐标(含偏移): {p_base}")
        self.switch_step(0.5, self.step5_move_horizontal)

    # ==================== Step 5: 水平移动 ====================
    def step5_move_horizontal(self):
        if 'step5' in self.locks: return
        self.locks.add('step5')
        tx, ty, tz = self.target_grasp_base
        cur_z = self.current_arm_pose.position.z
        self.get_logger().info("Step 5: 平移至目标上方...")
        self.send_movel(tx, ty, cur_z, self.target_grasp_quat, speed=40)
        self.switch_step(4.0, self.step6_move_vertical)

    # ==================== Step 6: 垂直下降 ====================
    def step6_move_vertical(self):
        if 'step6' in self.locks: return
        self.locks.add('step6')
        tx, ty, tz = self.target_grasp_base
        stop_height = tz + 0.14 # 目标上方 14cm
        self.get_logger().info(f"Step 6: 下降至抓取高度 Z={stop_height:.3f}...")
        self.send_movel(tx, ty, stop_height, self.target_grasp_quat, speed=40)
        self.switch_step(4.0, self.step7_close)

    # ==================== Step 7: 抓取 ====================
    def step7_close(self):
        if 'step7' in self.locks: return
        self.locks.add('step7')
        self.get_logger().info("Step 7: 闭合夹爪...")
        self.close_gripper()
        self.get_logger().info("等待抓紧 (4秒)...")
        self.switch_step(3.0, self.step8_lift_up)

    # ==================== Step 8: 抬起 ====================
    def step8_lift_up(self):
        if 'step8' in self.locks: return
        self.locks.add('step8')
        self.get_logger().info("Step 8: 抬起...")
        cur_x = self.current_arm_pose.position.x
        cur_y = self.current_arm_pose.position.y
        self.send_movel(cur_x, cur_y, self.home_z, self.target_grasp_quat, speed=40)
        self.switch_step(3.0, self.step9_return_home)

    # ==================== Step 9: 回家 ====================
    def step9_return_home(self):
        if 'step9' in self.locks: return
        self.locks.add('step9')
        self.get_logger().info("Step 9: 返回 Home，任务结束。")
        self.send_movej_p(self.home_x, self.home_y, self.home_z, self.home_quat, speed=40)

    # ==================== 发送指令 ====================
    def send_movel(self, x, y, z, quat, speed):
        cmd = Movel()
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = float(x), float(y), float(z)
        cmd.pose.orientation.x, cmd.pose.orientation.y = quat[0], quat[1]
        cmd.pose.orientation.z, cmd.pose.orientation.w = quat[2], quat[3]
        cmd.speed = speed
        self.movel_pub.publish(cmd)

    def send_movej_p(self, x, y, z, quat, speed):
        cmd = Movejp()
        cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = float(x), float(y), float(z)
        cmd.pose.orientation.x, cmd.pose.orientation.y = quat[0], quat[1]
        cmd.pose.orientation.z, cmd.pose.orientation.w = quat[2], quat[3]
        cmd.speed = speed
        self.movejp_pub.publish(cmd)

def main():
    rclpy.init()
    node = VisualServoYoloGrasp()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()