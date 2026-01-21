#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import time
import os
import yaml
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

        # ==================== 0. 加载配置文件 ====================
        self.config = self._load_config()
        if self.config is None:
            self.get_logger().error("配置文件加载失败，使用默认参数")
            return

        # ==================== 1. 初始化图片保存路径 ====================
        self.photo_dir = os.path.join(os.getcwd(), self.config['paths']['photo_save_dir'])
        if not os.path.exists(self.photo_dir):
            os.makedirs(self.photo_dir)
            self.get_logger().info(f"已创建图片保存目录: {self.photo_dir}")
        else:
            self.get_logger().info(f"图片保存目录已存在: {self.photo_dir}")

        # ==================== 2. 手眼标定参数 ====================
        self.ROT_CAM_TO_EE = np.array(self.config['camera']['rotation_cam_to_ee'])
        self.TRANS_CAM_TO_EE = np.array(self.config['camera']['translation_cam_to_ee'])

        # ==================== 3. RealSense 相机初始化 ====================
        self.pipeline = rs.pipeline()
        self.config_rs = rs.config()
        cam_cfg = self.config['camera']
        self.config_rs.enable_stream(rs.stream.color, cam_cfg['width'], cam_cfg['height'], rs.format.bgr8, cam_cfg['fps'])
        self.config_rs.enable_stream(rs.stream.depth, cam_cfg['width'], cam_cfg['height'], rs.format.z16, cam_cfg['fps'])
        self.align = rs.align(rs.stream.color)
        
        try:
            self.profile = self.pipeline.start(self.config_rs)
            self.get_logger().info("RealSense 启动成功，预热中...")
            time.sleep(self.config['system']['camera_warmup_time'])
            self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        except Exception as e:
            self.get_logger().error(f"相机启动失败: {e}")
            return

        # ==================== 4. ROS & YOLO 初始化 ====================
        model_cfg = self.config['model']
        model_path = model_cfg['yolo_model_path']
        self.target_class_name = model_cfg['target_class']

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

        # 从配置文件读取参数
        home_cfg = self.config['home_position']
        self.home_x, self.home_y, self.home_z = home_cfg['x'], home_cfg['y'], home_cfg['z']
        self.home_quat = None
        self.locks = set() 
        self.target_grasp_base = None
        self.target_grasp_quat = None
        # self.task_complete = False  # 任务完成标志
        
        self.timer = None
        self.timer = self.create_timer(1.0, self.check_ready_and_start)

    def destroy_node(self):
        if self.timer: self.timer.cancel()
        self.pipeline.stop()
        super().destroy_node()

    # ==================== 配置文件管理 ====================
    def _load_config(self):
        """从配置文件加载参数"""
        import sys
        from ament_index_python.packages import get_package_share_directory
        
        # 尝试多个可能的配置文件位置
        possible_paths = [
            # 方案1: 源代码目录 (开发时)
            os.path.join(os.path.dirname(__file__), 'config', 'grasp_config.yaml'),
            # 方案2: 使用ament_index查找已安装的包
            None,  # 后面处理
            # 方案3: 工作目录下的config
            os.path.join(os.getcwd(), 'src', 'vi_grab', 'vi_grab', 'config', 'grasp_config.yaml'),
        ]
        
        # 尝试用ament_index查找
        try:
            share_dir = get_package_share_directory('vi_grab')
            possible_paths[1] = os.path.join(share_dir, 'config', 'grasp_config.yaml')
        except:
            pass
        
        config_path = None
        for path in possible_paths:
            if path and os.path.exists(path):
                config_path = path
                break
        
        if config_path is None:
            self.get_logger().error(f"配置文件不存在，已尝试的路径:")
            for path in possible_paths:
                if path:
                    self.get_logger().error(f"  - {path}")
            return None
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f"配置文件加载成功: {config_path}")
            return config
        except Exception as e:
            self.get_logger().error(f"配置文件加载失败: {e}")
            return None

    # ==================== 夹爪控制 ====================
    def open_gripper(self):
        cmd = Gripperset()
        cmd.position = self.config['gripper']['open_position']
        self.gripper_set_pub.publish(cmd)
        self.get_logger().info(">>> 夹爪: 张开")

    def close_gripper(self):
        cmd = Gripperpick()
        gripper_cfg = self.config['gripper']
        cmd.speed = gripper_cfg['pick_speed']
        cmd.force = gripper_cfg['pick_force']
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
    
    # 【新增】放置姿态：Y->Base -Z, Z->Base -Y
    def get_place_quat(self):
        # 1. 目标 Z 轴 (末端指向) -> 基座 -Y
        z_axis = np.array([0.0, -1.0, 0.0])
        
        # 2. 目标 Y 轴 (末端上方) -> 基座 -Z
        y_axis = np.array([0.0, 0.0, -1.0])
        
        # 3. 目标 X 轴 (由右手定则决定 X = Y x Z)
        x_axis = np.cross(y_axis, z_axis) # 结果应为 [-1, 0, 0]
        
        # 4. 组合旋转矩阵 (按列)
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
        search_radius = self.config['depth']['search_radius']
        depths = []
        for i in range(-search_radius, search_radius + 1):
            for j in range(-search_radius, search_radius + 1):
                if 0 <= u+i < 640 and 0 <= v+j < 480:
                    d = depth_frame.get_distance(u+i, v+j)
                    depth_cfg = self.config['depth']
                    if depth_cfg['min_depth'] < d < depth_cfg['max_depth']: 
                        depths.append(d)
        return np.median(depths) if depths else 0.0

    # ==================== Step 1: 归位 ====================
    def step1_move_to_home(self):
        if 'step1' in self.locks: return
        self.locks.add('step1')
        self.open_gripper()
        self.get_logger().info("Step 1: 归位 (Home)...")
        self.send_movej_p(self.home_x, self.home_y, self.home_z, self.home_quat, speed=self.config['steps']['step1']['speed'])
        self.switch_step(self.config['steps']['step1']['timeout'], self.step2_align_and_descend)

    # ==================== Step 2: 对齐并下降 (拍照1：矩形检测) ====================
    def step2_align_and_descend(self):
        if 'step2' in self.locks: return
        self.locks.add('step2')
        self.get_logger().info("Step 2: 视觉对齐 & 下降 (Align & Descend)...")

        img, d_frame = self.get_latest_frames()
        if img is None: 
            self.switch_step(self.config['steps']['step2']['retry_interval'], self.step2_align_and_descend)
            return

        step2_cfg = self.config['steps']['step2']
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(step2_cfg['hsv_lower']), np.array(step2_cfg['hsv_upper']))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((step2_cfg['morphology_kernel_size'],step2_cfg['morphology_kernel_size']), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.get_logger().warn("无轮廓，直接下探...")
            target_z = self.current_arm_pose.position.z - step2_cfg['z_offset']
            self.send_movel(self.current_arm_pose.position.x, 
                            self.current_arm_pose.position.y, 
                            target_z, 
                            self.home_quat,
                            speed=step2_cfg['speed'])
            self.switch_step(self.config['steps']['step4']['timeout'], self.step4_yolo)
            return

        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        cx, cy = int(rect[0][0]), int(rect[0][1])
        
        # 1. 计算对齐的 XY 坐标 (Step 2 这里使用 cam_dy)
        real_depth = self.get_robust_depth(d_frame, cx, cy) or 0.4
        p_base = self.transform_pixel_to_base(cx, cy, real_depth, cam_dy=step2_cfg['camera_y_offset'])
        
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

        target_z = self.current_arm_pose.position.z - step2_cfg['descend_distance']
        if p_base is not None:
            self.send_movel(p_base[0], p_base[1], target_z, self.target_grasp_quat, speed=step2_cfg['speed'])
        
        self.switch_step(self.config['steps']['step2']['timeout'], self.step4_yolo)

    # ==================== Step 4: YOLO 识别 (拍照2：卡片检测) ====================
    def step4_yolo(self):
        if 'step4' in self.locks: return
        self.locks.add('step4')
        self.get_logger().info("Step 4: YOLO 最终精确定位...")
        
        img, d_frame = self.get_latest_frames()
        if img is None: 
            self.switch_step(self.config['steps']['step4']['retry_interval'], self.step4_yolo)
            return

        # 运行 YOLO
        model_cfg = self.config['model']
        results_list = self.model(img, conf=model_cfg['detection_confidence'], verbose=False)
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
        
        # 手动修正，使用新的坐标转换逻辑
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

        # 在相机画面中，Y轴向下为正，向上为负。 使用配置文件中的偏移值
        step6_cfg = self.config['steps']['step6']
        p_base = self.transform_pixel_to_base(target['cx'], target['cy'], target['d'], cam_dy=step6_cfg['camera_y_offset'])
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
        step5_cfg = self.config['steps']['step5']
        self.send_movel(tx, ty, cur_z, self.target_grasp_quat, speed=step5_cfg['speed'])
        self.switch_step(step5_cfg['timeout'], self.step6_move_vertical)

    # ==================== Step 6: 垂直下降 ====================
    def step6_move_vertical(self):
        if 'step6' in self.locks: return
        self.locks.add('step6')
        tx, ty, tz = self.target_grasp_base
        step6_cfg = self.config['steps']['step6']
        stop_height = tz + step6_cfg['grasp_height_offset']
        self.get_logger().info(f"Step 6: 下降至抓取高度 Z={stop_height:.3f}...")
        self.send_movel(tx, ty, stop_height, self.target_grasp_quat, speed=step6_cfg['speed'])
        self.switch_step(step6_cfg['timeout'], self.step7_close)

    # ==================== Step 7: 抓取 ====================
    def step7_close(self):
        if 'step7' in self.locks: return
        self.locks.add('step7')
        self.get_logger().info("Step 7: 闭合夹爪...")
        self.close_gripper()
        self.get_logger().info("等待抓紧 (4秒)...")
        step7_cfg = self.config['steps']['step7']
        self.switch_step(step7_cfg['timeout'], self.step8_lift_up)

    # ==================== Step 8: 抬起 ====================
    def step8_lift_up(self):
        if 'step8' in self.locks: return
        self.locks.add('step8')
        self.get_logger().info("Step 8: 抬起...")
        cur_x = self.current_arm_pose.position.x
        cur_y = self.current_arm_pose.position.y
        step8_cfg = self.config['steps']['step8']
        self.send_movel(cur_x, cur_y, self.home_z, self.target_grasp_quat, speed=step8_cfg['speed'])
        self.switch_step(step8_cfg['timeout'], self.step9_return_home)

    # ==================== Step 9: 前往放置点 ====================
    def step9_return_home(self):
        if 'step9' in self.locks: return
        self.locks.add('step9')
        
        # 从配置文件读取目标位置
        place_cfg = self.config['place_position']
        target_x, target_y, target_z = place_cfg['x'], place_cfg['y'], place_cfg['z']
        
        # 计算新姿态
        target_quat = self.get_place_quat()

        self.get_logger().info(f"Step 9: 前往放置点 ({target_x}, {target_y}, {target_z}) 并调整姿态...")
        # 涉及到大幅度旋转，使用关节运动(MoveJ)比直线运动(MoveL)更安全
        step9_cfg = self.config['steps']['step9']
        self.send_movej_p(target_x, target_y, target_z, target_quat, speed=step9_cfg['speed'])


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