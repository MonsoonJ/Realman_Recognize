#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from datetime import datetime
import os

from ultralytics import YOLO

from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import ( # pyright: ignore[reportMissingImports]
    Movejp,
    Movel,
    Gripperpick,
    Gripperset,
    ObjectInfo
)

# 手眼标定结果
ROT_CAM_TO_EE = np.array([
    [ 0.99984546, -0.00464906, -0.01695408],
    [ 0.00448498,  0.99994286, -0.00970348],
    [ 0.01699823,  0.00962594,  0.99980918]
])

TRANS_CAM_TO_EE = np.array([
     0.01213023,
    -0.06167299,
     0.026354
])


class GraspBottleExecutor(Node):
    """综合视觉抓取执行器"""

    def __init__(self):
        super().__init__('grasp_bottle_executor')

        # ========== YOLO 参数 ==========
        self.declare_parameter('model_path', 'models/yolov8n.pt')
        self.declare_parameter('target_class', 'bottle')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value

        if model_path == '':
            self.get_logger().fatal('model_path 参数未设置')
            raise RuntimeError('model_path is required')

        # YOLO 模型
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded YOLO model: {model_path}')

        # ========== RealSense 相机 ==========
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)
        self.intrinsics = (
            self.profile
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        self.get_logger().info('RealSense camera initialized.')

        # ========== 订阅机械臂位置 ==========
        self.current_pose = None
        self.create_subscription(
            Pose,
            '/rm_driver/udp_arm_position',
            self.arm_pose_callback,
            10
        )
        
        # ========== 发布执行器命令 ==========
        self.movejp_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.movel_pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.gripper_pick_pub = self.create_publisher(Gripperpick, '/rm_driver/set_gripper_pick_cmd', 10)
        self.gripper_set_pub = self.create_publisher(Gripperset, '/rm_driver/set_gripper_position_cmd', 10)

        # ========== 状态变量 ==========
        self.target = None
        self.executed = False
        self.state = 'IDLE'
        self.approach_orientation = None
        self.vision_processed = False

        self.get_logger().info('Vision Grasp Executor started.')
        self.get_logger().info('Target class: ' + self.target_class)
        self.get_logger().info('Waiting for arm pose and then will capture image...')

        # 创建图像保存目录
        self.img_save_dir = os.path.join(os.getcwd(), 'img_bottle')
        if not os.path.exists(self.img_save_dir):
            os.makedirs(self.img_save_dir)
            self.get_logger().info(f'Created image save directory: {self.img_save_dir}')
        else:
            self.get_logger().info(f'Image save directory: {self.img_save_dir}')

        # 延迟2秒后执行一次识别
        self.timer = self.create_timer(2.0, self.vision_process_once)

    def arm_pose_callback(self, msg: Pose):
        """更新机械臂位置"""
        self.current_pose = msg
        if not self.vision_processed:
            self.get_logger().debug(f'Received arm pose: ({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})')

    # ========== 视觉处理 获取一次图像，进行识别和坐标变换==========
    def vision_process_once(self):

        if self.vision_processed:
            return
        self.vision_processed = True

        # 销毁定时器
        self.destroy_timer(self.timer)

        if self.current_pose is None:
            self.get_logger().warn('Waiting for arm pose...')
            return

        self.get_logger().info('Starting image capture and object detection...')

        # 获取一帧图像和深度信息
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn('Failed to get frames')
            return

        color_image = np.asanyarray(color_frame.get_data())

        # YOLO 推理
        results = self.model(color_image, conf=0.5, verbose=False)[0]

        detected = False

        if results.boxes is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                conf = float(box.conf[0])

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                depth = depth_frame.get_distance(cx, cy)

                # 可视化检测框
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)

                label = f'{class_name} {conf:.2f}'
                if depth > 0:
                    label += f' {depth:.2f}m'

                cv2.putText(
                    color_image,
                    label,
                    (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

                if detected:
                    continue

                # 检查是否为目标类别
                if self.target_class and class_name != self.target_class:
                    continue

                # 检查深度有效性
                if depth <= 0.0:
                    continue

                # 相机坐标到3D点
                point = rs.rs2_deproject_pixel_to_point(
                    self.intrinsics,
                    [cx, cy],
                    depth
                )

                p_cam = np.array(point)

                # ========== 坐标变换：相机→末端执行器→基座 ==========
                T_cam_ee = np.eye(4)
                T_cam_ee[:3, :3] = ROT_CAM_TO_EE
                T_cam_ee[:3, 3] = TRANS_CAM_TO_EE

                pos = np.array([
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.current_pose.position.z
                ])

                quat = [
                    self.current_pose.orientation.x,
                    self.current_pose.orientation.y,
                    self.current_pose.orientation.z,
                    self.current_pose.orientation.w
                ]

                rot = R.from_quat(quat).as_matrix()

                T_base_ee = np.eye(4)
                T_base_ee[:3, :3] = rot
                T_base_ee[:3, 3] = pos

                # 执行变换
                p_cam_h = np.append(p_cam, 1.0)
                p_ee_h = T_cam_ee @ p_cam_h
                p_base = (T_base_ee @ p_ee_h)[:3]

                # 保存目标信息
                self.target = ObjectInfo()
                self.target.object_class = class_name
                self.target.x = float(p_base[0])
                self.target.y = float(p_base[1])
                self.target.z = float(p_base[2])

                self.get_logger().info(
                    f'Object detected: {class_name} | '
                    f'base xyz = ({self.target.x:.3f}, {self.target.y:.3f}, {self.target.z:.3f})'
                )

                detected = True

        # 显示检测结果
        cv2.imshow('YOLOv8 RealSense Detection', color_image)
        cv2.waitKey(1)

        # 保存带标注的图像
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        img_filename = os.path.join(self.img_save_dir, f'detection_{timestamp}.jpg')
        cv2.imwrite(img_filename, color_image)
        self.get_logger().info(f'Detection image saved: {img_filename}')

        # 处理完成后关闭相机
        self.pipeline.stop()
        cv2.destroyAllWindows()
        self.get_logger().info('Image processing complete. Camera closed.')

        # 如果检测到目标，开始抓取流程
        if self.target is not None and not self.executed:
            self.executed = True
            self.get_logger().info(f'Target Found: ({self.target.x:.3f}, {self.target.y:.3f}, {self.target.z:.3f})')
            self.open_gripper()

    # ========== 核心算法：Y轴向下, Z高度恒定 ==========
    def calculate_aligned_pose(self, target_pos, current_pos, distance_from_target):
        """计算对齐的接近位置和方向"""
        t_pos = np.array(target_pos)
        c_pos = np.array(current_pos)

        # 1. 计算 Z轴 (前进轴): 从TCP指向物体的方向
        direction_vec = t_pos - c_pos
        direction_vec[2] = 0.0  # 投影到水平面

        norm = np.linalg.norm(direction_vec)
        if norm < 0.001:
            z_axis = np.array([1.0, 0.0, 0.0])
        else:
            z_axis = direction_vec / norm

        # 2. Y轴: 强制指向地面（垂直向下）
        y_axis_global = np.array([0.0, 0.0, -1.0])

        # 3. 计算 X轴: 根据右手定则 X = Y × Z
        x_axis = np.cross(y_axis_global, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # 4. 重新正交化 Y轴
        y_axis = np.cross(z_axis, x_axis)

        # 5. 构建旋转矩阵
        R_mat = np.column_stack((x_axis, y_axis, z_axis))
        quat = R.from_matrix(R_mat).as_quat()

        # 6. 计算接近位置
        approach_pos = t_pos - (distance_from_target * z_axis)
        approach_pos[2] = t_pos[2]  # 锁定高度

        return approach_pos, quat
    # ========== 抓取流程 ==========
    def open_gripper(self):
        """打开夹爪"""
        self.state = 'OPEN'
        cmd = Gripperset()
        cmd.position = 1000
        self.gripper_set_pub.publish(cmd)
        self.get_logger().info('>> Opening gripper...')
        self.create_timer(1.0, self.align_and_approach)

    def align_and_approach(self):
        """对齐并接近目标"""
        if self.state != 'OPEN':
            return
        self.state = 'ALIGN_APPROACH'

        t_xyz = [self.target.x, self.target.y, self.target.z]
        c_xyz = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z]

        # 计算20cm处的预备点
        pos, quat = self.calculate_aligned_pose(t_xyz, c_xyz, distance_from_target=0.20)
        self.approach_orientation = quat

        cmd = Movejp()
        cmd.pose.position.x = float(pos[0])
        cmd.pose.position.y = float(pos[1])
        cmd.pose.position.z = float(t_xyz[2])
        cmd.pose.orientation.x = quat[0]
        cmd.pose.orientation.y = quat[1]
        cmd.pose.orientation.z = quat[2]
        cmd.pose.orientation.w = quat[3]
        cmd.speed = 30

        self.movejp_pub.publish(cmd)
        self.get_logger().info('>> MoveJ: Aligning (Y-Down)...')
        self.create_timer(3.0, self.move_forward_grasp)

    def move_forward_grasp(self):
        """向前移动进行抓取"""
        if self.state != 'ALIGN_APPROACH':
            return
        self.state = 'GRASP'

        t_xyz = [self.target.x, self.target.y, self.target.z]
        c_xyz = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z]

        # 计算9cm处的抓取点
        grasp_pos, _ = self.calculate_aligned_pose(t_xyz, c_xyz, distance_from_target=0.09)

        cmd = Movel()
        cmd.pose.position.x = float(grasp_pos[0])
        cmd.pose.position.y = float(grasp_pos[1])
        cmd.pose.position.z = float(t_xyz[2])
        cmd.pose.orientation.x = self.approach_orientation[0]
        cmd.pose.orientation.y = self.approach_orientation[1]
        cmd.pose.orientation.z = self.approach_orientation[2]
        cmd.pose.orientation.w = self.approach_orientation[3]
        cmd.speed = 20

        self.movel_pub.publish(cmd)
        self.get_logger().info('>> MoveL: Grasping...')
        self.create_timer(2.5, self.close_gripper)

    def close_gripper(self):
        """关闭夹爪"""
        if self.state != 'GRASP':
            return
        self.state = 'CLOSE'

        cmd = Gripperpick()
        cmd.speed = 500
        cmd.force = 500
        self.gripper_pick_pub.publish(cmd)
        self.get_logger().info('>> Closing gripper...')
        self.create_timer(2.0, self.retract)

    def retract(self):
        """退回"""
        if self.state != 'CLOSE':
            return
        self.state = 'RETRACT'

        t_xyz = [self.target.x, self.target.y, self.target.z]
        c_xyz = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z]

        # 计算40cm处的退回点
        back_pos, _ = self.calculate_aligned_pose(t_xyz, c_xyz, distance_from_target=0.40)

        cmd = Movel()
        cmd.pose.position.x = float(back_pos[0])
        cmd.pose.position.y = float(back_pos[1])
        cmd.pose.position.z = float(t_xyz[2])
        cmd.pose.orientation.x = self.approach_orientation[0]
        cmd.pose.orientation.y = self.approach_orientation[1]
        cmd.pose.orientation.z = self.approach_orientation[2]
        cmd.pose.orientation.w = self.approach_orientation[3]
        cmd.speed = 40

        self.movel_pub.publish(cmd)
        self.get_logger().info('>> Retracting...')
        self.state = 'DONE'
        self.get_logger().info('Grasping cycle completed!')
    
    def destroy_node(self):
        """清理资源"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    node = GraspBottleExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pipeline.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()