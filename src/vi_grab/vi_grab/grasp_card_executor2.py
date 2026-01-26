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
import threading  # [优化] 引入多线程支持
import torch      # [优化] 显存与半精度管理
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
            self.get_logger().error("配置文件加载失败")
            return

        # ==================== 1. 硬件加速与多线程初始化 ====================
        # [优化] 线程安全的数据交换缓存
        self.frame_lock = threading.Lock()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        
        # [优化] 预创建 GPU 矩阵，避免在循环中重复申请/释放显存
        try:
            self.gpu_mat_bgr = cv2.cuda_GpuMat()
            self.gpu_mat_hsv = cv2.cuda_GpuMat()
            self.use_cuda = True
            self.get_logger().info("已启用 OpenCV CUDA 硬件加速")
        except Exception:
            self.get_logger().warn("OpenCV CUDA 模块不可用，回退至 CPU 模式")
            self.use_cuda = False

        # [优化] 初始化图片保存路径
        self.photo_dir = os.path.join(os.getcwd(), self.config['paths']['photo_save_dir'])
        if not os.path.exists(self.photo_dir):
            os.makedirs(self.photo_dir)

        # ==================== 2. 手眼标定与相机初始化 ====================
        self.ROT_CAM_TO_EE = np.array(self.config['camera']['rotation_cam_to_ee'])
        self.TRANS_CAM_TO_EE = np.array(self.config['camera']['translation_cam_to_ee'])

        self.pipeline = rs.pipeline()
        self.config_rs = rs.config()
        cam_cfg = self.config['camera']
        # 建议：若要激活 NVJPG 硬件解码，可尝试将格式改为 rs.format.mjpeg
        self.config_rs.enable_stream(rs.stream.color, cam_cfg['width'], cam_cfg['height'], rs.format.bgr8, cam_cfg['fps'])
        self.config_rs.enable_stream(rs.stream.depth, cam_cfg['width'], cam_cfg['height'], rs.format.z16, cam_cfg['fps'])
        self.align = rs.align(rs.stream.color)
        
        try:
            self.profile = self.pipeline.start(self.config_rs)
            # [优化] 启动独立相机采集线程，消除 wait_for_frames 对 ROS 频率的影响
            self.cam_thread = threading.Thread(target=self._camera_polling_worker, daemon=True)
            self.cam_thread.start()
            self.get_logger().info("相机硬件加速采集线程已启动")
            self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        except Exception as e:
            self.get_logger().error(f"相机启动失败: {e}")
            return

        # ==================== 3. ROS & YOLO 初始化 ====================
        model_cfg = self.config['model']
        self.target_class_name = model_cfg['target_class']

        try:
            # [优化] 将模型加载至 GPU 并强制使用 FP16 半精度加速
            self.model = YOLO(model_cfg['yolo_model_path']).to('cuda')
            if hasattr(self.model, 'model'):
                self.model.model.half() 
            self.get_logger().info("YOLOv11 模型已加载至 GPU (FP16 模式)")
        except Exception as e:
            self.get_logger().error(f'YOLO 加载失败: {e}')

        self.create_subscription(Pose, '/rm_driver/udp_arm_position', self.arm_pose_callback, 10)
        self.current_arm_pose = None
        
        self.movejp_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.movel_pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.gripper_pick_pub = self.create_publisher(Gripperpick, '/rm_driver/set_gripper_pick_cmd', 10)
        self.gripper_set_pub = self.create_publisher(Gripperset, '/rm_driver/set_gripper_position_cmd', 10)

        home_cfg = self.config['home_position']
        self.home_x, self.home_y, self.home_z = home_cfg['x'], home_cfg['y'], home_cfg['z']
        self.home_quat = None
        self.locks = set() 
        self.target_grasp_base = None
        self.target_grasp_quat = None
        
        self.timer = self.create_timer(1.0, self.check_ready_and_start)

    # ==================== 硬件加速专用方法 ====================

    def _camera_polling_worker(self):
        """[新增] 生产者线程：持续从硬件获取帧并存入显存/内存缓存"""
        while rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                aligned_frames = self.align.process(frames)
                c = aligned_frames.get_color_frame()
                d = aligned_frames.get_depth_frame()
                if c and d:
                    img_bgr = np.asanyarray(c.get_data())
                    with self.frame_lock:
                        self.latest_color_frame = img_bgr
                        self.latest_depth_frame = d
            except Exception:
                continue

    def get_latest_frames_from_cache(self):
        """[优化] 消费者方法：非阻塞地从缓存获取最新画面"""
        with self.frame_lock:
            if self.latest_color_frame is None:
                return None, None
            return self.latest_color_frame.copy(), self.latest_depth_frame

    # ==================== 动作流程 (整合加速逻辑) ====================

    def step2_align_and_descend(self):
        if 'step2' in self.locks: return
        self.locks.add('step2')
        self.get_logger().info("Step 2: 视觉对齐 (GPU 加速预处理)...")

        img, d_frame = self.get_latest_frames_from_cache()
        if img is None: 
            self.switch_step(0.5, self.step2_align_and_descend)
            return

        # [优化] 使用 GPU (VIC 单元) 执行 BGR2HSV 转换
        if self.use_cuda:
            self.gpu_mat_bgr.upload(img)
            self.gpu_mat_hsv = cv2.cuda.cvtColor(self.gpu_mat_bgr, cv2.COLOR_BGR2HSV)
            hsv = self.gpu_mat_hsv.download()
        else:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        step2_cfg = self.config['steps']['step2']
        mask = cv2.inRange(hsv, np.array(step2_cfg['hsv_lower']), np.array(step2_cfg['hsv_upper']))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            target_z = self.current_arm_pose.position.z - step2_cfg['z_offset']
            self.send_movel(self.current_arm_pose.position.x, self.current_arm_pose.position.y, 
                            target_z, self.home_quat, speed=step2_cfg['speed'])
            self.switch_step(step2_cfg['timeout'], self.step4_yolo)
            return

        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        cx, cy = int(rect[0][0]), int(rect[0][1])
        real_depth = self.get_robust_depth(d_frame, cx, cy) or 0.4
        p_base = self.transform_pixel_to_base(cx, cy, real_depth, cam_dy=step2_cfg['camera_y_offset'])
        
        box = np.int32(cv2.boxPoints(rect))
        p0, p1 = box[0], box[1]
        angle = math.atan2(p1[1]-p0[1], p1[0]-p0[0])

        q_curr = [self.current_arm_pose.orientation.x, self.current_arm_pose.orientation.y, 
                  self.current_arm_pose.orientation.z, self.current_arm_pose.orientation.w]
        r_final = R.from_quat(q_curr) * R.from_euler('z', angle, degrees=False)
        self.target_grasp_quat = r_final.as_quat()

        target_z = self.current_arm_pose.position.z - step2_cfg['descend_distance']
        if p_base is not None:
            self.send_movel(p_base[0], p_base[1], target_z, self.target_grasp_quat, speed=step2_cfg['speed'])
        
        self.switch_step(step2_cfg['timeout'], self.step4_yolo)

    def step4_yolo(self):
        if 'step4' in self.locks: return
        self.locks.add('step4')
        self.get_logger().info("Step 4: YOLO 最终精确定位 (FP16 加速)...")
        
        img, d_frame = self.get_latest_frames_from_cache()
        if img is None: 
            self.switch_step(0.5, self.step4_yolo)
            return

        # [优化] 使用 FP16 推理并禁用梯度追踪，显著降低 CPU 负载与显存占用
        model_cfg = self.config['model']
        with torch.no_grad():
            results_list = self.model(img, 
                                     conf=model_cfg['detection_confidence'], 
                                     half=True, 
                                     device='cuda:0',
                                     verbose=False)
        results = results_list[0]
        
        candidates = []
        if results.boxes:
            for box in results.boxes:
                name = self.model.names[int(box.cls[0])]
                if self.target_class_name and name != self.target_class_name: continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                d = self.get_robust_depth(d_frame, cx, cy)
                if d > 0: candidates.append({'cx':cx, 'cy':cy, 'd':d})
        
        if not candidates:
            self.get_logger().warn("YOLO未发现目标，重试...")
            self.locks.remove('step4') # 允许重试
            self.switch_step(0.5, self.step4_yolo)
            return
        
        target = candidates[0]
        step6_cfg = self.config['steps']['step6']
        p_base = self.transform_pixel_to_base(target['cx'], target['cy'], target['d'], cam_dy=step6_cfg['camera_y_offset'])
        self.target_grasp_base = p_base
        
        # [优化] 显存清理，防止 8G 内存持续堆积
        torch.cuda.empty_cache()
        self.get_logger().info(f"锁定目标坐标: {p_base}")
        self.switch_step(0.5, self.step5_move_horizontal)

    # ==================== 整合原程序的工具函数与动作逻辑 ====================

    def _load_config(self):
        from ament_index_python.packages import get_package_share_directory
        possible_paths = [
            os.path.join(os.path.dirname(__file__), 'config', 'grasp_config.yaml'),
            os.path.join(os.getcwd(), 'src', 'vi_grab', 'vi_grab', 'config', 'grasp_config.yaml'),
        ]
        try:
            share_dir = get_package_share_directory('vi_grab')
            possible_paths.insert(1, os.path.join(share_dir, 'config', 'grasp_config.yaml'))
        except Exception: pass
        for path in possible_paths:
            if path and os.path.exists(path):
                with open(path, 'r', encoding='utf-8') as f:
                    self.get_logger().info(f"加载配置文件: {path}")
                    return yaml.safe_load(f)
        return None

    def arm_pose_callback(self, msg: Pose):
        self.current_arm_pose = msg

    def check_ready_and_start(self):
        if self.current_arm_pose is not None and 'start' not in self.locks:
            self.locks.add('start')
            self.home_quat = self.get_vertical_down_quat()
            self.step1_move_to_home()

    def get_vertical_down_quat(self):
        z_axis, y_axis = np.array([0.0, 0.0, -1.0]), np.array([1.0, 0.0, 0.0])
        x_axis = np.cross(y_axis, z_axis)
        return R.from_matrix(np.column_stack((x_axis, y_axis, z_axis))).as_quat()

    def get_place_quat(self):
        z_axis, y_axis = np.array([0.0, -1.0, 0.0]), np.array([0.0, 0.0, -1.0])
        x_axis = np.cross(y_axis, z_axis)
        return R.from_matrix(np.column_stack((x_axis, y_axis, z_axis))).as_quat()

    def transform_pixel_to_base(self, u, v, depth, cam_dy=0.0):
        if self.current_arm_pose is None: return None
        p_cam = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)
        p_cam_h = np.array([p_cam[0], p_cam[1] + cam_dy, p_cam[2], 1.0])
        T_ee_cam = np.eye(4)
        T_ee_cam[:3, :3], T_ee_cam[:3, 3] = self.ROT_CAM_TO_EE, self.TRANS_CAM_TO_EE
        q, p = self.current_arm_pose.orientation, self.current_arm_pose.position
        T_base_ee = np.eye(4)
        T_base_ee[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_base_ee[:3, 3] = [p.x, p.y, p.z]
        return (T_base_ee @ (T_ee_cam @ p_cam_h))[:3]

    def get_robust_depth(self, depth_frame, u, v):
        r, depths = 2, []
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                if 0 <= u+i < 640 and 0 <= v+j < 480:
                    d = depth_frame.get_distance(u+i, v+j)
                    if 0.1 < d < 1.0: depths.append(d)
        return np.median(depths) if depths else 0.0

    def send_movel(self, x, y, z, quat, speed):
        msg = Movel()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = [float(q) for q in quat]
        msg.speed = speed
        self.movel_pub.publish(msg)

    def send_movej_p(self, x, y, z, quat, speed):
        msg = Movejp()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = [float(q) for q in quat]
        msg.speed = speed
        self.movejp_pub.publish(msg)

    def open_gripper(self):
        msg = Gripperset()
        msg.position = self.config['gripper']['open_position']
        self.gripper_set_pub.publish(msg)

    def close_gripper(self):
        msg = Gripperpick()
        msg.speed, msg.force = self.config['gripper']['pick_speed'], self.config['gripper']['pick_force']
        self.gripper_pick_pub.publish(msg)

    def switch_step(self, delay, callback):
        if hasattr(self, 'timer_step'): self.timer_step.cancel()
        self.timer_step = self.create_timer(delay, callback)

    # ==================== 动作循环 ====================

    def step1_move_to_home(self):
        if 'step1' in self.locks: return
        self.locks.add('step1')
        self.open_gripper()
        self.send_movej_p(self.home_x, self.home_y, self.home_z, self.home_quat, self.config['steps']['step1']['speed'])
        self.switch_step(self.config['steps']['step1']['timeout'], self.step2_align_and_descend)

    def step5_move_horizontal(self):
        if 'step5' in self.locks: return
        self.locks.add('step5')
        tx, ty, tz = self.target_grasp_base
        self.send_movel(tx, ty, self.current_arm_pose.position.z, self.target_grasp_quat, self.config['steps']['step5']['speed'])
        self.switch_step(self.config['steps']['step5']['timeout'], self.step6_move_vertical)

    def step6_move_vertical(self):
        if 'step6' in self.locks: return
        self.locks.add('step6')
        tx, ty, tz = self.target_grasp_base
        z_target = tz + self.config['steps']['step6']['grasp_height_offset']
        self.send_movel(tx, ty, z_target, self.target_grasp_quat, self.config['steps']['step6']['speed'])
        self.switch_step(self.config['steps']['step6']['timeout'], self.step7_close)

    def step7_close(self):
        if 'step7' in self.locks: return
        self.locks.add('step7')
        self.close_gripper()
        self.switch_step(self.config['steps']['step7']['timeout'], self.step8_lift_up)

    def step8_lift_up(self):
        if 'step8' in self.locks: return
        self.locks.add('step8')
        self.send_movel(self.current_arm_pose.position.x, self.current_arm_pose.position.y, self.home_z, self.target_grasp_quat, self.config['steps']['step8']['speed'])
        self.switch_step(self.config['steps']['step8']['timeout'], self.step9_return_home)

    def step9_return_home(self):
        if 'step9' in self.locks: return
        self.locks.add('step9')
        p_cfg = self.config['place_position']
        self.send_movej_p(p_cfg['x'], p_cfg['y'], p_cfg['z'], self.get_place_quat(), self.config['steps']['step9']['speed'])

def main():
    rclpy.init()
    node = VisualServoYoloGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()