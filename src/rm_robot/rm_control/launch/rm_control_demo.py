#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time

# 导入 RealMan 自定义消息接口
# 依据: https://github.com/RealManRobot/ros2_rm_robot/tree/humble/rm_ros_interfaces
from rm_ros_interfaces.msg import MoveJ, GripperSet

class RMControlNode(Node):
    def __init__(self):
        super().__init__('rm_control_demo_node')

        # 1. 创建机械臂运动发布者
        # 话题名称通常为 /rm_driver/movej_cmd，用于发送关节角度指令
        self.arm_movej_pub = self.create_publisher(
            MoveJ, 
            '/rm_driver/movej_cmd', 
            10
        )

        # 2. 创建夹爪控制发布者
        # 依据文档: https://develop.realman-robotics.com/robot/ros2/ros2Description/#gripperset-msg
        # 话题名称通常为 /rm_driver/gripper_set_cmd 或 /rm_driver/gripper_cmd
        # 注意：请通过 'ros2 topic list' 确认实际话题名称，通常是下面这个：
        self.gripper_pub = self.create_publisher(
            GripperSet, 
            '/rm_driver/gripper_set_cmd', 
            10
        )

        self.get_logger().info('RealMan 机械臂控制节点已启动...')

    def move_joint(self, joint_angles, speed=0.5):
        """
        控制机械臂运动到指定关节角度
        :param joint_angles: 列表，包含6个或7个关节角度（弧度制）
        :param speed: 运动速度 (0.0 ~ 1.0)
        """
        msg = MoveJ()
        msg.joint = [float(x) for x in joint_angles]
        msg.speed = float(speed)
        
        # 0 代表默认轨迹规划
        # 注意：根据 msg 定义，有些版本可能不需要 mode 字段，视具体 msg 定义而定
        # 如果报错提示缺少字段，请检查 msg 定义
        
        self.arm_movej_pub.publish(msg)
        self.get_logger().info(f'发送关节运动指令: {joint_angles}, 速度: {speed}')

    def set_gripper(self, position):
        """
        控制夹爪开合
        :param position: 整数 (1 ~ 1000)。
                         通常 1000 代表全开，1 代表全闭（或者是反过来的，视具体夹爪型号而定）。
                         根据文档 GripperSet.msg 包含 uint16 position
        """
        msg = GripperSet()
        msg.position = int(position)
        
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'发送夹爪指令，位置: {position}')

def main(args=None):
    rclpy.init(args=args)
    node = RMControlNode()

    # 等待发布者与订阅者建立连接
    time.sleep(1.0)

    try:
        # === 动作 1: 张开夹爪 ===
        # 假设 1000 是最大行程（张开）
        node.set_gripper(1000)
        time.sleep(2.0) # 等待执行

        # === 动作 2: 移动到位置 A (单位：弧度) ===
        # 请确保这些角度对于你的机器人是安全的！
        # 这是一个示例姿态（大约垂直向上或初始位置）
        pose_a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        # 如果是7轴，需要7个数据
        if len(pose_a) == 6: # 假设是 RM65 等6轴
            node.move_joint(pose_a, speed=0.3)
        time.sleep(5.0) # 等待运动完成（开环控制简单延时）

        # === 动作 3: 移动到位置 B ===
        # 示例：第一个关节转动 30 度 (0.52 弧度)，第二个关节动一下
        pose_b = [0.5, 0.3, 0.0, 0.0, 0.0, 0.0]
        node.move_joint(pose_b, speed=0.3)
        time.sleep(5.0)

        # === 动作 4: 闭合夹爪 (抓取) ===
        # 假设 500 是中间位置，或者 1 是闭合
        node.set_gripper(100) 
        time.sleep(2.0)

        node.get_logger().info('任务序列完成。')

    except KeyboardInterrupt:
        node.get_logger().info('节点被手动停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()