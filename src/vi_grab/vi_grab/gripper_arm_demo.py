#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import math

from rm_ros_interfaces.msg import Movej, Gripperset, Gripperpick # pyright: ignore[reportMissingImports]
from std_msgs.msg import Bool

#==============================================================================
# RealMan 机械臂 固定流程抓取放置 示例
class RMPickPlaceNode(Node):
    def __init__(self):
        super().__init__('rm_pick_place_node')

        #机械臂
        self.movej_pub = self.create_publisher(
            Movej,
            '/rm_driver/movej_cmd',
            10
        )

        #夹爪
        self.gripper_pos_pub = self.create_publisher(
            Gripperset,
            '/rm_driver/set_gripper_position_cmd',
            10
        )

        self.gripper_pick_pub = self.create_publisher(
            Gripperpick,
            '/rm_driver/set_gripper_pick_cmd',
            10
        )

        self.create_subscription(
            Bool,
            '/rm_driver/set_gripper_position_result',
            self.gripper_result_cb,
            10
        )
        self.create_subscription(
            Bool,
            '/rm_driver/set_gripper_pick_result',
            self.gripper_result_cb,
            10
        )

        self.get_logger().info('RealMan Pick-and-Place 节点已启动')

    #工具函数

    @staticmethod
    def deg2rad(deg_list):
        """角度(°) → 弧度(rad)"""
        return [math.radians(d) for d in deg_list]

    def gripper_result_cb(self, msg: Bool):
        self.get_logger().info(
            f'Gripper result: {"Success" if msg.data else "Fail"}'
        )

    #手臂控制

    def move_joint_rad(self, joints_rad, speed=30):
        """底层接口：只接受弧度"""
        if len(joints_rad) not in (6, 7):
            self.get_logger().error('关节数量必须是 6 或 7')
            return

        msg = Movej()
        msg.joint = [float(j) for j in joints_rad]
        msg.speed = int(speed)

        self.movej_pub.publish(msg)
        self.get_logger().info(
            f'MoveJ(rad) -> {["%.2f" % j for j in joints_rad]}, speed={speed}'
        )

    def move_joint_deg(self, joints_deg, speed=30):
        """对外接口：用角度（°）"""
        joints_rad = self.deg2rad(joints_deg)
        self.move_joint_rad(joints_rad, speed)

    #夹爪控制

    def open_gripper(self, position=1000):
        msg = Gripperset()
        msg.position = position
        msg.block = True
        msg.timeout = 10

        self.gripper_pos_pub.publish(msg)
        self.get_logger().info(f'打开夹爪到位置 {position}')

    def force_grip(self, speed=500, force=400):
        msg = Gripperpick()
        msg.speed = speed
        msg.force = force
        msg.block = True
        msg.timeout = 10

        self.gripper_pick_pub.publish(msg)
        self.get_logger().info(
            f'力控抓取 speed={speed}, force={force}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RMPickPlaceNode()

    time.sleep(1.0)

    try:
        #1.初始位（角度）
        home = [0, 0, 0, 0, 0, 0]
        node.move_joint_deg(home, speed=30)
        time.sleep(2)

        #2.张开夹爪
        node.open_gripper(1000)
        time.sleep(1)

        #3.抓取位
        pre_grasp = [0, 10, -70, -30, 90, 0]
        node.move_joint_deg(pre_grasp, speed=25)
        time.sleep(3)

        #4.抓取位
        grasp = [0, 15, -80, -20, 90, 0]
        node.move_joint_deg(grasp, speed=15)
        time.sleep(4)

        #5.抓取
        node.force_grip(speed=500, force=400)
        time.sleep(2)

        #6.抬起
        node.move_joint_deg(pre_grasp, speed=20)
        time.sleep(2)

        #7.运输
        mid = [0, 10, -70, -30, 90, 0]
        node.move_joint_deg(mid, speed=25)
        time.sleep(4)

        #8.放置
        place = [0, 10, -70, -30, 90, 0]
        node.move_joint_deg(place, speed=25)
        time.sleep(3)

        #9.松开
        node.open_gripper(1000)
        time.sleep(3)

        #10.回Home
        node.move_joint_deg(home, speed=30)
        time.sleep(2)

        node.get_logger().info('Pick-and-Place 流程完成')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
