#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16

class SetToolVoltage(Node):
    def __init__(self):
        super().__init__('set_tool_voltage')
        pub = self.create_publisher(UInt16, '/rm_driver/set_tool_voltage_cmd', 10)
        msg = UInt16(data=2)  # 2 为12V（文档：0=0V, 2=12V, 3=24V）
        pub.publish(msg)
        self.get_logger().info('Set tool voltage to 12V')
        rclpy.shutdown()  # 发布后立即退出

def main():
    rclpy.init()
    SetToolVoltage()

if __name__ == '__main__':
    main()