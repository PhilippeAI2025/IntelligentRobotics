#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('monitor')

        self.sub = self.create_subscription(
            Float32,
            '/battery_voltage',
            self.callback,
            10)
        self.sub
        self.get_logger().info('Battery Monitor gestart. Wachten op data...')

    def callback(self, msg):
        voltage = msg.data

        if voltage < 11.5:
            self.get_logger().warn(f'WAARSCHUWING: Voltage te laag! ({voltage} V)')
        else:
            self.get_logger().info(f'Huidige voltage: {voltage} V')

def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
