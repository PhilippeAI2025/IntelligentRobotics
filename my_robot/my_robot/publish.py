#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class BatteryPublisher(Node):

    def __init__(self):
        super().__init__('publish')

        self.pub = self.create_publisher(Float32, '/battery_voltage', 10)

        timer_period = 0.50
        self.timer = self.create_timer(timer_period, self.tick)

        self.get_logger().info('Battery Publisher node is gestart. Publiceren elke 60 sec...')

    def tick(self):
        msg = Float32()
        msg.data = round(random.uniform(10.0, 11.8), 2)

        self.pub.publish(msg)
        self.get_logger().info(f'Publishing voltage: {msg.data} V')

def main():
    rclpy.init()
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
