#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')

        self.declare_parameter('topic', 'servo/cmd')
        self.declare_parameter('min_deg', 0.0)
        self.declare_parameter('max_deg', 180.0)
        self.declare_parameter('cycle_s', 4.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.min_deg = self.get_parameter('min_deg').get_parameter_value().double_value
        self.max_deg = self.get_parameter('max_deg').get_parameter_value().double_value
        self.cycle_s = self.get_parameter('cycle_s').get_parameter_value().double_value

        self.pub = self.create_publisher(Float32, self.topic, 10)

        self.dt = 0.05
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'Servo test node publishing on "{self.topic}" '
            f'({self.min_deg}° → {self.max_deg}°, cycle={self.cycle_s}s)'
        )

    def timer_callback(self):
        self.t += self.dt
        if self.cycle_s <= 0.0:
            angle = self.min_deg
        else:
            phase = 2.0 * math.pi * (self.t % self.cycle_s) / self.cycle_s
            norm = 0.5 * (math.sin(phase) + 1.0)  # 0 → 1
            angle = self.min_deg + norm * (self.max_deg - self.min_deg)

        msg = Float32()
        msg.data = float(angle)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
