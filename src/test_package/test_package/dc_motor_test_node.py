#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class DcMotorTestNode(Node):
    def __init__(self):
        super().__init__('dc_motor_test_node')

        self.declare_parameter('topic', 'dc_motor/cmd')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('cycle_s', 4.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.cycle_s = self.get_parameter('cycle_s').get_parameter_value().double_value

        self.pub = self.create_publisher(Float32, self.topic, 10)

        self.dt = 0.05
        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            f'DC motor test node publishing on "{self.topic}" '
            f'(max_speed={self.max_speed}, cycle={self.cycle_s}s)'
        )

    def timer_callback(self):
        self.t += self.dt
        if self.cycle_s <= 0.0:
            value = 0.0
        else:
            phase = 2.0 * math.pi * (self.t % self.cycle_s) / self.cycle_s
            value = self.max_speed * math.sin(phase)

        msg = Float32()
        msg.data = float(value)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DcMotorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
