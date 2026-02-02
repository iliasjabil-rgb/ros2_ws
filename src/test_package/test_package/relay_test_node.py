#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class RelayTestNode(Node):
    def __init__(self):
        super().__init__('relay_test_node')

        self.declare_parameter('topic', 'relay/cmd')
        self.declare_parameter('toggle_period_s', 1.0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.toggle_period_s = self.get_parameter('toggle_period_s').get_parameter_value().double_value

        self.pub = self.create_publisher(Bool, self.topic, 10)

        self.state = False
        self.timer = self.create_timer(self.toggle_period_s, self.timer_callback)

        self.get_logger().info(
            f'Relay test node toggling "{self.topic}" every {self.toggle_period_s}s'
        )

    def timer_callback(self):
        self.state = not self.state
        msg = Bool()
        msg.data = self.state
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RelayTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
