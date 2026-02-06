#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        # Publie 'True' sur /status/rpi chaque seconde
        self.pub = self.create_publisher(Bool, 'status/rpi', 10)
        self.create_timer(1.0, self.publish_heartbeat)
        self.get_logger().info("Moniteur Système Démarré (Heartbeat RPi Actif)")

    def publish_heartbeat(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
