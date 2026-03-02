#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        # --- PUBLISHERS (Vers Foxglove) ---
        self.status_rpi_pub = self.create_publisher(Bool, 'status/rpi', 10)
        self.status_mega_pub = self.create_publisher(Bool, 'status/mega', 10)
        self.status_uno_pub = self.create_publisher(Bool, 'status/uno', 10)

        # --- SUBSCRIBERS (Écoute du réseau) ---
        self.mega_sub = self.create_subscription(String, 'mega/raw', self.mega_cb, 10)
        self.uno_sub = self.create_subscription(String, 'uno/raw', self.uno_cb, 10)

        # --- CHRONOMÈTRES INTERNES ---
        now = self.get_clock().now()
        self.last_mega_time = now
        self.last_uno_time = now

        # Timer de vérification (1 fois par seconde)
        self.create_timer(1.0, self.check_status)
        
        self.get_logger().info("System Monitor (Watchdog) DÉMARRÉ. Surveillance active.")

    def mega_cb(self, msg):
        self.last_mega_time = self.get_clock().now()

    def uno_cb(self, msg):
        self.last_uno_time = self.get_clock().now()

    def check_status(self):
        now = self.get_clock().now()

        # 1. Analyse de la Mega
        delta_mega = (now - self.last_mega_time).nanoseconds / 1e9
        is_mega_alive = delta_mega < 3.0
        self.status_mega_pub.publish(Bool(data=is_mega_alive))

        # 2. Analyse de la Uno
        delta_uno = (now - self.last_uno_time).nanoseconds / 1e9
        is_uno_alive = delta_uno < 3.0
        self.status_uno_pub.publish(Bool(data=is_uno_alive))

        # 3. Analyse de la Raspberry Pi
        # Astuce : Si la Mega OU la Uno communiquent, c'est que la Pi fait bien le pont !
        # Si les deux ponts sont muets depuis 3 secondes, on considère la Pi comme déconnectée du réseau.
        is_rpi_alive = is_mega_alive or is_uno_alive
        self.status_rpi_pub.publish(Bool(data=is_rpi_alive))

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()