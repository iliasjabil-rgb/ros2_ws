#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json

from std_msgs.msg import String, Int32MultiArray, Float32MultiArray, Bool

class MegaDriver(Node):
    def __init__(self):
        super().__init__('mega_driver')
        
        # --- PUBLISHERS (Télémétrie Mega -> ROS 2) ---
        self.status_pub = self.create_publisher(Bool, '/status/mega', 10)
        self.pos_pub = self.create_publisher(Int32MultiArray, 'mega/positions', 10)
        self.pwr_pub = self.create_publisher(Float32MultiArray, 'mega/power', 10)
        self.lim_pub = self.create_publisher(Int32MultiArray, 'mega/limits', 10)
        self.alm_pub = self.create_publisher(Int32MultiArray, 'mega/alarms', 10)
        
        # Vers le Bridge
        self.cmd_raw_pub = self.create_publisher(String, 'mega/cmd_raw', 10)

        # --- SUBSCRIBERS (Commandes & Télémétrie) ---
        self.sub_cmd = self.create_subscription(String, 'mega/command', self.cmd_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/cmd/stop_all', self.stop_callback, 10)
        self.raw_sub = self.create_subscription(String, 'mega/raw', self.raw_callback, 10) # Depuis le bridge
        
        # --- WATCHDOG ---
        self.last_msg_time = self.get_clock().now()
        self.create_timer(1.0, self.check_connection_callback)

        self.get_logger().info('🧠 MEGA Driver unifié démarré (Intelligence séparée du transport).')

    def check_connection_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        is_connected = elapsed < 3.0 
        self.status_pub.publish(Bool(data=is_connected))

    def publish_cmd(self, obj: dict):
        """Envoie une commande JSON au bridge"""
        msg = String()
        msg.data = json.dumps(obj, separators=(',', ':'))
        self.cmd_raw_pub.publish(msg)

    def cmd_callback(self, msg: String):
        """Fait suivre les commandes du Joystick vers le bridge"""
        out_msg = String()
        out_msg.data = msg.data.strip()
        self.cmd_raw_pub.publish(out_msg)

    def stop_callback(self, msg: Bool):
        """Coupe immédiatement les moteurs si l'arrêt d'urgence est activé"""
        if msg.data:
            self.get_logger().error("🛑 ARRÊT D'URGENCE REÇU PAR LA MEGA !")
            self.publish_cmd({"cmd": "stop"})

    def raw_callback(self, msg: String):
        """Reçoit la télémétrie du Bridge"""
        self.last_msg_time = self.get_clock().now()
        line = msg.data.strip()
        if not line: return
        self.process_json(line)

    def process_json(self, line):
        """Parse le JSON et publie sur les topics correspondants"""
        try:
            data = json.loads(line)
            if data.get("src") == "mega":
                if "pos" in data:
                    self.pos_pub.publish(Int32MultiArray(data=[int(x) for x in data["pos"]]))
                if "pwr" in data:
                    self.pwr_pub.publish(Float32MultiArray(data=[float(x) for x in data["pwr"]]))
                if "mr" in data:
                    self.lim_pub.publish(Int32MultiArray(data=[int(x) for x in data["mr"]]))
                if "alm" in data:
                    alarms = [int(x) for x in data["alm"]]
                    self.alm_pub.publish(Int32MultiArray(data=alarms))
                    if sum(alarms) > 0:
                        self.get_logger().error(f"⚠️ ALARME DRIVER DÉTECTÉE (M1..M4) : {alarms}")
        except json.JSONDecodeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MegaDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()