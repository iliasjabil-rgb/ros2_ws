#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class RicaSrsTeleop(Node):
    def __init__(self):
        super().__init__('rica_srs_teleop')

        # --- ABONNEMENTS ---
        # 1. Flux Xbox pour la base RICA
        self.xbox_sub = self.create_subscription(Joy, '/xbox/joy', self.xbox_callback, 10)
        
        # 2. Flux Thrustmaster pour le robot SRS
        self.tm_sub = self.create_subscription(Joy, '/thrustmaster/joy', self.tm_callback, 10)

        # --- PUBLICATIONS ---
        self.mega_cmd_pub = self.create_publisher(String, 'mega/command', 10)
        self.uno_cmd_pub = self.create_publisher(String, 'uno/cmd', 10)
        # Note: Le client TCP pour RICA sera intégré ici ou via un autre topic dédié

        self.get_logger().info("🚀 Téléop unifiée prête : Xbox -> RICA | Thrustmaster -> SRS")

    def xbox_callback(self, msg):
        """ Logique pour la base mobile RICA (Navigation) """
        # Exemple : Homme-mort sur bouton RB (index 5)
        if msg.buttons[5]:
            # Logique TCP_NODELAY vers 192.168.0.2 ici
            pass

    def tm_callback(self, msg):
        """ Logique pour le robot SRS (Manipulation) """
        # Exemple : Le stick principal contrôle l'axe Z de la MEGA
        z_speed = msg.axes[1] # Stick vertical
        if abs(z_speed) > 0.05: # Filtre anti-tremblement
            cmd = f'{{"cmd":"move", "axis":1, "steps":{int(z_speed*100)}}}'
            self.mega_cmd_pub.publish(String(data=cmd))

def main(args=None):
    rclpy.init(args=args)
    node = RicaSrsTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
