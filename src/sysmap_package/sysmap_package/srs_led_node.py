#!/usr/bin/env python3
"""
============================================================================
FICHIER : srs_led_node.py
ROLE    : Nœud ROS 2 générant l'animation visuelle (Effet SRS).
          S'abonne à /led/effect/cmd (0=Off, 1=On) et publie sur /uno/cmd.
============================================================================
"""
 
import rclpy
from rclpy.node import Node
import random
import json
from std_msgs.msg import String, Int32, Bool
 
class SrsLedNode(Node):
    def __init__(self):
        super().__init__('srs_led_node')
        # Publisher vers l'Arduino Uno
        self.publisher_ = self.create_publisher(String, '/uno/cmd', 10)
        # Subscriber pour recevoir les ordres de Foxglove (0 ou 1)
        self.subscription = self.create_subscription(
            Bool,
            '/led_effect/cmd',
            self.effect_cmd_callback,
            10
        )
        # Timer d'animation (15ms)
        self.timer = self.create_timer(0.005, self.timer_callback) 
        self.led_count = 115 # À ajuster selon le nombre de LEDs
        # États
        self.effect_enabled = 0
        self.was_enabled = False
        self.get_logger().info("Nœud d'animation SRS démarré. En attente sur /led_effect/cmd")
 
    def effect_cmd_callback(self, msg):
        """Met à jour l'état de l'animation en fonction du message reçu"""
        if msg.data in [0, 1]:
            self.effect_enabled = msg.data
            state_str = "ACTIVÉE" if self.effect_enabled == 1 else "DÉSACTIVÉE"
            self.get_logger().info(f"Animation SRS {state_str}")
 
    def timer_callback(self):
        if self.effect_enabled == 1:
            # L'animation tourne
            self.was_enabled = True
            # 1. Éteindre 3 LEDs au hasard
            for _ in range(3):
                off_id = random.randint(0, self.led_count - 1)
                self.send_led_cmd(off_id, 0, 0, 0, 255)
 
            # 2. Allumer 1 LED avec une couleur aléatoire de la charte SRS
            pixel_id = random.randint(0, self.led_count - 1)
            choice = random.randint(0, 2)
            if choice == 0:
                r, g, b = 255, 255, 0   # Jaune
            elif choice == 1:
                r, g, b = 255, 255, 255 # Blanc
            else:
                r, g, b = 0, 0, 255     # Bleu foncé
 
            self.send_led_cmd(pixel_id, r, g, b, 255)
        else:
            # L'animation est coupée, on vérifie s'il faut éteindre le ruban
            if self.was_enabled:
                # On envoie un ordre global (id = -1) avec luminosité à 0
                self.send_led_cmd(-1, 0, 0, 0, 0)
                self.was_enabled = False
 
    def send_led_cmd(self, led_id, r, g, b, alpha):
        """Formate et envoie la trame JSON à l'Arduino"""
        msg = String()
        data = {
            "cmd": "led",
            "id": led_id,
            "r": r,
            "g": g,
            "b": b,
            "a": alpha
        }
        msg.data = json.dumps(data, separators=(',', ':'))
        self.publisher_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = SrsLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Par sécurité, on éteint les LEDs en quittant le nœud
        node.send_led_cmd(-1, 0, 0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()