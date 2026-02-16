
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')

        # --- SUBSCRIPTION (Ordres de Foxglove) ---
        self.create_subscription(String, '/cmd_mode', self.mode_callback, 10)

        # --- PUBLISHERS (Commandes vers les cartes) ---
        self.pub_uno = self.create_publisher(String, 'uno/cmd', 10)
        self.pub_mega = self.create_publisher(String, 'mega/cmd', 10)

        self.current_mode = "IDLE"
        self.get_logger().info("SYSMAP BRAIN : Prêt. En attente d'ordres...")

    def mode_callback(self, msg):
        # Récupère le mode (ex: {"data": "CLEAN"})
        # Si Foxglove envoie un JSON, il faut parfois le parser, 
        # mais si c'est un bouton "String", on prend direct msg.data
        raw_mode = msg.data.upper()

        # Petit nettoyage si on reçoit du JSON par erreur
        if "{" in raw_mode:
            try:
                obj = json.loads(msg.data)
                new_mode = obj.get("data", "IDLE").upper()
            except:
                new_mode = "IDLE"
        else:
            new_mode = raw_mode

        if new_mode == self.current_mode:
            return

        self.current_mode = new_mode
        self.get_logger().info(f"--- CHANGEMENT MODE : {self.current_mode} ---")
        self.apply_mode_logic()

    def apply_mode_logic(self):
        # --- IDLE : TOUT STOP ---
        if self.current_mode == "IDLE":
            self.send_uno(cmd="led", r=255, g=0, b=0) # ROUGE
            self.send_uno(cmd="relay", id=3, state=0) # Aspi OFF
            self.send_mega(cmd="stop")

        # --- MANUAL : VERT ---
        elif self.current_mode == "MANUAL":
            self.send_uno(cmd="led", r=0, g=255, b=0) # VERT
            self.send_mega(cmd="stop") # On attend les joysticks

        # --- CLEAN : BLEU + AVANCE ---
        elif self.current_mode == "CLEAN":
            self.send_uno(cmd="led", r=0, g=0, b=255) # BLEU
            self.send_uno(cmd="relay", id=3, state=1) # Aspi ON
            # Exemple : Avancer l'axe 1 de 5000 pas
            self.send_mega(cmd="move", axis=1, steps=5000, speed=800)

    # --- Helpers JSON ---
    def send_uno(self, **kwargs):
        msg = String()
        msg.data = json.dumps(kwargs)
        self.pub_uno.publish(msg)

    def send_mega(self, **kwargs):
        msg = String()
        msg.data = json.dumps(kwargs)
        self.pub_mega.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RobotBrain())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
