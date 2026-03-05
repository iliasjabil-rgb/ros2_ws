#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32
import json
import math

class JointStateConverter(Node):
    def __init__(self):
        super().__init__('joint_state_converter')
        
        # --- PUBLISHER (Vers le modèle 3D URDF) ---
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # --- SUBSCRIBERS (Depuis les Arduino) ---
        # 1. Position des 4 moteurs pas-à-pas (Mega)
        self.create_subscription(String, '/mega/telemetry', self.mega_callback, 10)
        # 2. Angle du servo de la pince (Uno) - Remplacez par le bon topic si besoin
        self.create_subscription(Float32, 'uno/servo4/cmd', self.servo_callback, 10) 
        
        # --- FACTEURS DE CONVERSION (LE RÉGLAGE MAGIQUE) ---
        # C'est ici que vous direz combien de mètres représente 1 "pas" moteur.
        # Par exemple : Si 800 pas = 1 cm (0.01 m), alors 1 pas = 0.01 / 800 = 0.0000125
        # Mettez un signe négatif "-" si l'axe 3D part dans le sens inverse du vrai robot.
        self.steps_to_m_z       = 0.0001  # Axe 1 : gliss_poteaux
        self.steps_to_m_stock   = 0.0001  # Axe 2 : gliss_boites
        self.steps_to_m_y       = 0.0001  # Axe 3 : gliss_bras
        self.steps_to_m_retract = 0.0001  # Axe 4 : verrin
        
        # --- MÉMOIRE DES POSITIONS ---
        self.current_z = 0.0
        self.current_stock = 0.0
        self.current_y = 0.0
        self.current_retract = 0.0
        self.current_rot_pince = 0.0 # En radians
        
        self.get_logger().info("🔄 Joint State Converter DÉMARRÉ ! Lien Mega <-> URDF actif.")

    def mega_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if "pos" in data:
                p = data["pos"]
                # p = [z, stock, y, retract] selon votre code Mega
                self.current_z       = p[0] * self.steps_to_m_z
                self.current_stock   = p[1] * self.steps_to_m_stock
                self.current_y       = p[2] * self.steps_to_m_y
                self.current_retract = p[3] * self.steps_to_m_retract
                
                self.publish_joints()
        except Exception as e:
            # On ignore les erreurs de parsing si un JSON est mal formé
            pass

    def servo_callback(self, msg):
        # L'URDF veut des radians (0 à 3.14). Le joystick envoie des degrés (0 à 180).
        deg = msg.data
        # On convertit en radians en centrant la pince (90° = 0 rad)
        self.current_rot_pince = math.radians(deg - 90.0)
        self.publish_joints()

    def publish_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        
        # Ces noms DOIVENT être exactement ceux de votre fichier URDF
        js.name = [
            'gliss_poteaux', 
            'gliss_boites', 
            'gliss_bras', 
            'verrin', 
            'rot_pince'
        ]
        
        # Les positions mathématiques calculées
        js.position = [
            self.current_z,
            self.current_stock,
            self.current_y,
            self.current_retract,
            self.current_rot_pince
        ]
        
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()