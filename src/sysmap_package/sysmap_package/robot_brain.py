#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# Assurez-vous d'avoir le type de message utilisé par votre IA YOLO
# (sudo apt install ros-jazzy-vision-msgs)
from vision_msgs.msg import Detection2DArray 

class RobotBrain(Node):
    def __init__(self):
        super().__init__('robot_brain')

        # --- 1. ÉTATS DE L'IA (FSM) ---
        self.STATE_SEARCH = "SEARCH"
        self.STATE_ALIGN = "ALIGN"
        self.STATE_COLLECT = "COLLECT"
        self.ia_state = self.STATE_SEARCH

        # --- 2. PARAMÈTRES DE CONTRÔLE VISUEL ---
        self.img_width = 640        
        self.img_height = 480
        self.target_x = self.img_width / 2
        self.target_y = self.img_height / 2
        self.center_tolerance = 30
        self.kp = 0.8 # Gain proportionnel pour le mouvement

        # --- 3. PUBLISHERS ---
        # Vos helpers envoient le JSON ici
        self.pub_uno = self.create_publisher(String, 'uno/cmd', 10)
        # J'ai mis 'mega/command' car c'était le topic dans votre sysmap_joystick
        self.pub_mega = self.create_publisher(String, 'mega/command', 10) 
        self.pub_mode = self.create_publisher(String, '/cmd_mode', 10) # Pour forcer le changement de mode

        # --- 4. SUBSCRIBERS ---
        self.create_subscription(String, '/cmd_mode', self.mode_callback, 10)
        self.create_subscription(Detection2DArray, '/video_result/compressed', self.vision_callback, 10)

        self.current_mode = "IDLE"
        self.get_logger().info("🧠 SYSMAP BRAIN : Prêt. En attente d'ordres...")

    # --- CALLBACK CHANGEMENT DE MODE ---
    def mode_callback(self, msg):
        raw_mode = msg.data.upper()
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
        
        # Si on passe en AUTO, on réinitialise l'IA sur la recherche
        if self.current_mode == "AUTO":
            self.ia_state = self.STATE_SEARCH
            
        self.apply_mode_logic()

    # --- LOGIQUE DES MODES (LES COULEURS LED) ---
    def apply_mode_logic(self):
        # --- IDLE : ROUGE ---
        if self.current_mode == "IDLE":
            self.send_uno(cmd="led", r=255, g=0, b=0, a= 255) 
            self.send_uno(cmd="relay", id=3, state=0) # Aspi OFF
            self.send_mega(cmd="stop")

        # --- MANUAL : VERT ---
        elif self.current_mode == "MANUAL":
            self.send_uno(cmd="led", r=0, g=255, b=0, a= 255) 
            self.send_mega(cmd="stop") # On attend les joysticks

        # --- CLEAN : BLEU ---
        elif self.current_mode == "CLEAN":
            self.send_uno(cmd="led", r=0, g=0, b=255, a= 255) 
            self.send_uno(cmd="relay", id=3, state=1) # Aspi ON
            self.send_mega(cmd="move", axis=1, steps=5000, speed=800)

        # --- AUTO (IA) : MAGENTA/VIOLET ---
        elif self.current_mode == "AUTO":
            self.send_uno(cmd="led", r=255, g=0, b=255, a= 255) 
            self.send_uno(cmd="relay", id=3, state=0) # Aspi OFF par sécurité

    # --- LA BOUCLE DE L'IA (ASSERVISSEMENT VISUEL) ---
    def vision_callback(self, msg):
        # L'IA ne tourne QUE si on est en mode AUTO
        if self.current_mode != "AUTO":
            return

        # S'il n'y a aucune détection
        if len(msg.detections) == 0:
            if self.ia_state != self.STATE_SEARCH:
                 self.get_logger().info("Cible perdue. Retour en recherche.")
                 self.ia_state = self.STATE_SEARCH
            return

        # Si on voit une cible pour la première fois
        if self.ia_state == self.STATE_SEARCH:
            self.get_logger().info("🎯 Cible acquise ! Début de l'alignement.")
            self.ia_state = self.STATE_ALIGN

        # --- ALIGNEMENT ---
        elif self.ia_state == self.STATE_ALIGN:
            det = msg.detections[0] # On cible le premier objet détecté
            
            obj_x = det.bbox.center.position.x
            obj_y = det.bbox.center.position.y

            err_x = obj_x - self.target_x
            err_y = obj_y - self.target_y

            centered_x = abs(err_x) <= self.center_tolerance
            centered_y = abs(err_y) <= self.center_tolerance

            if not centered_x:
                # Axe 3 = Tige X (A adapter selon votre robot)
                steps_x = int(err_x * self.kp)
                self.send_mega(cmd="move", axis=3, steps=steps_x, speed=600)

            elif not centered_y:
                # Axe 1 = Colonne Y (A adapter selon votre robot)
                steps_y = int(err_y * self.kp)
                self.send_mega(cmd="move", axis=1, steps=steps_y, speed=600)
            
            if centered_x and centered_y:
                self.get_logger().info("✅ Cible PARFAITEMENT CENTRÉE ! Lancement de la collecte.")
                self.ia_state = self.STATE_COLLECT
                self.execute_collection()

    # --- SÉQUENCE DE COLLECTE ---
    def execute_collection(self):
        AXIS_RETRACT = 4  # Vérin Z

        # 1. Descente
        self.send_uno(cmd="led", r=255, g=165, b=0) # Orange pendant la collecte
        self.get_logger().info("⬇️ Descente du vérin...")
        self.send_mega(cmd="move", axis=AXIS_RETRACT, steps=1500, speed=1000)
        time.sleep(3.0) 

        # 2. Aspiration
        self.get_logger().info("💨 Activation aspirateur...")
        self.send_uno(cmd="relay", id=3, state=1)
        time.sleep(2.0) 

        # 3. Remontée
        self.get_logger().info("⬆️ Remontée du vérin...")
        self.send_mega(cmd="move", axis=AXIS_RETRACT, steps=-1500, speed=1000)
        time.sleep(3.0)

        # 4. Fin
        self.get_logger().info("🛑 Arrêt aspirateur.")
        self.send_uno(cmd="relay", id=3, state=0)

        # 5. Retour en mode Manuel automatique
        self.get_logger().info("🏁 Mission IA terminée. Retour au mode MANUEL.")
        
        msg = String()
        msg.data = "MANUAL"
        self.pub_mode.publish(msg) # On dit à tout le monde qu'on repasse en manuel

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