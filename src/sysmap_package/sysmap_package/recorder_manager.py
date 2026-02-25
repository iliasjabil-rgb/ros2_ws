#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
from datetime import datetime
import os

class RecorderManager(Node):
    def __init__(self):
        super().__init__('recorder_manager')

        # Souscription au topic des boutons Foxglove
        self.sub = self.create_subscription(String, '/ihm/recorder_cmd', self.cmd_callback, 10)

        # Variables pour stocker les processus en cours
        self.data_process = None
        self.macro_process = None
        self.replay_process = None
        
        # Mémorise la dernière macro enregistrée pour le bouton "REPLAY"
        self.last_macro_name = ""

        # Dossier de sauvegarde (dans le dossier Home de l'utilisateur)
        self.save_dir = os.path.expanduser('~/rosbags_srs')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info('📼 Recorder Manager prêt (Écoute sur /ihm/recorder_cmd)')

    def cmd_callback(self, msg):
        command = msg.data.strip().split(" ")
        action = command[0]
        
        # --- Récupération d'un nom optionnel, sinon horodatage ---
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        custom_name = command[1] if len(command) > 1 else ""

        # ==========================================
        # 1. GESTION DES MACROS (Mouvements)
        # ==========================================
        if action == "REC_MACRO":
            if self.macro_process:
                self.get_logger().warning("Une macro est DÉJÀ en cours d'enregistrement !")
                return
                
            name = custom_name if custom_name else f"macro_{timestamp}"
            self.last_macro_name = os.path.join(self.save_dir, name)
            
            self.macro_process = subprocess.Popen([
                'ros2', 'bag', 'record', '-o', self.last_macro_name, 
                '/mega/command', '/uno/cmd'
            ])
            self.get_logger().info(f"👻 Enregistrement Macro démarré : {name}")

        elif action == "STOP_MACRO":
            if self.macro_process:
                self.macro_process.terminate()
                self.macro_process = None
                self.get_logger().info(f"⬛ Enregistrement Macro terminé. (Prêt pour le REPLAY)")

        elif action == "PLAY_MACRO":
            # On tue le replay précédent s'il y en avait un en cours
            if self.replay_process:
                self.replay_process.terminate()
                
            if not self.last_macro_name or not os.path.exists(self.last_macro_name):
                self.get_logger().error("Aucune macro récente à rejouer !")
                return
                
            self.replay_process = subprocess.Popen(['ros2', 'bag', 'play', self.last_macro_name])
            self.get_logger().info(f"▶️ Lecture de la Macro {self.last_macro_name} en cours...")

        # ==========================================
        # 2. GESTION DE LA BOÎTE NOIRE (Analyse Vidéo/Courant)
        # ==========================================
        elif action == "REC_DATA":
            if self.data_process:
                self.get_logger().warning("L'enregistrement DATA est DÉJÀ en cours !")
                return
                
            name = custom_name if custom_name else f"data_{timestamp}"
            filepath = os.path.join(self.save_dir, name)
            
            # On enregistre tout SAUF les commandes !
            self.data_process = subprocess.Popen([
                'ros2', 'bag', 'record', '-o', filepath, 
                '/mega/power', '/mega/positions', '/mega/limits', '/mega/alarms',
                '/video_cam/compressed', '/video_cam_2/compressed', '/vision/detections'
            ])
            self.get_logger().info(f"📼 Enregistrement Boîte Noire démarré : {name}")

        elif action == "STOP_DATA":
            if self.data_process:
                self.data_process.terminate()
                self.data_process = None
                self.get_logger().info("⬛ Enregistrement Boîte Noire terminé.")

def main(args=None):
    rclpy.init(args=args)
    node = RecorderManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Nettoyage à l'arrêt du nœud
        if node.data_process: node.data_process.terminate()
        if node.macro_process: node.macro_process.terminate()
        if node.replay_process: node.replay_process.terminate()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()