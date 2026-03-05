#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu  # <--- Ajout pour lire l'orientation 3D
import csv
import os
from datetime import datetime
import json
import math # <--- Ajout pour la trigonométrie (Quaternions -> Degrés)

class PrelevementRecorder(Node):
    def __init__(self):
        super().__init__('csv_recorder')
        
        # --- CONFIGURATION DU FICHIER ---
        self.save_dir = os.path.expanduser('~/srs_csv_logs')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # On crée un fichier unique par jour
        timestamp = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
        self.filepath = os.path.join(self.save_dir, f"prelevements_{timestamp}.csv")
        
        # Initialisation du fichier avec les NOUVELLES en-têtes (Ajout de Roulis, Tangage, Lacet)
        if not os.path.exists(self.filepath):
            with open(self.filepath, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Date_Heure', 'Type_Prelevement', 'Numero', 'Pos_X', 'Pos_Y', 'Pos_Z', 'Roll_deg', 'Pitch_deg', 'Yaw_deg'])
        
        # --- VARIABLES INTERNES ---
        self.compteurs = {
            "Solide": 0,
            "Liquide": 0,
            "Poussiere": 0,
            "Frottis": 0  # <--- NOUVEAU TYPE AJOUTÉ ICI
        }
        
        # Stockage des dernières données reçues
        self.last_positions = [0.0, 0.0, 0.0]
        self.last_orientation_deg = [0.0, 0.0, 0.0] # Roll (Roulis), Pitch (Tangage), Yaw (Lacet)

        # --- SUBSCRIBERS ---
        # 1. Écoute de la MEGA (J'ai mis /mega/raw car c'est le topic standard de votre pont)
        self.mega_sub = self.create_subscription(String, '/mega/raw', self.telemetry_cb, 10)
        
        # 2. Écoute du filtre Madgwick pour l'orientation 3D
        self.imu_sub = self.create_subscription(Imu, '/uno/imu_filtered', self.imu_cb, 10)
        
        # 3. Écoute des boutons de l'IHM Foxglove
        self.ihm_sub = self.create_subscription(String, '/ihm/prelevement', self.trigger_cb, 10)

        self.get_logger().info(f"📼 Nœud CSV prêt (INCLUANT FROTTIS ET 3D). Fichier actif : {self.filepath}")

    def imu_cb(self, msg: Imu):
        """ Reçoit les Quaternions du filtre et les convertit en angles (degrés) lisibles par un humain """
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Conversion mathématique (Quaternion vers Angles d'Euler)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.degrees(math.atan2(t0, t1))
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.degrees(math.asin(t2))
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.degrees(math.atan2(t3, t4))

        # Arrondi à 2 décimales pour la propreté du CSV
        self.last_orientation_deg = [round(roll_x, 2), round(pitch_y, 2), round(yaw_z, 2)]

    def telemetry_cb(self, msg: String):
        """ Met à jour les coordonnées X,Y,Z en mémoire à chaque message de la Mega """
        try:
            data = json.loads(msg.data)
            if "pos" in data and len(data["pos"]) >= 4:
                z = data["pos"][0]
                y = data["pos"][2]
                x = data["pos"][3]
                self.last_positions = [x, y, z]
        except Exception:
            pass 

    def trigger_cb(self, msg: String):
        """ Déclenché uniquement quand on clique sur un bouton dans Foxglove """
        # On nettoie et on met la première lettre en majuscule pour éviter les erreurs de casse (ex: "frottis" -> "Frottis")
        type_prelev = msg.data.strip().capitalize()
        
        if type_prelev not in self.compteurs:
            self.get_logger().warn(f"⚠️ Type de prélèvement inconnu ignoré : {type_prelev}")
            return
            
        # Incrémentation du compteur
        self.compteurs[type_prelev] += 1
        num = self.compteurs[type_prelev]
        
        date_str = datetime.now().strftime("%H:%M:%S")
        pos_x, pos_y, pos_z = self.last_positions
        roll, pitch, yaw = self.last_orientation_deg

        # Écriture immédiate dans le CSV
        with open(self.filepath, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([date_str, type_prelev, num, pos_x, pos_y, pos_z, roll, pitch, yaw])
            
        self.get_logger().info(f"✅ {type_prelev} n°{num} enregistré ! Pos:({pos_x}, {pos_y}, {pos_z}) | Inclinaison:({roll}°, {pitch}°, {yaw}°)")

def main(args=None):
    rclpy.init(args=args)
    node = PrelevementRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()