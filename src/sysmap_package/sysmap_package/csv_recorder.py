#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os
from datetime import datetime
import json

class PrelevementRecorder(Node):
    def __init__(self):
        super().__init__('csv_recorder')
        
        # --- CONFIGURATION DU FICHIER ---
        self.save_dir = os.path.expanduser('~/srs_csv_logs')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # On crée un fichier unique par jour pour ne pas tout mélanger
# Nouvelles lignes (avec l'heure exacte)
        timestamp = datetime.now().strftime("%Y-%m-%d_%Hh%Mm%Ss")
        self.filepath = os.path.join(self.save_dir, f"prelevements_{timestamp}.csv")
        
        # Initialisation du fichier avec les en-têtes (si c'est un nouveau fichier)
        if not os.path.exists(self.filepath):
            with open(self.filepath, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Date_Heure', 'Type_Prelevement', 'Numero', 'Pos_X', 'Pos_Y', 'Pos_Z'])
        
        # --- VARIABLES INTERNES ---
        self.compteurs = {
            "Solide": 0,
            "Liquide": 0,
            "Poussiere": 0
        }
        # Stockage de la dernière position reçue [X, Y, Z]
        self.last_positions = [0.0, 0.0, 0.0]

        # --- SUBSCRIBERS ---
        # 1. Écoute de la télémétrie de la MEGA pour mettre à jour les positions en temps réel
        self.mega_sub = self.create_subscription(String, '/mega/telemetry', self.telemetry_cb, 10)
        # (Si votre topic Mega est différent, ex: /mega/positions, ajustez-le ci-dessus)
        
        # 2. Écoute des boutons de l'IHM Foxglove
        self.ihm_sub = self.create_subscription(String, '/ihm/prelevement', self.trigger_cb, 10)

        self.get_logger().info(f"📼 Nœud CSV prêt. Fichier actif : {self.filepath}")

    def telemetry_cb(self, msg: String):
        """ Met à jour les coordonnées X,Y,Z en mémoire à chaque message de la Mega """
        try:
            data = json.loads(msg.data)
            # D'après votre dico d'API, pos = [Z, Magasin, Y, Verin]
            if "pos" in data and len(data["pos"]) >= 4:
                # On extrait les axes (à adapter selon votre vraie mécanique)
                z = data["pos"][0]
                y = data["pos"][2]
                x = data["pos"][3] # Supposons que le vérin/tige est l'axe X
                self.last_positions = [x, y, z]
        except Exception:
            pass # On ignore si le JSON est mal formé ou s'il s'agit d'un autre type de message

    def trigger_cb(self, msg: String):
        """ Déclenché uniquement quand on clique sur un bouton dans Foxglove """
        type_prelev = msg.data.strip()
        
        # Vérification de sécurité (au cas où on reçoit un message inconnu)
        if type_prelev not in self.compteurs:
            self.get_logger().warn(f"Type de prélèvement inconnu : {type_prelev}")
            return
            
        # Incrémentation du compteur
        self.compteurs[type_prelev] += 1
        num = self.compteurs[type_prelev]
        
        date_str = datetime.now().strftime("%H:%M:%S")
        pos_x, pos_y, pos_z = self.last_positions

        # Écriture immédiate dans le CSV (mode 'a' = append / ajouter à la fin)
        with open(self.filepath, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([date_str, type_prelev, num, pos_x, pos_y, pos_z])
            
        self.get_logger().info(f"✅ Prélèvement {type_prelev} n°{num} enregistré ! Coords: X={pos_x}, Y={pos_y}, Z={pos_z}")


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