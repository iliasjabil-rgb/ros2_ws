#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
<<<<<<< HEAD
# IMPORT CORRECT : On a besoin de ces types
=======
>>>>>>> 9a6e3af0 (MAJ camera et driver mega pour cap courant)
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray, Bool

class MegaDriver(Node):
    def __init__(self):
        super().__init__('mega_driver')

        # --- Publishers Télémétrie ---
        self.pub_pos    = self.create_publisher(Int32MultiArray,   'mega/pos',    10)
        self.pub_limits = self.create_publisher(Int32MultiArray,   'mega/limits', 10)
        self.pub_events = self.create_publisher(String,            'mega/events', 10)
<<<<<<< HEAD
        # Publisher pour le voyant Foxglove
        self.pub_status = self.create_publisher(Bool,              'status/mega', 10)
        
        # Gestion du temps pour le Watchdog
        self.last_msg_time = self.get_clock().now()
        self.create_timer(1.0, self.check_connection)
=======
        self.pub_status = self.create_publisher(Bool,              'status/mega', 10)
>>>>>>> 9a6e3af0 (MAJ camera et driver mega pour cap courant)

        # --- NOUVEAU : Publishers Puissance Séparés ---
        # On garde 'mega/pwr' global pour le debug, et on ajoute les détaillés
        self.pub_pwr_global = self.create_publisher(Float32MultiArray, 'mega/pwr', 10)
        
        self.pub_pwr_m1 = self.create_publisher(Float32MultiArray, 'mega/pwr/m1', 10)
        self.pub_pwr_m2 = self.create_publisher(Float32MultiArray, 'mega/pwr/m2', 10)
        self.pub_pwr_m3 = self.create_publisher(Float32MultiArray, 'mega/pwr/m3', 10)
        self.pub_pwr_m4 = self.create_publisher(Float32MultiArray, 'mega/pwr/m4', 10)
        
        # Gestion du Watchdog
        self.last_msg_time = self.get_clock().now()
        self.create_timer(1.0, self.check_connection)

        # Subscriber (Entrée depuis le Bridge)
        self.sub_raw = self.create_subscription(String, 'mega/raw', self.raw_cb, 10)

<<<<<<< HEAD
        self.get_logger().info('MEGA Driver démarré (JSON Parser).')
    # Fonction qui vérifie la connexion chaque seconde
    def check_connection(self):
        # Calcul du temps écoulé depuis le dernier message reçu
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        # Si moins de 3 secondes de silence -> Connecté (Vrai), sinon Déconnecté (Faux)
=======
        self.get_logger().info('MEGA Driver (4-Axes Power Edition) démarré.')

    def check_connection(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
>>>>>>> 9a6e3af0 (MAJ camera et driver mega pour cap courant)
        is_connected = elapsed < 3.0
        self.pub_status.publish(Bool(data=is_connected))

    def raw_cb(self, msg: String):
<<<<<<< HEAD
        self.last_msg_time = self.get_clock().now()
=======
        # 1. Reset Watchdog
        self.last_msg_time = self.get_clock().now()

        # 2. Parsing JSON
>>>>>>> 9a6e3af0 (MAJ camera et driver mega pour cap courant)
        line = msg.data
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return 

        if data.get('src') != 'mega': return

        # --- TRAITEMENT DES DONNÉES ---

        # 1. Positions
        if 'pos' in data and isinstance(data['pos'], list):
            self.pub_pos.publish(Int32MultiArray(data=data['pos']))

        # 2. Fins de course
        elif 'mr' in data and isinstance(data['mr'], list):
            self.pub_limits.publish(Int32MultiArray(data=data['mr']))

        # 3. PUISSANCE (C'est ici que ça change !)
        elif 'pwr' in data and isinstance(data['pwr'], list):
            raw_list = data['pwr']
            count = len(raw_list)
            
            # Publie toujours la liste complète sur le topic global
            self.pub_pwr_global.publish(Float32MultiArray(data=raw_list))

            # Si on a bien 12 valeurs (4 capteurs x 3 valeurs)
            if count >= 12:
                # Moteur 1 (Index 0, 1, 2)
                self.pub_pwr_m1.publish(Float32MultiArray(data=raw_list[0:3]))
                # Moteur 2 (Index 3, 4, 5)
                self.pub_pwr_m2.publish(Float32MultiArray(data=raw_list[3:6]))
                # Moteur 3 (Index 6, 7, 8)
                self.pub_pwr_m3.publish(Float32MultiArray(data=raw_list[6:9]))
                # Moteur 4 (Index 9, 10, 11)
                self.pub_pwr_m4.publish(Float32MultiArray(data=raw_list[9:12]))

        # 4. Événements
        elif 'event' in data:
            self.pub_events.publish(String(data=line))

def main(args=None):
    rclpy.init(args=args)
    node = MegaDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()