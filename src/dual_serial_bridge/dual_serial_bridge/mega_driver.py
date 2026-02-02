#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
# IMPORT CORRECT : On a besoin de ces types
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

class MegaDriver(Node):
    def __init__(self):
        super().__init__('mega_driver')

        # --- Publishers Télémétrie (Sorties vers ROS) ---
        # Position en steps [M1, M2, M3, M4]
        self.pub_pos    = self.create_publisher(Int32MultiArray,   'mega/pos',    10)
        # État des capteurs [Lim1, Lim2, Lim3, Lim4] (0 ou 1)
        self.pub_limits = self.create_publisher(Int32MultiArray,   'mega/limits', 10)
        # Puissance [Voltage(mV), Current(mA), Power(mW)]
        self.pub_power  = self.create_publisher(Float32MultiArray, 'mega/pwr',    10)
        # Messages systèmes (Boot, erreurs...)
        self.pub_events = self.create_publisher(String,            'mega/events', 10)

        # --- Subscriber (Entrée depuis le Bridge) ---
        self.sub_raw = self.create_subscription(String, 'mega/raw', self.raw_cb, 10)

        self.get_logger().info('MEGA Driver démarré (JSON Parser).')

    def raw_cb(self, msg: String):
        line = msg.data
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            return # Ignore les lignes corrompues

        # Vérification de la source
        if data.get('src') != 'mega':
            return

        # 1. Positions : {"pos":[...]}
        if 'pos' in data and isinstance(data['pos'], list):
            out = Int32MultiArray(data=data['pos'])
            self.pub_pos.publish(out)

        # 2. Fins de course : {"mr":[...]}
        elif 'mr' in data and isinstance(data['mr'], list):
            out = Int32MultiArray(data=data['mr'])
            self.pub_limits.publish(out)

        # 3. Puissance : {"pwr":[mV, mA, mW]}
        elif 'pwr' in data and isinstance(data['pwr'], list):
            out = Float32MultiArray(data=data['pwr'])
            self.pub_power.publish(out)

        # 4. Événements divers
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