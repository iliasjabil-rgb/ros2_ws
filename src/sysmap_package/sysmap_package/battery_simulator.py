#!/usr/bin/env python3
"""
============================================================================
FICHIER : battery_simulator.py
ROLE    : Nœud ROS 2 simulant la décharge d'une batterie de 100% à 0% en 45 min.
============================================================================
"""
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
 
class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        # Topic de publication : /battery
        self.publisher_ = self.create_publisher(Float32, 'battery', 10)
        # Paramètres de simulation
        self.update_period = 1.0  # Publication à 1 Hz
        self.timer = self.create_timer(self.update_period, self.timer_callback)
        self.battery_level = 100.0
        # Calcul du pas de décharge : 100% / (45 minutes * 60 secondes)
        self.discharge_step = 100.0 / (20.0 * 60.0)
        self.get_logger().info('Simulateur de batterie démarré (Décharge : 45 min).')
 
    def timer_callback(self):
        # Création et publication du message
        msg = Float32()
        msg.data = self.battery_level
        self.publisher_.publish(msg)
        # Affichage dans le terminal toutes les 60 secondes (pour le monitoring)
        if int(self.battery_level % 1.0) == 0 and self.battery_level > 0:
            self.get_logger().info(f'Niveau de batterie : {self.battery_level:.1f}%')
        # Mise à jour du niveau (décrémentation)
        self.battery_level -= self.discharge_step
        # Saturation à 0% pour éviter les valeurs négatives
        if self.battery_level <= 0.0:
            self.battery_level = 0.0
            self.get_logger().info('Batterie vide !', once=True)
 
def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()