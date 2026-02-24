#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Importation de la gestion des broches de la Raspberry Pi
try:
    from gpiozero import OutputDevice
except ImportError:
    print("Erreur : la bibliothèque gpiozero n'est pas installée.")

class RpiRelayNode(Node):
    def __init__(self):
        super().__init__('rpi_relay_node')

        # --- PARAMÈTRES ---
        # Remplacez 17 par le numéro BCM de la broche GPIO que vous avez choisie
        self.declare_parameter('relay_pin', 26)
        self.pin_number = self.get_parameter('relay_pin').value

        # Initialisation de la broche (active_high=True est standard pour les relais)
        self.relay = OutputDevice(self.pin_number, active_high=True, initial_value=False)

        # --- SUBSCRIBER ---
        # Topic pour contrôler le relais (ex: depuis Foxglove ou la manette)
        self.sub = self.create_subscription(Bool, '/rpi/relay_24v/cmd', self.relay_cb, 10)

        # Publisher pour le retour d'état (utile pour afficher un voyant dans Foxglove)
        self.state_pub = self.create_publisher(Bool, '/rpi/relay_24v/state', 10)

        self.get_logger().info(f"⚡ Nœud Relais RPi démarré sur la broche GPIO {self.pin_number}")

    def relay_cb(self, msg: Bool):
        if msg.data:
            self.relay.on()
            self.get_logger().info("Relais 24V : ACTIVÉ 🔴")
        else:
            self.relay.off()
            self.get_logger().info("Relais 24V : DÉSACTIVÉ ⬛")
            
        # On publie l'état réel pour Foxglove
        state_msg = Bool()
        state_msg.data = self.relay.is_active
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RpiRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Sécurité : on éteint le relais si on coupe le nœud
        node.relay.off()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
