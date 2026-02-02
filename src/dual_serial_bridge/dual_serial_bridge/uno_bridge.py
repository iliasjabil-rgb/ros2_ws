#!/usr/bin/env python3
"""
uno_bridge.py
Petit pont série <-> ROS2 pour la carte Arduino UNO.

Rôle :
  - ouvrir le port série
  - publier chaque ligne reçue sur /uno/raw (std_msgs/String)
  - écouter /uno/cmd (std_msgs/String) et envoyer la ligne sur le port série

Toute l'intelligence protocolaire (JSON -> topics typés, et inversement)
est déléguée à uno_driver.py.
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class UnoBridge(Node):
    def __init__(self):
        super().__init__('uno_bridge')

        # --- Paramètres ROS ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- Série ---
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Impossible d'ouvrir le port série {port}: {e}")
            raise

        self.ser_lock = threading.Lock()

        # --- ROS I/O ---
        self.raw_pub = self.create_publisher(String, 'uno/raw', 10)
        self.cmd_sub = self.create_subscription(String, 'uno/cmd', self.cmd_cb, 10)

        # --- Thread de lecture série ---
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(
            f'UNO serial bridge démarré sur {port} @ {baudrate} bauds'
        )

    # ------------------------------------------------------------------
    # Callbacks ROS -> Série
    # ------------------------------------------------------------------
    def cmd_cb(self, msg: String):
        """
        /uno/cmd : contient une ligne (JSON ou autre) à envoyer sur le port série.
        On s'assure qu'elle se termine par '\n'.
        """
        line = msg.data
        if not line.endswith('\n'):
            line += '\n'
        data = line.encode('utf-8', errors='ignore')

        with self.ser_lock:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                self.get_logger().error(f'Erreur écriture série UNO: {e}')

    # ------------------------------------------------------------------
    # Boucle Série -> ROS
    # ------------------------------------------------------------------
    def read_serial_loop(self):
        buffer = ""
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Erreur lecture série UNO: {e}')
                break

            if not chunk:
                continue

            try:
                text = chunk.decode('utf-8', errors='ignore')
            except UnicodeDecodeError:
                continue

            buffer += text
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.rstrip('\r\n')
                if not line:
                    continue

                msg = String()
                msg.data = line
                self.raw_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.stop_event.set()
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
