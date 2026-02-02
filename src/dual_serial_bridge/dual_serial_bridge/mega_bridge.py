#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MegaBridge(Node):
    def __init__(self):
        super().__init__('mega_bridge')

        # Paramètres
        self.declare_parameter('port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_7513030393535130D120-if00')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # Connexion Série
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'MEGA serial bridge démarré sur {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f"Erreur ouverture port MEGA: {e}")
            raise

        self.ser_lock = threading.Lock()

        # ROS I/O
        self.raw_pub = self.create_publisher(String, 'mega/raw', 10)
        self.cmd_sub = self.create_subscription(String, 'mega/cmd', self.cmd_cb, 10)

        # Thread de lecture
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.reader_thread.start()

    def cmd_cb(self, msg: String):
        """ Reçoit une commande JSON brute (depuis Joystick) et l'envoie à l'Arduino """
        line = msg.data
        if not line.endswith('\n'):
            line += '\n'
        data = line.encode('utf-8', errors='ignore')
        
        with self.ser_lock:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                self.get_logger().error(f'Erreur écriture MEGA: {e}')

    def read_loop(self):
        """ Lit le port série en boucle et publie sur mega/raw """
        buffer = ""
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception:
                break
            
            if not chunk: continue
            
            try:
                text = chunk.decode('utf-8', errors='ignore')
            except UnicodeDecodeError:
                continue

            buffer += text
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.raw_pub.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MegaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()