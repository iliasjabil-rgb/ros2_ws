#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String

class MegaBridge(Node):
    def __init__(self):
        super().__init__('mega_bridge')
        
        # --- PARAMÈTRES MATÉRIELS ---
        self.declare_parameter('port', '/dev/ttyACM0') 
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"🔌 Bridge MEGA connecté sur {port} à {baudrate} bauds.")
        except Exception as e:
            self.get_logger().error(f"❌ Impossible d'ouvrir le port série MEGA: {e}")
            raise SystemExit

        # --- TOPICS DE TRANSPORT BRUT ---
        self.raw_pub = self.create_publisher(String, 'mega/raw', 10)
        self.cmd_sub = self.create_subscription(String, 'mega/cmd_raw', self.cmd_callback, 10)

        # Thread de lecture
        self.is_running = True
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

    def cmd_callback(self, msg: String):
        """Prend le texte de ROS 2 et l'envoie sur l'USB"""
        if self.ser.is_open:
            cmd_str = msg.data.strip() + '\n'
            self.ser.write(cmd_str.encode('utf-8'))

    def read_serial(self):
        """Prend le texte de l'USB et l'envoie dans ROS 2"""
        while rclpy.ok() and self.ser.is_open and self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        msg = String()
                        msg.data = line
                        self.raw_pub.publish(msg)
            except Exception:
                pass

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MegaBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()