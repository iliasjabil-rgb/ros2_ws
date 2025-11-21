import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from .serial_utils import SerialReader
from .json_utils import parse_json_line

class MegaBridge(Node):
    def __init__(self):
        super().__init__('mega_bridge')
        self.declare_parameter('port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_7513030393535130D120-if00')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.pub_pos    = self.create_publisher(Int32MultiArray, 'mega/pos',    10)
        self.pub_limits = self.create_publisher(Int32MultiArray, 'mega/limits', 10)
        self.pub_events = self.create_publisher(String,           'mega/events', 10)

        self.sub_cmd = self.create_subscription(String, 'mega/cmd', self.on_cmd, 10)

        self.get_logger().info(f'Opening MEGA serial {port} @ {baud}')
        try:
            self.ser = SerialReader(port, baud, self.on_line, self.on_serial_error)
            self.ser.start()
        except Exception as e:
            self.get_logger().error(f'Cannot open serial: {e}')
            raise

    def on_cmd(self, msg: String):
        # forward tel quel (doit être une ligne JSON)
        self.ser.write_line(msg.data)

    def on_line(self, line: str):
        data = parse_json_line(line)
        if data is None:
            return
        # Positions : {"src":"mega","pos":[...]}
        if isinstance(data, dict) and 'pos' in data and isinstance(data['pos'], list):
            out = Int32MultiArray(data=data['pos'])
            self.pub_pos.publish(out)
        # Fins de course : {"src":"mega","mr":[...]}
        elif isinstance(data, dict) and 'mr' in data and isinstance(data['mr'], list):
            out = Int32MultiArray(data=data['mr'])
            self.pub_limits.publish(out)
        # Tout le reste -> events (JSON brut ou _raw)
        else:
            txt = line.strip()
            self.pub_events.publish(String(data=txt))

    def on_serial_error(self, e: Exception):
        self.get_logger().error(f'Serial error: {e}')

def main():
    rclpy.init()
    node = MegaBridge()
    try:
        rclpy.spin(node)
    finally:
        try: node.ser.stop()
        except: pass
        node.destroy_node()
        rclpy.shutdown()
