import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
from .serial_utils import SerialReader
from .json_utils import parse_json_line

class UnoBridge(Node):
    def __init__(self):
        super().__init__('uno_bridge')
        self.declare_parameter('port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_4423831383835151A2D1-if00')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.pub_analog = self.create_publisher(Int32MultiArray, 'uno/analog', 10)
        self.pub_events = self.create_publisher(String,           'uno/events', 10)
        self.sub_cmd    = self.create_subscription(String,        'uno/cmd', self.on_cmd, 10)

        self.get_logger().info(f'Opening UNO serial {port} @ {baud}')
        try:
            self.ser = SerialReader(port, baud, self.on_line, self.on_serial_error)
            self.ser.start()
        except Exception as e:
            self.get_logger().error(f'Cannot open serial: {e}')
            raise

    def on_cmd(self, msg: String):
        self.ser.write_line(msg.data)

    def on_line(self, line: str):
        data = parse_json_line(line)
        if data is None:
            return
        # Télémétrie analogique : {"src":"uno","a0":..,"a1":..,"a2":..,"dist":..,"temp":..}
        if isinstance(data, dict) and ('a0' in data or 'dist' in data or 'temp' in data):
            arr = [
                int(data.get('a0', 0)),
                int(data.get('a1', 0)),
                int(data.get('a2', 0)),
                int(data.get('dist', 0)),
                int(data.get('temp', 0)),
            ]
            self.pub_analog.publish(Int32MultiArray(data=arr))
        else:
            self.pub_events.publish(String(data=line.strip()))

    def on_serial_error(self, e: Exception):
        self.get_logger().error(f'Serial error: {e}')

def main():
    rclpy.init()
    node = UnoBridge()
    try:
        rclpy.spin(node)
    finally:
        try: node.ser.stop()
        except: pass
        node.destroy_node()
        rclpy.shutdown()
