#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from smbus2 import SMBus

MCP9808_DEFAULT_ADDR = 0x18
MCP9808_REG_TEMP = 0x05


class Mcp9808Node(Node):
    def __init__(self):
        super().__init__('temperature_sensor_test_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', MCP9808_DEFAULT_ADDR)
        self.declare_parameter('publish_rate_hz', 2.0)

        bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        addr = self.get_parameter('address').get_parameter_value().integer_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.bus = SMBus(bus_num)
        self.address = addr

        self.pub = self.create_publisher(Temperature, 'temperature', 10)
        period = 1.0 / rate if rate > 0.0 else 0.5
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'MCP9808 node started on I2C bus {bus_num}, addr 0x{addr:02X}'
        )

    def read_temperature_c(self) -> float:
        raw = self.bus.read_word_data(self.address, MCP9808_REG_TEMP)
        # swap des octets
        raw = ((raw & 0xFF) << 8) | (raw >> 8)
        temp = raw & 0x0FFF
        if raw & 0x1000:  # bit de signe
            temp -= 1 << 13
        temp_c = temp * 0.0625
        return temp_c

    def timer_callback(self):
        try:
            temp_c = self.read_temperature_c()
        except OSError as e:
            self.get_logger().warning(f'I2C error while reading MCP9808: {e}')
            return

        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mcp9808_link'
        msg.temperature = float(temp_c)
        msg.variance = 0.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mcp9808Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
