#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from smbus2 import SMBus

INA260_DEFAULT_ADDR = 0x40
INA260_REG_CURRENT = 0x01
INA260_REG_BUS_VOLTAGE = 0x02
INA260_REG_POWER = 0x03


class Ina260Node(Node):
    def __init__(self):
        super().__init__('current_sensor_test_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', INA260_DEFAULT_ADDR)
        self.declare_parameter('publish_rate_hz', 10.0)

        bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        addr = self.get_parameter('address').get_parameter_value().integer_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.bus = SMBus(bus_num)
        self.address = addr

        self.current_pub = self.create_publisher(Float32, 'ina260/current', 10)
        self.voltage_pub = self.create_publisher(Float32, 'ina260/voltage', 10)
        self.power_pub = self.create_publisher(Float32, 'ina260/power', 10)

        period = 1.0 / rate if rate > 0.0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'INA260 node started on I2C bus {bus_num}, address 0x{addr:02X}'
        )

    def _read_register(self, reg: int) -> int:
        raw = self.bus.read_word_data(self.address, reg)
        # SMBus renvoie LSB|MSB, le capteur est MSB|LSB → on swap
        value = ((raw & 0xFF) << 8) | (raw >> 8)
        return value

    def read_measurements(self):
        # Courant (signed 16 bits, 1.25 mA / LSB)
        raw_current = self._read_register(INA260_REG_CURRENT)
        if raw_current & 0x8000:
            raw_current -= 1 << 16
        current_a = raw_current * 1.25e-3  # A

        # Tension bus (unsigned, 1.25 mV / LSB)
        raw_voltage = self._read_register(INA260_REG_BUS_VOLTAGE)
        voltage_v = raw_voltage * 1.25e-3  # V

        # Puissance (unsigned, 10 mW / LSB)
        raw_power = self._read_register(INA260_REG_POWER)
        power_w = raw_power * 10e-3  # W

        return current_a, voltage_v, power_w

    def timer_callback(self):
        try:
            current_a, voltage_v, power_w = self.read_measurements()
        except OSError as e:
            self.get_logger().warning(f'I2C error while reading INA260: {e}')
            return

        msg_i = Float32()
        msg_i.data = float(current_a)
        self.current_pub.publish(msg_i)

        msg_v = Float32()
        msg_v.data = float(voltage_v)
        self.voltage_pub.publish(msg_v)

        msg_p = Float32()
        msg_p.data = float(power_w)
        self.power_pub.publish(msg_p)


def main(args=None):
    rclpy.init(args=args)
    node = Ina260Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
