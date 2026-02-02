#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

import board
import busio
import adafruit_vl53l0x


class Vl53l0xNode(Node):
    def __init__(self):
        super().__init__('distance_test_node')

        self.declare_parameter('publish_rate_hz', 10.0)

        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # I2C via Blinka
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_vl53l0x.VL53L0X(i2c)

        # Optionnel : temps de mesure (µs)
        # self.sensor.measurement_timing_budget = 200000  # 200 ms

        self.pub = self.create_publisher(Range, 'distance', 10)

        period = 1.0 / rate if rate > 0.0 else 0.1
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info('VL53L0X node started')

    def timer_callback(self):
        try:
            distance_mm = self.sensor.range  # propriété .range en mm
        except OSError as e:
            self.get_logger().warning(f'I2C error while reading VL53L0X: {e}')
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'vl53l0x_link'

        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 35.0 * 3.1415926535 / 180.0  # ~35°
        msg.min_range = 0.03  # 3 cm
        msg.max_range = 2.0   # 2 m (à adapter)
        msg.range = float(distance_mm) / 1000.0  # en mètres

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Vl53l0xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
