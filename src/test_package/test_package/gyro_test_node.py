#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from smbus2 import SMBus

LSM6DS33_ADDR_DEFAULT = 0x6B  # ou 0x6A suivant SA0

REG_WHO_AM_I = 0x0F
REG_CTRL1_XL = 0x10
REG_CTRL2_G = 0x11
REG_CTRL3_C = 0x12
REG_OUT_TEMP_L = 0x20  # puis H, Gx, Gy, Gz, Ax, Ay, Az


class Lsm6ds33Node(Node):
    def __init__(self):
        super().__init__('gyro_test_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', LSM6DS33_ADDR_DEFAULT)
        self.declare_parameter('publish_rate_hz', 50.0)

        bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        addr = self.get_parameter('address').get_parameter_value().integer_value
        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.bus = SMBus(bus_num)
        self.address = addr

        self._configure_sensor()

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)

        period = 1.0 / rate if rate > 0.0 else 0.02
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'LSM6DS33 node started on I2C bus {bus_num}, addr 0x{addr:02X}'
        )

    def _write_reg(self, reg: int, value: int):
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def _configure_sensor(self):
        who = self.bus.read_byte_data(self.address, REG_WHO_AM_I)
        if who != 0x69:
            self.get_logger().warn(
                f'LSM6DS33 WHO_AM_I=0x{who:02X} (attendu 0x69), vérifie l\'adresse I2C'
            )

        # CTRL3_C : BDU=1 (bit6), IF_INC=1 (bit2) → 0x44
        self._write_reg(REG_CTRL3_C, 0x44)

        # CTRL1_XL : Accéléro 104 Hz, ±2g → 0x40
        self._write_reg(REG_CTRL1_XL, 0x40)

        # CTRL2_G : Gyro 104 Hz, ±245 dps → 0x40
        self._write_reg(REG_CTRL2_G, 0x40)

        time.sleep(0.1)

    @staticmethod
    def _to_int16(lo: int, hi: int) -> int:
        value = (hi << 8) | lo
        if value & 0x8000:
            value -= 1 << 16
        return value

    def _read_all(self):
        # 14 octets : Temp_L, Temp_H, Gx_L, Gx_H, Gy_L, Gy_H, Gz_L, Gz_H,
        #             Ax_L, Ax_H, Ay_L, Ay_H, Az_L, Az_H
        data = self.bus.read_i2c_block_data(self.address, REG_OUT_TEMP_L, 14)

        temp_raw = self._to_int16(data[0], data[1])
        gx_raw = self._to_int16(data[2], data[3])
        gy_raw = self._to_int16(data[4], data[5])
        gz_raw = self._to_int16(data[6], data[7])
        ax_raw = self._to_int16(data[8], data[9])
        ay_raw = self._to_int16(data[10], data[11])
        az_raw = self._to_int16(data[12], data[13])

        # Température : 0 LSB à 25 °C, 16 LSB/°C
        temp_c = 25.0 + (temp_raw / 16.0)

        # Gyro : 8.75 mdps/LSB pour ±245 dps
        gyro_dps_x = gx_raw * 8.75e-3
        gyro_dps_y = gy_raw * 8.75e-3
        gyro_dps_z = gz_raw * 8.75e-3

        gyro_rad_x = math.radians(gyro_dps_x)
        gyro_rad_y = math.radians(gyro_dps_y)
        gyro_rad_z = math.radians(gyro_dps_z)

        # Accéléro : 0.061 mg/LSB pour ±2g
        acc_g_x = ax_raw * 0.061e-3
        acc_g_y = ay_raw * 0.061e-3
        acc_g_z = az_raw * 0.061e-3

        g = 9.80665
        acc_ms2_x = acc_g_x * g
        acc_ms2_y = acc_g_y * g
        acc_ms2_z = acc_g_z * g

        return (
            acc_ms2_x,
            acc_ms2_y,
            acc_ms2_z,
            gyro_rad_x,
            gyro_rad_y,
            gyro_rad_z,
            temp_c,
        )

    def timer_callback(self):
        try:
            (
                ax, ay, az,
                gx, gy, gz,
                temp_c,
            ) = self._read_all()
        except OSError as e:
            self.get_logger().warning(f'I2C error while reading LSM6DS33: {e}')
            return

        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.linear_acceleration_covariance[0] = -1.0  # inconnue

        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.angular_velocity_covariance[0] = -1.0

        # On ne fournit pas l'orientation (pas de fusion de capteurs ici)
        imu_msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        temp_msg = Temperature()
        temp_msg.header.stamp = now
        temp_msg.header.frame_id = 'imu_link'
        temp_msg.temperature = float(temp_c)
        temp_msg.variance = 0.0
        self.temp_pub.publish(temp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Lsm6ds33Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
