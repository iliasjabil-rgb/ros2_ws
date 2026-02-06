#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, Bool, String, ColorRGBA
from sensor_msgs.msg import Range, Imu, MagneticField


class UnoDriver(Node):
    def __init__(self):
        super().__init__('uno_driver')

        # --- Publishers capteurs ---
        self.temp_pub = self.create_publisher(Float32, 'uno/temperature', 10)
        self.dist_pub = self.create_publisher(Range,   'uno/distance',    10)
        self.imu_pub  = self.create_publisher(Imu,     'uno/imu',         10)
        self.mag_pub  = self.create_publisher(MagneticField, 'uno/mag',   10)

        # --- Publisher JSON -> UNO ---
        self.cmd_pub  = self.create_publisher(String, 'uno/cmd', 10)

        # --- NOUVEAU : STATUS WATCHDOG (Voyant Foxglove) ---
        self.status_pub = self.create_publisher(Bool, 'status/uno', 10)
        self.last_msg_time = self.get_clock().now()
        self.create_timer(1.0, self.check_connection_callback)

        # --- Subscribers télémétrie & commandes ---
        self.raw_sub = self.create_subscription(String, 'uno/raw', self.raw_cb, 10)

        # 4 servos
        self.create_subscription(Float32, 'uno/servo1/cmd', self.servo1_cmd_cb, 10)
        self.create_subscription(Float32, 'uno/servo2/cmd', self.servo2_cmd_cb, 10)
        self.create_subscription(Float32, 'uno/servo3/cmd', self.servo3_cmd_cb, 10)
        self.create_subscription(Float32, 'uno/servo4/cmd', self.servo4_cmd_cb, 10)

        # 6 relais
        self.create_subscription(Bool, 'uno/relay_led_white/cmd',  self.relay1_cmd_cb, 10)
        self.create_subscription(Bool, 'uno/relay_led_rgb/cmd',    self.relay2_cmd_cb, 10)
        self.create_subscription(Bool, 'uno/relay_aspirateur/cmd', self.relay3_cmd_cb, 10)
        self.create_subscription(Bool, 'uno/relay_ev1/cmd',        self.relay4_cmd_cb, 10)
        self.create_subscription(Bool, 'uno/relay_ev2/cmd',        self.relay5_cmd_cb, 10)
        self.create_subscription(Bool, 'uno/relay_ev3/cmd',        self.relay6_cmd_cb, 10)

        # LED RGB (via Topic standard)
        self.create_subscription(ColorRGBA, 'led/cmd', self.led_cmd_cb, 10)

        # Effet scintillant SRS
        self.create_subscription(Bool, 'led_effect/cmd', self.led_effect_cmd_cb, 10)

        # --- NOUVEAU : Paramètres ROS 2 pour Foxglove ---
        # Cela crée les champs dans le panneau "ROS 2 Parameters"
        self.declare_parameter('led_red', 0)
        self.declare_parameter('led_green', 0)
        self.declare_parameter('led_blue', 0)
        self.declare_parameter('led_intensity', 0)
        
        # Active la mise à jour automatique quand on touche Foxglove
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('UNO driver démarré (LEDs + Watchdog Actifs).')

        # --- WATCHDOG ---
    def check_connection_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        is_connected = elapsed < 3.0 
        self.status_pub.publish(Bool(data=is_connected))

    # ------------------------------------------------------------------
    # GESTION DES PARAMÈTRES FOXGLOVE
    # ------------------------------------------------------------------
    def parameters_callback(self, params):
        # Récupère les anciennes valeurs
        r = self.get_parameter('led_red').value
        g = self.get_parameter('led_green').value
        b = self.get_parameter('led_blue').value
        a = self.get_parameter('led_intensity').value

        # Met à jour avec les nouvelles valeurs reçues
        for param in params:
            if param.name == 'led_red': r = param.value
            if param.name == 'led_green': g = param.value
            if param.name == 'led_blue': b = param.value
            if param.name == 'led_intensity': a = param.value

        self.get_logger().info(f"PARAM CHANGE: R={r} G={g} B={b} A={a}")
        self.send_led_manual(r, g, b, a)
        return SetParametersResult(successful=True)

    def send_led_manual(self, r, g, b, a):
        # Sécurité : Borner entre 0 et 255
        r = max(0, min(255, int(r)))
        g = max(0, min(255, int(g)))
        b = max(0, min(255, int(b)))
        a = max(0, min(255, int(a)))

        cmd = {
            "cmd": "led",
            "r": r, "g": g, "b": b, "a": a
        }
        self.publish_cmd(cmd)

    # ------------------------------------------------------------------
    # ROS commandes -> JSON /uno/cmd
    # ------------------------------------------------------------------
    def publish_cmd(self, obj: dict):
        line = json.dumps(obj, separators=(',', ':'))
        msg = String()
        msg.data = line
        self.cmd_pub.publish(msg)

    # ---------- servos ----------
    def _send_servo(self, servo_id: int, msg: Float32):
        angle = float(msg.data)
        cmd = {
            "cmd": "servo",
            "id": servo_id,
            "angle": int(angle),
        }
        self.publish_cmd(cmd)

    def servo1_cmd_cb(self, msg: Float32): self._send_servo(1, msg)
    def servo2_cmd_cb(self, msg: Float32): self._send_servo(2, msg)
    def servo3_cmd_cb(self, msg: Float32): self._send_servo(3, msg)
    def servo4_cmd_cb(self, msg: Float32): self._send_servo(4, msg)

    # ---------- relais ----------
    def _send_relay(self, relay_id: int, msg: Bool):
        cmd = {
            "cmd": "relay",
            "id": relay_id,
            "state": 1 if msg.data else 0,
        }
        self.publish_cmd(cmd)

    def relay1_cmd_cb(self, msg: Bool): self._send_relay(1, msg)  # LED blanche
    def relay2_cmd_cb(self, msg: Bool): self._send_relay(2, msg)  # LED RGB
    def relay3_cmd_cb(self, msg: Bool): self._send_relay(3, msg)  # Aspirateur
    def relay4_cmd_cb(self, msg: Bool): self._send_relay(4, msg)  # EV1
    def relay5_cmd_cb(self, msg: Bool): self._send_relay(5, msg)  # EV2
    def relay6_cmd_cb(self, msg: Bool): self._send_relay(6, msg)  # EV3

    # ---------- LED & effet (Via Topic) ----------
    def led_cmd_cb(self, msg: ColorRGBA):
        r = int(max(0.0, min(1.0, msg.r)) * 255.0)
        g = int(max(0.0, min(1.0, msg.g)) * 255.0)
        b = int(max(0.0, min(1.0, msg.b)) * 255.0)
        a = int(max(0.0, min(1.0, msg.a)) * 255.0)

        cmd = {
            "cmd": "led",
            "r": r,
            "g": g,
            "b": b,
            "a": a,
        }
        self.publish_cmd(cmd)

    def led_effect_cmd_cb(self, msg: Bool):
        cmd = {
            "cmd": "led_effect",
            "state": 1 if msg.data else 0,
        }
        self.publish_cmd(cmd)

    # ------------------------------------------------------------------
    # /uno/raw -> parsing JSON -> topics typés
    # ------------------------------------------------------------------
    # --- RECEPTION RAW ---
    def raw_cb(self, msg: String):
        self.last_msg_time = self.get_clock().now() # Reset Watchdog
        line = msg.data.strip()
        if not line: return
        try:
            obj = json.loads(line)
        except json.JSONDecodeError: return
        if obj.get("src", "") != "uno": return
        self.handle_telemetry(obj)
        
    def handle_telemetry(self, obj: dict):
        # Events simples
        if "event" in obj and "temp_c" not in obj and "dist_mm" not in obj:
            self.get_logger().info(f"UNO event: {obj['event']}")
            return

        # Température
        if "temp_c" in obj:
            try:
                t = float(obj["temp_c"])
                m = Float32()
                m.data = t
                self.temp_pub.publish(m)
            except (TypeError, ValueError):
                pass

        # Distance
        if "dist_mm" in obj:
            try:
                d_mm = float(obj["dist_mm"])
                d_m = d_mm / 1000.0
                m = Range()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'uno_distance'
                m.radiation_type = Range.INFRARED
                m.min_range = 0.0
                m.max_range = 5.0
                m.field_of_view = 0.5
                m.range = d_m
                self.dist_pub.publish(m)
            except (TypeError, ValueError):
                pass

        # IMU
        try:
            imu_ok = int(obj.get("imu_ok", 0))
        except (TypeError, ValueError):
            imu_ok = 0

        if imu_ok:
            def safe_float(key, default=0.0):
                try:
                    return float(obj.get(key, default))
                except (TypeError, ValueError):
                    return default

            ax = safe_float("ax")
            ay = safe_float("ay")
            az = safe_float("az")
            gx = safe_float("gx")
            gy = safe_float("gy")
            gz = safe_float("gz")

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'uno_imu'

            imu_msg.orientation_covariance[0] = -1.0

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            self.imu_pub.publish(imu_msg)

            mx = safe_float("mx")
            my = safe_float("my")
            mz = safe_float("mz")

            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            mag_msg.magnetic_field.x = mx * 1e-6
            mag_msg.magnetic_field.y = my * 1e-6
            mag_msg.magnetic_field.z = mz * 1e-6
            self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()