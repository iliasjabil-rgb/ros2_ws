#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float32, Bool, String, ColorRGBA
from sensor_msgs.msg import Range, Imu, MagneticField


class UnoDriver(Node):
    """
    Nœud ROS 2 gérant la logique de l'Arduino Uno.
    Fait l'interface entre les topics ROS (Foxglove) et les messages JSON bruts.
    """
    def __init__(self):
        super().__init__('uno_driver')

        # --- 1. PARAMÈTRES FOXGLOVE (Contrôle LED Manuel) ---
        self.declare_parameter('led_red', 0)
        self.declare_parameter('led_green', 0)
        self.declare_parameter('led_blue', 0)
        self.declare_parameter('led_intensity', 0)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # --- 2. PUBLISHERS (Télémétrie vers ROS 2) ---
        self.temp_pub   = self.create_publisher(Float32, 'uno/temperature', 10)
        self.dist_pub   = self.create_publisher(Range,   'uno/distance',    10)
        self.imu_pub    = self.create_publisher(Imu,     'uno/imu',         10)
        self.mag_pub    = self.create_publisher(MagneticField, 'uno/mag',   10)
        
        # Publisher du statut (Watchdog) pour le tableau de bord
        self.status_pub = self.create_publisher(Bool, 'status/uno', 10)

        # Publisher pour envoyer le JSON formaté vers le bridge série
        self.cmd_pub    = self.create_publisher(String, 'uno/cmd', 10)

        # --- 3. SUBSCRIBERS (Commandes depuis ROS 2) ---
        # Télémétrie brute venant du bridge série
        self.raw_sub = self.create_subscription(String, 'uno/raw', self.raw_cb, 10)

        # Commandes Servomoteurs
        self.create_subscription(Float32, 'uno/servo1/cmd', lambda msg: self._send_servo(1, msg), 10)
        self.create_subscription(Float32, 'uno/servo2/cmd', lambda msg: self._send_servo(2, msg), 10)
        self.create_subscription(Float32, 'uno/servo3/cmd', lambda msg: self._send_servo(3, msg), 10)
        self.create_subscription(Float32, 'uno/servo4/cmd', lambda msg: self._send_servo(4, msg), 10)

        # Commandes Relais
        self.create_subscription(Bool, 'uno/relay_led_white/cmd',  lambda msg: self._send_relay(1, msg), 10)
        self.create_subscription(Bool, 'uno/relay_led_rgb/cmd',    lambda msg: self._send_relay(2, msg), 10)
        self.create_subscription(Bool, 'uno/relay_aspirateur/cmd', lambda msg: self._send_relay(3, msg), 10)
        self.create_subscription(Bool, 'uno/relay_ev1/cmd',        lambda msg: self._send_relay(4, msg), 10)
        self.create_subscription(Bool, 'uno/relay_ev2/cmd',        lambda msg: self._send_relay(5, msg), 10)
        self.create_subscription(Bool, 'uno/relay_ev3/cmd',        lambda msg: self._send_relay(6, msg), 10)

        # Commandes LEDs via Topic Standard
        self.create_subscription(ColorRGBA, 'led/cmd', self.led_cmd_cb, 10)
        self.create_subscription(Bool, 'led_effect/cmd', self.led_effect_cmd_cb, 10)

        # --- 4. TIMERS (Watchdog) ---
        self.last_msg_time = self.get_clock().now()
        self.create_timer(1.0, self.check_connection_callback)

        self.get_logger().info('✅ UNO Driver démarré (Télémétrie, LEDs & Watchdog Actifs).')

    # ==================================================================
    # GESTION DU WATCHDOG
    # ==================================================================
    def check_connection_callback(self):
        """Vérifie si on a reçu des données récemment et met à jour le statut."""
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        is_connected = elapsed < 3.0 
        self.status_pub.publish(Bool(data=is_connected))

    # ==================================================================
    # GESTION DES PARAMÈTRES (Panneau Foxglove)
    # ==================================================================
    def parameters_callback(self, params):
        """Callback déclenché quand on modifie un curseur dans Foxglove."""
        # Valeurs actuelles
        r = self.get_parameter('led_red').value
        g = self.get_parameter('led_green').value
        b = self.get_parameter('led_blue').value
        a = self.get_parameter('led_intensity').value

        # Application des modifications
        for param in params:
            if param.name == 'led_red': r = param.value
            elif param.name == 'led_green': g = param.value
            elif param.name == 'led_blue': b = param.value
            elif param.name == 'led_intensity': a = param.value

        self.get_logger().info(f"🎨 Paramètres LED modifiés : R={r} G={g} B={b} A={a}")
        self.send_led_manual(r, g, b, a)
        return SetParametersResult(successful=True)

    def send_led_manual(self, r, g, b, a):
        """Formate et envoie la commande LED."""
        cmd = {
            "cmd": "led",
            "r": max(0, min(255, int(r))),
            "g": max(0, min(255, int(g))),
            "b": max(0, min(255, int(b))),
            "a": max(0, min(255, int(a)))
        }
        self.publish_cmd(cmd)

    # ==================================================================
    # ENVOI DES COMMANDES VERS L'UNO
    # ==================================================================
    def publish_cmd(self, obj: dict):
        """Convertit le dictionnaire en JSON et le publie."""
        msg = String()
        msg.data = json.dumps(obj, separators=(',', ':'))
        self.cmd_pub.publish(msg)

    def _send_servo(self, servo_id: int, msg: Float32):
        self.publish_cmd({
            "cmd": "servo",
            "id": servo_id,
            "angle": int(msg.data)
        })

    def _send_relay(self, relay_id: int, msg: Bool):
        self.publish_cmd({
            "cmd": "relay",
            "id": relay_id,
            "state": 1 if msg.data else 0
        })

    def led_cmd_cb(self, msg: ColorRGBA):
        """Traduit un message ColorRGBA standard en JSON pour l'Uno."""
        self.publish_cmd({
            "cmd": "led",
            "r": int(max(0.0, min(1.0, msg.r)) * 255.0),
            "g": int(max(0.0, min(1.0, msg.g)) * 255.0),
            "b": int(max(0.0, min(1.0, msg.b)) * 255.0),
            "a": int(max(0.0, min(1.0, msg.a)) * 255.0)
        })

    def led_effect_cmd_cb(self, msg: Bool):
        self.publish_cmd({
            "cmd": "led_effect",
            "state": 1 if msg.data else 0
        })

    # ==================================================================
    # RÉCEPTION ET PARSING DE LA TÉLÉMÉTRIE
    # ==================================================================
    def raw_cb(self, msg: String):
        """Reçoit le JSON brut, reset le watchdog et parse les données."""
        self.last_msg_time = self.get_clock().now()
        line = msg.data.strip()
        
        if not line: return
        
        try:
            obj = json.loads(line)
        except json.JSONDecodeError: 
            return
            
        if obj.get("src") != "uno": 
            return
            
        self.handle_telemetry(obj)
        
    def _safe_float(self, data: dict, key: str, default: float = 0.0) -> float:
        """Helper pour extraire proprement un float depuis le dict JSON."""
        try:
            return float(data.get(key, default))
        except (TypeError, ValueError):
            return default

    def handle_telemetry(self, obj: dict):
        """Dispatche les données JSON vers les bons topics ROS 2."""
        now = self.get_clock().now().to_msg()

        # 1. Événements simples (Boot, init failed, etc.)
        if "event" in obj and "temp_c" not in obj and "dist_mm" not in obj:
            self.get_logger().info(f"🔔 UNO Event: {obj['event']}")
            return

        # 2. Température
        if "temp_c" in obj:
            msg = Float32()
            msg.data = self._safe_float(obj, "temp_c")
            self.temp_pub.publish(msg)

        # 3. Distance (ToF Laser)
        if "dist_mm" in obj:
            dist_m = self._safe_float(obj, "dist_mm") / 1000.0
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = 'uno_distance'
            msg.radiation_type = Range.INFRARED
            msg.min_range = 0.0
            msg.max_range = 5.0
            msg.field_of_view = 0.5
            msg.range = dist_m
            self.dist_pub.publish(msg)

        # 4. IMU & Magnétomètre
        if int(obj.get("imu_ok", 0)) == 1:
            # Création du message IMU
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'uno_imu'
            imu_msg.orientation_covariance[0] = -1.0 # Orientation non calculée par l'Arduino

            imu_msg.angular_velocity.x = self._safe_float(obj, "gx")
            imu_msg.angular_velocity.y = self._safe_float(obj, "gy")
            imu_msg.angular_velocity.z = self._safe_float(obj, "gz")

            imu_msg.linear_acceleration.x = self._safe_float(obj, "ax")
            imu_msg.linear_acceleration.y = self._safe_float(obj, "ay")
            imu_msg.linear_acceleration.z = self._safe_float(obj, "az")

            self.imu_pub.publish(imu_msg)

            # Création du message MagneticField (Conversion en Tesla)
            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            mag_msg.magnetic_field.x = self._safe_float(obj, "mx") * 1e-6
            mag_msg.magnetic_field.y = self._safe_float(obj, "my") * 1e-6
            mag_msg.magnetic_field.z = self._safe_float(obj, "mz") * 1e-6
            
            self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud UnoDriver.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()