#!/usr/bin/env python3
import socket, threading, time, math
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32, Bool, String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros

RICA_IP_DEFAULT = "192.168.0.2"
RICA_PORT_DEFAULT = 2009

def clamp(x, a, b): return max(a, min(b, x))

class RICAClient:
    def __init__(self, ip, port, logger):
        self.ip, self.port = ip, port
        self.sock: Optional[socket.socket] = None
        self.lock = threading.Lock()
        self.log = logger

    def connect(self):
        if self.sock:
            return
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3.0)
        s.connect((self.ip, self.port))
        s.settimeout(2.0)
        self.sock = s
        self.log.info(f"[RICA] Connecté à {self.ip}:{self.port}")

    def close(self):
        with self.lock:
            try:
                if self.sock:
                    self.sock.close()
            finally:
                self.sock = None

    def send_cmd(self, line: str) -> Optional[str]:
        """Envoie une ligne ASCII, lit une ligne (ou None si timeout)."""
        with self.lock:
            if not self.sock:
                self.connect()
            if not line.endswith("\n"):
                line += "\n"
            try:
                self.sock.sendall(line.encode("utf-8"))
                # lire une ligne
                data = bytearray()
                while True:
                    ch = self.sock.recv(1)
                    if not ch:
                        break
                    data += ch
                    if ch == b"\n":
                        break
                return data.decode("utf-8", errors="replace").strip() if data else None
            except (socket.timeout, OSError) as e:
                self.log.warn(f"[RICA] Erreur socket: {e}; reconnexion…")
                self.close()
                return None

    # --- Helpers protocole RICA ---
    def power(self, on: bool) -> bool:
        rep = self.send_cmd(f"POWER {1 if on else 0}")
        return bool(rep) and "TRAME_OK" in rep

    def set_wheels(self, left_pct: int, right_pct: int) -> bool:
        left_pct = int(clamp(left_pct, -100, 100))
        right_pct = int(clamp(right_pct, -100, 100))
        if left_pct >= 0:
            repg = self.send_cmd(f"AVANCER_ROUES_G {left_pct}")
        else:
            repg = self.send_cmd(f"RECULER_ROUES_G {abs(left_pct)}")
        if right_pct >= 0:
            repd = self.send_cmd(f"AVANCER_ROUES_D {right_pct}")
        else:
            repd = self.send_cmd(f"RECULER_ROUES_D {abs(right_pct)}")
        return bool(repg and "TRAME_OK" in repg and repd and "TRAME_OK" in repd)

    def read_cod_roues(self) -> Optional[Tuple[int, int]]:
        rep = self.send_cmd("COD_ROUES")
        if not rep or (len(rep) % 3 != 0):
            return None
        b = [int(rep[i:i+3]) for i in range(0, len(rep), 3)]
        if len(b) != 8:
            return None
        def u32_be(o): return (b[o]<<24) | (b[o+1]<<16) | (b[o+2]<<8) | b[o+3]
        def i32_be(o):
            v = u32_be(o)
            return v - (1<<32) if v & 0x80000000 else v
        return i32_be(0), i32_be(4)

    def read_fins_course(self) -> Optional[Tuple[int, int]]:
        rep = self.send_cmd("FINS_COURSE")
        if not rep or len(rep) != 6:
            return None
        return int(rep[0:3]), int(rep[3:6])


class RICABridge(Node):
    def __init__(self):
        super().__init__("rica_bridge")
        # --- Params
        self.declare_parameter("ip", RICA_IP_DEFAULT)
        self.declare_parameter("port", RICA_PORT_DEFAULT)
        self.declare_parameter("wheel_separation", 0.30)  # m
        self.declare_parameter("wheel_radius", 0.05)       # m
        self.declare_parameter("ticks_per_rev", 2048)      # codeur / tour
        self.declare_parameter("max_speed_mps", 0.6)       # m/s -> 100%

        self.client = RICAClient(
            self.get_parameter("ip").value,
            int(self.get_parameter("port").value),
            self.get_logger()
        )

        # --- Pub/Sub/Services
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.pub_codeur_g = self.create_publisher(Int32, "rica/encoder_left", 10)
        self.pub_codeur_d = self.create_publisher(Int32, "rica/encoder_right", 10)
        self.pub_fin_av   = self.create_publisher(Bool,  "rica/limit_front", 10)
        self.pub_fin_ar   = self.create_publisher(Bool,  "rica/limit_rear", 10)
        self.pub_status   = self.create_publisher(String,"rica/status", 10)
        self.pub_odom     = self.create_publisher(Odometry, "odom", 10)
        self.pub_js       = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br        = tf2_ros.TransformBroadcaster(self)

        # --- État odométrique
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.prev_enc: Optional[Tuple[int,int]] = None
        self.wheel_pos: List[float] = [0.0, 0.0]  # rad
        self.last_odo_time = self.get_clock().now()

        # --- Timers
        self.poll_timer = self.create_timer(0.1, self.poll_feedback)  # 10 Hz
        self.stop_failsafe_timer = self.create_timer(0.5, self.stop_if_idle)

        self.last_cmd_time = self.get_clock().now()
        self.powered = False

        # POWER ON optionnel
        try:
            if self.client.power(True):
                self.powered = True
                self.pub_status.publish(String(data="POWER ON"))
        except Exception as e:
            self.get_logger().warn(f"[RICA] POWER ON échoué: {e}")

    # ----------------- ODOM -----------------
    def odom_from_encoders(self, enc_g: int, enc_d: int):
        now = self.get_clock().now()
        dt = (now - self.last_odo_time).nanoseconds * 1e-9
        self.last_odo_time = now
        if dt <= 0.0:
            dt = 1e-3

        R = float(self.get_parameter("wheel_radius").value)
        L = float(self.get_parameter("wheel_separation").value)
        ticks = float(self.get_parameter("ticks_per_rev").value)

        if self.prev_enc is None:
            self.prev_enc = (enc_g, enc_d)
            return

        dg_ticks = enc_g - self.prev_enc[0]
        dd_ticks = enc_d - self.prev_enc[1]
        self.prev_enc = (enc_g, enc_d)

        # distances par roue
        s_g = (2.0*math.pi*R) * (dg_ticks / ticks)
        s_d = (2.0*math.pi*R) * (dd_ticks / ticks)

        # positions angulaires des “roues”
        self.wheel_pos[0] += (2.0*math.pi) * (dg_ticks / ticks)
        self.wheel_pos[1] += (2.0*math.pi) * (dd_ticks / ticks)

        s = 0.5*(s_g + s_d)
        dth = (s_d - s_g) / L

        if abs(dth) < 1e-9:
            dx = s * math.cos(self.th)
            dy = s * math.sin(self.th)
        else:
            R_icc = s / dth
            dx = R_icc*(math.sin(self.th + dth) - math.sin(self.th))
            dy = -R_icc*(math.cos(self.th + dth) - math.cos(self.th))

        self.x += dx
        self.y += dy
        self.th = (self.th + dth + math.pi) % (2*math.pi) - math.pi

        # TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.th/2.0)
        t.transform.rotation.w = math.cos(self.th/2.0)
        self.tf_br.sendTransform(t)

        # Odometry
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = t.transform.rotation.z
        od.pose.pose.orientation.w = t.transform.rotation.w
        self.pub_odom.publish(od)

        # JointState (si tu veux animer les “pattes”, change les noms)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["base_to_front_left_leg", "base_to_front_right_leg",
                   "base_to_back_left_leg",  "base_to_back_right_leg"]
        js.position = [self.wheel_pos[0], self.wheel_pos[1],
                       self.wheel_pos[0], self.wheel_pos[1]]
        self.pub_js.publish(js)

    # -------------- Callbacks / Timers --------------
    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx   = msg.linear.x
        wz   = msg.angular.z
        L    = float(self.get_parameter("wheel_separation").value)
        vmax = float(self.get_parameter("max_speed_mps").value)

        v_l = vx - 0.5 * L * wz
        v_r = vx + 0.5 * L * wz

        pct_l = int(clamp((v_l / vmax) * 100.0, -100, 100))
        pct_r = int(clamp((v_r / vmax) * 100.0, -100, 100))

        ok = self.client.set_wheels(pct_l, pct_r)
        self.pub_status.publish(String(data=f"CMD {pct_l},{pct_r}" if ok else "CMD NOK"))

    def poll_feedback(self):
        try:
            cod = self.client.read_cod_roues()
            if cod:
                g, d = cod
                self.pub_codeur_g.publish(Int32(data=g))
                self.pub_codeur_d.publish(Int32(data=d))
                self.odom_from_encoders(g, d)  # <-- corrige l’erreur
        except Exception as e:
            self.get_logger().warn(f"[RICA] COD_ROUES err: {e}")

        try:
            fc = self.client.read_fins_course()
            if fc:
                f_av, f_ar = fc
                self.pub_fin_av.publish(Bool(data=bool(f_av)))
                self.pub_fin_ar.publish(Bool(data=bool(f_ar)))
        except Exception as e:
            self.get_logger().warn(f"[RICA] FINS_COURSE err: {e}")

    def stop_if_idle(self):
        # stop de sécurité si plus de /cmd_vel depuis 0.7s
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 7e8:
            self.client.set_wheels(0, 0)

def main():
    rclpy.init()
    node = RICABridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.client.set_wheels(0,0)
            node.client.power(False)
        except Exception:
            pass
        node.destroy_node()
        # évite l’exception "rcl_shutdown déjà appelé"
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
 