#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros

class DiffDriveSim(Node):
    def __init__(self):
        super().__init__("diff_drive_sim")

        # --- paramètres ---
        self.declare_parameter("wheel_separation", 0.30)  # distance entre roues/chenilles (m)
        self.declare_parameter("wheel_radius",     0.05)  # rayon roue/chenille (m)
        self.declare_parameter("rate_hz",          50.0)
        self.declare_parameter("frame_odom",       "odom")
        self.declare_parameter("frame_base",       "base_link")

        self.L = float(self.get_parameter("wheel_separation").value)
        self.R = float(self.get_parameter("wheel_radius").value)
        self.frame_odom = self.get_parameter("frame_odom").value
        self.frame_base = self.get_parameter("frame_base").value

        # --- état ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.cmd = Twist()

        # animation "cosmétique" des chenilles
        self.pos_side_left = 0.0
        self.pos_side_right = 0.0

        # --- IO ---
        self.sub_cmd  = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_js   = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br    = tf2_ros.TransformBroadcaster(self)

        # --- timer (définir self.dt AVANT create_timer !) ---
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / max(1.0, rate_hz)
        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info("[SIM] DiffDriveSim prêt")

    def on_cmd_vel(self, msg: Twist):
        self.cmd = msg

    def step(self):
        # 1) lire la commande
        vx = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)
        # anti-bruit
        if abs(vx) < 0.05: vx = 0.0
        if abs(wz) < 0.05: wz = 0.0

        # 2) intégrer la pose
        dt = self.dt
        if abs(wz) < 1e-9:
            self.x += vx * math.cos(self.th) * dt
            self.y += vx * math.sin(self.th) * dt
        else:
            self.x += (vx / wz) * ( math.sin(self.th + wz*dt) - math.sin(self.th) )
            self.y += (vx / wz) * ( -math.cos(self.th + wz*dt) + math.cos(self.th) )
        self.th = (self.th + wz*dt + math.pi) % (2*math.pi) - math.pi

        # 3) TF + Odom
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_odom
        t.child_frame_id  = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.th/2.0)
        t.transform.rotation.w = math.cos(self.th/2.0)
        self.tf_br.sendTransform(t)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.frame_odom
        od.child_frame_id  = self.frame_base
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = t.transform.rotation.z
        od.pose.pose.orientation.w = t.transform.rotation.w
        od.twist.twist.linear.x = vx
        od.twist.twist.angular.z = wz
        self.pub_odom.publish(od)

        # 4) vitesses côté gauche/droit (m/s)
        v_l = vx - 0.5 * self.L * wz
        v_r = vx + 0.5 * self.L * wz
        if abs(v_l) < 1e-3: v_l = 0.0
        if abs(v_r) < 1e-3: v_r = 0.0

        # 5) animer les "chenilles" (rad)
        if self.R > 1e-9:
            self.pos_side_left  += (v_l / self.R) * dt
            self.pos_side_right += (v_r / self.R) * dt

        js = JointState()
        js.header.stamp = now
        js.name     = ["base_to_left_side", "base_to_right_side"]
        js.position = [ self.pos_side_left,   self.pos_side_right ]
        self.pub_js.publish(js)

def main():
    rclpy.init()
    node = DiffDriveSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
