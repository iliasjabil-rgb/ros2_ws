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

        # Paramètres (ajuste si besoin)
        self.declare_parameter("wheel_separation", 0.30)  # m: entraxe roues
        self.declare_parameter("wheel_radius", 0.05)      # m: rayon roue
        self.declare_parameter("rate_hz", 50.0)           # fréquence intégration
        self.declare_parameter("frame_odom", "odom")
        self.declare_parameter("frame_base", "base_link")

        self.L = float(self.get_parameter("wheel_separation").value)
        self.R = float(self.get_parameter("wheel_radius").value)
        self.frame_odom = self.get_parameter("frame_odom").value
        self.frame_base = self.get_parameter("frame_base").value

        # État
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.cmd = Twist()  # dernière commande

        # Simu des “roues” pour animer tes quatre joints URDF
        self.pos_left = 0.0   # rad
        self.pos_right = 0.0  # rad

        # ROS IO
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_js   = self.create_publisher(JointState, "joint_states", 10)
        self.tf_br    = tf2_ros.TransformBroadcaster(self)

        # Timer d’intégration
        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info("[SIM] DiffDriveSim prêt (manette -> /cmd_vel -> odom/TF/joint_states)")

    def on_cmd_vel(self, msg: Twist):
        self.cmd = msg

    def step(self):
        # Intégration pose base_link dans odom
        dt = self.timer.timer_period_ns * 1e-9
        vx = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)

        # Intégration SE(2)
        if abs(wz) < 1e-9:
            self.x += vx * math.cos(self.th) * dt
            self.y += vx * math.sin(self.th) * dt
        else:
            self.x += (vx / wz) * ( math.sin(self.th + wz*dt) - math.sin(self.th) )
            self.y += (vx / wz) * ( -math.cos(self.th + wz*dt) + math.cos(self.th) )
        self.th = (self.th + wz*dt + math.pi) % (2*math.pi) - math.pi

        # TF odom -> base_link
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.th/2.0)
        t.transform.rotation.w = math.cos(self.th/2.0)
        self.tf_br.sendTransform(t)

        # Odom
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.frame_odom
        od.child_frame_id = self.frame_base
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = t.transform.rotation.z
        od.pose.pose.orientation.w = t.transform.rotation.w
        od.twist.twist.linear.x = vx
        od.twist.twist.angular.z = wz
        self.pub_odom.publish(od)

        # Animation “roues” → tes 4 articulations URDF (pattes)
        # v_l, v_r (m/s) puis ω = v/R
        v_l = vx - 0.5 * self.L * wz
        v_r = vx + 0.5 * self.L * wz
        if self.R > 1e-9:
            self.pos_left  += (v_l / self.R) * dt
            self.pos_right += (v_r / self.R) * dt

        js = JointState()
        js.header.stamp = now
        # NOMS = ceux de ton URDF
        js.name = [
            "base_to_front_left_leg",
            "base_to_front_right_leg",
            "base_to_back_left_leg",
            "base_to_back_right_leg"
        ]
        # On duplique gauche/droite pour l’avant/arrière
        js.position = [self.pos_left, self.pos_right, self.pos_left, self.pos_right]
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
