#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from rclpy.time import Time

class ManipulatorTeleop(Node):
    def __init__(self):
        super().__init__('manipulator_teleop')

        # Configuration des joints (Noms tirés de votre URDF)
        self.declare_parameter('joints', ['gliss_poteaux','gliss_bras', 'verrin', 'rot_pince', 'gliss_boites'])
        
        # Mapping T16000M : Axe 1 (Y), Axe 0 (X), Axe 5 (Joystick gachette), Axe 2 (Twist)
        self.declare_parameter('axis_for_joint', [1, 0, 5, 2, 4])
        
        # Vitesse max (m/s ou rad/s)
        # Note: 'gliss_poteaux' est inversé (-0.1) pour que "pousser" = monter
        self.declare_parameter('scale_for_joint', [-0.1, 0.1, -0.1, 1.0, 0.1])

        # Limites logicielles (puisque l'URDF est à 0)
        self.declare_parameter('joint_limits_min', [0.0, 0.0, -0.22, -3.14, -0.2])
        self.declare_parameter('joint_limits_max', [0.2, 0.3, 0.0,  3.14, 0.0])

        # Sécurité (Gâchette bouton 0)
        self.declare_parameter('enable_button', 0)
        self.declare_parameter('require_enable_button', True)

        # Chargement des params
        self.joints = self.get_parameter('joints').value
        self.axis_ids = list(self.get_parameter('axis_for_joint').value)
        self.scales = list(self.get_parameter('scale_for_joint').value)
        self.limits_min = list(self.get_parameter('joint_limits_min').value)
        self.limits_max = list(self.get_parameter('joint_limits_max').value)
        self.enable_btn = self.get_parameter('enable_button').value
        self.use_safety = self.get_parameter('require_enable_button').value

        # Variables d'état
        self.positions = [0.0] * len(self.joints)
        self.velocities = [0.0] * len(self.joints)
        self.last_axes = []
        self.last_buttons = []
        self.last_time = self.get_clock().now()

        # ROS
        self.sub_joy = self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.update_loop) # 50Hz

        self.get_logger().info("Sysmap Teleop prêt. Maintenez la gâchette pour bouger.")

    def cb_joy(self, msg: Joy):
        self.last_axes = msg.axes
        self.last_buttons = msg.buttons

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.last_axes:
            return

        # Sécurité Homme-mort
        if self.use_safety:
            if len(self.last_buttons) <= self.enable_btn or self.last_buttons[self.enable_btn] == 0:
                # Si bouton relâché, vitesse 0, on ne met pas à jour la position
                self.velocities = [0.0] * len(self.joints)
                self.publish_state(now)
                return

        # Calcul mouvement
        for i in range(len(self.joints)):
            ax_idx = self.axis_ids[i]
            if ax_idx < len(self.last_axes):
                # Vitesse = input joystick * scale
                cmd_vel = self.last_axes[ax_idx] * self.scales[i]
                self.velocities[i] = cmd_vel
                
                # Nouvelle position
                new_pos = self.positions[i] + (cmd_vel * dt)
                # Clamp limites
                new_pos = max(self.limits_min[i], min(new_pos, self.limits_max[i]))
                self.positions[i] = new_pos

        self.publish_state(now)

    def publish_state(self, now_time):
        msg = JointState()
        msg.header.stamp = now_time.to_msg()
        msg.name = self.joints
        msg.position = self.positions
        msg.velocity = self.velocities
        self.pub_joints.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()