#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from rclpy.time import Time


class CartesianTeleop(Node):
    def __init__(self):
        super().__init__('cartesian_teleop')

        # === Paramètres ===
        # NOMS DES JOINTS = ton URDF
        self.declare_parameter(
            'joints',
            ['gliss_poteaux', 'verrin', 'rot_pince']
        )

        # Indice d’axe joystick pour chaque joint (tableau axes[] de /joy)
        # Proposé :
        #   gliss_poteaux -> axe 1 (avant / arrière)
        #   verrin       -> axe 0 (gauche / droite)
        #   rot_pince    -> axe 2 (twist du manche)
        self.declare_parameter('axis_for_joint', [1, 0, 2])

        # Vitesse max par joint
        #   prismatic : ~5 cm/s
        #   revolute  : ~0.5 rad/s
        self.declare_parameter('scale_for_joint', [0.05, 0.05, 0.5])

        # Bouton "deadman" (gâchette)
        self.declare_parameter('enable_button', 0)
        self.declare_parameter('require_enable_button', True)

        # Fréquence d’update
        self.declare_parameter('rate', 50.0)

        # Lecture des paramètres (en listes Python)
        self.joints = self.get_parameter('joints').value
        self.axis_for_joint = list(self.get_parameter('axis_for_joint').value)
        self.scale_for_joint = list(self.get_parameter('scale_for_joint').value)
        self.enable_button = int(self.get_parameter('enable_button').value)
        self.require_enable_button = bool(
            self.get_parameter('require_enable_button').value
        )
        self.rate = float(self.get_parameter('rate').value)

        n = len(self.joints)
        if len(self.axis_for_joint) != n or len(self.scale_for_joint) != n:
            self.get_logger().error(
                'joints, axis_for_joint et scale_for_joint doivent '
                'avoir la même longueur.'
            )
            raise RuntimeError('Config paramètres invalide')

        # État interne des joints
        self.positions = [0.0] * n
        self.velocities = [0.0] * n

        # Dernier /joy
        self.last_axes = []
        self.last_buttons = []
        self.last_time = self.get_clock().now()

        # ROS I/O
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        dt = 1.0 / self.rate
        self.timer = self.create_timer(dt, self.update)

        self.get_logger().info(
            f'cartesian_teleop démarré pour joints: {self.joints}'
        )

    def joy_callback(self, msg: Joy):
        self.last_axes = list(msg.axes)
        self.last_buttons = list(msg.buttons)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        if not self.last_axes:
            # aucun /joy reçu pour l'instant
            return

        # Deadman
        if self.require_enable_button:
            if self.enable_button >= len(self.last_buttons):
                self.get_logger().warn_once(
                    f'Bouton enable_button={self.enable_button} inexistant '
                    f'(len(buttons)={len(self.last_buttons)})'
                )
                return
            if self.last_buttons[self.enable_button] == 0:
                # Pas appuyé -> on republie seulement la position actuelle
                self.publish_joint_states(now)
                return

        # Intégration des positions à partir des vitesses
        for i, joint in enumerate(self.joints):
            axis_idx = self.axis_for_joint[i]
            scale = self.scale_for_joint[i]

            if axis_idx < 0 or axis_idx >= len(self.last_axes):
                self.velocities[i] = 0.0
                continue

            axis_val = self.last_axes[axis_idx]   # -1..1
            self.velocities[i] = axis_val * scale
            self.positions[i] += self.velocities[i] * dt

            # Ici tu pourrais ajouter des limites si tu connais la course réelle

        self.publish_joint_states(now)

    def publish_joint_states(self, now: Time):
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(self.joints)
        js.position = list(self.positions)
        js.velocity = list(self.velocities)
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
