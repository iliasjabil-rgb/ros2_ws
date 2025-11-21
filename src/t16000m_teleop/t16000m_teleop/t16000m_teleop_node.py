#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class T16000MTeleop(Node):
    def __init__(self):
        super().__init__('t16000m_teleop')

        # Paramètres configurables
        # (valeurs par défaut raisonnables pour un joystick type T.16000M)
        self.declare_parameter('axis_linear', 1)          # Axe Y
        self.declare_parameter('axis_angular', 0)         # Axe X
        self.declare_parameter('scale_linear', 0.8)
        self.declare_parameter('scale_angular', 1.5)
        self.declare_parameter('enable_button', 0)        # Gâchette
        self.declare_parameter('require_enable_button', True)
        self.declare_parameter('invert_linear', -1.0)     # Y souvent inversé
        self.declare_parameter('invert_angular', 1.0)

        # Lecture des paramètres
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)
        self.enable_button = self.get_parameter('enable_button').value
        self.require_enable_button = bool(
            self.get_parameter('require_enable_button').value
        )
        self.invert_linear = float(self.get_parameter('invert_linear').value)
        self.invert_angular = float(self.get_parameter('invert_angular').value)

        # Publisher /cmd_vel et subscriber /joy
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10
        )

        self.get_logger().info('T16000M teleop node started')

    def joy_callback(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        # Sécurité : bouton "deadman"
        if self.require_enable_button:
            if self.enable_button >= len(buttons):
                self.get_logger().warn_once(
                    f'Bouton enable_button={self.enable_button} inexistant '
                    f'(len(buttons)={len(buttons)}).'
                )
                return

            if buttons[self.enable_button] == 0:
                # Bouton non pressé → envoie un /cmd_vel nul
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                return

        # Vérif indices axes
        if self.axis_linear >= len(axes) or self.axis_angular >= len(axes):
            self.get_logger().warn_once(
                f'Axes joystick insuffisants. axis_linear={self.axis_linear}, '
                f'axis_angular={self.axis_angular}, len(axes)={len(axes)}'
            )
            return

        # Conversion joystick → cmd_vel
        twist = Twist()
        twist.linear.x = (
            self.scale_linear * self.invert_linear * axes[self.axis_linear]
        )
        twist.angular.z = (
            self.scale_angular * self.invert_angular * axes[self.axis_angular]
        )

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = T16000MTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
