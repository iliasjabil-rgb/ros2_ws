#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class StepperTest(Node):
    """
    Node de test pour un moteur pas-à-pas de la MEGA.
    - Envoie des JSON sur le topic 'mega/cmd' que mega_bridge renvoie vers l'Arduino.
    - Mode JOG CONTINU : tant que le joystick est incliné, on envoie des petits moves.
    """

    def __init__(self):
        super().__init__('stepper_test')

        # --- Paramètres ---
        # Axe MEGA à piloter : 1..4 (cf MEGA_main.ino)
        self.declare_parameter('axis', 1)

        # Index de l'axe joystick (T16000M : 1 = Y, 0 = X, etc.)
        self.declare_parameter('joy_axis', 1)

        # Inversion de l'axe joystick (si haut/bas sont inversés)
        self.declare_parameter('invert_axis', False)

        # "Pas" ajouté à CHAQUE callback Joy à fond de course
        # (ex: 100, 200, 500... à régler selon la fluidité désirée)
        self.declare_parameter('steps_per_cycle', 200)

        # Vitesse de move (steps/s) côté AccelStepper
        self.declare_parameter('speed', 800)

        # Zone morte joystick
        self.declare_parameter('deadband', 0.2)

        # Sécurité type "homme mort" (gâchette)
        self.declare_parameter('enable_button', 0)
        self.declare_parameter('require_enable_button', True)

        # Boutons pour home/stop (indices dans Joy.buttons)
        self.declare_parameter('home_button', 1)
        self.declare_parameter('stop_button', 2)

        # --- Récup paramètres ---
        self.axis_id = int(self.get_parameter('axis').value)
        self.joy_axis = int(self.get_parameter('joy_axis').value)
        self.invert_axis = bool(self.get_parameter('invert_axis').value)
        self.steps_per_cycle = int(self.get_parameter('steps_per_cycle').value)
        self.speed = int(self.get_parameter('speed').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.enable_btn = int(self.get_parameter('enable_button').value)
        self.require_enable = bool(self.get_parameter('require_enable_button').value)
        self.home_btn = int(self.get_parameter('home_button').value)
        self.stop_btn = int(self.get_parameter('stop_button').value)

        # --- ROS I/O ---
        self.pub_cmd = self.create_publisher(String, 'mega/cmd', 10)
        self.sub_joy = self.create_subscription(Joy, 'joy', self.cb_joy, 10)

        # État interne
        self.prev_buttons = []
        self.last_cmd = None  # pour limiter les logs

        self.get_logger().info(
            f"MegaStepperTest JOG prêt pour l'axe {self.axis_id} "
            f"(joy_axis={self.joy_axis}, steps_per_cycle={self.steps_per_cycle}, "
            f"invert_axis={self.invert_axis})"
        )

    def send_cmd(self, json_str: str):
        msg = String()
        msg.data = json_str
        self.pub_cmd.publish(msg)

        # Option : éviter de spammer les logs si commande identique
        if json_str != self.last_cmd:
            self.get_logger().debug(f"TX mega/cmd: {json_str}")
        self.last_cmd = json_str

    def cb_joy(self, msg: Joy):
        axes = msg.axes
        buttons = msg.buttons

        # --- Sécurité homme-mort ---
        enabled = True
        if self.require_enable:
            if self.enable_btn >= len(buttons) or buttons[self.enable_btn] == 0:
                enabled = False

        # --- Gestion boutons home/stop (front montant) ---
        prev = self.prev_buttons
        self.prev_buttons = buttons

        def is_pressed(idx, arr):
            return idx < len(arr) and arr[idx] == 1

        # HOME
        if is_pressed(self.home_btn, buttons) and not is_pressed(self.home_btn, prev):
            self.send_cmd('{"cmd":"home"}')

        # STOP
        if is_pressed(self.stop_btn, buttons) and not is_pressed(self.stop_btn, prev):
            self.send_cmd('{"cmd":"stop"}')

        # Si pas de sécurité active -> pas de jog
        if not enabled:
            return

        # --- Lecture de l'axe joystick choisi ---
        if self.joy_axis >= len(axes):
            return
        val = axes[self.joy_axis]

        # Inversion éventuelle de l'axe
        if self.invert_axis:
            val = -val

        # Log de debug pour voir les valeurs réelles
        self.get_logger().debug(
            f"Joy axis {self.joy_axis} value = {val:.3f}"
        )

        # Zone morte
        if abs(val) < self.deadband:
            return  # joystick au neutre -> pas de nouveau move

        # --- JOG CONTINU ---
        # On convertit la valeur [-1,1] en "quantité de steps à ajouter"
        # par callback Joy. À fond de course, on ajoute +-steps_per_cycle.
        steps = int(val * self.steps_per_cycle)

        if steps == 0:
            return

        cmd = (
            f'{{"cmd":"move","axis":{self.axis_id},'
            f'"steps":{steps},"speed":{self.speed}}}'
        )
        self.send_cmd(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StepperTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
