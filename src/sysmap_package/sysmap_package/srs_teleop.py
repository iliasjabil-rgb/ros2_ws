#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, ColorRGBA, String


MODE_CAM     = 'CAMERA'
MODE_STEPPER = 'STEPPER'


class SRSTeleop(Node):
    def __init__(self):
        super().__init__('srs_teleop')

        # Paramètres des axes/boutons
        self.declare_parameter('axis_pan',  0)   # joystick horizontal
        self.declare_parameter('axis_tilt', 1)   # joystick vertical
        self.declare_parameter('axis_brightness', 5)  # ex: gâchette

        self.declare_parameter('button_mode', 2)   # change de mode
        self.declare_parameter('button_relay', 0)  # relais ON/OFF
        self.declare_parameter('button_led_on', 1) # toggle LED

        self.axis_pan        = self.get_parameter('axis_pan').value
        self.axis_tilt       = self.get_parameter('axis_tilt').value
        self.axis_brightness = self.get_parameter('axis_brightness').value
        self.button_mode     = self.get_parameter('button_mode').value
        self.button_relay    = self.get_parameter('button_relay').value
        self.button_led_on   = self.get_parameter('button_led_on').value

        # Publishers vers les autres nodes
        # → UNO
        self.servo_cmd_pub  = self.create_publisher(Float32, 'servo/cmd', 10)
        self.relay_cmd_pub  = self.create_publisher(Bool, 'relay/cmd', 10)
        self.led_cmd_pub    = self.create_publisher(ColorRGBA, 'led/cmd', 10)
        # → MEGA (moteurs pas à pas), à adapter à ton bridge méga
        self.stepper_cmd_pub = self.create_publisher(Float32, 'stepper/cmd', 10)

        # Pour mise à jour URDF
        self.cam_pan_pub  = self.create_publisher(Float32, 'camera/pan_cmd', 10)
        self.cam_tilt_pub = self.create_publisher(Float32, 'camera/tilt_cmd', 10)
        self.mode_pub     = self.create_publisher(String, 'control_mode', 10)

        # Sub joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        # État interne
        self.mode = MODE_CAM
        self.relay_state = False
        self.led_enabled = False
        self.last_button_mode  = 0
        self.last_button_relay = 0
        self.last_button_led   = 0

        self.get_logger().info(f'SRS teleop démarré, mode initial = {self.mode}')

    def joy_cb(self, msg: Joy):

        # ---- 1) Bouton de mode ----
        b_mode = msg.buttons[self.button_mode] if self.button_mode < len(msg.buttons) else 0
        if b_mode == 1 and self.last_button_mode == 0:
            self.mode = MODE_STEPPER if self.mode == MODE_CAM else MODE_CAM
            m = String(); m.data = self.mode
            self.mode_pub.publish(m)
            self.get_logger().info(f'Mode → {self.mode}')
        self.last_button_mode = b_mode

        # ---- 2) Toggle relais ----
        b_rel = msg.buttons[self.button_relay] if self.button_relay < len(msg.buttons) else 0
        if b_rel == 1 and self.last_button_relay == 0:
            self.relay_state = not self.relay_state
            r = Bool(); r.data = self.relay_state
            self.relay_cmd_pub.publish(r)
        self.last_button_relay = b_rel

        # ---- 3) Toggle LED ----
        b_led = msg.buttons[self.button_led_on] if self.button_led_on < len(msg.buttons) else 0
        if b_led == 1 and self.last_button_led == 0:
            self.led_enabled = not self.led_enabled
        self.last_button_led = b_led

        # ---- 4) LED : couleur + luminosité via axes ----
        brightness = 1.0
        if self.axis_brightness < len(msg.axes):
            raw = msg.axes[self.axis_brightness]  # [-1..1]
            brightness = (1.0 - raw) / 2.0        # map to [0..1]

        led_msg = ColorRGBA()
        if self.led_enabled:
            # ex: couleur en fonction du mode
            if self.mode == MODE_CAM:
                led_msg.r = brightness
                led_msg.g = brightness
                led_msg.b = 0.0
            else:
                led_msg.r = 0.0
                led_msg.g = brightness
                led_msg.b = brightness
            led_msg.a = brightness   # utilisé comme "intensité" côté Arduino
        else:
            led_msg.r = led_msg.g = led_msg.b = 0.0
            led_msg.a = 0.0

        self.led_cmd_pub.publish(led_msg)

        # ---- 5) Contrôle en fonction du mode ----
        if self.mode == MODE_CAM:
            self.handle_camera_mode(msg)
        else:
            self.handle_stepper_mode(msg)

    def handle_camera_mode(self, msg: Joy):
        # joystick → pan/tilt en degrés
        pan  = 0.0
        tilt = 0.0
        if self.axis_pan < len(msg.axes):
            pan = msg.axes[self.axis_pan] * 90.0     # [-90°, +90°]
        if self.axis_tilt < len(msg.axes):
            tilt = msg.axes[self.axis_tilt] * 60.0   # [-60°, +60°] par ex.

        # Publication commandes "logiques" pour joint state bridge
        pan_msg = Float32();  pan_msg.data = pan
        tilt_msg = Float32(); tilt_msg.data = tilt
        self.cam_pan_pub.publish(pan_msg)
        self.cam_tilt_pub.publish(tilt_msg)

        # Publication vers UNO (servo principal pour la caméra)
        servo_cmd = Float32()
        servo_cmd.data = pan + 90.0   # ex: map [-90..+90] -> [0..180] si tu veux
        self.servo_cmd_pub.publish(servo_cmd)

    def handle_stepper_mode(self, msg: Joy):
        # Ex: un axe contrôle une vitesse ou une consigne pour tes moteurs pas à pas
        if self.axis_pan < len(msg.axes):
            v = msg.axes[self.axis_pan]   # [-1..1]
            cmd = Float32()
            cmd.data = v   # à interpréter côté méga (vitesse / vitesse cible)
            self.stepper_cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SRSTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
