#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

class JointTeleop(Node):
    """
    RB = deadman
    - (no LT/RT)  : right stick Y -> base_to_left_side / base_to_right_side
    - (LT held)   : left  stick Y -> back_left  / back_right
    - (RT held)   : left  stick Y -> front_left / front_right
    LT/RT peuvent être des BOUTONS ou des AXES (gérés tous les deux).
    """
    def __init__(self):
        super().__init__('joint_teleop')

        # joints (doivent correspondre à l’URDF)
        self.declare_parameter('joint_front_left',  'base_to_front_left_leg')
        self.declare_parameter('joint_front_right', 'base_to_front_right_leg')
        self.declare_parameter('joint_back_left',   'base_to_back_left_leg')
        self.declare_parameter('joint_back_right',  'base_to_back_right_leg')

        # mapping boutons/axes
        self.declare_parameter('btn_deadman_rb', 5)
        self.declare_parameter('btn_lt',        -1)   # -1 si non utilisé
        self.declare_parameter('btn_rt',        -1)

        self.declare_parameter('axis_left_y',  1)
        self.declare_parameter('axis_right_y', 4)

        # LT/RT comme AXES (très courant sous Linux)
        self.declare_parameter('lt_axis_index', 2)
        self.declare_parameter('lt_axis_active_when', 'low')   # 'low' => actif si < threshold
        self.declare_parameter('rt_axis_index', 5)
        self.declare_parameter('rt_axis_active_when', 'high')  # 'high' => actif si > threshold
        self.declare_parameter('trigger_threshold', 0.5)

        # vitesses/limites
        self.declare_parameter('rate_rad_per_s', 1.2)
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('limit_low',  -3.14)
        self.declare_parameter('limit_high',  3.14)
        self.declare_parameter('rate_hz', 50.0)

        # état
        self.pos = {k: 0.0 for k in
            ['front_left','front_right','back_left','back_right','side_left','side_right']}

        # IO
        self.sub = self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # timer
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(self.dt, self.tick)

        self.last_joy = None
        self.get_logger().info('[JOINT_TELEOP] prêt (RB sécurité, LT/RT modes)')

    def on_joy(self, msg: Joy):
        self.last_joy = msg

    def _dz(self, v: float) -> float:
        dz = float(self.get_parameter('deadzone').value)
        return 0.0 if abs(v) < dz else v

    def _clamp(self, x: float) -> float:
        lo = float(self.get_parameter('limit_low').value)
        hi = float(self.get_parameter('limit_high').value)
        return min(hi, max(lo, x))

    def _is_pressed(self, buttons, idx: int) -> bool:
        return (idx >= 0) and (idx < len(buttons)) and (buttons[idx] == 1)

    def _axis_active(self, axes, idx: int, mode: str, thr: float) -> bool:
        if not (0 <= idx < len(axes)):
            return False
        val = axes[idx]
        return (val < -thr) if mode == 'low' else (val > thr)

    def tick(self):
        if self.last_joy is None:
            return

        # Récupération des paramètres (compat rclpy Jazzy)
        btn_rb = int(self.get_parameter('btn_deadman_rb').value)
        btn_lt = int(self.get_parameter('btn_lt').value)
        btn_rt = int(self.get_parameter('btn_rt').value)

        ax_ly  = int(self.get_parameter('axis_left_y').value)
        ax_ry  = int(self.get_parameter('axis_right_y').value)

        lt_ax   = int(self.get_parameter('lt_axis_index').value)
        lt_mode = str(self.get_parameter('lt_axis_active_when').value)
        rt_ax   = int(self.get_parameter('rt_axis_index').value)
        rt_mode = str(self.get_parameter('rt_axis_active_when').value)
        thr     = float(self.get_parameter('trigger_threshold').value)

        rate    = float(self.get_parameter('rate_rad_per_s').value)

        joy = self.last_joy

        # Deadman RB obligatoire
        if not self._is_pressed(joy.buttons, btn_rb):
            return

        # LT/RT : boutons OU axes
        lt_pressed = self._is_pressed(joy.buttons, btn_lt) or self._axis_active(joy.axes, lt_ax, lt_mode, thr)
        rt_pressed = self._is_pressed(joy.buttons, btn_rt) or self._axis_active(joy.axes, rt_ax, rt_mode, thr)

        # sticks
        ly = self._dz(joy.axes[ax_ly]) if 0 <= ax_ly < len(joy.axes) else 0.0
        ry = self._dz(joy.axes[ax_ry]) if 0 <= ax_ry < len(joy.axes) else 0.0

        d = rate * self.dt

        
        if lt_pressed:
            # Mode “BACK legs” : joystick gauche vertical
            self.pos['back_left']  = self._clamp(self.pos['back_left']  + (-ly) * d)
            self.pos['back_right'] = self._clamp(self.pos['back_right'] + (-ly) * d)
        elif rt_pressed:
            # Mode “FRONT legs” : joystick gauche vertical
            self.pos['front_left']  = self._clamp(self.pos['front_left']  + (-ly) * d)
            self.pos['front_right'] = self._clamp(self.pos['front_right'] + (-ly) * d)

        # Publier JointState
        names = [
            self.get_parameter('joint_front_left').value,
            self.get_parameter('joint_front_right').value,
            self.get_parameter('joint_back_left').value,
            self.get_parameter('joint_back_right').value,
        ]
        positions = [
            self.pos['front_left'], self.pos['front_right'],
            self.pos['back_left'],  self.pos['back_right'],
        ]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = positions
        self.pub.publish(js)

def main():
    rclpy.init()
    node = JointTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
