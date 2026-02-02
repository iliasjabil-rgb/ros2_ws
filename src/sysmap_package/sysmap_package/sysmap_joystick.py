#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, ColorRGBA, String, Int32MultiArray

class SysmapJoystick(Node):
    def __init__(self):
        super().__init__('sysmap_joystick')

        # ----------------------------
        # 1. PARAMÈTRES (VOTRE MAPPING)
        # ----------------------------
        # Axes Analogiques
        self.declare_parameter('axis_servo_1', 4)  # Caméra Pan
        self.declare_parameter('axis_servo_2', 5)  # Caméra Tilt
        self.declare_parameter('axis_servo_3', 2)  # Rotation Pince
        self.declare_parameter('axis_led_brightness', 3)
        self.declare_parameter('axis_stepper_x', 0) # Joy Gauche/Droite
        self.declare_parameter('axis_stepper_y', 1) # Joy Haut/Bas
        
        # Boutons Steppers
        self.declare_parameter('button_stepper_stockage_sens_a', 2) 
        self.declare_parameter('button_stepper_stockage_sens_b', 3) 
        self.declare_parameter('button_stepper_z_up', 4)   
        self.declare_parameter('button_stepper_z_down', 9) 

        # Boutons Homing
        self.declare_parameter('button_home_x', 5)  
        self.declare_parameter('button_home_y', 6)  
        self.declare_parameter('button_home_z', 8) 

        # Boutons Relais
        self.declare_parameter('button_relay_EV1', 12)
        self.declare_parameter('button_relay_EV2', 11)
        self.declare_parameter('button_relay_EV3', 10)
        self.declare_parameter('button_relay_Aspirateur', 13)
        self.declare_parameter('button_relay_led_white', 14)
        self.declare_parameter('button_relay_led_rgb', 15)
        
        # Boutons Pince (Servo 4 continu)
        self.declare_parameter('button_servo4_cw', 1)  # Serrage 
        self.declare_parameter('button_servo4_ccw', 7) # Desserrage

        # ----------------------------
        # 2. RÉGLAGES PHYSIQUES
        # ----------------------------
        self.declare_parameter('servo_speed_deg_per_s', 60.0)
        self.declare_parameter('servo3_neutral_deg', 90.0)
        self.declare_parameter('joy_deadzone', 0.15)
        self.declare_parameter('stepper_speed', 800)       
        self.declare_parameter('stepper_steps_per_loop', 200)
        self.declare_parameter('smooth_factor', 0.15) 

        # --- Récupération des valeurs ---
        p = self.get_parameter
        self.axis_s1 = p('axis_servo_1').value
        self.axis_s2 = p('axis_servo_2').value
        self.axis_s3 = p('axis_servo_3').value
        self.axis_led = p('axis_led_brightness').value
        self.axis_st_x = p('axis_stepper_x').value
        self.axis_st_y = p('axis_stepper_y').value

        # Steppers Z
        self.btn_st_z_up = p('button_stepper_z_up').value
        self.btn_st_z_down = p('button_stepper_z_down').value
        
        # Steppers Stockage
        self.btn_st_stock_a = p('button_stepper_stockage_sens_a').value
        self.btn_st_stock_b = p('button_stepper_stockage_sens_b').value

        # Relais
        self.btn_ev1 = p('button_relay_EV1').value
        self.btn_ev2 = p('button_relay_EV2').value
        self.btn_ev3 = p('button_relay_EV3').value
        self.btn_aspi = p('button_relay_Aspirateur').value
        self.btn_l_w = p('button_relay_led_white').value
        self.btn_l_rgb = p('button_relay_led_rgb').value
        
        # Homing
        self.btn_home_x = p('button_home_x').value
        self.btn_home_y = p('button_home_y').value
        self.btn_home_z = p('button_home_z').value

        # Pince
        self.btn_s4_cw = p('button_servo4_cw').value
        self.btn_s4_ccw = p('button_servo4_ccw').value

        # Physique
        self.step_speed = p('stepper_speed').value
        self.step_inc = p('stepper_steps_per_loop').value
        self.deadzone = p('joy_deadzone').value
        self.servo_speed = p('servo_speed_deg_per_s').value
        self.s3_neutral = p('servo3_neutral_deg').value
        self.smooth_k = p('smooth_factor').value

        # ----------------------------
        # 3. PUBLISHERS & SUBSCRIBERS
        # ----------------------------
        # Vers UNO
        self.servo1_pub = self.create_publisher(Float32, 'uno/servo1/cmd', 10)
        self.servo2_pub = self.create_publisher(Float32, 'uno/servo2/cmd', 10)
        self.servo3_pub = self.create_publisher(Float32, 'uno/servo3/cmd', 10)
        self.servo4_pub = self.create_publisher(Float32, 'uno/servo4/cmd', 10)

        self.relay_ev1_pub = self.create_publisher(Bool, 'uno/relay_ev1/cmd', 10)
        self.relay_ev2_pub = self.create_publisher(Bool, 'uno/relay_ev2/cmd', 10)
        self.relay_ev3_pub = self.create_publisher(Bool, 'uno/relay_ev3/cmd', 10)
        self.relay_aspi_pub = self.create_publisher(Bool, 'uno/relay_aspirateur/cmd', 10)
        self.relay_w_pub = self.create_publisher(Bool, 'uno/relay_led_white/cmd', 10)
        self.relay_rgb_pub = self.create_publisher(Bool, 'uno/relay_led_rgb/cmd', 10)
        
        self.led_pub = self.create_publisher(ColorRGBA, 'led/cmd', 10)
        self.srs_pub = self.create_publisher(Bool, 'led_effect/cmd', 10)
        
        # Vers MEGA
        self.mega_cmd_pub = self.create_publisher(String, 'mega/cmd', 10)

        # Inputs
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/cmd/stop_all', self.stop_callback, 10)

        # États internes
        self.prev_buttons = []
        self.servo1_angle = 90.0
        self.servo2_angle = 90.0
        self.last_time = self.get_clock().now()

        # Variables de lissage
        self.smooth_joy_s1 = 0.0
        self.smooth_joy_s2 = 0.0

        # Anti-spam
        self.last_sent_s1 = -1
        self.last_sent_s2 = -1
        self.last_sent_s3 = -1
        self.last_sent_s4 = -1
        
        self.states = {
            'ev1': False, 'ev2': False, 'ev3': False, 'aspi': False,
            'white': False, 'rgb': False
        }

        self.get_logger().info("SysmapJoystick DÉMARRÉ (Axes: 1=Col, 2=Stock, 3=Tige, 4=Retr)")

        # --- NOUVEAU : PUBLISHERS DE STATUS ---
        self.status_rpi_pub = self.create_publisher(Bool, 'status/rpi', 10)
        self.status_mega_pub = self.create_publisher(Bool, 'status/mega', 10)

        # --- NOUVEAU : MONITORING MEGA ---
        # On écoute ce que la Mega raconte (ex: sa position)
        # Assurez-vous que votre bridge Mega publie sur ce topic !
        self.mega_monitor_sub = self.create_subscription(String, 'mega/log', self.mega_watchdog_cb, 10)
        # OU si vous avez des positions :
        # self.mega_monitor_sub = self.create_subscription(Int32MultiArray, 'mega/pos', self.mega_watchdog_cb, 10)

        self.last_mega_time = self.get_clock().now()
        
        # Timer de vérification (1Hz)
        self.create_timer(1.0, self.system_check_callback)

        self.get_logger().info("SysmapJoystick DÉMARRÉ (+ Status Monitors)")

    # --- CALLBACKS DE STATUS ---
    def mega_watchdog_cb(self, msg):
        # On a reçu un signe de vie de la Mega
        self.last_mega_time = self.get_clock().now()

    def system_check_callback(self):
        # 1. RASPBERRY PI
        # Si ce code s'exécute, c'est que la RPi et ROS tournent -> True
        self.status_rpi_pub.publish(Bool(data=True))

        # 2. MEGA
        # Calcul du temps depuis le dernier message reçu
        delta = (self.get_clock().now() - self.last_mega_time).nanoseconds / 1e9
        is_mega_alive = delta < 3.0 # Timeout de 3 secondes
        self.status_mega_pub.publish(Bool(data=is_mega_alive))

    def send_mega(self, data_dict):
        msg = String()
        msg.data = json.dumps(data_dict)
        self.mega_cmd_pub.publish(msg)

    # --- CALLBACK ARRÊT D'URGENCE ---
    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("⚠️ STOP GÉNÉRAL ACTIVÉ ⚠️")
            # Stop Moteurs
            self.send_mega({"cmd": "stop"})
            # Reset Servos
            self.servo1_angle = 0.0
            self.servo2_angle = 0.0
            self.servo1_pub.publish(Float32(data=0.0))
            self.servo2_pub.publish(Float32(data=0.0))
            self.servo3_pub.publish(Float32(data=0.0))
            self.servo4_pub.publish(Float32(data=0.0))
            # Reset Relais
            off = Bool(data=False)
            self.relay_ev1_pub.publish(off)
            self.relay_ev2_pub.publish(off)
            self.relay_ev3_pub.publish(off)
            self.relay_aspi_pub.publish(off)
            self.relay_w_pub.publish(off)
            self.relay_rgb_pub.publish(off)
            # Reset États
            for key in self.states: self.states[key] = False

    # --- CALLBACK JOYSTICK ---
    def joy_callback(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        if not self.prev_buttons:
            self.prev_buttons = buttons
            return

        def pressed(idx):
            if idx >= len(buttons): return False
            return buttons[idx] == 1 and self.prev_buttons[idx] == 0

        def held(idx):
            if idx >= len(buttons): return False
            return buttons[idx] == 1

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt > 0.1: dt = 0.02
        self.last_time = now

        # ==========================
        # 1. GESTION STEPPERS (MEGA)
        # ==========================
        
        # --- DEFINITION DES AXES PHYSIQUES ---
        AXIS_COLONNE = 1    # 001MO
        AXIS_STOCKAGE = 2   # 002MO
        AXIS_TIGE = 3       # 003MO
        AXIS_RETRACT = 4    # 004MO

        # --- HOMING ---
        # Btn Home X -> Axe 3 (Tige)
        if pressed(self.btn_home_x): self.send_mega({"cmd": "home", "axis": AXIS_TIGE})
        # Btn Home Y -> Axe 1 (Colonne)
        if pressed(self.btn_home_y): self.send_mega({"cmd": "home", "axis": AXIS_COLONNE})
        # Btn Home Z -> Axe 4 (Rétractable)
        if pressed(self.btn_home_z): self.send_mega({"cmd": "home", "axis": AXIS_RETRACT})

        # --- JOG MANUEL JOYSTICK ---
        
        # JOYSTICK GAUCHE/DROITE -> Axe 3 (Tige)
        if self.axis_st_x < len(axes):
            val_x = axes[self.axis_st_x]
            if abs(val_x) > self.deadzone:
                steps = int(val_x * self.step_inc)
                self.send_mega({"cmd": "move", "axis": AXIS_TIGE, "steps": steps, "speed": self.step_speed})

        # JOYSTICK HAUT/BAS -> Axe 1 (Colonne)
        if self.axis_st_y < len(axes):
            val_y = axes[self.axis_st_y]
            if abs(val_y) > self.deadzone:
                steps = int(val_y * self.step_inc)
                self.send_mega({"cmd": "move", "axis": AXIS_COLONNE, "steps": steps, "speed": self.step_speed})

        # --- JOG MANUEL BOUTONS (Z UP/DOWN) ---
        # Boutons Z -> Axe 3 (Tige) - Même axe que Joy X
        if held(self.btn_st_z_up):
             self.send_mega({"cmd": "move", "axis": AXIS_TIGE, "steps": self.step_inc, "speed": self.step_speed})
        elif held(self.btn_st_z_down):
             self.send_mega({"cmd": "move", "axis": AXIS_TIGE, "steps": -self.step_inc, "speed": self.step_speed})

        # --- JOG MANUEL STOCKAGE (AXE 2) ---
        if held(self.btn_st_stock_a):
             self.send_mega({"cmd": "move", "axis": AXIS_STOCKAGE, "steps": self.step_inc, "speed": self.step_speed})
        elif held(self.btn_st_stock_b):
             self.send_mega({"cmd": "move", "axis": AXIS_STOCKAGE, "steps": -self.step_inc, "speed": self.step_speed})

        # ==========================
        # 2. GESTION SERVOS (UNO)
        # ==========================
        # Servo 1 (Caméra Pan) - Lissé
        if self.axis_s1 < len(axes):
            raw_val = axes[self.axis_s1]
            if abs(raw_val) < self.deadzone: raw_val = 0.0
            
            self.smooth_joy_s1 = (self.smooth_joy_s1 * (1.0 - self.smooth_k)) + (raw_val * self.smooth_k)
            
            if abs(self.smooth_joy_s1) > 0.01:
                self.servo1_angle += self.smooth_joy_s1 * self.servo_speed * dt
                self.servo1_angle = max(0.0, min(180.0, self.servo1_angle))
                
                current_int = int(self.servo1_angle)
                if current_int != self.last_sent_s1:
                    m = Float32(); m.data = float(current_int)
                    self.servo1_pub.publish(m)
                    self.last_sent_s1 = current_int

        # Servo 2 (Caméra Tilt) - Lissé
        if self.axis_s2 < len(axes):
            raw_val = axes[self.axis_s2]
            if abs(raw_val) < self.deadzone: raw_val = 0.0
            self.smooth_joy_s2 = (self.smooth_joy_s2 * (1.0 - self.smooth_k)) + (raw_val * self.smooth_k)

            if abs(self.smooth_joy_s2) > 0.01:
                self.servo2_angle += self.smooth_joy_s2 * self.servo_speed * dt
                self.servo2_angle = max(0.0, min(180.0, self.servo2_angle))
                
                current_int = int(self.servo2_angle)
                if current_int != self.last_sent_s2:
                    m = Float32(); m.data = float(current_int)
                    self.servo2_pub.publish(m)
                    self.last_sent_s2 = current_int

        # Servo 3 (Rotation Pince)
        if self.axis_s3 < len(axes):
            val = axes[self.axis_s3]
            angle = self.s3_neutral 
            if abs(val) > self.deadzone:
                angle += (val * 60.0)
            
            target_int = int(max(0.0, min(180.0, angle)))
            if target_int != self.last_sent_s3:
                m = Float32(); m.data = float(target_int)
                self.servo3_pub.publish(m)
                self.last_sent_s3 = target_int

        # Servo 4 (Ouverture/Fermeture - Rotation Continue)
        s4_val = 90.0
        if held(self.btn_s4_cw):    s4_val = 180.0
        elif held(self.btn_s4_ccw): s4_val = 0.0
        
        if int(s4_val) != self.last_sent_s4:
            m4 = Float32(); m4.data = s4_val
            self.servo4_pub.publish(m4)
            self.last_sent_s4 = int(s4_val)

        # ==========================
        # 3. RELAIS
        # ==========================
        def toggle(btn_idx, key, pub):
            if pressed(btn_idx):
                self.states[key] = not self.states[key]
                m = Bool(); m.data = self.states[key]; pub.publish(m)
                self.get_logger().info(f"Relais {key} -> {self.states[key]}")

        toggle(self.btn_ev1, 'ev1', self.relay_ev1_pub)
        toggle(self.btn_ev2, 'ev2', self.relay_ev2_pub)
        toggle(self.btn_ev3, 'ev3', self.relay_ev3_pub)
        toggle(self.btn_aspi, 'aspi', self.relay_aspi_pub)
        toggle(self.btn_l_w, 'white', self.relay_w_pub)
        toggle(self.btn_l_rgb, 'rgb', self.relay_rgb_pub)

        # Luminosité LED
        if self.axis_led < len(axes):
            brightness = (axes[self.axis_led] + 1.0) / 2.0
            c = ColorRGBA()
            c.r = c.g = c.b = 1.0; c.a = brightness
            self.led_pub.publish(c)

        self.prev_buttons = buttons

def main(args=None):
    rclpy.init(args=args)
    node = SysmapJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()