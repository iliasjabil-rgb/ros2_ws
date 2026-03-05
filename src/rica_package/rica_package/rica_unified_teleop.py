#!/usr/bin/env python3
import socket, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String, Empty

def clamp(v, a, b): return a if v < a else b if v > b else v

class TcpClient:
    def __init__(self, ip, port, timeout=0.6, backoff=0.8, log=None):
        self.ip, self.port, self.timeout, self.backoff = ip, port, timeout, backoff
        self.log = log; self.sock=None; self.lock=threading.RLock()
        self._connect()
        
    def _log(self, lvl, msg):
        (self.log.info if lvl=="i" else self.log.warn if lvl=="w" else print)(msg)
        
    def _connect(self):
        with self.lock:
            try:
                if self.sock: self.sock.close()
                s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                
                # 🚀 LA LIGNE MAGIQUE ANTI-LATENCE 🚀
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                
                s.settimeout(self.timeout); s.connect((self.ip,self.port))
                self.sock=s; self._log("i", f"[TCP] connecté {self.ip}:{self.port}")
            except Exception as e:
                self.sock=None; self._log("w", f"[TCP] connexion échouée: {e}")

    def send(self, line:str):
        data=(line+"\r\n").encode("ascii","ignore")
        for _ in range(2):
            with self.lock:
                if not self.sock: self._connect()
                if not self.sock: time.sleep(self.backoff); continue
                try: self.sock.sendall(data); return True
                except Exception as e:
                    self._log("w", f"[TCP] erreur '{line}': {e} (reconnect)")
                    try: self.sock.close()
                    except: pass
                    self.sock=None
            time.sleep(self.backoff)
        return False
        
    def query(self, line:str, expect_bytes=256):
        data=(line+"\r\n").encode("ascii","ignore")
        for _ in range(2):
            with self.lock:
                if not self.sock: self._connect()
                if not self.sock: time.sleep(self.backoff); continue
                try:
                    self.sock.sendall(data)
                    self.sock.settimeout(self.timeout)
                    resp = self.sock.recv(expect_bytes)
                    return resp.decode("ascii","ignore").strip()
                except Exception as e:
                    self._log("w", f"[TCP] erreur query '{line}': {e} (reconnect)")
                    try: self.sock.close()
                    except: pass
                    self.sock=None
            time.sleep(self.backoff)
        return ""

class UnifiedTeleop(Node):
    def __init__(self):
        super().__init__("rica_unified_teleop")
        
        # Réseau & Mapping de base
        self.declare_parameter("ip", "192.168.0.2")
        self.declare_parameter("port", 2009)
        self.declare_parameter("max_speed_mps", 0.6)
        self.declare_parameter("deadzone_leg", 0.10)
        self.declare_parameter("btn_rb", 5)
        self.declare_parameter("btn_lt", -1)
        self.declare_parameter("btn_rt", -1)
        self.declare_parameter("ax_lt", 2)
        self.declare_parameter("ax_rt", 5)
        self.declare_parameter("trig_thr", 0.5)
        self.declare_parameter("ax_leg", 1)
        self.declare_parameter("inv_leg", False)
        self.declare_parameter("poll_hz", 4.0)

        # 🛡️ Paramètres de sécurité (Limites des bras) 🛡️
        self.declare_parameter("bras_ar_max", 50000) 
        self.declare_parameter("bras_ar_min", -1000)

        # Récupération des valeurs
        ip = self.get_parameter("ip").value
        port = int(self.get_parameter("port").value)
        self.tcp = TcpClient(ip, port, log=self.get_logger())

        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.deadzone_leg = float(self.get_parameter("deadzone_leg").value)
        self.btn_rb = int(self.get_parameter("btn_rb").value)
        self.btn_lt = int(self.get_parameter("btn_lt").value)
        self.btn_rt = int(self.get_parameter("btn_rt").value)
        self.ax_lt = int(self.get_parameter("ax_lt").value)
        self.ax_rt = int(self.get_parameter("ax_rt").value)
        self.trig_thr = float(self.get_parameter("trig_thr").value)
        self.ax_leg = int(self.get_parameter("ax_leg").value)
        self.inv_leg = bool(self.get_parameter("inv_leg").value)
        
        self.bras_ar_max = int(self.get_parameter("bras_ar_max").value)
        self.bras_ar_min = int(self.get_parameter("bras_ar_min").value)

        # États internes
        self.last_leg_state = None
        self.last_cmd_lr = (None,None)
        self.power_on = False
        self.connected = False
        
        # Variables de capteurs pour la sécurité
        self.pos_bras_av = 0
        self.pos_bras_ar = 0
        self.fdc_av = False
        self.fdc_ar = False

        # Publishers IHM
        self.pub_connected = self.create_publisher(Bool, "/rica/connected", 1)
        self.pub_power     = self.create_publisher(Bool, "/rica/power", 1)
        self.pub_cmdlog    = self.create_publisher(String, "/rica/cmd_log", 20)
        self.pub_cod_roues = self.create_publisher(String, "/rica/cod_roues_raw", 10)
        self.pub_cod_bras  = self.create_publisher(String, "/rica/cod_bras_raw", 10)

        # Subscriptions
        self.create_subscription(Empty, "/rica/appli_stop", self.on_appli_stop, 1)
        self.create_subscription(Bool,  "/rica/power_cmd",  self.on_power_cmd, 1)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Joy,   "/joy",     self.on_joy,     10)

        # --- Initialisation du robot : Force le Power à ON ---
        time.sleep(0.5) 
        self.get_logger().info("Envoi de la commande POWER 1 (Activation forcée)...")
        self._send_and_log("POWER 1")
        self.power_on = True
        self._pub_states()

        # Timers pour le polling
        self.create_timer(1.0, self.heartbeat)
        self.create_timer(1.0 / max(0.5, float(self.get_parameter("poll_hz").value)), self.poll_encoders)

        self.get_logger().info("Unified: Prêt avec BOUCLIER DE SÉCURITÉ activé pour les bras.")

    # --- Utilitaires de parsing (traduction Trame -> Variables) ---
    def _parse_encodeur_24c(self, rep: str):
        if not rep or len(rep) < 24: return None
        try:
            b = [int(rep[i:i+3]) for i in range(0, 24, 3)]
            def i32_be(o):
                v = (b[o]<<24) | (b[o+1]<<16) | (b[o+2]<<8) | b[o+3]
                return v - (1<<32) if v & 0x80000000 else v
            return i32_be(0), i32_be(4)
        except Exception:
            return None

    def _parse_fdc(self, rep: str):
        if not rep or len(rep) < 6: return None
        try:
            return int(rep[0:3]) > 0, int(rep[3:6]) > 0
        except Exception:
            return None

    # --- Helpers IHM ---
    def _pub_states(self):
        self.pub_connected.publish(Bool(data=self.connected))
        self.pub_power.publish(Bool(data=self.power_on))
        
    def _send_and_log(self, line: str, query=False):
        if query:
            resp = self.tcp.query(line) or ""
            self.connected = bool(resp)
            self._pub_states()
            self.pub_cmdlog.publish(String(data=f"> {line} | < {resp}"))
            return resp
        else:
            ok = self.tcp.send(line)
            self.connected = self.connected or ok
            self._pub_states()
            self.pub_cmdlog.publish(String(data=f"> {line}"))
            return ""

    # --- IHM callbacks ---
    def on_appli_stop(self, _): 
        self._send_and_log("APPLI_STOP")
        
    def on_power_cmd(self, msg: Bool):
        self._send_and_log(f"POWER {'1' if msg.data else '0'}")
        self.power_on = bool(msg.data)
        self._pub_states()

    # --- Base roulante (/cmd_vel → roues) ---
    def on_cmd_vel(self, msg: Twist):
        vx, wz = msg.linear.x, msg.angular.z
        l = int(clamp((vx - 0.5*wz*self.max_speed)/self.max_speed, -1.0, 1.0)*100 + 0.5)
        r = int(clamp((vx + 0.5*wz*self.max_speed)/self.max_speed, -1.0, 1.0)*100 + 0.5)
        l = int(clamp(l, -100, 100)); r = int(clamp(r, -100, 100))
        
        last_l, last_r = self.last_cmd_lr
        is_stop = (l == 0 and r == 0)
        
        # Filtre anti-saturation
        if not is_stop and last_l is not None and last_r is not None:
            if abs(l - last_l) < 4 and abs(r - last_r) < 4:
                return 

        if (l,r) == self.last_cmd_lr: return
        self.last_cmd_lr = (l,r)
        
        self._send_and_log(f"{'AVANCER_ROUES_G' if l>=0 else 'RECULER_ROUES_G'} {abs(l)}")
        self._send_and_log(f"{'AVANCER_ROUES_D' if r>=0 else 'RECULER_ROUES_D'} {abs(r)}")

    # --- Contrôle des pattes AV/AR avec SÉCURITÉS ---
    def on_joy(self, msg: Joy):
        # Sécurité Homme-mort (RB)
        if not self._btn(msg, self.btn_rb): 
            self._stop_all_legs_if_needed(); return
            
        # Détection des modes (Gâchettes)
        lt_held = self._btn(msg, self.btn_lt) or self._trig(msg, self.ax_lt, "L")
        rt_held = self._btn(msg, self.btn_rt) or self._trig(msg, self.ax_rt, "R")
        
        if not (lt_held or rt_held): 
            self._stop_all_legs_if_needed(); return
            
        # Lecture de la valeur du stick (Vitesse)
        val = self._axis(msg, self.ax_leg)
        val *= -1.0 if self.inv_leg else 1.0
        
        if abs(val) < self.deadzone_leg: 
            self._stop_all_legs_if_needed(); return
            
        # Bridgade à 60% pour éviter la chute de tension
        speed = int(clamp(abs(val), 0, 1)*60 + 0.5)
        up = val > 0.0
        direction = "UP" if up else "DOWN"

        # --- CAS 1 : BRAS ARRIÈRE (Mode LT) ---
        if lt_held:
            # Sécurité Bouclier
            if up and (self.fdc_ar or self.pos_bras_ar >= self.bras_ar_max):
                speed = 0
                self.get_logger().warn("⚠️ Limite HAUTE Arrière !")
            elif not up and (self.pos_bras_ar <= self.bras_ar_min):
                speed = 0
                self.get_logger().warn("⚠️ Limite BASSE Arrière !")
            
            new_state = (direction, speed, "AR")
            if self.last_leg_state != new_state:
                self._send_and_log(f"BRAS_AR_{direction} {speed}")
                self.last_leg_state = new_state

        # --- CAS 2 : BRAS AVANT (Mode RT) ---
        elif rt_held:
            # Sécurité Bouclier (à adapter avec des limites AV si besoin)
            if up and (self.fdc_av or self.pos_bras_av >= self.bras_ar_max): 
                speed = 0
                self.get_logger().warn("⚠️ Limite HAUTE Avant !")
            elif not up and (self.pos_bras_av <= self.bras_ar_min):
                speed = 0
                self.get_logger().warn("⚠️ Limite BASSE Avant !")

            new_state = (direction, speed, "AV")
            if self.last_leg_state != new_state:
                self._send_and_log(f"BRAS_AV_{direction} {speed}")
                self.last_leg_state = new_state

    def _stop_all_legs_if_needed(self):
        if not self.last_leg_state or self.last_leg_state[1] == 0:
            return
        
        direction, _, target = self.last_leg_state
        # Envoi de la commande d'arrêt au bon bras (AV ou AR)
        self._send_and_log(f"BRAS_{target}_{direction} 0")
        self.last_leg_state = (direction, 0, target)
        
    # --- Heartbeat + Polling Capteurs ---
    def heartbeat(self):
        rep_fdc = self._send_and_log("FINS_COURSE", query=True)
        if rep_fdc:
            parsed = self._parse_fdc(rep_fdc)
            if parsed:
                self.fdc_av, self.fdc_ar = parsed

    def poll_encoders(self):
        r = self._send_and_log("COD_ROUES", query=True)
        b = self._send_and_log("COD_BRAS", query=True)
        
        if r: self.pub_cod_roues.publish(String(data=r))
        if b: 
            self.pub_cod_bras.publish(String(data=b))
            parsed = self._parse_encodeur_24c(b)
            if parsed:
                self.pos_bras_av, self.pos_bras_ar = parsed

    # --- Utils Manette ---
    @staticmethod
    def _btn(msg, idx): return 0<=idx<len(msg.buttons) and msg.buttons[idx]==1
    def _trig(self,msg,idx,side):
        if not (0<=idx<len(msg.axes)): return False
        v=msg.axes[idx]; thr=self.trig_thr
        return (v<-thr) if side=="L" else (v>+thr)
    @staticmethod
    def _axis(msg, idx): return msg.axes[idx] if 0<=idx<len(msg.axes) else 0.0

# --- C'EST ICI QUE LE PROBLÈME SE TROUVAIT ---
def main(args=None):
    rclpy.init(args=args)
    n = UnifiedTeleop()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n._stop_all_legs_if_needed()
        except:
            pass
        n.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()