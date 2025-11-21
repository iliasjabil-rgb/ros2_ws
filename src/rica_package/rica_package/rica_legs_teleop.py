#!/usr/bin/env python3
import socket, time, threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

def clamp(v,a,b): return max(a,min(b,v))

class TcpClient:
    def __init__(self, ip, port, timeout=1.0, backoff=1.0, log=None):
        self.ip, self.port, self.timeout, self.backoff = ip, port, timeout, backoff
        self.sock=None; self.lock=threading.Lock(); self.log=log
    def _print(self, lvl, msg):
        (self.log.info if lvl=="i" else self.log.warn if lvl=="w" else print)(msg)
    def _connect(self):
        with self.lock:
            try:
                if self.sock: self.sock.close()
                s=socket.socket(socket.AF_INET,socket.SOCK_STREAM); s.settimeout(self.timeout); s.connect((self.ip,self.port))
                self.sock=s; self._print("i", f"[TCP] connecté {self.ip}:{self.port}")
            except Exception as e:
                self.sock=None; self._print("w", f"[TCP] connexion échouée: {e}")
    def send(self, line:str):
        data=(line+"\r\n").encode("ascii","ignore")
        for _ in range(2):
            with self.lock:
                if not self.sock: self._connect()
                if not self.sock: time.sleep(self.backoff); continue
                try:
                    self.sock.sendall(data); return True
                except Exception as e:
                    self._print("w", f"[TCP] erreur sur '{line}': {e} (reconnect)")
                    try: self.sock.close()
                    except: pass
                    self.sock=None
            time.sleep(self.backoff)
        return False

class LegsTeleop(Node):
    def __init__(self):
        super().__init__("rica_legs_teleop")
        # params
        self.declare_parameter("ip","192.168.0.2")
        self.declare_parameter("port",2009)
        self.declare_parameter("deadzone",0.10)
        self.declare_parameter("rate_hz",30.0)
        # mapping manette (adapte si besoin)
        self.declare_parameter("btn_rb",5)       # RB = deadman
        self.declare_parameter("btn_lt", -1)     # si LT/RT exposés en boutons
        self.declare_parameter("btn_rt", -1)
        self.declare_parameter("ax_lt",2)        # LT axe (souvent 2)  mode 'low' actif
        self.declare_parameter("ax_rt",5)        # RT axe (souvent 5)  mode 'high' actif
        self.declare_parameter("trig_thr",0.5)
        self.declare_parameter("ax_front",1)     # stick G vertical par défaut
        self.declare_parameter("ax_rear",4)      # stick D vertical par défaut
        self.declare_parameter("inv_front",False)
        self.declare_parameter("inv_rear",False)

        self.tcp = TcpClient(self.get_param("ip"), int(self.get_param("port")), log=self.get_logger())
        self.dt = 1.0/max(1.0,float(self.get_param("rate_hz")))
        self.deadzone=float(self.get_param("deadzone"))
        self.btn_rb=int(self.get_param("btn_rb"))
        self.btn_lt=int(self.get_param("btn_lt")); self.btn_rt=int(self.get_param("btn_rt"))
        self.ax_lt=int(self.get_param("ax_lt")); self.ax_rt=int(self.get_param("ax_rt"))
        self.thr=float(self.get_param("trig_thr"))
        self.ax_front=int(self.get_param("ax_front")); self.ax_rear=int(self.get_param("ax_rear"))
        self.inv_front=bool(self.get_param("inv_front")); self.inv_rear=bool(self.get_param("inv_rear"))
        self.last_front=None; self.last_rear=None; self.last_joy=None
        self.create_subscription(Joy,"/joy",self.on_joy,10)
        self.create_timer(self.dt,self.tick)
        self.get_logger().info("LegsTeleop prêt (RB + (LT|RT) + sticks => BRAS_*)")

    def get_param(self,k): return self.get_parameter(k).value
    def on_joy(self,msg): self.last_joy=msg
    def pressed(self,buttons,i): return 0<=i<len(buttons) and buttons[i]==1
    def axis_ok(self,axes,i): return 0<=i<len(axes)
    def gate_ok(self,joy):
        # Autorisation pattes si LT ou RT (bouton OU axe)
        lt = self.pressed(joy.buttons,self.btn_lt) or (self.axis_ok(joy.axes,self.ax_lt) and joy.axes[self.ax_lt] < -self.thr)
        rt = self.pressed(joy.buttons,self.btn_rt) or (self.axis_ok(joy.axes,self.ax_rt) and joy.axes[self.ax_rt] >  self.thr)
        return lt or rt

    def tick(self):
        j=self.last_joy
        if j is None: return
        if not self.pressed(j.buttons,self.btn_rb):
            self.stop_if_needed(); return
        if not self.gate_ok(j):
            self.stop_if_needed(); return

        # FRONT (pattes avant) = axe ax_front
        front = j.axes[self.ax_front] if self.axis_ok(j.axes,self.ax_front) else 0.0
        if self.inv_front: front*=-1.0
        self.apply_axis(front,"front")

        # REAR (pattes arrière) = axe ax_rear
        rear = j.axes[self.ax_rear] if self.axis_ok(j.axes,self.ax_rear) else 0.0
        if self.inv_rear: rear*=-1.0
        self.apply_axis(rear,"rear")

    def apply_axis(self, val, which):
        if abs(val) < self.deadzone:
            self.send_stop(which); return
        speed = int(clamp(abs(val),0.0,1.0)*100 + 0.5)  # 0..100 %
        up = val > 0.0
        if which=="front":
            st=("UP" if up else "DOWN", speed)
            if self.last_front!=st:
                self.tcp.send(f"BRAS_AV_{st[0]} {st[1]}"); self.last_front=st
        else:
            st=("UP" if up else "DOWN", speed)
            if self.last_rear!=st:
                self.tcp.send(f"BRAS_AR_{st[0]} {st[1]}"); self.last_rear=st

    def send_stop(self, which):
        if which=="front":
            if self.last_front is None or self.last_front[1]!=0:
                base="UP" if (self.last_front and self.last_front[0]=="UP") else "DOWN"
                self.tcp.send(f"BRAS_AV_{base} 0"); self.last_front=(base,0)
        else:
            if self.last_rear is None or self.last_rear[1]!=0:
                base="UP" if (self.last_rear and self.last_rear[0]=="UP") else "DOWN"
                self.tcp.send(f"BRAS_AR_{base} 0"); self.last_rear=(base,0)

def main():
    rclpy.init(); n=LegsTeleop()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.send_stop("front"); n.send_stop("rear")
        except Exception: pass
        n.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
