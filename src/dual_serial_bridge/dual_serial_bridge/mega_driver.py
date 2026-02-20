#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import threading

# Importation des types de messages standards ROS 2
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray

class MegaDriver(Node):
    def __init__(self):
        super().__init__('mega_driver')
        
        # --- PARAMÈTRES ---
        self.declare_parameter('port', '/dev/ttyACM0') # À adapter selon votre port
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # --- CONNEXION SÉRIE ---
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"✅ Connecté à l'Arduino Mega sur {port} à {baudrate} bauds.")
        except Exception as e:
            self.get_logger().error(f"❌ Impossible d'ouvrir le port série: {e}")
            raise SystemExit
            
        # --- PUBLISHERS (Télémétrie Mega -> ROS 2) ---
        # mega/positions : [M1_pos, M2_pos, M3_pos, M4_pos]
        self.pos_pub = self.create_publisher(Int32MultiArray, 'mega/positions', 10)
        
        # mega/power : [V1, I1, P1, V2, I2, P2, V3, I3, P3, V4, I4, P4]
        self.pwr_pub = self.create_publisher(Float32MultiArray, 'mega/power', 10)
        
        # mega/limits : [M1_Min, M1_Max, M2_Min, M2_Max, M3_Min, M3_Max, M4_Min, M4_Max] (1 = touché)
        self.lim_pub = self.create_publisher(Int32MultiArray, 'mega/limits', 10)
        
        # mega/alarms : [M1_Alm, M2_Alm, M3_Alm, M4_Alm] (1 = driver en erreur)
        self.alm_pub = self.create_publisher(Int32MultiArray, 'mega/alarms', 10)
        
        # --- SUBSCRIBER (Commandes ROS 2 -> Mega) ---
        # Reçoit du JSON brut sous forme de String (ex: '{"cmd":"move", "axis":1, "steps":100}')
        self.sub_cmd = self.create_subscription(String, 'mega/command', self.cmd_callback, 10)
        
        # --- DÉMARRAGE DU THREAD DE LECTURE ---
        self.is_running = True
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()

    def cmd_callback(self, msg):
        """Envoie les commandes reçues de ROS 2 vers l'Arduino via le port Série."""
        if self.ser.is_open:
            cmd_str = msg.data.strip() + '\n'
            self.ser.write(cmd_str.encode('utf-8'))
            self.get_logger().debug(f"Commande envoyée: {cmd_str.strip()}")

    def read_serial(self):
        """Boucle infinie lue dans un thread séparé pour ne pas bloquer ROS 2."""
        while rclpy.ok() and self.ser.is_open and self.is_running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        self.process_json(line)
            except Exception as e:
                self.get_logger().warning(f"Erreur de lecture série: {e}")

    def process_json(self, line):
        """Parse le JSON et publie sur les topics correspondants."""
        try:
            data = json.loads(line)
            
            # On vérifie que le message vient bien de la Mega
            if data.get("src") == "mega":
                
                # 1. POSITIONS MOTEURS
                if "pos" in data:
                    msg = Int32MultiArray()
                    msg.data = [int(x) for x in data["pos"]]
                    self.pos_pub.publish(msg)
                    
                # 2. CAPTEURS DE PUISSANCE (INA260)
                if "pwr" in data:
                    msg = Float32MultiArray()
                    msg.data = [float(x) for x in data["pwr"]]
                    self.pwr_pub.publish(msg)
                    
                # 3. FINS DE COURSE (Sécurité Min/Max)
                if "mr" in data:
                    msg = Int32MultiArray()
                    msg.data = [int(x) for x in data["mr"]]
                    self.lim_pub.publish(msg)
                    
                # 4. ALARMES DRIVERS (ALM)
                if "alm" in data:
                    msg = Int32MultiArray()
                    msg.data = [int(x) for x in data["alm"]]
                    self.alm_pub.publish(msg)
                    
                    # Log critique dans le terminal ROS 2 si un driver plante !
                    if sum(msg.data) > 0:
                        self.get_logger().error(f"⚠️ ALARME DRIVER DÉTECTÉE (M1..M4) : {msg.data}")

        except json.JSONDecodeError:
            # On ignore silencieusement les lignes qui ne sont pas du JSON valide
            pass

    def destroy_node(self):
        """Ferme proprement le port série à l'arrêt du nœud."""
        self.is_running = False
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MegaDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()