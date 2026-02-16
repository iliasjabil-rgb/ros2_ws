#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Importations cruciales pour le Multithreading et la QoS
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

class TwoCameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        
        # --- 1. Paramètres de Configuration ---
        self.declare_parameter('camera_index_1', 0)
        self.declare_parameter('topic_name_1', '/video_cam') 
        cam_index_1 = self.get_parameter('camera_index_1').get_parameter_value().integer_value
        topic_name_1 = self.get_parameter('topic_name_1').get_parameter_value().string_value

        self.declare_parameter('camera_index_2', 2)
        self.declare_parameter('topic_name_2', '/video_cam_2') 
        cam_index_2 = self.get_parameter('camera_index_2').get_parameter_value().integer_value
        topic_name_2 = self.get_parameter('topic_name_2').get_parameter_value().string_value

        self.bridge = CvBridge()
        
        # Création d'un groupe de callbacks qui permet l'exécution en parallèle
        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info("Démarrage du nœud double caméra (Mode Multithread & MJPEG).")

        # --- 2. Initialisation Caméra 1 ---
        self.cap1 = self._init_camera(cam_index_1)
        if self.cap1:
            # Utilisation de qos_profile_sensor_data (Best Effort, Queue = 5)
            self.publisher_1 = self.create_publisher(Image, topic_name_1, qos_profile_sensor_data)
            # Timer dédié à la caméra 1, rattaché au groupe parallèle
            self.timer1 = self.create_timer(0.033, self.publish_cam1, callback_group=self.callback_group)

        # --- 3. Initialisation Caméra 2 ---
        self.cap2 = self._init_camera(cam_index_2)
        if self.cap2:
            self.publisher_2 = self.create_publisher(Image, topic_name_2, qos_profile_sensor_data)
            # Timer dédié à la caméra 2, rattaché au même groupe parallèle
            self.timer2 = self.create_timer(0.033, self.publish_cam2, callback_group=self.callback_group)

    def _init_camera(self, index):
        """Fonction utilitaire pour configurer une caméra de manière optimisée."""
        video_path = f"/dev/video{index}"
        cap = cv2.VideoCapture(video_path, cv2.CAP_V4L2)
        
        if cap.isOpened():
            # 1. Forcer le format MJPEG pour ne pas saturer le bus USB
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            
            # 2. Forcer la résolution
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # 3. Réduire le buffer interne de la caméra à 1 pour toujours avoir l'image la plus récente
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            self.get_logger().info(f"SUCCES : {video_path} ouvert en MJPEG (640x480).")
            return cap
        else:
            self.get_logger().error(f"ECHEC : Impossible d'ouvrir {video_path}")
            return None

    def publish_cam1(self):
        self._publish_single_frame(self.cap1, self.publisher_1, "Caméra 1")

    def publish_cam2(self):
        self._publish_single_frame(self.cap2, self.publisher_2, "Caméra 2")

    def _publish_single_frame(self, cap, publisher, log_name):
        ret, frame = cap.read()
        if not ret:
            self.get_logger().warning(f"Erreur lecture frame {log_name}", throttle_duration_sec=5.0)
            return

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Erreur publication {log_name}: {e}")

    def destroy_node(self):
        self.get_logger().info("Arrêt du nœud, libération des caméras...")
        if hasattr(self, 'cap1') and self.cap1:
            self.cap1.release()
        if hasattr(self, 'cap2') and self.cap2:
            self.cap2.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwoCameraNode()
    
    # Utilisation d'un exécuteur multithread (par défaut, il utilise le nombre de cœurs de ton CPU)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # On fait tourner l'exécuteur au lieu du simple rclpy.spin(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()