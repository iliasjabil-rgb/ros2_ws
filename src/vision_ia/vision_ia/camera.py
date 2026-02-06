#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class TwoCameraNode(Node):
    def __init__(self):
        # Le nom du nœud ROS 2 est 'camera_node'
        super().__init__('camera')
        
        # --- 1. Paramètres de Configuration ---

        # Caméra 1: Index 0 par défaut
        self.declare_parameter('camera_index_1', 0)
        self.declare_parameter('topic_name_1', '/video_cam') 
        
        cam_index_1 = self.get_parameter('camera_index_1').get_parameter_value().integer_value
        topic_name_1 = self.get_parameter('topic_name_1').get_parameter_value().string_value

        # Caméra 2: Index 2 par défaut
        self.declare_parameter('camera_index_2', 2)
        self.declare_parameter('topic_name_2', '/video_cam_2') 
        
        cam_index_2 = self.get_parameter('camera_index_2').get_parameter_value().integer_value
        topic_name_2 = self.get_parameter('topic_name_2').get_parameter_value().string_value

        self.bridge = CvBridge()
        
        self.get_logger().info(f"Démarrage du nœud double caméra.")
        self.get_logger().info(f"Config C1: {cam_index_1} -> {topic_name_1}")
        self.get_logger().info(f"Config C2: {cam_index_2} -> {topic_name_2}")

        # --- 2. Initialisation avec Résolution Forcée ---

        # --- CAMÉRA 1 ---
        video_path_1 = f"/dev/video{cam_index_1}"
        # Utilisation de V4L2
        self.cap1 = cv2.VideoCapture(video_path_1, cv2.CAP_V4L2)
        
        # TENTATIVE DE FORCER LA RÉSOLUTION (Crucial pour la stabilité)
        if self.cap1.isOpened():
            self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Vérification finale C1
        if not self.cap1.isOpened():
            self.get_logger().error(f"ECHEC C1: Impossible d'ouvrir {video_path_1}")
            self.cap1 = None
        else:
            self.get_logger().info(f"SUCCES C1: {video_path_1} ouvert.")
            
        self.publisher_1 = self.create_publisher(Image, topic_name_1, 10)


        # --- CAMÉRA 2 ---
        video_path_2 = f"/dev/video{cam_index_2}"
        # Utilisation de V4L2
        self.cap2 = cv2.VideoCapture(video_path_2, cv2.CAP_V4L2)
        
        # TENTATIVE DE FORCER LA RÉSOLUTION (Crucial pour C2)
        if self.cap2.isOpened():
            self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Vérification finale C2
        if not self.cap2.isOpened():
            self.get_logger().error(f"ECHEC C2: Impossible d'ouvrir {video_path_2}")
            self.cap2 = None
        else:
            self.get_logger().info(f"SUCCES C2: {video_path_2} ouvert.")

        self.publisher_2 = self.create_publisher(Image, topic_name_2, 10)

        # Timer pour la boucle principale (~30 FPS)
        self.timer = self.create_timer(0.033, self.publish_frames) 

    def publish_frames(self):
        # Publie les deux flux séquentiellement
        self._publish_single_frame(self.cap1, self.publisher_1, "Caméra 1")
        self._publish_single_frame(self.cap2, self.publisher_2, "Caméra 2")

    def _publish_single_frame(self, cap, publisher, log_name):
        """Fonction utilitaire pour lire et publier une frame."""
        # Si la caméra n'a pas été ouverte, on ne fait rien
        if cap is None:
            return

        ret, frame = cap.read()
        
        if not ret:
            # On log en debug ou warning pour ne pas spammer si la caméra est débranchée
            self.get_logger().warning(f"Erreur lecture frame {log_name}", throttle_duration_sec=5.0)
            return

        try:
            # Conversion BGR8 (Standard OpenCV) -> ROS Image
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            publisher.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Erreur publication {log_name}: {e}")

    def destroy_node(self):
        # Libération propre des ressources
        self.get_logger().info("Arrêt du nœud, libération des caméras...")
        if self.cap1:
            self.cap1.release()
        if self.cap2:
            self.cap2.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()