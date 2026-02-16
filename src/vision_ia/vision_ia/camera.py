#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import subprocess
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

class TwoCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # --- Paramètres Caméra 1 ---
        self.declare_parameter('camera_index_1', 0)
        self.declare_parameter('topic_name_1', '/video_cam') 
        cam_index_1 = self.get_parameter('camera_index_1').value
        topic_name_1 = self.get_parameter('topic_name_1').value

        # --- Paramètres Caméra 2 ---
        self.declare_parameter('camera_index_2', 2)
        self.declare_parameter('topic_name_2', '/video_cam_2') 
        cam_index_2 = self.get_parameter('camera_index_2').value
        topic_name_2 = self.get_parameter('topic_name_2').value

        self.callback_group = ReentrantCallbackGroup()

        self.get_logger().info("Démarrage Nœud Double Caméra (V4L2 Hack + ROS 2 Compression)")

        # --- Initialisation Caméra 1 ---
        self.cap1 = self._init_camera(cam_index_1)
        if self.cap1:
            # On publie directement sur un topic "compressed"
            self.publisher_1 = self.create_publisher(CompressedImage, topic_name_1 + '/compressed', qos_profile_sensor_data)
            self.timer1 = self.create_timer(0.033, self.publish_cam1, callback_group=self.callback_group)

        # --- Initialisation Caméra 2 ---
        self.cap2 = self._init_camera(cam_index_2)
        if self.cap2:
            self.publisher_2 = self.create_publisher(CompressedImage, topic_name_2 + '/compressed', qos_profile_sensor_data)
            self.timer2 = self.create_timer(0.033, self.publish_cam2, callback_group=self.callback_group)

    def _init_camera(self, index):
        video_path = f"/dev/video{index}"
        
        # 1. LE HACK V4L2 (Sauve la bande passante USB de la Pi)
        # On force le matériel en MJPEG *avant* qu'OpenCV ne touche à la caméra
        try:
            cmd = ["v4l2-ctl", "-d", video_path, "-v", "pixelformat=MJPG,width=640,height=480"]
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"Driver V4L2 forcé en MJPEG pour {video_path}")
        except FileNotFoundError:
            self.get_logger().warning("v4l2-ctl non installé. Installez-le avec: sudo apt install v4l-utils")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Impossible de forcer v4l2-ctl sur {video_path}: {e}")

        # 2. Ouverture OpenCV
        cap = cv2.VideoCapture(video_path, cv2.CAP_V4L2)
        
        if cap.isOpened():
            # On confirme à OpenCV qu'on veut du MJPG et la bonne résolution
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            return cap
        else:
            self.get_logger().error(f"ECHEC : Impossible d'ouvrir {video_path}")
            return None

    def publish_cam1(self):
        self._publish_single_frame(self.cap1, self.publisher_1, "cam1")

    def publish_cam2(self):
        self._publish_single_frame(self.cap2, self.publisher_2, "cam2")

    def _publish_single_frame(self, cap, publisher, frame_id):
        ret, frame = cap.read()
        if not ret:
            self.get_logger().warning(f"Erreur lecture frame {frame_id}", throttle_duration_sec=5.0)
            return

        try:
            # 2. LA COMPRESSION ROS 2 (Sauve la bande passante Wi-Fi)
            # On compresse l'image en JPEG (qualité 80%) pour l'envoi réseau
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, encoded_image = cv2.imencode('.jpg', frame, encode_param)
            
            if result:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = frame_id
                msg.format = "jpeg"
                msg.data = np.array(encoded_image).tobytes()
                publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur publication {frame_id}: {e}")

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()