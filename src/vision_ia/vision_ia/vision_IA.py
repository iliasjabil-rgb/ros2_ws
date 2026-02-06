#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import torch

MODEL_PATH = "yolov8n.pt"
CONF_THRESHOLD = 0.5
USE_GPU = True
SHOW_WINDOW = True


class VisionIA(Node):
    def __init__(self):
        super().__init__('vision_IA_node')

        self.bridge = CvBridge()

        self.get_logger().info("🔄 Chargement du modèle YOLOv8...")
        self.model = YOLO(MODEL_PATH)

        if torch.cuda.is_available() and USE_GPU:
            self.get_logger().info("✅ Utilisation du GPU (CUDA)")
            self.model.to('cuda')
        else:
            self.get_logger().info("⚙️ Utilisation du CPU")
            self.model.to('cpu')

        self.window_created = False

        # Souscription aux images
        self.subscription = self.create_subscription(
            Image,
            '/video_cam',
            self.image_callback,
            10
        )

        # Publication des coordonnées et du flux annoté
        self.publisher_ = self.create_publisher(Float32MultiArray, '/position_personne', 10)
        self.result_publisher = self.create_publisher(Image, '/video_result', 10)

        self.get_logger().info("✅ Nœud YOLO prêt !")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        orig_frame = frame.copy()

        # --------- YOLO DETECTION (tous objets) ---------
        results = self.model(frame)[0]

        positions = Float32MultiArray()

        for box in results.boxes:
            if box.conf[0] < CONF_THRESHOLD:
                continue

            cls = int(box.cls[0])
            label = self.model.names[cls]

            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

            # Ajouter les coordonnées au message Float32MultiArray
            positions.data.extend([x1, y1, x2, y2])

            # Dessiner les boîtes et labels
            cv2.rectangle(orig_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                orig_frame,
                f"{label}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        # Publier les coordonnées
        self.publisher_.publish(positions)

        # Publier l'image annotée
        img_msg = self.bridge.cv2_to_imgmsg(orig_frame, encoding='bgr8')
        self.result_publisher.publish(img_msg)

        # Affichage OpenCV
        if SHOW_WINDOW:
            if not self.window_created:
                cv2.namedWindow("Détection Objets - YOLO", cv2.WINDOW_NORMAL)
                self.window_created = True

            cv2.imshow("Détection Objets - YOLO", orig_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("🛑 Fermeture de la fenêtre vidéo.")
                cv2.destroyAllWindows()
                rclpy.shutdown()

    def destroy_node(self):
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionIA()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()