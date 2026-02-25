#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
import cv2
import torch
import numpy as np

class VisionIA(Node):
    def __init__(self):
        super().__init__('vision_IA_node')

        # --- PARAMÈTRES ROS 2 ---
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('show_window', False) # Désactivé par défaut (on utilise Foxglove)
        self.declare_parameter('input_topic', '/video_cam/compressed')

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        use_gpu = self.get_parameter('use_gpu').value
        self.show_window = self.get_parameter('show_window').value
        input_topic = self.get_parameter('input_topic').value

        # --- CHARGEMENT DU MODÈLE ---
        self.get_logger().info(f"🔄 Chargement du modèle {model_path}...")
        self.model = YOLO(model_path)

        if torch.cuda.is_available() and use_gpu:
            self.get_logger().info("✅ Utilisation du GPU (CUDA)")
            self.model.to('cuda')
        else:
            self.get_logger().info("⚙️ Utilisation du CPU")
            self.model.to('cpu')

        self.window_created = False

        # --- SUBSCRIBERS & PUBLISHERS ---
        self.subscription = self.create_subscription(CompressedImage, input_topic, self.image_callback, 10)
        
        # Nouveau topic générique pour les détections
        self.publisher_ = self.create_publisher(Float32MultiArray, '/vision/detections', 10)
        self.result_publisher = self.create_publisher(CompressedImage, '/video_result/compressed', 10)

        self.get_logger().info("✅ Nœud YOLO prêt ! En attente des images...")

    def image_callback(self, msg):
        # --------- DÉCODAGE DU FLUX COMPRESSÉ ---------
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is None:
            return

        orig_frame = frame.copy()

        # --------- YOLO DETECTION ---------
        results = self.model(orig_frame)[0]

        positions = Float32MultiArray()

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < self.conf_threshold:
                continue

            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]

            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

            # NOUVEAU FORMAT : [Classe, Confiance, X1, Y1, X2, Y2]
            # Cela permet au robot de trier ce qu'il voit !
            positions.data.extend([float(cls_id), conf, float(x1), float(y1), float(x2), float(y2)])

            # Dessin de la boîte
            cv2.rectangle(orig_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                orig_frame,
                f"{label} {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        # Publier les coordonnées si on a trouvé quelque chose
        if len(positions.data) > 0:
            self.publisher_.publish(positions)

        # --------- ENCODAGE ET PUBLICATION DU RÉSULTAT ---------
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        result, encoded_image = cv2.imencode('.jpg', orig_frame, encode_param)
        
        if result:
            msg_out = CompressedImage()
            msg_out.header = msg.header
            msg_out.format = "jpeg"
            msg_out.data = np.array(encoded_image).tobytes()
            self.result_publisher.publish(msg_out)

        # --------- AFFICHAGE LOCAL (Si activé) ---------
        if self.show_window:
            if not self.window_created:
                cv2.namedWindow("Détection YOLO", cv2.WINDOW_NORMAL)
                self.window_created = True

            cv2.imshow("Détection YOLO", orig_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("🛑 Fermeture de la fenêtre vidéo.")
                cv2.destroyAllWindows()
                rclpy.shutdown()

    def destroy_node(self):
        if self.show_window:
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