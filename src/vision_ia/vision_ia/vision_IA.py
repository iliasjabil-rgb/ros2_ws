#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage # <--- Changement ici
from ultralytics import YOLO
import cv2
import torch
import numpy as np # <--- Nécessaire pour décoder le JPEG

MODEL_PATH = "yolov8n.pt"
CONF_THRESHOLD = 0.5
USE_GPU = True
SHOW_WINDOW = True


class VisionIA(Node):
    def __init__(self):
        super().__init__('vision_IA_node')

        self.get_logger().info("🔄 Chargement du modèle YOLOv8...")
        self.model = YOLO(MODEL_PATH)

        if torch.cuda.is_available() and USE_GPU:
            self.get_logger().info("✅ Utilisation du GPU (CUDA)")
            self.model.to('cuda')
        else:
            self.get_logger().info("⚙️ Utilisation du CPU")
            self.model.to('cpu')

        self.window_created = False

        # 1. Souscription au topic COMPRESSÉ
        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_cam/compressed', # <--- On écoute le nouveau flux
            self.image_callback,
            10
        )

        # 2. Publication (Positions + Flux COMPRESSÉ)
        self.publisher_ = self.create_publisher(Float32MultiArray, '/position_personne', 10)
        
        # On publie le résultat en compressé pour ne pas tuer le Wi-Fi vers Foxglove !
        self.result_publisher = self.create_publisher(CompressedImage, '/video_result/compressed', 10)

        self.get_logger().info("✅ Nœud YOLO prêt (Compatible CompressedImage) !")

    def image_callback(self, msg):
        # --------- DÉCODAGE DU FLUX COMPRESSÉ ---------
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if frame is None:
            self.get_logger().warning("Erreur de décodage de l'image.")
            return

        orig_frame = frame.copy()

        # --------- YOLO DETECTION ---------
        results = self.model(frame)[0]

        positions = Float32MultiArray()

        for box in results.boxes:
            if box.conf[0] < CONF_THRESHOLD:
                continue

            cls = int(box.cls[0])
            label = self.model.names[cls]

            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

            # Ajouter les coordonnées
            positions.data.extend([x1, y1, x2, y2])

            # Dessiner les boîtes
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
        if len(positions.data) > 0:
            self.publisher_.publish(positions)

        # --------- ENCODAGE ET PUBLICATION DU RÉSULTAT ---------
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        result, encoded_image = cv2.imencode('.jpg', orig_frame, encode_param)
        
        if result:
            msg_out = CompressedImage()
            msg_out.header = msg.header # On garde le même timestamp
            msg_out.format = "jpeg"
            msg_out.data = np.array(encoded_image).tobytes()
            self.result_publisher.publish(msg_out)

        # --------- AFFICHAGE LOCAL OPENCV ---------
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