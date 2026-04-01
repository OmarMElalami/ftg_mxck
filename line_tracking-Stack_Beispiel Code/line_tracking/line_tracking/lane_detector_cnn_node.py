#!/usr/bin/env python3
"""
Lane Detector Node — Version CNN (PyTorch pur)

Charge un fichier .pth et fait l'inférence directement avec PyTorch.
Pas besoin d'ONNX Runtime.

Pour mettre à jour le modèle :
  1. Copie le nouveau best_model.pth sur le robot
  2. Relance le node (ros2 launch ... ou Ctrl+C + relance)
  C'est tout — l'architecture ne change jamais.

Publie (Float32MultiArray) — MÊME FORMAT que l'ancien node OpenCV :
    [valid, mode_int,
     lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,
     rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import time
import torch
import torch.nn as nn
from torchvision import models
import os
# Pour trouver les dossiers d'installation ROS 2 de manière dynamique
from ament_index_python.packages import get_package_share_directory


# ════════════════════════════════════════════════════════════════
# MODÈLE — identique à celui dans le notebook Colab
# ════════════════════════════════════════════════════════════════
class LaneNet(nn.Module):
    """
    ResNet18 + tête de régression → 16 coordonnées [0,1].
    Cette classe doit être IDENTIQUE à celle utilisée pour l'entraînement.
    Ne la modifie jamais sans réentraîner le modèle.
    """
    def __init__(self, num_outputs=16, pretrained=False):
        super().__init__()
        resnet = models.resnet18(weights=None)
        self.backbone = nn.Sequential(
            resnet.conv1, resnet.bn1, resnet.relu, resnet.maxpool,
            resnet.layer1, resnet.layer2, resnet.layer3, resnet.layer4
        )
        self.pool = nn.AdaptiveAvgPool2d((1, 1))
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(512, 256), nn.ReLU(), nn.Dropout(0.3),
            nn.Linear(256, 128), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(128, num_outputs), nn.Sigmoid()
        )

    def forward(self, x):
        return self.head(self.pool(self.backbone(x)))


# ════════════════════════════════════════════════════════════════
# NODE ROS2
# ════════════════════════════════════════════════════════════════
class LaneDetectorCNNNode(Node):

    def __init__(self):
        super().__init__('lane_detector_node')

        # ── Paramètres ──
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 360)
        self.declare_parameter('model_filename', '')
        self.declare_parameter('input_size', 224)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('use_cv_bridge', True)

        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.input_size = self.get_parameter('input_size').value
        model_filename = self.get_parameter('model_filename').value

        # ── LOCALISATION DYNAMIQUE DU MODÈLE ──
        # Au lieu d'un chemin fixe, on demande à ROS où est installé le package
        try:
            package_share = get_package_share_directory('line_tracking')
            model_filename = os.path.join(package_share, 'models', model_filename)
            self.get_logger().info(f'Recherche du modèle : {model_filename}')
        except Exception as e:
            self.get_logger().error(f"Erreur de localisation du package : {e}")
            raise e

        # ── Charger le modèle PyTorch ──
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'PyTorch device: {self.device}')

        self.model = LaneNet(num_outputs=16, pretrained=False)
        checkpoint = torch.load(model_filename, map_location=self.device, weights_only=True)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.to(self.device)
        self.model.eval()

        if 'input_size' in checkpoint:
            saved_size = checkpoint['input_size']
            if saved_size != self.input_size:
                self.get_logger().warn(
                    f'input_size param ({self.input_size}) != modèle ({saved_size})')
                self.input_size = saved_size

        self.get_logger().info(f'LaneNet chargé : {model_filename}')
        self.get_logger().info(
            f'  val_loss: {checkpoint.get("val_loss", "?")}'
            f'  input_size: {self.input_size}')

        # ── Normalisation ImageNet ──
        self.mean = torch.tensor([0.485, 0.456, 0.406],
                                 device=self.device).view(1, 3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225],
                                device=self.device).view(1, 3, 1, 1)

        # ── Subscribers ──
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # ── Publishers ──
        self.lane_fits_pub = self.create_publisher(
            Float32MultiArray, '/lane_detection/lane_fits', 10)

        debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.debug_image_pub = self.create_publisher(
            Image, '/lane_detection/debug_image', debug_qos)

        # ── Bridge ──
        self.use_cv_bridge = bool(self.get_parameter('use_cv_bridge').value)
        self.bridge = None
        if self.use_cv_bridge:
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
            except ImportError:
                self.use_cv_bridge = False

        self._frame_count = 0
        self._valid_count = 0
        self._total_ms = 0
        self._size_warned = False

        self.get_logger().info(f'LaneNet Detector prêt — écoute {image_topic}')

    # ================================================================
    #  Prétraitement
    # ================================================================
    def preprocess(self, img_bgr):
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (self.input_size, self.input_size))
        t = torch.from_numpy(img_resized).permute(2, 0, 1).unsqueeze(0)
        t = t.float().to(self.device) / 255.0
        return (t - self.mean) / self.std

    # ================================================================
    #  Inférence
    # ================================================================
    @torch.no_grad()
    def infer(self, img_bgr):
        input_t = self.preprocess(img_bgr)
        t0 = time.monotonic()
        output = self.model(input_t)
        infer_ms = (time.monotonic() - t0) * 1000
        coords_norm = output[0].cpu().numpy()
        coords_px = coords_norm.copy()
        coords_px[0::2] *= self.img_w
        coords_px[1::2] *= self.img_h
        return coords_px, infer_ms

    # ================================================================
    #  Callback
    # ================================================================
    def image_callback(self, msg):
        try:
            cv_image = self._msg_to_bgr8(msg)
            self._frame_count += 1

            h, w = cv_image.shape[:2]
            if w != self.img_w or h != self.img_h:
                if not self._size_warned:
                    self.get_logger().warn(
                        f'Résolution {w}x{h} != {self.img_w}x{self.img_h} — resize')
                    self._size_warned = True
                cv_image = cv2.resize(cv_image, (self.img_w, self.img_h))

            coords_px, infer_ms = self.infer(cv_image)
            self._total_ms += infer_ms

            left_pts = coords_px[:8].reshape(4, 2)
            right_pts = coords_px[8:].reshape(4, 2)

            valid = np.mean(left_pts[:, 0]) < np.mean(right_pts[:, 0])
            mode = 'paire' if valid else 'aucune'
            if valid:
                self._valid_count += 1

            if self._frame_count % 100 == 0:
                rate = (self._valid_count / self._frame_count) * 100
                avg = self._total_ms / self._frame_count
                self.get_logger().info(
                    f'Frame {self._frame_count} | OK:{rate:.0f}% | '
                    f'{avg:.1f}ms | {mode}')

            self._publish(left_pts, right_pts, mode)

            if self.get_parameter('publish_debug_image').value:
                debug = self._draw_debug(
                    cv_image, left_pts, right_pts, mode, infer_ms)
                dmsg = self._bgr8_to_msg(debug, msg.header.frame_id)
                dmsg.header.stamp = msg.header.stamp
                self.debug_image_pub.publish(dmsg)

        except Exception as e:
            self.get_logger().error(f'Erreur frame {self._frame_count}: {e}')

    # ================================================================
    #  Publication — même format que l'ancien node
    # ================================================================
    def _publish(self, left_pts, right_pts, mode):
        msg = Float32MultiArray()
        mode_int = {'paire': 2.0, 'dominante': 1.0, 'aucune': 0.0}.get(mode, 0.0)
        valid = 1.0 if mode != 'aucune' else 0.0
        msg.data = ([valid, mode_int]
                    + [float(v) for v in left_pts.flatten()]
                    + [float(v) for v in right_pts.flatten()])
        self.lane_fits_pub.publish(msg)

    # ================================================================
    #  Debug overlay
    # ================================================================
    def _draw_debug(self, img, left_pts, right_pts, mode, ms):
        out = img.copy()
        H, W = out.shape[:2]
        for pts, color, label in [
            (left_pts, (0, 255, 100), 'G'),
            (right_pts, (255, 200, 0), 'D'),
        ]:
            for i, (px, py) in enumerate(pts):
                px, py = int(px), int(py)
                cv2.circle(out, (px, py), 6, color, -1)
                cv2.circle(out, (px, py), 6, (255, 255, 255), 1)
                cv2.putText(out, f'{label}{i+1}', (px+8, py-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            for i in range(len(pts)-1):
                cv2.line(out, (int(pts[i][0]), int(pts[i][1])),
                         (int(pts[i+1][0]), int(pts[i+1][1])), color, 2)
        cx = int((left_pts[0, 0] + right_pts[0, 0]) / 2)
        cy = int((left_pts[0, 1] + right_pts[0, 1]) / 2)
        cv2.circle(out, (cx, cy), 10, (0, 165, 255), -1)
        mc = (0, 255, 100) if mode == 'paire' else (0, 130, 255)
        cv2.rectangle(out, (0, 0), (W, 50), (20, 20, 20), -1)
        cv2.putText(out, f'LaneNet | {ms:.1f}ms | {mode.upper()}',
                    (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mc, 1)
        return out

    # ================================================================
    #  Conversion images ROS ↔ OpenCV
    # ================================================================
    def _msg_to_bgr8(self, msg):
        if self.use_cv_bridge and self.bridge:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        enc = msg.encoding.lower()
        ch = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}.get(enc)
        if ch is None:
            raise ValueError(f'Encodage non supporté: {msg.encoding}')
        data = np.frombuffer(msg.data, dtype=np.uint8)
        rs = msg.width * ch
        if msg.step == rs:
            img = data.reshape((msg.height, msg.width, ch))
        else:
            img = data.reshape((msg.height, msg.step))[:, :rs]
            img = img.reshape((msg.height, msg.width, ch))
        cvt = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
               'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
        return cv2.cvtColor(img, cvt[enc]) if enc in cvt else img

    def _bgr8_to_msg(self, image, frame_id=''):
        if self.use_cv_bridge and self.bridge:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.frame_id = frame_id
            return msg
        msg = Image()
        msg.height, msg.width = image.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = image.shape[1] * 3
        msg.data = image.tobytes()
        msg.header.frame_id = frame_id
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorCNNNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# """
# Lane Detector Node — Version CNN (LaneNet)

# RÔLE : Remplace le pipeline OpenCV par une inférence CNN (ONNX Runtime).
#        Même interface ROS2, même format de publication.

#     /camera/image_raw ──> [LaneNet CNN] ──> /lane_detection/lane_fits
#                                          ──> /lane_detection/debug_image

# Publie (Float32MultiArray) :
#     [valid, mode_int,
#      lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,   <- ligne gauche (bas -> haut)
#      rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]    <- ligne droite (bas -> haut)

# Installation sur le robot :
#     pip install onnxruntime   (ou onnxruntime-gpu si CUDA dispo)
#     Copier lanenet.onnx dans le package ROS2
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray
# import cv2
# import numpy as np
# import time


# class LaneDetectorCNNNode(Node):

#     def __init__(self):
#         super().__init__('lane_detector_node')

#         # ── Paramètres ──
#         self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
#         self.declare_parameter('image_width', 640)
#         self.declare_parameter('image_height', 360)
#         self.declare_parameter('model_filename', 'lanenet.onnx')
#         self.declare_parameter('input_size', 224)
#         self.declare_parameter('confidence_threshold', 0.1)  # erreur max pour valider
#         self.declare_parameter('publish_debug_image', True)
#         self.declare_parameter('use_cv_bridge', True)
#         self.declare_parameter('use_gpu', True)  # True si onnxruntime-gpu installé

#         self.img_w = self.get_parameter('image_width').value
#         self.img_h = self.get_parameter('image_height').value
#         self.input_size = self.get_parameter('input_size').value
#         model_filename = self.get_parameter('model_filename').value
#         use_gpu = self.get_parameter('use_gpu').value

#         # ── Charger le modèle ONNX ──
#         try:
#             import onnxruntime as ort
#             providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if use_gpu \
#                 else ['CPUExecutionProvider']
#             self.session = ort.InferenceSession(model_filename, providers=providers)
#             self.input_name = self.session.get_inputs()[0].name
#             actual_provider = self.session.get_providers()[0]
#             self.get_logger().info(f'LaneNet ONNX chargé : {model_filename}')
#             self.get_logger().info(f'Provider : {actual_provider}')
#         except Exception as e:
#             self.get_logger().error(f'Impossible de charger le modèle ONNX : {e}')
#             raise

#         # ── Normalisation ImageNet ──
#         self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
#         self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

#         # ── Subscribers ──
#         image_topic = self.get_parameter('image_topic').value
#         self.image_sub = self.create_subscription(
#             Image, image_topic, self.image_callback, 10)

#         # ── Publishers ──
#         self.lane_fits_pub = self.create_publisher(
#             Float32MultiArray, '/lane_detection/lane_fits', 10)

#         debug_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST, depth=1)
#         self.debug_image_pub = self.create_publisher(
#             Image, '/lane_detection/debug_image', debug_qos)

#         # ── Bridge ──
#         self.use_cv_bridge = bool(self.get_parameter('use_cv_bridge').value)
#         self.bridge = None
#         if self.use_cv_bridge:
#             try:
#                 from cv_bridge import CvBridge
#                 self.bridge = CvBridge()
#             except ImportError:
#                 self.use_cv_bridge = False

#         # ── Stats ──
#         self._frame_count = 0
#         self._valid_count = 0
#         self._total_infer_ms = 0

#         self.get_logger().info(f'LaneNet CNN Detector prêt — écoute {image_topic}')

#     # ================================================================
#     #  Prétraitement pour le CNN
#     # ================================================================
#     def preprocess(self, img_bgr):
#         """
#         BGR (640×360) → RGB (224×224) normalisé → NCHW float32
#         """
#         img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
#         img_resized = cv2.resize(img_rgb, (self.input_size, self.input_size))
#         img_float = img_resized.astype(np.float32) / 255.0

#         # Normalisation ImageNet
#         img_float = (img_float - self.mean) / self.std

#         # HWC → NCHW
#         img_nchw = np.transpose(img_float, (2, 0, 1))[np.newaxis, ...]
#         return img_nchw.astype(np.float32)

#     # ================================================================
#     #  Inférence
#     # ================================================================
#     def infer(self, img_bgr):
#         """
#         Exécute le CNN et retourne les 16 coordonnées en pixels.

#         Retourne :
#             coords_px : np.array (16,) en pixels image originale
#             infer_ms  : temps d'inférence en ms
#         """
#         input_data = self.preprocess(img_bgr)

#         t0 = time.monotonic()
#         outputs = self.session.run(None, {self.input_name: input_data})
#         infer_ms = (time.monotonic() - t0) * 1000

#         coords_norm = outputs[0][0]  # (16,) normalisé [0, 1]

#         # Dénormaliser vers pixels
#         coords_px = coords_norm.copy()
#         coords_px[0::2] *= self.img_w   # x
#         coords_px[1::2] *= self.img_h   # y

#         return coords_px, infer_ms

#     # ================================================================
#     #  Callback principal
#     # ================================================================
#     def image_callback(self, msg):
#         try:
#             cv_image = self.image_msg_to_bgr8(msg)
#             self._frame_count += 1

#             # Resize si nécessaire
#             h, w = cv_image.shape[:2]
#             if w != self.img_w or h != self.img_h:
#                 cv_image = cv2.resize(cv_image, (self.img_w, self.img_h))

#             # ── Inférence CNN ──
#             coords_px, infer_ms = self.infer(cv_image)
#             self._total_infer_ms += infer_ms

#             # Extraire les points
#             left_pts = coords_px[:8].reshape(4, 2)
#             right_pts = coords_px[8:].reshape(4, 2)

#             # Vérifier la cohérence (gauche.x < droite.x en moyenne)
#             valid = np.mean(left_pts[:, 0]) < np.mean(right_pts[:, 0])
#             mode = 'paire' if valid else 'aucune'

#             if valid:
#                 self._valid_count += 1

#             # ── Log périodique ──
#             if self._frame_count % 100 == 0:
#                 rate = (self._valid_count / self._frame_count) * 100
#                 avg_ms = self._total_infer_ms / self._frame_count
#                 self.get_logger().info(
#                     f'Frame {self._frame_count} | '
#                     f'Détection OK: {rate:.0f}% | '
#                     f'Inférence: {avg_ms:.1f}ms | '
#                     f'Mode: {mode}'
#                 )

#             # ── Publication ──
#             self.publish_lane_points(left_pts, right_pts, mode)

#             if self.get_parameter('publish_debug_image').value:
#                 debug_img = self.draw_debug(cv_image, left_pts, right_pts, mode, infer_ms)
#                 debug_msg = self.bgr8_to_image_msg(debug_img, msg.header.frame_id)
#                 debug_msg.header.stamp = msg.header.stamp
#                 self.debug_image_pub.publish(debug_msg)

#         except Exception as e:
#             self.get_logger().error(f'Erreur frame {self._frame_count}: {str(e)}')

#     # ================================================================
#     #  Publication (même format que l'ancien node)
#     # ================================================================
#     def publish_lane_points(self, left_pts, right_pts, mode):
#         msg = Float32MultiArray()
#         mode_int = {'paire': 2.0, 'dominante': 1.0, 'aucune': 0.0}.get(mode, 0.0)
#         valid = 1.0 if mode != 'aucune' else 0.0

#         left_flat = [float(v) for v in left_pts.flatten()]
#         right_flat = [float(v) for v in right_pts.flatten()]
#         msg.data = [valid, mode_int] + left_flat + right_flat
#         self.lane_fits_pub.publish(msg)

#     # ================================================================
#     #  Visualisation debug
#     # ================================================================
#     def draw_debug(self, img, left_pts, right_pts, mode, infer_ms):
#         out = img.copy()
#         H, W = out.shape[:2]

#         # Points gauche (cyan) et droite (jaune)
#         for pts, color, label in [
#             (left_pts, (0, 255, 100), 'G'),
#             (right_pts, (255, 200, 0), 'D'),
#         ]:
#             for i, (px, py) in enumerate(pts):
#                 px, py = int(px), int(py)
#                 cv2.circle(out, (px, py), 6, color, -1)
#                 cv2.circle(out, (px, py), 6, (255, 255, 255), 1)
#                 cv2.putText(out, f'{label}{i+1}', (px + 8, py - 4),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
#             for i in range(len(pts) - 1):
#                 p1 = (int(pts[i][0]), int(pts[i][1]))
#                 p2 = (int(pts[i+1][0]), int(pts[i+1][1]))
#                 cv2.line(out, p1, p2, color, 2)

#         # Centre du couloir
#         cx = int((left_pts[0, 0] + right_pts[0, 0]) / 2)
#         cy = int((left_pts[0, 1] + right_pts[0, 1]) / 2)
#         cv2.circle(out, (cx, cy), 10, (0, 165, 255), -1)

#         # Banner
#         mode_color = (0, 255, 100) if mode == 'paire' else (0, 130, 255)
#         cv2.rectangle(out, (0, 0), (W, 50), (20, 20, 20), -1)
#         cv2.putText(out,
#                     f'LaneNet CNN | {infer_ms:.1f}ms | MODE: {mode.upper()}',
#                     (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 1)

#         return out

#     # ================================================================
#     #  Conversion images ROS <-> OpenCV
#     # ================================================================
#     def image_msg_to_bgr8(self, msg):
#         if self.use_cv_bridge and self.bridge is not None:
#             return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         encoding = msg.encoding.lower()
#         if encoding in ('bgr8', 'rgb8'):
#             channels = 3
#         elif encoding in ('bgra8', 'rgba8'):
#             channels = 4
#         elif encoding == 'mono8':
#             channels = 1
#         else:
#             raise ValueError(f'Encodage non supporté: {msg.encoding}')

#         data = np.frombuffer(msg.data, dtype=np.uint8)
#         row_size = msg.width * channels
#         if msg.step == row_size:
#             img = data.reshape((msg.height, msg.width, channels))
#         else:
#             img = data.reshape((msg.height, msg.step))[:, :row_size]
#             img = img.reshape((msg.height, msg.width, channels))

#         if encoding == 'rgb8':   return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
#         if encoding == 'rgba8':  return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
#         if encoding == 'bgra8':  return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
#         if encoding == 'mono8':  return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
#         return img

#     def bgr8_to_image_msg(self, image, frame_id=''):
#         if self.use_cv_bridge and self.bridge is not None:
#             msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
#             msg.header.frame_id = frame_id
#             return msg

#         msg = Image()
#         msg.height = image.shape[0]
#         msg.width = image.shape[1]
#         msg.encoding = 'bgr8'
#         msg.is_bigendian = 0
#         msg.step = image.shape[1] * 3
#         msg.data = image.tobytes()
#         msg.header.frame_id = frame_id
#         return msg


# def main(args=None):
#     rclpy.init(args=args)
#     node = LaneDetectorCNNNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

