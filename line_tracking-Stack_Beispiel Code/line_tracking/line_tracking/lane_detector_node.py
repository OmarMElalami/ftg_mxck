#!/usr/bin/env python3
"""
Lane Detector Node — Detection des lignes de voie

ROLE : Lit les images camera -> detecte les 2 lignes blanches -> publie 4 points par ligne

    /camera/image_raw ──> [Lane Detector] ──> /lane_detection/lane_fits
                                           ──> /lane_detection/debug_image

Publie (Float32MultiArray) :
    [valid, mode_int,
     lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,   <- ligne gauche (bas -> haut)
     rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]    <- ligne droite (bas -> haut)

    valid    = 1.0 OK | 0.0 echec
    mode_int = 2.0 paire | 1.0 dominante | 0.0 aucune
    Points en pixels, coordonnees image complete
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

from .utils.image_processing import ImageProcessor


class LaneDetectorNode(Node):

    def __init__(self):
        super().__init__('lane_detector_node')

        # ── Parametres ──
        required_params = [
            'image_topic',
            'image_width',
            'image_height',
            'hsv_lower',
            'hsv_upper',
            'roi_top_ratio',
            'roi_bottom_cut_ratio',
            'lane_width_min_px',
            'lane_width_max_px',
            'min_blob_area',
            'publish_debug_image',
            'use_cv_bridge',
        ]
        for name in required_params:
            self.declare_parameter(name)

        # ── Image Processor ──
        img_w = self._require_param('image_width')
        img_h = self._require_param('image_height')
        self.processor = ImageProcessor(img_w, img_h, logger=self.get_logger())

        self.processor.roi_top_ratio        = float(self._require_param('roi_top_ratio'))
        self.processor.roi_bottom_cut_ratio = float(self._require_param('roi_bottom_cut_ratio'))
        self.processor.lane_width_min_px    = float(self._require_param('lane_width_min_px'))
        self.processor.lane_width_max_px    = float(self._require_param('lane_width_max_px'))
        self.processor.min_blob_area        = int(self._require_param('min_blob_area'))
        self.processor.hsv_lower = np.array(self._require_param('hsv_lower'), dtype=np.uint8)
        self.processor.hsv_upper = np.array(self._require_param('hsv_upper'), dtype=np.uint8)

        self.get_logger().info(
            f'Pipeline: Gauss+CLAHE+HSV+Contours | '
            f'ROI top={self.processor.roi_top_ratio} bot={self.processor.roi_bottom_cut_ratio} | '
            f'Lane [{self.processor.lane_width_min_px}, {self.processor.lane_width_max_px}]px'
        )

        # ── Subscribers ──
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.info_callback, 10)

        image_topic = self._require_param('image_topic')
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # ── Publishers ──
        self.lane_fits_pub = self.create_publisher(
            Float32MultiArray, '/lane_detection/lane_fits', 10)

        debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.debug_image_pub = self.create_publisher(
            Image, '/lane_detection/debug_image', debug_qos)

        # ── Bridge ──
        self.use_cv_bridge = bool(self._require_param('use_cv_bridge'))
        self.bridge = None
        if self.use_cv_bridge:
            try:
                from cv_bridge import CvBridge
                self.bridge = CvBridge()
                self.get_logger().info('Conversion image: cv_bridge')
            except ImportError:
                self.use_cv_bridge = False
                self.get_logger().warn('cv_bridge indisponible, conversion NumPy activee.')

        self.get_logger().info(f'Lane Detector pret — ecoute {image_topic}')
        self.get_logger().info(
            'Publie : /lane_detection/lane_fits  '
            '[valid, mode, lx1,ly1,...,lx4,ly4, rx1,ry1,...,rx4,ry4]'
        )

        self._frame_count = 0
        self._valid_count = 0
        self._size_warned = False

    def _require_param(self, name):
        param = self.get_parameter(name)
        if param is None or param.value is None:
            raise ValueError(f"Parametre '{name}' absent dans config/params.yaml")
        return param.value

    # ================================================================
    #  Calibration camera
    # ================================================================
    def info_callback(self, msg):
        if self.processor.camera_matrix is None:
            self.processor.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.processor.dist_coeffs   = np.array(msg.d)
            k = self.processor.camera_matrix
            d = self.processor.dist_coeffs
            self.get_logger().info('=' * 50)
            self.get_logger().info('[CAMERA_INFO] Calibration recue')
            self.get_logger().info(f'  Resolution : {msg.width}x{msg.height}')
            self.get_logger().info(f'  focal_x={k[0,0]:.2f}  focal_y={k[1,1]:.2f}')
            self.get_logger().info(f'  center_x={k[0,2]:.2f}  center_y={k[1,2]:.2f}')
            self.get_logger().info(f'  Distorsion : {d.tolist()}')
            if np.all(d == 0):
                self.get_logger().warn('  Coefficients de distorsion nuls.')
            self.get_logger().info('=' * 50)

    # ================================================================
    #  Callback principal
    # ================================================================
    def image_callback(self, msg):
        try:
            cv_image = self.image_msg_to_bgr8(msg)
            self._frame_count += 1

            expected_w = int(self.get_parameter('image_width').value)
            expected_h = int(self.get_parameter('image_height').value)
            if not self._size_warned:
                h, w = cv_image.shape[:2]
                if w != expected_w or h != expected_h:
                    self.get_logger().warn(
                        f'Resolution camera {w}x{h} != params {expected_w}x{expected_h}'
                        f' -> redimensionnement automatique'
                    )
                self._size_warned = True
            h, w = cv_image.shape[:2]
            if w != expected_w or h != expected_h:
                cv_image = cv2.resize(cv_image, (expected_w, expected_h))

            # ── Pipeline detection ──
            left_pts, right_pts, mode, cx_lane, debug_img = \
                self.processor.detect_lanes(cv_image)

            valid = mode in ('paire', 'dominante')
            if valid:
                self._valid_count += 1

            # Log periodique
            if self._frame_count % 100 == 0:
                rate = (self._valid_count / self._frame_count) * 100
                self.get_logger().info(
                    f'Frame {self._frame_count} | '
                    f'Detection OK: {rate:.0f}% | '
                    f'Mode: {mode} | cx={cx_lane}px'
                )

            # ── Publication ──
            self.publish_lane_points(left_pts, right_pts, mode)

            if self.get_parameter('publish_debug_image').value:
                debug_msg = self.bgr8_to_image_msg(debug_img, msg.header.frame_id)
                debug_msg.header.stamp = msg.header.stamp
                self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Erreur frame {self._frame_count}: {str(e)}')

    # ================================================================
    #  Publication
    # ================================================================
    def publish_lane_points(self, left_pts, right_pts, mode):
        """
        Publie les 4 points par ligne.
        Format : [valid, mode_int,
                  lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,
                  rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]

        Points ordonnes bas -> haut (proche robot -> loin robot).
        """
        msg = Float32MultiArray()

        mode_int = {'paire': 2.0, 'dominante': 1.0, 'aucune': 0.0}.get(mode, 0.0)

        if mode == 'aucune' or left_pts is None or right_pts is None:
            msg.data = [float(0.0), float(mode_int)] + [float(0.0)] * 16
            self.lane_fits_pub.publish(msg)
            return

        # Aplatir les points et forcer float Python (ROS2 refuse int/np.int32)
        left_flat  = [float(v) for v in left_pts.flatten()]
        right_flat = [float(v) for v in right_pts.flatten()]

        msg.data = [1.0, mode_int] + left_flat + right_flat
        self.lane_fits_pub.publish(msg)

    # ================================================================
    #  Conversion images ROS <-> OpenCV
    # ================================================================
    def image_msg_to_bgr8(self, msg):
        if self.use_cv_bridge and self.bridge is not None:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        encoding = msg.encoding.lower()
        if encoding in ('bgr8', 'rgb8'):
            channels = 3
        elif encoding in ('bgra8', 'rgba8'):
            channels = 4
        elif encoding == 'mono8':
            channels = 1
        else:
            raise ValueError(f'Encodage non supporte: {msg.encoding}')

        data     = np.frombuffer(msg.data, dtype=np.uint8)
        row_size = msg.width * channels
        if msg.step == row_size:
            img = data.reshape((msg.height, msg.width, channels))
        else:
            img = data.reshape((msg.height, msg.step))[:, :row_size]
            img = img.reshape((msg.height, msg.width, channels))

        if encoding == 'rgb8':   return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':  return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':  return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if encoding == 'mono8':  return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    def bgr8_to_image_msg(self, image, frame_id=''):
        if self.use_cv_bridge and self.bridge is not None:
            msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.frame_id = frame_id
            return msg

        msg = Image()
        msg.height       = image.shape[0]
        msg.width        = image.shape[1]
        msg.encoding     = 'bgr8'
        msg.is_bigendian = 0
        msg.step         = image.shape[1] * 3
        msg.data         = image.tobytes()
        msg.header.frame_id = frame_id
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
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
# Lane Detector Node — Détection des lignes de voie

# RÔLE : Lit les images caméra → détecte les 2 lignes blanches → publie leurs polynômes

#     /camera/image_raw ──→ [Lane Detector] ──→ /lane_detection/lane_fits
#                                            ──→ /lane_detection/debug_image
#                                            ──→ /lane_detection/debug_birdseye

# Ce node ne calcule PAS la midline ni le point cible.
# C'est le Controller Node qui fait ça.

# Publie (Float32MultiArray) :
#     [valid, left_a, left_b, left_c, right_a, right_b, right_c]
    
#     valid = 1.0 si détection OK, 0.0 sinon
#     x = a*y² + b*y + c  (polynôme en espace bird's eye pixels)
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import Image, CameraInfo
# from std_msgs.msg import Float32MultiArray
# import cv2
# import numpy as np

# from .utils.image_processing import ImageProcessor


# class LaneDetectorNode(Node):

#     def __init__(self):
#         super().__init__('lane_detector_node')

#         # ── Paramètres (obligatoires, définis dans config/params.yaml) ──
#         required_params = [
#             'image_topic',
#             'image_width',
#             'image_height',
#             'hsv_lower',
#             'hsv_upper',
#             'preprocess_mode',      # 'hsv_canny' | 'canny_only' | 'hsv_only'
#             'canny_low',            # seuil bas Canny
#             'canny_high',           # seuil haut Canny
#             'n_windows',
#             'window_margin',
#             'min_pixels',
#             'roi_top_ratio',
#             'roi_bottom_cut_ratio',
#             'roi_side_cut_ratio',
#             'min_component_area',
#             'allow_single_line_fallback',
#             'enable_dynamic_warp',
#             'dynamic_warp_period',
#             'lane_width_tolerance_ratio',
#             'min_lane_width_ratio',
#             'max_lane_width_ratio',
#             'expected_lane_width_ratio',
#             'warp_src',
#             'warp_dst',
#             'publish_debug_image',
#             'publish_birdseye',
#             'use_cv_bridge',
#         ]
#         for name in required_params:
#             self.declare_parameter(name)

#         # ── Image Processor (utils) ──
#         img_w = self._require_param('image_width')
#         img_h = self._require_param('image_height')
#         self.processor = ImageProcessor(img_w, img_h, logger=self.get_logger())

#         self.processor.n_windows = self._require_param('n_windows')
#         self.processor.window_margin = self._require_param('window_margin')
#         self.processor.min_pixels = self._require_param('min_pixels')
#         self.processor.roi_top_ratio = float(self._require_param('roi_top_ratio'))
#         self.processor.roi_bottom_cut_ratio = float(
#             self._require_param('roi_bottom_cut_ratio')
#         )
#         self.processor.roi_side_cut_ratio = float(
#             self._require_param('roi_side_cut_ratio')
#         )
#         self.processor.min_component_area = int(
#             self._require_param('min_component_area')
#         )
#         self.processor.allow_single_line_fallback = bool(
#             self._require_param('allow_single_line_fallback')
#         )
#         self.processor.enable_dynamic_warp = bool(
#             self._require_param('enable_dynamic_warp')
#         )
#         self.processor.dynamic_warp_period = int(
#             self._require_param('dynamic_warp_period')
#         )
#         self.processor.lane_width_tolerance_ratio = float(
#             self._require_param('lane_width_tolerance_ratio')
#         )
#         self.processor.min_lane_width_px = float(
#             self._require_param('min_lane_width_ratio')
#         ) * img_w
#         self.processor.max_lane_width_px = float(
#             self._require_param('max_lane_width_ratio')
#         ) * img_w
#         self.processor.expected_lane_width_px = float(
#             self._require_param('expected_lane_width_ratio')
#         ) * img_w

#         # ── Prétraitement : mode + seuils Canny ──
#         self.processor.preprocess_mode = str(self._require_param('preprocess_mode'))
#         self.processor.canny_low       = int(self._require_param('canny_low'))
#         self.processor.canny_high      = int(self._require_param('canny_high'))
#         self.get_logger().info(
#             f'Prétraitement : mode={self.processor.preprocess_mode} | '
#             f'Canny=[{self.processor.canny_low}, {self.processor.canny_high}]'
#         )

#         # Calibration perspective si fournie
#         warp_src = self._require_param('warp_src')
#         warp_dst = self._require_param('warp_dst')
#         if len(warp_src) == 8 and len(warp_dst) == 8:
#             src = [warp_src[i:i+2] for i in range(0, 8, 2)]
#             dst = [warp_dst[i:i+2] for i in range(0, 8, 2)]
#             self.processor.set_warp_points(src, dst)
#             self.get_logger().info('Perspective warp calibrée depuis params')

#         # Subscriber : On écoute camera_info pour avoir la calibration intrinsèque
#         self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
#             self.info_callback, 10)
        
#         # ── Subscriber : images caméra ──
#         image_topic = self._require_param('image_topic')
#         self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

#         # ── Publishers ──
#         # Polynômes des lignes → pour le Controller Node
#         self.lane_fits_pub = self.create_publisher(Float32MultiArray, '/lane_detection/lane_fits', 10)

#         # Images debug → pour Foxglove / enregistrement
#         self.debug_image_pub = self.create_publisher(Image, '/lane_detection/debug_image', 10)
#         self.birdseye_pub = self.create_publisher(Image, '/lane_detection/debug_birdseye', 10)

#         # Bridge Image
#         self.use_cv_bridge = bool(self._require_param('use_cv_bridge'))
#         self.bridge = None
#         if self.use_cv_bridge:
#             try:
#                 from cv_bridge import CvBridge
#                 self.bridge = CvBridge()
#             except ImportError:
#                 self.get_logger().warn("CvBridge non trouvé, mode NumPy activé.")
#                 self.use_cv_bridge = False

#         self.get_logger().info(f'Lane Detector prêt — écoute {image_topic}')
#         self.get_logger().info('Publie : /lane_detection/lane_fits')

#         # Compteur de frames pour log périodique
#         self._frame_count = 0
#         self._valid_count = 0
#         self._size_warned = False

#     def _require_param(self, name):
#         """Lit un paramètre obligatoire (défini dans config/params.yaml)."""
#         param = self.get_parameter(name)
#         if param is None or param.value is None:
#             raise ValueError(
#                 f"Paramètre '{name}' absent : ajoute-le dans config/params.yaml"
#             )
#         return param.value
    
#     def info_callback(self, msg):
#         """Récupère la calibration de la caméra une seule fois au démarrage
#         et affiche les paramètres lus pour validation technique."""
#         if self.processor.camera_matrix is None:
#             # Conversion des données ROS (listes) en matrices NumPy pour OpenCV
#             self.processor.camera_matrix = np.array(msg.k).reshape((3, 3))
#             self.processor.dist_coeffs = np.array(msg.d)
            
#             # --- LOG TECHNIQUE DES PARAMÈTRES ---
#             self.get_logger().info('='*50)
#             self.get_logger().info(' [CAMERA_INFO] : CALIBRATION MATÉRIELLE DÉTECTÉE')
#             self.get_logger().info(f' RÉSOLUTION : {msg.width}x{msg.height}')
#             self.get_logger().info(f' MODÈLE DE DISTORSION : {msg.distortion_model}')
            
#             # Affichage de la matrice intrinsèque K
#             # fx/fy = focale, cx/cy = centre optique
#             k = self.processor.camera_matrix
#             self.get_logger().info(f' MATRICE K (Intrinsèque) :')
#             self.get_logger().info(f'   focal_x: {k[0,0]:.2f}, focal_y: {k[1,1]:.2f}')
#             self.get_logger().info(f'   center_x: {k[0,2]:.2f}, center_y: {k[1,2]:.2f}')
            
#             # Affichage des coefficients de distorsion D (k1, k2, p1, p2, k3)
#             d = self.processor.dist_coeffs
#             self.get_logger().info(f' COEFFS D (Distorsion) : {d.tolist()}')
            
#             # Alerte si les données semblent vides
#             if np.all(d == 0):
#                 self.get_logger().warn(" ATTENTION : Coefficients de distorsion nuls. L'image ne sera pas redressée.")
            
#             self.get_logger().info("Calibration reçue et appliquée à l'ImageProcessor")
#             self.get_logger().info('='*50)

#     def image_callback(self, msg):
#         """Reçoit une image, détecte les lignes, publie les résultats."""
#         try:
#             cv_image = self.image_msg_to_bgr8(msg)
#             self._frame_count += 1

#             if not self._size_warned:
#                 expected_w = int(self.get_parameter('image_width').value)
#                 expected_h = int(self.get_parameter('image_height').value)
#                 actual_h, actual_w = cv_image.shape[:2]
#                 if actual_w != expected_w or actual_h != expected_h:
#                     self.get_logger().warn(
#                         f'Résolution caméra {actual_w}x{actual_h} ≠ params {expected_w}x{expected_h} '
#                         f'→ redimensionnement automatique activé'
#                     )
#                 self._size_warned = True

#             # Redimensionner si nécessaire (CRITIQUE pour le warp)
#             expected_w = int(self.get_parameter('image_width').value)
#             expected_h = int(self.get_parameter('image_height').value)
#             actual_h, actual_w = cv_image.shape[:2]
#             if actual_w != expected_w or actual_h != expected_h:
#                 cv_image = cv2.resize(cv_image, (expected_w, expected_h))

#             hsv_lower = self.get_parameter('hsv_lower').value
#             hsv_upper = self.get_parameter('hsv_upper').value

#             # ── Pipeline de détection ──
#             # 1. Pré-traitement (HSV + masque)
#             binary = self.processor.preprocess(cv_image, hsv_lower, hsv_upper)

#             # 2. Bird's eye view
#             binary_warped = self.processor.warp_to_birdseye(binary)

#             # 3. Sliding window
#             left_x, left_y, right_x, right_y, window_img = \
#                 self.processor.find_lane_pixels(binary_warped)

#             # 4. Fit polynômes
#             left_fit, right_fit, valid = \
#                 self.processor.fit_polynomials(left_x, left_y, right_x, right_y)

#             if valid:
#                 self._valid_count += 1

#             # ── Log périodique (toutes les 100 frames ≈ 3.3s à 30fps) ──
#             if self._frame_count % 100 == 0:
#                 rate = (self._valid_count / self._frame_count) * 100
#                 self.get_logger().info(
#                     f'Frame {self._frame_count} | '
#                     f'Détection OK: {rate:.0f}% | '
#                     f'Pixels G={len(left_x)} D={len(right_x)}'
#                 )

#             # ── Publier les polynômes ──
#             self.publish_lane_fits(left_fit, right_fit, valid)

#             # ── Publier images debug ──
#             if self.get_parameter('publish_debug_image').value:
#                 overlay = self.processor.draw_lane_overlay(cv_image, left_fit, right_fit)
#                 debug_msg = self.bgr8_to_image_msg(overlay, msg.header.frame_id)
#                 debug_msg.header.stamp = msg.header.stamp
#                 self.debug_image_pub.publish(debug_msg)

#             if self.get_parameter('publish_birdseye').value:
#                 birdseye = self.processor.draw_birdseye_debug(
#                     binary_warped, window_img, left_fit, right_fit)
#                 be_msg = self.bgr8_to_image_msg(birdseye, msg.header.frame_id)
#                 be_msg.header.stamp = msg.header.stamp
#                 self.birdseye_pub.publish(be_msg)

#         except Exception as e:
#             self.get_logger().error(f'Erreur frame {self._frame_count}: {str(e)}')

#     def image_msg_to_bgr8(self, msg):
#         """Convertit sensor_msgs/Image vers OpenCV BGR."""
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

#         if encoding == 'rgb8':
#             return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
#         if encoding == 'rgba8':
#             return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
#         if encoding == 'bgra8':
#             return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
#         if encoding == 'mono8':
#             return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
#         return img

#     def bgr8_to_image_msg(self, image, frame_id=''):
#         """Convertit une image OpenCV BGR vers sensor_msgs/Image."""
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

#     def publish_lane_fits(self, left_fit, right_fit, valid):
#         """
#         Publie les coefficients des polynômes.
#         Format : [valid, left_a, left_b, left_c, right_a, right_b, right_c]
#         """
#         msg = Float32MultiArray()

#         if not valid or left_fit is None or right_fit is None:
#             msg.data = [0.0]
#         else:
#             msg.data = [
#                 1.0,                                    # valid
#                 float(left_fit[0]),  float(left_fit[1]),  float(left_fit[2]),   # gauche
#                 float(right_fit[0]), float(right_fit[1]), float(right_fit[2]),  # droite
#             ]
#             #self.get_logger(msg.data)
#         self.lane_fits_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LaneDetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# #!/usr/bin/env python3
# """
# Lane Detector Node — Détection des lignes de voie

# RÔLE : Lit les images caméra → détecte les 2 lignes blanches → publie leurs polynômes

#     /camera/image_raw ──→ [Lane Detector] ──→ /lane_detection/lane_fits
#                                            ──→ /lane_detection/debug_image
#                                            ──→ /lane_detection/debug_birdseye

# Ce node ne calcule PAS la midline ni le point cible.
# C'est le Controller Node qui fait ça.

# Publie (Float32MultiArray) :
#     [valid, left_a, left_b, left_c, right_a, right_b, right_c]
    
#     valid = 1.0 si détection OK, 0.0 sinon
#     x = a*y² + b*y + c  (polynôme en espace bird's eye pixels)
# """

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# from sensor_msgs.msg import Image, CameraInfo
# from std_msgs.msg import Float32MultiArray
# import cv2
# import numpy as np

# from .utils.image_processing import ImageProcessor


# class LaneDetectorNode(Node):

#     def __init__(self):
#         super().__init__('lane_detector_node')

#         # ── Paramètres (obligatoires, définis dans config/params.yaml) ──
#         required_params = [
#             'image_topic',
#             'image_width',
#             'image_height',
#             'hsv_lower',
#             'hsv_upper',
#             'n_windows',
#             'window_margin',
#             'min_pixels',
#             'roi_top_ratio',
#             'roi_bottom_cut_ratio',
#             'roi_side_cut_ratio',
#             'min_component_area',
#             'allow_single_line_fallback',
#             'enable_dynamic_warp',
#             'dynamic_warp_period',
#             'lane_width_tolerance_ratio',
#             'min_lane_width_ratio',
#             'max_lane_width_ratio',
#             'expected_lane_width_ratio',
#             'warp_src',
#             'warp_dst',
#             'publish_debug_image',
#             'publish_birdseye',
#             'use_cv_bridge',
#         ]
#         for name in required_params:
#             self.declare_parameter(name)

#         # ── Image Processor (utils) ──
#         img_w = self._require_param('image_width')
#         img_h = self._require_param('image_height')
#         self.processor = ImageProcessor(img_w, img_h, logger=self.get_logger())

#         self.processor.n_windows = self._require_param('n_windows')
#         self.processor.window_margin = self._require_param('window_margin')
#         self.processor.min_pixels = self._require_param('min_pixels')
#         self.processor.roi_top_ratio = float(self._require_param('roi_top_ratio'))
#         self.processor.roi_bottom_cut_ratio = float(
#             self._require_param('roi_bottom_cut_ratio')
#         )
#         self.processor.roi_side_cut_ratio = float(
#             self._require_param('roi_side_cut_ratio')
#         )
#         self.processor.min_component_area = int(
#             self._require_param('min_component_area')
#         )
#         self.processor.allow_single_line_fallback = bool(
#             self._require_param('allow_single_line_fallback')
#         )
#         self.processor.enable_dynamic_warp = bool(
#             self._require_param('enable_dynamic_warp')
#         )
#         self.processor.dynamic_warp_period = int(
#             self._require_param('dynamic_warp_period')
#         )
#         self.processor.lane_width_tolerance_ratio = float(
#             self._require_param('lane_width_tolerance_ratio')
#         )
#         self.processor.min_lane_width_px = float(
#             self._require_param('min_lane_width_ratio')
#         ) * img_w
#         self.processor.max_lane_width_px = float(
#             self._require_param('max_lane_width_ratio')
#         ) * img_w
#         self.processor.expected_lane_width_px = float(
#             self._require_param('expected_lane_width_ratio')
#         ) * img_w

#         # Calibration perspective si fournie
#         warp_src = self._require_param('warp_src')
#         warp_dst = self._require_param('warp_dst')
#         if len(warp_src) == 8 and len(warp_dst) == 8:
#             src = [warp_src[i:i+2] for i in range(0, 8, 2)]
#             dst = [warp_dst[i:i+2] for i in range(0, 8, 2)]
#             self.processor.set_warp_points(src, dst)
#             self.get_logger().info('Perspective warp calibrée depuis params')

#         # Subscriber : On écoute camera_info pour avoir la calibration intrinsèque
#         self.info_sub = self.create_subscription(
#             CameraInfo, '/camera/camera/color/camera_info',
#             self.info_callback, 10)
        
#         # ── Subscriber : images caméra ──
#         image_topic = self._require_param('image_topic')
#         self.image_sub = self.create_subscription(
#             Image, image_topic, self.image_callback, 10)

#         # ── Publishers ──
#         # Polynômes des lignes → pour le Controller Node
#         self.lane_fits_pub = self.create_publisher(
#             Float32MultiArray, '/lane_detection/lane_fits', 10)

#         # Images debug → pour Foxglove / enregistrement
#         # self.debug_image_pub = self.create_publisher(
#         #     Image, '/lane_detection/debug_image', 10)
#         # self.birdseye_pub = self.create_publisher(
#         #     Image, '/lane_detection/debug_birdseye', 10)

#         # QoS best_effort + keep_last(1) pour compatibilité Foxglove bridge
#         debug_qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#         self.debug_image_pub = self.create_publisher(
#             Image, '/lane_detection/debug_image', debug_qos)
#         self.birdseye_pub = self.create_publisher(
#             Image, '/lane_detection/debug_birdseye', debug_qos)

#         self.use_cv_bridge = bool(self._require_param('use_cv_bridge'))
#         self.bridge = None
#         if self.use_cv_bridge:
#             try:
#                 from cv_bridge import CvBridge  # import local pour éviter crash si cv_bridge cassé
#                 self.bridge = CvBridge()
#                 self.get_logger().info('Conversion image: cv_bridge')
#             except Exception as e:
#                 self.use_cv_bridge = False
#                 self.get_logger().warn(
#                     f'cv_bridge indisponible ({str(e)}), conversion NumPy activée.'
#                 )
#         else:
#             self.get_logger().info('Conversion image: NumPy (sans cv_bridge)')

#         self.get_logger().info(f'Lane Detector prêt — écoute {image_topic}')
#         self.get_logger().info('Publie : /lane_detection/lane_fits')

#         # Compteur de frames pour log périodique
#         self._frame_count = 0
#         self._valid_count = 0
#         self._size_warned = False

#     def _require_param(self, name):
#         """Lit un paramètre obligatoire (défini dans config/params.yaml)."""
#         param = self.get_parameter(name)
#         if param is None or param.value is None:
#             raise ValueError(
#                 f"Paramètre '{name}' absent : ajoute-le dans config/params.yaml"
#             )
#         return param.value
    
#     def info_callback(self, msg):
#         """Récupère la calibration de la caméra une seule fois au démarrage
#         et affiche les paramètres lus pour validation technique."""
#         if self.processor.camera_matrix is None:
#             # Conversion des données ROS (listes) en matrices NumPy pour OpenCV
#             self.processor.camera_matrix = np.array(msg.k).reshape((3, 3))
#             self.processor.dist_coeffs = np.array(msg.d)
            
#             # --- LOG TECHNIQUE DES PARAMÈTRES ---
#             self.get_logger().info('='*50)
#             self.get_logger().info(' [CAMERA_INFO] : CALIBRATION MATÉRIELLE DÉTECTÉE')
#             self.get_logger().info(f' RÉSOLUTION : {msg.width}x{msg.height}')
#             self.get_logger().info(f' MODÈLE DE DISTORSION : {msg.distortion_model}')
            
#             # Affichage de la matrice intrinsèque K
#             # fx/fy = focale, cx/cy = centre optique
#             k = self.processor.camera_matrix
#             self.get_logger().info(f' MATRICE K (Intrinsèque) :')
#             self.get_logger().info(f'   focal_x: {k[0,0]:.2f}, focal_y: {k[1,1]:.2f}')
#             self.get_logger().info(f'   center_x: {k[0,2]:.2f}, center_y: {k[1,2]:.2f}')
            
#             # Affichage des coefficients de distorsion D (k1, k2, p1, p2, k3)
#             d = self.processor.dist_coeffs
#             self.get_logger().info(f' COEFFS D (Distorsion) : {d.tolist()}')
            
#             # Alerte si les données semblent vides
#             if np.all(d == 0):
#                 self.get_logger().warn(" ATTENTION : Coefficients de distorsion nuls. L'image ne sera pas redressée.")
            
#             self.get_logger().info("Calibration reçue et appliquée à l'ImageProcessor")
#             self.get_logger().info('='*50)

#     def image_callback(self, msg):
#         """Reçoit une image, détecte les lignes, publie les résultats."""
#         try:
#             cv_image = self.image_msg_to_bgr8(msg)
#             self._frame_count += 1

#             if not self._size_warned:
#                 expected_w = int(self.get_parameter('image_width').value)
#                 expected_h = int(self.get_parameter('image_height').value)
#                 actual_h, actual_w = cv_image.shape[:2]
#                 if actual_w != expected_w or actual_h != expected_h:
#                     self.get_logger().warn(
#                         f'Résolution caméra {actual_w}x{actual_h} ≠ params {expected_w}x{expected_h} '
#                         f'→ redimensionnement automatique activé'
#                     )
#                 self._size_warned = True

#             # Redimensionner si nécessaire (CRITIQUE pour le warp)
#             expected_w = int(self.get_parameter('image_width').value)
#             expected_h = int(self.get_parameter('image_height').value)
#             actual_h, actual_w = cv_image.shape[:2]
#             if actual_w != expected_w or actual_h != expected_h:
#                 cv_image = cv2.resize(cv_image, (expected_w, expected_h))

#             hsv_lower = self.get_parameter('hsv_lower').value
#             hsv_upper = self.get_parameter('hsv_upper').value

#             # ── Pipeline de détection ──
#             # 1. Pré-traitement (HSV + masque)
#             binary = self.processor.preprocess(cv_image, hsv_lower, hsv_upper)

#             # 2. Bird's eye view
#             binary_warped = self.processor.warp_to_birdseye(binary)

#             # 3. Sliding window
#             left_x, left_y, right_x, right_y, window_img = \
#                 self.processor.find_lane_pixels(binary_warped)

#             # 4. Fit polynômes
#             left_fit, right_fit, valid = \
#                 self.processor.fit_polynomials(left_x, left_y, right_x, right_y)

#             if valid:
#                 self._valid_count += 1

#             # ── Log périodique (toutes les 100 frames ≈ 3.3s à 30fps) ──
#             if self._frame_count % 100 == 0:
#                 rate = (self._valid_count / self._frame_count) * 100
#                 self.get_logger().info(
#                     f'Frame {self._frame_count} | '
#                     f'Détection OK: {rate:.0f}% | '
#                     f'Pixels G={len(left_x)} D={len(right_x)}'
#                 )

#             # ── Publier les polynômes ──
#             self.publish_lane_fits(left_fit, right_fit, valid)

#             # ── Publier images debug ──
#             if self.get_parameter('publish_debug_image').value:
#                 overlay = self.processor.draw_lane_overlay(cv_image, left_fit, right_fit)
#                 debug_msg = self.bgr8_to_image_msg(overlay, msg.header.frame_id)
#                 debug_msg.header.stamp = msg.header.stamp
#                 self.debug_image_pub.publish(debug_msg)

#             if self.get_parameter('publish_birdseye').value:
#                 birdseye = self.processor.draw_birdseye_debug(
#                     binary_warped, window_img, left_fit, right_fit)
#                 be_msg = self.bgr8_to_image_msg(birdseye, msg.header.frame_id)
#                 be_msg.header.stamp = msg.header.stamp
#                 self.birdseye_pub.publish(be_msg)

#         except Exception as e:
#             self.get_logger().error(f'Erreur frame {self._frame_count}: {str(e)}')

#     def image_msg_to_bgr8(self, msg):
#         """Convertit sensor_msgs/Image vers OpenCV BGR."""
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

#         if encoding == 'rgb8':
#             return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
#         if encoding == 'rgba8':
#             return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
#         if encoding == 'bgra8':
#             return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
#         if encoding == 'mono8':
#             return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
#         return img

#     def bgr8_to_image_msg(self, image, frame_id=''):
#         """Convertit une image OpenCV BGR vers sensor_msgs/Image."""
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

#     def publish_lane_fits(self, left_fit, right_fit, valid):
#         """
#         Publie les coefficients des polynômes.
#         Format : [valid, left_a, left_b, left_c, right_a, right_b, right_c]
#         """
#         msg = Float32MultiArray()

#         if not valid or left_fit is None or right_fit is None:
#             msg.data = [0.0]
#         else:
#             msg.data = [
#                 1.0,                                    # valid
#                 float(left_fit[0]),  float(left_fit[1]),  float(left_fit[2]),   # gauche
#                 float(right_fit[0]), float(right_fit[1]), float(right_fit[2]),  # droite
#             ]
#             #self.get_logger(msg.data)
#         self.lane_fits_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LaneDetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
