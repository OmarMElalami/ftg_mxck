#!/usr/bin/env python3
"""
Obstacle Detector Node (depth camera).

RÔLE:
  Lit l'image de profondeur RealSense et publie un STOP sécurité
  quand un obstacle est trop proche devant le robot.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class ObstacleDetectorNode(Node):
    """Safety node based on forward depth measurements."""

    def __init__(self):
        super().__init__('obstacle_detector_node')

        default_params = {
            'depth_topic': '/camera/camera/depth/image_rect_raw',
            'stop_distance_m': 0.10,
            'clear_distance_m': 0.20,
            'min_valid_distance_m': 0.10,
            'max_valid_distance_m': 5.00,
            'roi_width_ratio': 0.35,
            'roi_height_ratio': 0.45,
            'min_valid_pixels': 80,
        }
        for name, default_value in default_params.items():
            self.declare_parameter(name, default_value)

        depth_topic = self._require_param('depth_topic')
        self.stop_distance = float(self._require_param('stop_distance_m'))
        self.clear_distance = float(self._require_param('clear_distance_m'))
        self.min_valid_distance = float(self._require_param('min_valid_distance_m'))
        self.max_valid_distance = float(self._require_param('max_valid_distance_m'))
        self.roi_width_ratio = float(self._require_param('roi_width_ratio'))
        self.roi_height_ratio = float(self._require_param('roi_height_ratio'))
        self.min_valid_pixels = int(self._require_param('min_valid_pixels'))

        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.stop_pub = self.create_publisher(Bool, '/safety/obstacle_stop', 10)
        self.distance_pub = self.create_publisher(Float32, '/safety/obstacle_distance_m', 10)

        self.stop_active = False
        self._log_counter = 0

        self.get_logger().info('=== Obstacle Detector Node ===')
        self.get_logger().info(f'Écoute : {depth_topic}')
        self.get_logger().info('Publie : /safety/obstacle_stop, /safety/obstacle_distance_m')
        self.get_logger().info(
            f'Seuils: stop<{self.stop_distance:.2f}m, clear>{self.clear_distance:.2f}m'
        )

    def depth_callback(self, msg: Image):
        try:
            depth_m = self.depth_msg_to_meters(msg)
            h, w = depth_m.shape[:2]

            roi_w = max(1, int(w * self.roi_width_ratio))
            roi_h = max(1, int(h * self.roi_height_ratio))
            x0 = max(0, (w - roi_w) // 2)
            y0 = max(0, h - roi_h)  # moitié basse (zone de roulage)
            roi = depth_m[y0:y0 + roi_h, x0:x0 + roi_w]

            valid = np.isfinite(roi)
            valid &= (roi >= self.min_valid_distance) & (roi <= self.max_valid_distance)
            valid_values = roi[valid]

            min_distance = math.inf
            if valid_values.size >= self.min_valid_pixels:
                min_distance = float(np.percentile(valid_values, 10))

            if self.stop_active:
                if min_distance > self.clear_distance:
                    self.stop_active = False
            else:
                if min_distance < self.stop_distance:
                    self.stop_active = True

            self.stop_pub.publish(Bool(data=self.stop_active))
            self.distance_pub.publish(
                Float32(data=min_distance if math.isfinite(min_distance) else -1.0)
            )

            self._log_counter += 1
            if self._log_counter % 30 == 0:
                d = f'{min_distance:.2f}m' if math.isfinite(min_distance) else 'invalid'
                self.get_logger().info(f'Obstacle: stop={self.stop_active} | min_dist={d}')

        except Exception as e:
            self.get_logger().error(f'Erreur depth_callback: {str(e)}')

    def depth_msg_to_meters(self, msg: Image) -> np.ndarray:
        """Convert depth image to meters (supports 16UC1 and 32FC1)."""
        enc = msg.encoding.upper()

        if enc in ('16UC1', 'MONO16'):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            return arr.astype(np.float32) / 1000.0

        if enc == '32FC1':
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            return arr

        raise ValueError(f'Encodage profondeur non supporté: {msg.encoding}')

    def _require_param(self, name):
        """Lit un paramètre obligatoire depuis YAML."""
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(
                f"Paramètre '{name}' absent: ajoute-le dans config/params.yaml"
            )
        return value


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# import numpy as np
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# # Il faut importer un utilitaire pour lire les données binaires du nuage
# import sensor_msgs_py.point_cloud2 as pc2 
# from std_msgs.msg import Bool, Float32

# class ObstacleDetectorNode(Node):
#     def __init__(self):
#         super().__init__('obstacle_detector_node')
        
#         # Nouveau topic
#         self.declare_parameter('pointcloud_topic', '/camera/camera/depth/color/points')
        
#         # Définition de la "Safety Box" en mètres
#         self.box_x_min = -0.15  # Largeur gauche
#         self.box_x_max = 0.15   # Largeur droite
#         self.box_y_min = -0.10  # Hauteur (sous la caméra)
#         self.box_y_max = 0.10   # Hauteur (au dessus)
#         self.stop_dist = 0.20   # Distance Z (devant)
        
#         self.pc_sub = self.create_subscription(
#             PointCloud2, 
#             self.get_parameter('pointcloud_topic').value, 
#             self.pc_callback, 
#             10)
#         # ... reste des publishers identiques ...

#     def pc_callback(self, msg):
#         # 1. Convertir le message en tableau NumPy (X, Y, Z)
#         # C'est l'étape la plus lourde techniquement
#         points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
#         if len(points) == 0:
#             return

#         # 2. Filtrage spatial (On ne garde que ce qui est "dans le passage" du robot)
#         # x = latéral, y = vertical, z = profondeur
#         mask = (points[:, 0] > self.box_x_min) & (points[:, 0] < self.box_x_max) & \
#                (points[:, 1] > self.box_y_min) & (points[:, 1] < self.box_y_max) & \
#                (points[:, 2] > 0.01) # On ignore les points trop proches de la lentille
        
#         relevant_points = points[mask]

#         if len(relevant_points) > 0:
#             # 3. Trouver le point le plus proche sur l'axe Z
#             min_distance = np.min(relevant_points[:, 2])
            
#             # Logique de stop
#             self.stop_active = min_distance < self.stop_dist
#         else:
#             min_distance = 10.0 # Rien en vue
#             self.stop_active = False

#         # 4. Publication
#         self.stop_pub.publish(Bool(data=self.stop_active))
#         self.distance_pub.publish(Float32(data=float(min_distance)))