#!/usr/bin/env python3
"""
Data Recorder Node pour MXCarKit
Ce noeud sauvegarde les images brutes et traitées pour l'entraînement.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from datetime import datetime


class DataRecorderNode(Node):
    """
    Noeud ROS2 pour enregistrer les images.
    
    Écoute :
        - /camera/camera/color/image_raw : images brutes de la caméra
        - /lane_detection/debug_image : images avec lignes détectées
    
    Sauvegarde dans :
        Image_run/
        └── run_YYYY_MM_DD_HHhMMmSSs/
            ├── camera_raw/
            └── lines_detected/
    """
    
    def __init__(self):
        super().__init__('data_recorder_node')
        
        # ==================== PARAMÈTRES ====================
        default_params = {
            'raw_image_topic': '/camera/camera/color/image_raw',
            'processed_image_topic': '/lane_detection/debug_image',
            'save_path': 'Image_run',
            'save_interval': 5,
            'recording': True,
            'use_cv_bridge': False,
        }
        for name, default_value in default_params.items():
            self.declare_parameter(name, default_value)
        
        # ==================== VARIABLES ====================
        
        self.raw_frame_count = 0
        self.processed_frame_count = 0
        self.raw_images_dir = None
        self.processed_images_dir = None
        self.raw_image_topic = self._require_param('raw_image_topic')
        self.processed_image_topic = self._require_param('processed_image_topic')
        self.save_path = self._require_param('save_path')
        self.save_interval = int(self._require_param('save_interval'))
        self.recording_enabled = bool(self._require_param('recording'))
        self.use_cv_bridge = bool(self._require_param('use_cv_bridge'))
        self.bridge = None

        if self.use_cv_bridge:
            try:
                from cv_bridge import CvBridge  # import local pour éviter crash si cv_bridge cassé
                self.bridge = CvBridge()
                self.get_logger().info('Conversion image: cv_bridge')
            except Exception as e:
                self.use_cv_bridge = False
                self.get_logger().warn(
                    f'cv_bridge indisponible ({str(e)}), fallback conversion NumPy activé.'
                )
        else:
            self.get_logger().info('Conversion image: NumPy (sans cv_bridge)')
        
        # ==================== CRÉER LES DOSSIERS ====================
        
        self.setup_save_directories()
        
        # ==================== SUBSCRIBERS ====================
        
        # Subscriber pour les images brutes de la caméra
        self.raw_image_sub = self.create_subscription(
            Image,
            self.raw_image_topic,
            self.raw_image_callback,
            10
        )
        
        # Subscriber pour les images avec lignes détectées
        self.processed_image_sub = self.create_subscription(
            Image,
            self.processed_image_topic,
            self.processed_image_callback,
            10
        )
        
        # ==================== LOGS ====================
        
        self.get_logger().info('=== Data Recorder Node initialisé ===')
        self.get_logger().info(f'Images brutes depuis: {self.raw_image_topic}')
        self.get_logger().info(f'Images avec lignes depuis: {self.processed_image_topic}')
        self.get_logger().info(f'Sauvegarde dans: {self.raw_images_dir}')
    
    def setup_save_directories(self):
        """
        Crée les dossiers pour sauvegarder les images.
        """
        base_path = self.save_path
        timestamp = datetime.now().strftime('%Y_%m_%d_%Hh%Mm%Ss')
        run_folder = f'run_{timestamp}'
        
        run_path = os.path.join(base_path, run_folder)
        self.raw_images_dir = os.path.join(run_path, 'camera_raw')
        self.processed_images_dir = os.path.join(run_path, 'lines_detected')
        
        os.makedirs(self.raw_images_dir, exist_ok=True)
        os.makedirs(self.processed_images_dir, exist_ok=True)
        
        self.get_logger().info(f'Dossiers créés:')
        self.get_logger().info(f'  - Images brutes: {self.raw_images_dir}')
        self.get_logger().info(f'  - Lignes détectées: {self.processed_images_dir}')
    
    def raw_image_callback(self, msg):
        """
        Callback pour les images brutes de la caméra.
        """
        if not self.recording_enabled:
            return
        
        self.raw_frame_count += 1
        
        if self.raw_frame_count % self.save_interval != 0:
            return
        
        try:
            cv_image = self.image_msg_to_bgr8(msg)
            filename = f'frame_{self.raw_frame_count:06d}.jpg'
            filepath = os.path.join(self.raw_images_dir, filename)
            cv2.imwrite(filepath, cv_image)
            
            if (self.raw_frame_count // self.save_interval) % 50 == 0:
                self.get_logger().info(f'Image brute sauvegardée: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Erreur sauvegarde image brute: {str(e)}')
    
    def processed_image_callback(self, msg):
        """
        Callback pour les images avec lignes détectées.
        """
        if not self.recording_enabled:
            return
        
        self.processed_frame_count += 1
        
        if self.processed_frame_count % self.save_interval != 0:
            return
        
        try:
            cv_image = self.image_msg_to_bgr8(msg)
            filename = f'frame_{self.processed_frame_count:06d}.jpg'
            filepath = os.path.join(self.processed_images_dir, filename)
            cv2.imwrite(filepath, cv_image)
            
            if (self.processed_frame_count // self.save_interval) % 50 == 0:
                self.get_logger().info(f'Image lignes sauvegardée: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'Erreur sauvegarde image traitée: {str(e)}')

    def image_msg_to_bgr8(self, msg):
        """
        Convertit un sensor_msgs/Image vers image OpenCV BGR.
        """
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
            raise ValueError(f'Encodage non supporté: {msg.encoding}')

        data = np.frombuffer(msg.data, dtype=np.uint8)
        row_size = msg.width * channels

        if msg.step == row_size:
            img = data.reshape((msg.height, msg.width, channels))
        else:
            # Gère les images avec padding de ligne éventuel
            img = data.reshape((msg.height, msg.step))[:, :row_size]
            img = img.reshape((msg.height, msg.width, channels))

        if encoding == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if encoding == 'mono8':
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

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
    node = DataRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('=== Enregistrement terminé ===')
        node.get_logger().info(f'Images brutes: {node.raw_frame_count // node.save_interval}')
        node.get_logger().info(f'Images lignes: {node.processed_frame_count // node.save_interval}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
