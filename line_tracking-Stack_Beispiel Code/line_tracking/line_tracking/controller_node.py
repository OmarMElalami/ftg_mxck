#!/usr/bin/env python3
"""
Controller Node — avec fallback IMU

ROLE : Lit les points des lignes -> calcule la trajectoire -> publie le point cible
       Quand la detection est perdue -> utilise le gyroscope IMU pour tenir le cap

    /lane_detection/lane_fits ──> [Controller] ──> /control/target_point
    /imu                      ──> [Controller]

Recoit (Float32MultiArray) :
    [valid, mode_int,
     lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,   <- ligne gauche (bas -> haut)
     rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]    <- ligne droite (bas -> haut)

Publie (Float32MultiArray) :
    /control/target_point : [valid, target_x, target_y, center_offset,
                             heading_error, suggested_speed]
    Tout en metres, referentiel robot :
        x = avant (+) / arriere (-)
        y = gauche (+) / droite (-)

Fallback IMU :
    Quand lane_valid = False, le node integre angular_velocity.z du topic /imu
    pour estimer combien le robot a tourne depuis la perte de detection.
    Il continue d avancer doucement (imu_fallback_speed) avec heading_error=0
    jusqu a ce que :
      - la detection reprenne  (retour normal)
      - OU le robot ait tourne de plus de imu_max_rotation_deg (STOP securite)
      - OU le fallback dure plus de imu_max_duration_s (STOP securite)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import numpy as np
import math


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # ── Parametres existants ──
        default_params = {
            'image_width':        640,
            'image_height':       360,
            'pixels_per_meter':   905,
            'lookahead_distance': 0.6,
            'min_lookahead':      0.25,
            'max_lookahead':      2.5,
            'lookahead_gain':     0.4,
            'target_speed':       0.5,
            'min_speed':          0.1,
            'n_waypoints':        10,
            'control_rate':       50.0,
            'no_data_timeout':    0.5,
        }
        # ── Nouveaux parametres IMU fallback ──
        imu_params = {
            'imu_topic':              '/imu',
            'imu_fallback_speed':     0.20,
            'imu_max_rotation_deg':   35.0,
            'imu_max_duration_s':     1.5,
            'imu_heading_decay':      0.85,
            'max_heading_error_deg':  35.0,     # NOUVEAU — clamp heading pour eviter les valeurs aberrantes
        }

        for name, val in {**default_params, **imu_params}.items():
            self.declare_parameter(name, val)

        # Parametres existants
        self.image_width        = int(self._require_param('image_width'))
        self.image_height       = int(self._require_param('image_height'))
        self.pixels_per_meter   = float(self._require_param('pixels_per_meter'))
        self.lookahead_distance = float(self._require_param('lookahead_distance'))
        self.min_lookahead      = float(self._require_param('min_lookahead'))
        self.max_lookahead      = float(self._require_param('max_lookahead'))
        self.lookahead_gain     = float(self._require_param('lookahead_gain'))
        self.target_speed       = float(self._require_param('target_speed'))
        self.min_speed          = float(self._require_param('min_speed'))
        self.n_waypoints        = int(self._require_param('n_waypoints'))
        self.control_rate       = float(self._require_param('control_rate'))
        self.no_data_timeout    = float(self._require_param('no_data_timeout'))

        # Parametres IMU fallback
        self.imu_topic             = str(self._require_param('imu_topic'))
        self.imu_fallback_speed    = float(self._require_param('imu_fallback_speed'))
        self.imu_max_rotation_rad  = math.radians(float(self._require_param('imu_max_rotation_deg')))
        self.imu_max_duration_s    = float(self._require_param('imu_max_duration_s'))
        self.imu_heading_decay     = float(self._require_param('imu_heading_decay'))
        self.max_heading_error_rad = math.radians(float(self._require_param('max_heading_error_deg')))

        # ── Etat detection visuelle ──
        self.left_pts       = None
        self.right_pts      = None
        self.lane_mode      = 'aucune'
        self.lane_valid     = False
        self.last_data_time = None
        self.current_speed  = 0.0
        self._log_counter   = 0

        # ── Lissage temporel heading + offset (filtre les sauts du CNN) ──
        self.smoothed_heading = 0.0
        self.smoothed_offset  = 0.0
        self.heading_smooth_alpha = 0.4   # 0 = pas de lissage, 1 = figé sur l ancien

        # ── Etat fallback IMU ──
        self.imu_gz               = 0.0    # angular_velocity.z lu depuis /imu (rad/s)
        self.imu_fallback_active  = False  # True quand on est en mode fallback
        self.imu_fallback_start   = None   # instant de debut du fallback
        self.imu_rotation_accum   = 0.0    # rotation integree depuis debut fallback (rad)
        self.last_imu_time        = None   # pour dt d integration
        self.last_heading_error   = 0.0    # dernier heading_error valide (rad)
        self.last_center_offset   = 0.0    # dernier center_offset valide
        self.imu_available        = False  # True quand au moins 1 message IMU recu

        # ── Subscribers ──
        self.lane_fits_sub = self.create_subscription(
            Float32MultiArray,
            '/lane_detection/lane_fits',
            self.lane_fits_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_callback,
            10
        )

        # ── Publisher ──
        self.target_pub = self.create_publisher(
            Float32MultiArray, '/control/target_point', 10)

        # ── Timer boucle de controle ──
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('=== Controller Node (+ fallback IMU) ===')
        self.get_logger().info(f'Ecoute : /lane_detection/lane_fits')
        self.get_logger().info(f'Ecoute : {self.imu_topic}  (fallback gyroscope)')
        self.get_logger().info(f'Fallback : vitesse={self.imu_fallback_speed}m/s | '
                               f'rotation_max={math.degrees(self.imu_max_rotation_rad):.0f}deg | '
                               f'duree_max={self.imu_max_duration_s}s')
        self.get_logger().info('Publie : /control/target_point')

    # ================================================================
    #  Callback IMU — lit angular_velocity.z en continu
    # ================================================================
    def _imu_callback(self, msg: Imu):
        """
        Lit la vitesse angulaire autour de Z (lacet = yaw rate).
        Si fallback actif : integre pour estimer la rotation accumulee.
        """
        self.imu_gz        = float(msg.angular_velocity.z)
        self.imu_available = True

        if self.imu_fallback_active and self.last_imu_time is not None:
            now = self.get_clock().now()
            dt  = (now - self.last_imu_time).nanoseconds / 1e9
            if 0.0 < dt < 0.5:
                self.imu_rotation_accum += self.imu_gz * dt

        self.last_imu_time = self.get_clock().now()

    # ================================================================
    #  Callback detection lignes
    # ================================================================
    def lane_fits_callback(self, msg):
        data = list(msg.data)
        if len(data) < 18 or data[0] < 0.5:
            self.lane_valid = False
            return

        mode_int = data[1]
        self.lane_mode = {2.0: 'paire', 1.0: 'dominante', 0.0: 'aucune'}.get(mode_int, 'aucune')

        left_flat  = data[2:10]
        right_flat = data[10:18]
        self.left_pts  = np.array(left_flat,  dtype=np.float32).reshape(4, 2)
        self.right_pts = np.array(right_flat, dtype=np.float32).reshape(4, 2)

        self.lane_valid     = True
        self.last_data_time = self.get_clock().now()

        # Detection retrouvee -> sortir du fallback
        if self.imu_fallback_active:
            self.get_logger().info(
                f'[IMU fallback] Detection retrouvee apres '
                f'{math.degrees(self.imu_rotation_accum):.1f}deg de rotation — retour normal'
            )
            self._reset_fallback()

    # ================================================================
    #  Boucle de controle principale
    # ================================================================
    def control_loop(self):
        msg = Float32MultiArray()

        # ── Timeout detection ──
        if self.last_data_time is not None:
            elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
            if elapsed > self.no_data_timeout:
                self.lane_valid = False

        # ── Pas de detection : fallback IMU ou STOP ──
        if not self.lane_valid or self.left_pts is None or self.right_pts is None:
            self._publish_fallback(msg)
            return

        # ── Detection valide : pipeline normal ──
        self._publish_normal(msg)

    # ================================================================
    #  Pipeline normal (detection valide)
    # ================================================================
    def _publish_normal(self, msg):
        # 1. Midline
        mid_pts = (self.left_pts + self.right_pts) / 2.0

        # 2. Fit polynomial deg 2
        try:
            mid_fit = np.polyfit(mid_pts[:, 1], mid_pts[:, 0], 2)
        except Exception:
            msg.data = [0.0]
            self.target_pub.publish(msg)
            return

        # 3. Waypoints
        y_values = np.linspace(self.image_height - 1, 0, self.n_waypoints)
        x_values = np.polyval(mid_fit, y_values)
        x_values = np.clip(x_values, 0, self.image_width - 1)

        # 4. Conversion pixels -> metres
        center_x = self.image_width / 2.0
        waypoints_robot = []
        for px, py in zip(x_values, y_values):
            x_robot = (self.image_height - py) / self.pixels_per_meter
            y_robot = -(px - center_x) / self.pixels_per_meter
            waypoints_robot.append([x_robot, y_robot])
        waypoints_robot = np.array(waypoints_robot)

        # 5. Center offset
        closest_px    = x_values[0]
        center_offset = (closest_px - center_x) / (self.image_width / 2.0)

        # 6. Heading error — avec CLAMP et LISSAGE
        y_bottom      = self.image_height - 1
        dxdy          = 2 * mid_fit[0] * y_bottom + mid_fit[1]
        heading_raw   = np.arctan(dxdy)

        # Clamp : le CNN peut donner des headings aberrants (±80°)
        heading_clamped = float(np.clip(heading_raw,
                                        -self.max_heading_error_rad,
                                        self.max_heading_error_rad))

        # Lissage temporel : filtre passe-bas pour absorber les sauts frame-a-frame
        alpha = self.heading_smooth_alpha
        self.smoothed_heading = alpha * self.smoothed_heading + (1.0 - alpha) * heading_clamped
        self.smoothed_offset  = alpha * self.smoothed_offset  + (1.0 - alpha) * center_offset

        heading_error = self.smoothed_heading
        center_offset = self.smoothed_offset

        # 7. Courbure
        curvature = abs(2.0 * mid_fit[0])

        # 8. Lookahead adaptatif
        lookahead_dist = self.min_lookahead + self.lookahead_gain * abs(self.current_speed)
        lookahead_dist = np.clip(lookahead_dist, self.min_lookahead, self.max_lookahead)

        target_point = self._find_lookahead_point(waypoints_robot, lookahead_dist)
        if target_point is None:
            msg.data = [0.0]
            self.target_pub.publish(msg)
            return

        # 9. Vitesse adaptative
        speed_factor    = 1.0 / (1.0 + 5.0 * curvature)
        suggested_speed = max(self.target_speed * speed_factor, self.min_speed)
        self.current_speed = suggested_speed

        # Sauvegarder pour le fallback
        self.last_heading_error  = float(heading_error)
        self.last_center_offset  = float(center_offset)

        # 10. Publier
        msg.data = [
            1.0,
            float(target_point[0]),
            float(target_point[1]),
            float(center_offset),
            float(heading_error),
            float(suggested_speed),
        ]
        self.target_pub.publish(msg)

        self._log_counter += 1
        if self._log_counter % 50 == 0:
            self.get_logger().info(
                f'Mode: {self.lane_mode} | '
                f'Offset: {center_offset:+.2f} | '
                f'Heading: {math.degrees(heading_error):+.1f}deg | '
                f'Target: ({target_point[0]:.2f}, {target_point[1]:.2f})m | '
                f'Speed: {suggested_speed:.2f} m/s | '
                f'Curvature: {curvature:.4f}'
            )

    # ================================================================
    #  Fallback IMU (detection perdue)
    # ================================================================
    def _publish_fallback(self, msg):
        """
        Comportement quand la detection visuelle est perdue :

        SI l IMU est disponible :
          - Activer le fallback et avancer doucement
          - Integrer angular_velocity.z pour suivre la rotation
          - Si rotation > imu_max_rotation_deg -> STOP securite
          - Si duree > imu_max_duration_s -> STOP securite
          - heading_error amorti vers 0 (le robot se redresse progressivement)

        SI l IMU n est pas disponible :
          - STOP (comportement original)
        """
        if not self.imu_available:
            # Pas d IMU -> comportement original : STOP
            msg.data = [0.0]
            self.target_pub.publish(msg)
            return

        now = self.get_clock().now()

        # Initialiser le fallback au premier appel
        if not self.imu_fallback_active:
            self.imu_fallback_active = True
            self.imu_fallback_start  = now
            self.imu_rotation_accum  = 0.0
            self.last_imu_time       = now
            self.get_logger().warn(
                f'[IMU fallback] Detection perdue — continuation gyroscope | '
                f'vitesse={self.imu_fallback_speed}m/s'
            )

        # Verifier les conditions de securite
        elapsed = (now - self.imu_fallback_start).nanoseconds / 1e9

        if abs(self.imu_rotation_accum) > self.imu_max_rotation_rad:
            self.get_logger().error(
                f'[IMU fallback] STOP securite : rotation accumulee '
                f'{math.degrees(self.imu_rotation_accum):.1f}deg > '
                f'{math.degrees(self.imu_max_rotation_rad):.0f}deg limite'
            )
            self._reset_fallback()
            msg.data = [0.0]
            self.target_pub.publish(msg)
            return

        if elapsed > self.imu_max_duration_s:
            self.get_logger().error(
                f'[IMU fallback] STOP securite : duree {elapsed:.1f}s > '
                f'{self.imu_max_duration_s}s limite'
            )
            self._reset_fallback()
            msg.data = [0.0]
            self.target_pub.publish(msg)
            return

        # Amortir le heading error vers 0 (le robot se stabilise)
        fallback_heading = self.last_heading_error * self.imu_heading_decay
        self.last_heading_error = fallback_heading

        # center_offset amorti aussi
        fallback_offset = self.last_center_offset * self.imu_heading_decay
        self.last_center_offset = fallback_offset

        # Point cible : tout droit devant a imu_fallback_speed
        fallback_target_x = self.min_lookahead
        fallback_target_y = 0.0

        msg.data = [
            1.0,
            float(fallback_target_x),
            float(fallback_target_y),
            float(fallback_offset),     # offset amorti
            float(fallback_heading),    # heading amorti
            float(self.imu_fallback_speed),
        ]
        self.target_pub.publish(msg)

        self._log_counter += 1
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                f'[IMU fallback] t={elapsed:.2f}s | '
                f'rotation={math.degrees(self.imu_rotation_accum):+.1f}deg | '
                f'gz={math.degrees(self.imu_gz):+.1f}deg/s | '
                f'speed={self.imu_fallback_speed}m/s'
            )

    # ================================================================
    #  Reset etat fallback
    # ================================================================
    def _reset_fallback(self):
        self.imu_fallback_active = False
        self.imu_fallback_start  = None
        self.imu_rotation_accum  = 0.0

    # ================================================================
    #  Lookahead point
    # ================================================================
    def _find_lookahead_point(self, waypoints, lookahead_dist):
        if waypoints is None or len(waypoints) < 2:
            return None
        distances = np.sqrt(waypoints[:, 0]**2 + waypoints[:, 1]**2)
        for i in range(len(distances) - 1):
            if distances[i] <= lookahead_dist <= distances[i + 1]:
                ratio = (lookahead_dist - distances[i]) / \
                        (distances[i + 1] - distances[i] + 1e-6)
                return waypoints[i] + ratio * (waypoints[i + 1] - waypoints[i])
        if lookahead_dist >= distances[-1]:
            return waypoints[-1]
        return waypoints[0]

    def _require_param(self, name):
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Parametre '{name}' absent dans config/params.yaml")
        return value


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
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
# Controller Node — Cerveau strategique

# ROLE : Lit les points des lignes -> calcule la trajectoire -> publie le point cible

#     /lane_detection/lane_fits ──> [Controller] ──> /control/target_point

# Recoit (Float32MultiArray) :
#     [valid, mode_int,
#      lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,   <- ligne gauche (bas -> haut)
#      rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]    <- ligne droite (bas -> haut)

# Publie (Float32MultiArray) :
#     /control/target_point : [valid, target_x, target_y, center_offset,
#                              heading_error, suggested_speed]
#     Tout en metres, referentiel robot :
#         x = avant (+) / arriere (-)
#         y = gauche (+) / droite (-)
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import math


# class ControllerNode(Node):

#     def __init__(self):
#         super().__init__('controller_node')

#         # ── Parametres ──
#         default_params = {
#             'image_width':        640,
#             'image_height':       360,
#             'pixels_per_meter':   905,
#             'lookahead_distance': 0.6,
#             'min_lookahead':      0.25,
#             'max_lookahead':      2.5,
#             'lookahead_gain':     0.4,
#             'target_speed':       0.5,
#             'min_speed':          0.1,
#             'n_waypoints':        10,
#             'control_rate':       50.0,
#             'no_data_timeout':    0.5,
#         }
#         for name, default_value in default_params.items():
#             self.declare_parameter(name, default_value)

#         self.image_width       = int(self._require_param('image_width'))
#         self.image_height      = int(self._require_param('image_height'))
#         self.pixels_per_meter  = float(self._require_param('pixels_per_meter'))
#         self.lookahead_distance = float(self._require_param('lookahead_distance'))
#         self.min_lookahead     = float(self._require_param('min_lookahead'))
#         self.max_lookahead     = float(self._require_param('max_lookahead'))
#         self.lookahead_gain    = float(self._require_param('lookahead_gain'))
#         self.target_speed      = float(self._require_param('target_speed'))
#         self.min_speed         = float(self._require_param('min_speed'))
#         self.n_waypoints       = int(self._require_param('n_waypoints'))
#         self.control_rate      = float(self._require_param('control_rate'))
#         self.no_data_timeout   = float(self._require_param('no_data_timeout'))

#         # ── Etat ──
#         self.left_pts    = None   # np.array (4,2)
#         self.right_pts   = None   # np.array (4,2)
#         self.lane_mode   = 'aucune'
#         self.lane_valid  = False
#         self.last_data_time  = None
#         self.current_speed   = 0.0
#         self._log_counter    = 0

#         # ── Subscriber ──
#         self.lane_fits_sub = self.create_subscription(
#             Float32MultiArray,
#             '/lane_detection/lane_fits',
#             self.lane_fits_callback,
#             10
#         )

#         # ── Publisher ──
#         self.target_pub = self.create_publisher(
#             Float32MultiArray, '/control/target_point', 10)

#         # ── Timer boucle de controle ──
#         self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

#         self.get_logger().info('=== Controller Node ===')
#         self.get_logger().info('Ecoute : /lane_detection/lane_fits  [valid, mode, 4pts_G, 4pts_D]')
#         self.get_logger().info('Publie : /control/target_point')

#     # ================================================================
#     #  Callback : reception des points de lignes
#     # ================================================================
#     def lane_fits_callback(self, msg):
#         """
#         Recoit [valid, mode_int,
#                 lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,
#                 rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]
#         """
#         data = list(msg.data)

#         # Format attendu : 2 + 8 + 8 = 18 valeurs
#         if len(data) < 18 or data[0] < 0.5:
#             self.lane_valid = False
#             return

#         mode_int = data[1]
#         self.lane_mode = {2.0: 'paire', 1.0: 'dominante', 0.0: 'aucune'}.get(mode_int, 'aucune')

#         # Reconstruire les arrays (4,2)
#         left_flat  = data[2:10]    # 8 valeurs = 4 points x,y
#         right_flat = data[10:18]

#         self.left_pts  = np.array(left_flat,  dtype=np.float32).reshape(4, 2)
#         self.right_pts = np.array(right_flat, dtype=np.float32).reshape(4, 2)

#         self.lane_valid    = True
#         self.last_data_time = self.get_clock().now()

#     # ================================================================
#     #  Boucle de controle
#     # ================================================================
#     def control_loop(self):
#         msg = Float32MultiArray()

#         # Timeout
#         if self.last_data_time is not None:
#             elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
#             if elapsed > self.no_data_timeout:
#                 self.lane_valid = False

#         if not self.lane_valid or self.left_pts is None or self.right_pts is None:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 1. Midline : moyenne des points gauche et droite ──
#         # Les 4 points sont deja alignes (meme indices = meme hauteur)
#         mid_pts = (self.left_pts + self.right_pts) / 2.0  # shape (4,2)

#         # ── 2. Fit polynomial deg 2 sur la midline (pixels) ──
#         # mid_pts[:,1] = y (hauteur), mid_pts[:,0] = x (lateral)
#         # On fit x = f(y) comme avant
#         try:
#             mid_fit = np.polyfit(mid_pts[:, 1], mid_pts[:, 0], 2)
#         except Exception:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 3. Echantillonner N waypoints sur la midline ──
#         y_values = np.linspace(self.image_height - 1, 0, self.n_waypoints)
#         x_values = np.polyval(mid_fit, y_values)
#         x_values = np.clip(x_values, 0, self.image_width - 1)

#         # ── 4. Convertir en metres (referentiel robot) ──
#         center_x = self.image_width / 2.0
#         waypoints_robot = []
#         for px, py in zip(x_values, y_values):
#             x_robot = (self.image_height - py) / self.pixels_per_meter
#             y_robot = -(px - center_x) / self.pixels_per_meter
#             waypoints_robot.append([x_robot, y_robot])
#         waypoints_robot = np.array(waypoints_robot)

#         # ── 5. Center offset ──
#         closest_px    = x_values[0]
#         center_offset = (closest_px - center_x) / (self.image_width / 2.0)

#         # ── 6. Heading error ──
#         y_bottom   = self.image_height - 1
#         dxdy       = 2 * mid_fit[0] * y_bottom + mid_fit[1]
#         heading_error = np.arctan(dxdy)

#         # ── 7. Courbure (depuis le fit polynomial) ──
#         curvature = abs(2.0 * mid_fit[0])

#         # ── 8. Lookahead adaptatif ──
#         lookahead_dist = self.min_lookahead + self.lookahead_gain * abs(self.current_speed)
#         lookahead_dist = np.clip(lookahead_dist, self.min_lookahead, self.max_lookahead)

#         target_point = self._find_lookahead_point(waypoints_robot, lookahead_dist)
#         if target_point is None:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 9. Vitesse adaptative (ralentir en courbe) ──
#         speed_factor   = 1.0 / (1.0 + 5.0 * curvature)
#         suggested_speed = max(self.target_speed * speed_factor, self.min_speed)
#         self.current_speed = suggested_speed

#         # ── 10. Publier ──
#         msg.data = [
#             1.0,
#             float(target_point[0]),   # target_x (metres, avant)
#             float(target_point[1]),   # target_y (metres, lateral)
#             float(center_offset),     # [-1, 1]
#             float(heading_error),     # radians
#             float(suggested_speed),   # m/s
#         ]
#         self.target_pub.publish(msg)

#         # Log
#         self._log_counter += 1
#         if self._log_counter % 50 == 0:
#             self.get_logger().info(
#                 f'Mode: {self.lane_mode} | '
#                 f'Offset: {center_offset:+.2f} | '
#                 f'Heading: {math.degrees(heading_error):+.1f}deg | '
#                 f'Target: ({target_point[0]:.2f}, {target_point[1]:.2f})m | '
#                 f'Speed: {suggested_speed:.2f} m/s | '
#                 f'Curvature: {curvature:.4f}'
#             )

#     def _require_param(self, name):
#         value = self.get_parameter(name).value
#         if value is None:
#             raise ValueError(f"Parametre '{name}' absent dans config/params.yaml")
#         return value

#     # ================================================================
#     #  Lookahead point (Pure Pursuit)
#     # ================================================================
#     def _find_lookahead_point(self, waypoints, lookahead_dist):
#         if waypoints is None or len(waypoints) < 2:
#             return None

#         distances = np.sqrt(waypoints[:, 0]**2 + waypoints[:, 1]**2)

#         for i in range(len(distances) - 1):
#             if distances[i] <= lookahead_dist <= distances[i + 1]:
#                 ratio = (lookahead_dist - distances[i]) / \
#                         (distances[i + 1] - distances[i] + 1e-6)
#                 return waypoints[i] + ratio * (waypoints[i + 1] - waypoints[i])

#         if lookahead_dist >= distances[-1]:
#             return waypoints[-1]

#         return waypoints[0]


# def main(args=None):
#     rclpy.init(args=args)
#     node = ControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
# """
# Controller Node — Cerveau strategique

# ROLE : Lit les points des lignes -> calcule la trajectoire -> publie le point cible

#     /lane_detection/lane_fits ──> [Controller] ──> /control/target_point

# Recoit (Float32MultiArray) :
#     [valid, mode_int,
#      lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,   <- ligne gauche (bas -> haut)
#      rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]    <- ligne droite (bas -> haut)

# Publie (Float32MultiArray) :
#     /control/target_point : [valid, target_x, target_y, center_offset,
#                              heading_error, suggested_speed]
#     Tout en metres, referentiel robot :
#         x = avant (+) / arriere (-)
#         y = gauche (+) / droite (-)
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import math


# class ControllerNode(Node):

#     def __init__(self):
#         super().__init__('controller_node')

#         # ── Parametres ──
#         default_params = {
#             'image_width':        640,
#             'image_height':       360,
#             'pixels_per_meter':   905,
#             'lookahead_distance': 0.6,
#             'min_lookahead':      0.25,
#             'max_lookahead':      2.5,
#             'lookahead_gain':     0.4,
#             'target_speed':       0.5,
#             'min_speed':          0.1,
#             'n_waypoints':        10,
#             'control_rate':       50.0,
#             'no_data_timeout':    0.5,
#         }
#         for name, default_value in default_params.items():
#             self.declare_parameter(name, default_value)

#         self.image_width       = int(self._require_param('image_width'))
#         self.image_height      = int(self._require_param('image_height'))
#         self.pixels_per_meter  = float(self._require_param('pixels_per_meter'))
#         self.lookahead_distance = float(self._require_param('lookahead_distance'))
#         self.min_lookahead     = float(self._require_param('min_lookahead'))
#         self.max_lookahead     = float(self._require_param('max_lookahead'))
#         self.lookahead_gain    = float(self._require_param('lookahead_gain'))
#         self.target_speed      = float(self._require_param('target_speed'))
#         self.min_speed         = float(self._require_param('min_speed'))
#         self.n_waypoints       = int(self._require_param('n_waypoints'))
#         self.control_rate      = float(self._require_param('control_rate'))
#         self.no_data_timeout   = float(self._require_param('no_data_timeout'))

#         # ── Etat ──
#         self.left_pts    = None   # np.array (4,2)
#         self.right_pts   = None   # np.array (4,2)
#         self.lane_mode   = 'aucune'
#         self.lane_valid  = False
#         self.last_data_time  = None
#         self.current_speed   = 0.0
#         self._log_counter    = 0

#         # ── Subscriber ──
#         self.lane_fits_sub = self.create_subscription(
#             Float32MultiArray,
#             '/lane_detection/lane_fits',
#             self.lane_fits_callback,
#             10
#         )

#         # ── Publisher ──
#         self.target_pub = self.create_publisher(
#             Float32MultiArray, '/control/target_point', 10)

#         # ── Timer boucle de controle ──
#         self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

#         self.get_logger().info('=== Controller Node ===')
#         self.get_logger().info('Ecoute : /lane_detection/lane_fits  [valid, mode, 4pts_G, 4pts_D]')
#         self.get_logger().info('Publie : /control/target_point')

#     # ================================================================
#     #  Callback : reception des points de lignes
#     # ================================================================
#     def lane_fits_callback(self, msg):
#         """
#         Recoit [valid, mode_int,
#                 lx1,ly1, lx2,ly2, lx3,ly3, lx4,ly4,
#                 rx1,ry1, rx2,ry2, rx3,ry3, rx4,ry4]
#         """
#         data = list(msg.data)

#         # Format attendu : 2 + 8 + 8 = 18 valeurs
#         if len(data) < 18 or data[0] < 0.5:
#             self.lane_valid = False
#             return

#         mode_int = data[1]
#         self.lane_mode = {2.0: 'paire', 1.0: 'dominante', 0.0: 'aucune'}.get(mode_int, 'aucune')

#         # Reconstruire les arrays (4,2)
#         left_flat  = data[2:10]    # 8 valeurs = 4 points x,y
#         right_flat = data[10:18]

#         self.left_pts  = np.array(left_flat,  dtype=np.float32).reshape(4, 2)
#         self.right_pts = np.array(right_flat, dtype=np.float32).reshape(4, 2)

#         self.lane_valid    = True
#         self.last_data_time = self.get_clock().now()

#     # ================================================================
#     #  Boucle de controle
#     # ================================================================
#     def control_loop(self):
#         msg = Float32MultiArray()

#         # Timeout
#         if self.last_data_time is not None:
#             elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
#             if elapsed > self.no_data_timeout:
#                 self.lane_valid = False

#         if not self.lane_valid or self.left_pts is None or self.right_pts is None:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 1. Midline : moyenne des points gauche et droite ──
#         # Les 4 points sont deja alignes (meme indices = meme hauteur)
#         mid_pts = (self.left_pts + self.right_pts) / 2.0  # shape (4,2)

#         # ── 2. Fit polynomial deg 2 sur la midline (pixels) ──
#         # mid_pts[:,1] = y (hauteur), mid_pts[:,0] = x (lateral)
#         # On fit x = f(y) comme avant
#         try:
#             mid_fit = np.polyfit(mid_pts[:, 1], mid_pts[:, 0], 2)
#         except Exception:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 3. Echantillonner N waypoints sur la midline ──
#         y_values = np.linspace(self.image_height - 1, 0, self.n_waypoints)
#         x_values = np.polyval(mid_fit, y_values)
#         x_values = np.clip(x_values, 0, self.image_width - 1)

#         # ── 4. Convertir en metres (referentiel robot) ──
#         center_x = self.image_width / 2.0
#         waypoints_robot = []
#         for px, py in zip(x_values, y_values):
#             x_robot = (self.image_height - py) / self.pixels_per_meter
#             y_robot = -(px - center_x) / self.pixels_per_meter
#             waypoints_robot.append([x_robot, y_robot])
#         waypoints_robot = np.array(waypoints_robot)

#         # ── 5. Center offset ──
#         closest_px    = x_values[0]
#         center_offset = (closest_px - center_x) / (self.image_width / 2.0)

#         # ── 6. Heading error ──
#         y_bottom   = self.image_height - 1
#         dxdy       = 2 * mid_fit[0] * y_bottom + mid_fit[1]
#         heading_error = np.arctan(dxdy)

#         # ── 7. Courbure (depuis le fit polynomial) ──
#         curvature = abs(2.0 * mid_fit[0])

#         # ── 8. Lookahead adaptatif ──
#         lookahead_dist = self.min_lookahead + self.lookahead_gain * abs(self.current_speed)
#         lookahead_dist = np.clip(lookahead_dist, self.min_lookahead, self.max_lookahead)

#         target_point = self._find_lookahead_point(waypoints_robot, lookahead_dist)
#         if target_point is None:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 9. Vitesse adaptative (ralentir en courbe) ──
#         speed_factor   = 1.0 / (1.0 + 5.0 * curvature)
#         suggested_speed = max(self.target_speed * speed_factor, self.min_speed)
#         self.current_speed = suggested_speed

#         # ── 10. Publier ──
#         msg.data = [
#             1.0,
#             float(target_point[0]),   # target_x (metres, avant)
#             float(target_point[1]),   # target_y (metres, lateral)
#             float(center_offset),     # [-1, 1]
#             float(heading_error),     # radians
#             float(suggested_speed),   # m/s
#         ]
#         self.target_pub.publish(msg)

#         # Log
#         self._log_counter += 1
#         if self._log_counter % 50 == 0:
#             self.get_logger().info(
#                 f'Mode: {self.lane_mode} | '
#                 f'Offset: {center_offset:+.2f} | '
#                 f'Heading: {math.degrees(heading_error):+.1f}deg | '
#                 f'Target: ({target_point[0]:.2f}, {target_point[1]:.2f})m | '
#                 f'Speed: {suggested_speed:.2f} m/s | '
#                 f'Curvature: {curvature:.4f}'
#             )

#     def _require_param(self, name):
#         value = self.get_parameter(name).value
#         if value is None:
#             raise ValueError(f"Parametre '{name}' absent dans config/params.yaml")
#         return value

#     # ================================================================
#     #  Lookahead point (Pure Pursuit)
#     # ================================================================
#     def _find_lookahead_point(self, waypoints, lookahead_dist):
#         if waypoints is None or len(waypoints) < 2:
#             return None

#         distances = np.sqrt(waypoints[:, 0]**2 + waypoints[:, 1]**2)

#         for i in range(len(distances) - 1):
#             if distances[i] <= lookahead_dist <= distances[i + 1]:
#                 ratio = (lookahead_dist - distances[i]) / \
#                         (distances[i + 1] - distances[i] + 1e-6)
#                 return waypoints[i] + ratio * (waypoints[i + 1] - waypoints[i])

#         if lookahead_dist >= distances[-1]:
#             return waypoints[-1]

#         return waypoints[0]


# def main(args=None):
#     rclpy.init(args=args)
#     node = ControllerNode()
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
# Controller Node — Cerveau stratégique (NOUVEAU)

# RÔLE : Lit les polynômes des lignes → calcule la trajectoire → publie le point cible

#     /lane_detection/lane_fits ──→ [Controller] ──→ /control/target_point
#                                                 ──→ /control/debug_info

# Ce node fait le lien entre la VISION et le CONTRÔLE MOTEUR.
# Il ne touche pas aux images, il ne touche pas aux moteurs.

# Calculs :
#     1. Midline = moyenne des 2 polynômes (milieu de la piste)
#     2. Waypoints = points échantillonnés sur la midline (en mètres)
#     3. Lookahead point = le point cible à viser (Pure Pursuit)
#     4. Center offset = décalage de la voiture par rapport au milieu
#     5. Heading error = angle entre la voiture et la trajectoire

# Publie (Float32MultiArray) :
#     /control/target_point : [valid, target_x, target_y, center_offset,
#                              heading_error, suggested_speed]
    
#     Tout en mètres, référentiel robot :
#         x = avant (+) / arrière (-)
#         y = gauche (+) / droite (-)

# PLUS TARD (Stanley) : ajouter subscriber /odom pour position exacte
# """

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import math


# class ControllerNode(Node):

#     def __init__(self):
#         super().__init__('controller_node')

#         # ── Paramètres (defaults robustes: ros2 run sans YAML) ──
#         default_params = {
#             'image_width': 640,
#             'image_height': 360,
#             'pixels_per_meter': 905,
#             'lookahead_distance': 0.6,
#             'min_lookahead': 0.25,
#             'max_lookahead': 2.5,
#             'lookahead_gain': 0.4,
#             'target_speed': 0.5,
#             'min_speed': 0.1,
#             'n_waypoints': 10,
#             'control_rate': 50.0,
#             'no_data_timeout': 0.5,
#         }
#         for name, default_value in default_params.items():
#             self.declare_parameter(name, default_value)

#         # ── Variables ──
#         self.image_width = int(self._require_param('image_width'))
#         self.image_height = int(self._require_param('image_height'))
#         self.pixels_per_meter = float(self._require_param('pixels_per_meter'))
#         self.lookahead_distance = float(self._require_param('lookahead_distance'))
#         self.min_lookahead = float(self._require_param('min_lookahead'))
#         self.max_lookahead = float(self._require_param('max_lookahead'))
#         self.lookahead_gain = float(self._require_param('lookahead_gain'))
#         self.target_speed = float(self._require_param('target_speed'))
#         self.min_speed = float(self._require_param('min_speed'))
#         self.n_waypoints = int(self._require_param('n_waypoints'))
#         self.control_rate = float(self._require_param('control_rate'))
#         self.no_data_timeout = float(self._require_param('no_data_timeout'))

#         self.left_fit = None
#         self.right_fit = None
#         self.lane_valid = False
#         self.last_data_time = None
#         self.current_speed = 0.0
#         self._log_counter = 0

#         # ── Subscriber : polynômes du Lane Detector ──
#         self.lane_fits_sub = self.create_subscription(
#             Float32MultiArray,
#             '/lane_detection/lane_fits',
#             self.lane_fits_callback,
#             10
#         )

#         # ── Publisher : point cible pour Pure Pursuit ──
#         self.target_pub = self.create_publisher(
#             Float32MultiArray,
#             '/control/target_point',
#             10
#         )

#         # ── Timer : boucle de calcul ──
#         self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

#         self.get_logger().info('=== Controller Node ===')
#         self.get_logger().info('Écoute : /lane_detection/lane_fits')
#         self.get_logger().info('Publie : /control/target_point')
#         self.get_logger().info('(Camera only — pas d\'odométrie)')

#     # ================================================================
#     #  Callback : réception des polynômes
#     # ================================================================
#     def lane_fits_callback(self, msg):
#         """Reçoit [valid, left_a, left_b, left_c, right_a, right_b, right_c]."""
#         data = msg.data

#         if len(data) < 7 or data[0] < 0.5:
#             self.lane_valid = False
#             return

#         self.left_fit = np.array([data[1], data[2], data[3]])
#         self.right_fit = np.array([data[4], data[5], data[6]])
#         self.lane_valid = True
#         self.last_data_time = self.get_clock().now()

#     # ================================================================
#     #  Boucle de contrôle (20 Hz)
#     # ================================================================
#     def control_loop(self):
#         """Calcule et publie le point cible."""
#         msg = Float32MultiArray()

#         # ── Timeout ? ──
#         if self.last_data_time is not None:
#             elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
#             if elapsed > self.no_data_timeout:
#                 self.lane_valid = False

#         # ── Pas de données valides → STOP ──
#         if not self.lane_valid or self.left_fit is None:
#             msg.data = [0.0]  # valid = 0
#             self.target_pub.publish(msg)
#             return

#         # ── 1. Calculer la midline ──
#         mid_fit = (self.left_fit + self.right_fit) / 2.0

#         # ── 2. Échantillonner des waypoints (pixels bird's eye) ──
#         y_values = np.linspace(self.image_height - 1, 0, self.n_waypoints)
#         x_values = np.polyval(mid_fit, y_values)
#         x_values = np.clip(x_values, 0, self.image_width - 1)

#         # ── 3. Convertir en mètres (référentiel robot) ──
#         center_x = self.image_width / 2.0

#         waypoints_robot = []
#         for px, py in zip(x_values, y_values):
#             x_robot = (self.image_height - py) / self.pixels_per_meter   # pixel bas = 0m, pixel haut = loin
#             y_robot = -(px - center_x) / self.pixels_per_meter           # pixel gauche = +, pixel droite = -
#             waypoints_robot.append([x_robot, y_robot])

#         waypoints_robot = np.array(waypoints_robot)

#         # ── 4. Center offset (au point le plus proche) ──
#         closest_px = x_values[0]  # Premier point = bas de l'image
#         center_offset = (closest_px - center_x) / (self.image_width / 2.0)

#         # ── 5. Heading error (tangente de la midline en bas) ──
#         y_bottom = self.image_height - 1
#         dxdy = 2 * mid_fit[0] * y_bottom + mid_fit[1]  # dérivée dx/dy
#         heading_error = np.arctan(dxdy)

#         # ── 6. Trouver le lookahead point ──
#         # Adapter la distance de lookahead à la vitesse
#         lookahead_dist = self.min_lookahead + self.lookahead_gain * abs(self.current_speed)
#         lookahead_dist = np.clip(lookahead_dist, self.min_lookahead, self.max_lookahead)

#         target_point = self.find_lookahead_point(waypoints_robot, lookahead_dist)

#         if target_point is None:
#             msg.data = [0.0]
#             self.target_pub.publish(msg)
#             return

#         # ── 7. Vitesse suggérée (ralentir en courbe) ──
#         curvature = abs(2.0 * mid_fit[0])  # Courbure ≈ 2a du polynôme

#         # Plus la courbure est forte → plus on ralentit
#         speed_factor = 1.0 / (1.0 + 5.0 * curvature)  # Décroît avec la courbure
#         suggested_speed = max(self.target_speed * speed_factor, self.min_speed)

#         self.current_speed = suggested_speed  # Pour le lookahead adaptatif

#         # ── 8. Publier ──
#         msg.data = [
#             1.0,                            # valid
#             float(target_point[0]),         # target_x (mètres, avant)
#             float(target_point[1]),         # target_y (mètres, latéral)
#             float(center_offset),           # décalage normalisé [-1, 1]
#             float(heading_error),           # radians
#             float(suggested_speed),         # m/s
#         ]
#         self.target_pub.publish(msg)

#         # ── Log ──
#         self._log_counter += 1
#         if self._log_counter % 50 == 0:
#             self.get_logger().info(
#                 f'Offset: {center_offset:+.2f} | '
#                 f'Heading: {math.degrees(heading_error):+.1f}° | '
#                 f'Target: ({target_point[0]:.2f}, {target_point[1]:.2f})m | '
#                 f'Speed: {suggested_speed:.2f} m/s | '
#                 f'Curvature: {curvature:.4f}'
#             )

#     def _require_param(self, name):
#         """Lit un paramètre obligatoire depuis YAML."""
#         value = self.get_parameter(name).value
#         if value is None:
#             raise ValueError(
#                 f"Paramètre '{name}' absent: ajoute-le dans config/params.yaml"
#             )
#         return value

#     # ================================================================
#     #  Trouver le lookahead point
#     # ================================================================
#     def find_lookahead_point(self, waypoints, lookahead_dist):
#         """
#         Cherche le point sur la midline à la distance de lookahead.
#         Interpole entre les waypoints.

#         Args:
#             waypoints: (N, 2) en coordonnées robot
#             lookahead_dist: distance souhaitée (mètres)

#         Returns:
#             [x, y] ou None
#         """
#         if waypoints is None or len(waypoints) < 2:
#             return None

#         # Distance de chaque waypoint depuis le robot (0, 0)
#         distances = np.sqrt(waypoints[:, 0]**2 + waypoints[:, 1]**2)

#         # Chercher le segment qui croise la distance de lookahead
#         for i in range(len(distances) - 1):
#             if distances[i] <= lookahead_dist <= distances[i + 1]:
#                 # Interpolation linéaire
#                 ratio = (lookahead_dist - distances[i]) / (distances[i + 1] - distances[i] + 1e-6)
#                 point = waypoints[i] + ratio * (waypoints[i + 1] - waypoints[i])
#                 return point

#         # Lookahead plus loin que tous les waypoints → dernier point
#         if lookahead_dist >= distances[-1]:
#             return waypoints[-1]

#         # Lookahead plus proche que le premier → premier point
#         return waypoints[0]


# def main(args=None):
#     rclpy.init(args=args)
#     node = ControllerNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

