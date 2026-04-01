# from datetime import datetime
# import os

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.clock import Clock
# from ackermann_msgs.msg import AckermannDriveStamped
# from std_msgs.msg import Float32MultiArray
# import csv

# class PurePursuitNode(Node):

#     def __init__(self):
#         super().__init__('pure_pursuit_node')

#         # ── Paramètres ──
#         self.declare_parameter('wheel_base', 0.3)
#         self.declare_parameter('max_steer', 0.4)
#         self.declare_parameter('steer_smoothing', 0.2)

#         self.wheelbase = self.get_parameter('wheel_base').value
#         self.max_steer = self.get_parameter('max_steer').value
#         self.smoothing = self.get_parameter('steer_smoothing').value

#         self.target_point = None
#         self.speed = 0.0
#         self.steering = 0.0
#         self.data_valid = False

#         # dossier de logs
#         self.log_dir = "pure_pursuit_logs"
#         os.makedirs(self.log_dir, exist_ok=True)

#         # buffer mémoire
#         self.log_buffer = []

#         # timer sauvegarde
#         self.log_timer = self.create_timer(5.0, self.save_log)

#         self.pub = self.create_publisher(
#             AckermannDriveStamped,
#             '/autonomous/ackermann_cmd',
#             10
#         )

#         self.sub = self.create_subscription(
#             Float32MultiArray,
#             '/control/target_point',
#             self.update_target,
#             50
#         )

#         #self.timer = self.create_timer(0.05, self.send_cmd)

#         self.get_logger().info("=== PURE PURSUIT DRIVE NODE ===")

#     # ======================================================
#     # Reception target point
#     # ======================================================
#     def update_target(self, msg):

#         if len(msg.data) < 4 or msg.data[0] < 0.5:
#             self.data_valid = False
#             return

#         self.target_point = np.array([msg.data[1], msg.data[2]])
#         self.speed = msg.data[5]
#         self.get_logger().info(f"Zielpunkt : ( {msg.data[1]}, {msg.data[2]} ) , Geschwindigkeit = {self.speed}")
#         self.data_valid = True
#         self.send_cmd()

#         Ld = np.sqrt(msg.data[1]**2+ msg.data[2]**2)

#         t = self.get_clock().now().nanoseconds * 1e-9

#         self.log_buffer.append([t,msg.data[1],msg.data[2],msg.data[3],msg.data[4],Ld,self.speed,self.steering])

#     # ======================================================
#     # Pure Pursuit
#     # ======================================================
#     def compute_steering(self):

#         if not self.data_valid or self.target_point is None:
#             return 0.0

#         x_t, y_t = self.target_point
#         Ld = np.sqrt(x_t**2 + y_t**2)

#         if Ld < 1e-6:
#             return 0.0

#         delta = np.arctan2(2 * self.wheelbase * y_t,Ld**2)

#         delta = np.clip(delta, -self.max_steer, self.max_steer)

#         # Lissage
#         self.steering = ((1 - self.smoothing) * self.steering+ self.smoothing * delta)

#         return self.steering

#     # ======================================================
#     # Envoi commande
#     # ======================================================
#     def send_cmd(self):

#         msg = AckermannDriveStamped()
#         msg.header.stamp = Clock().now().to_msg()

#         if not self.data_valid:
#             msg.drive.speed = 0.0
#             msg.drive.steering_angle = 0.0
#         else:
#             steering = self.compute_steering()
#             msg.drive.speed = float(self.speed)
#             msg.drive.steering_angle = float(steering)

#         self.get_logger().info(f"Berechnete Winkel : ( {msg.drive.steering_angle} ) , Fahrgeschwindigkeit = {msg.drive.speed}")
#         self.pub.publish(msg)

#     def save_log(self):

#         if len(self.log_buffer) == 0:
#             return

#         # nom unique avec timestamp
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         filename = os.path.join(self.log_dir, f"log_{timestamp}.csv")

#         with open(filename, "w", newline="") as f:

#             writer = csv.writer(f)

#             writer.writerow(["time","target_x","target_y","center_offset","heading_error","lookahead","speed","steering"])

#             writer.writerows(self.log_buffer)

#         self.get_logger().info(f"Saved {len(self.log_buffer)} samples → {filename}")

#         # vider buffer
#         self.log_buffer = []

# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




#!/usr/bin/env python3
"""
Stanley Controller Node — Exécution moteur

ROLE : Reçoit center_offset + heading_error → calcule angle de braquage → envoie aux moteurs

    /control/target_point ──→ [Stanley] ──→ /autonomous/ackermann_cmd

Reçoit (Float32MultiArray) :
    [valid, target_x, target_y, center_offset, heading_error, suggested_speed]

    center_offset : [-1, +1]  — 0.0 = centré, +1 = bord droit, -1 = bord gauche
    heading_error : radians   — 0.0 = aligné piste, + = robot pointe à gauche

Publie (AckermannDriveStamped) :
    /autonomous/ackermann_cmd

Formule Stanley :
    δ = heading_error + arctan(k * center_offset / (v + ε))

    heading_error corrige l'angle du robot par rapport à la piste
    arctan(...)   corrige le décalage latéral

Comportement :
    - Démarre et avance dès que des données valides arrivent
    - S'arrête immédiatement si plus de données (Ctrl+C ou lane detector perdu)
    - Logs CSV toutes les 5 secondes dans pure_pursuit_logs/
"""

import os
import math
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
import csv


class StanleyNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ── Paramètres ──
        self.declare_parameter('wheel_base',      0.30)   # empattement robot (m)
        self.declare_parameter('max_steer',        0.50)  # angle max braquage (rad) ~28°
        self.declare_parameter('stanley_k',        1.0)   # gain correction latérale
        self.declare_parameter('steer_smoothing',  0.25)  # lissage steering [0=aucun, 1=figé]
        self.declare_parameter('speed_smoothing',  0.30)  # lissage vitesse
        self.declare_parameter('min_speed',        0.05)  # vitesse plancher (m/s)

        self.wheel_base      = float(self.get_parameter('wheel_base').value)
        self.max_steer       = float(self.get_parameter('max_steer').value)
        self.stanley_k       = float(self.get_parameter('stanley_k').value)
        self.steer_smoothing = float(self.get_parameter('steer_smoothing').value)
        self.speed_smoothing = float(self.get_parameter('speed_smoothing').value)
        self.min_speed       = float(self.get_parameter('min_speed').value)

        # ── Etat interne ──
        self.steering      = 0.0   # dernier angle de braquage envoyé (rad)
        self.speed         = 0.0   # dernière vitesse envoyée (m/s)
        self._log_counter  = 0

        # ── Logs CSV ──
        self.log_dir    = "pure_pursuit_logs"
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_buffer = []
        self.log_timer  = self.create_timer(5.0, self._save_log)

        # ── Publisher ──
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/autonomous/ackermann_cmd',
            10
        )

        # ── Subscriber ──
        # Pas de timer — on réagit à chaque message du controller (~30Hz)
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/control/target_point',
            self._on_target,
            10
        )

        self.get_logger().info('=== Stanley Controller Node ===')
        self.get_logger().info(f'Wheelbase: {self.wheel_base}m | '
                               f'K: {self.stanley_k} | '
                               f'Max steer: {math.degrees(self.max_steer):.1f}°')
        self.get_logger().info('En attente de données...')

    # ================================================================
    #  Callback principal — déclenché à chaque frame (~30Hz)
    # ================================================================
    def _on_target(self, msg):
        data = list(msg.data)

        # ── Données invalides ou absentes → STOP immédiat ──
        if len(data) < 6 or data[0] < 0.5:
            self._send(speed=0.0, steering=0.0)
            return

        center_offset  = float(data[3])   # [-1, +1]
        heading_error  = float(data[4])   # radians
        target_speed   = float(data[5])   # m/s suggéré par le controller

        # ── Formule Stanley ──
        #
        #   δ = heading_error + arctan(k * center_offset / (v + ε))
        #
        #   heading_error : corrige l'angle entre le robot et la piste
        #   arctan(...)   : corrige le décalage latéral, amorti par la vitesse
        #
        # Note sur les signes :
        #   center_offset > 0 → robot décalé à droite → il faut braquer à gauche → δ > 0
        #   heading_error > 0 → robot pointe à gauche → δ > 0 (déjà en bonne direction)

        v_eff          = max(abs(self.speed), self.min_speed)
        lateral_corr   = np.arctan(self.stanley_k * center_offset / v_eff)
        delta_raw      = heading_error + lateral_corr

        # Clamp
        delta_clamped  = float(np.clip(delta_raw, -self.max_steer, self.max_steer))

        # Lissage steering (filtre passe-bas)
        self.steering  = ((1.0 - self.steer_smoothing) * self.steering
                          + self.steer_smoothing * delta_clamped)

        # Lissage vitesse (accélération/décélération progressive)
        self.speed     = ((1.0 - self.speed_smoothing) * self.speed
                          + self.speed_smoothing * target_speed)

        self._send(speed=self.speed, steering=self.steering)

        # ── Log buffer ──
        t = self.get_clock().now().nanoseconds * 1e-9
        self.log_buffer.append([
            t,
            round(center_offset,  4),
            round(heading_error,  4),
            round(lateral_corr,   4),
            round(delta_raw,      4),
            round(self.steering,  4),
            round(self.speed,     4),
        ])

        # ── Log console (1x/seconde à 30Hz) ──
        self._log_counter += 1
        if self._log_counter % 30 == 0:
            self.get_logger().info(
                f'Offset: {center_offset:+.2f} | '
                f'Heading: {math.degrees(heading_error):+.1f}° | '
                f'Steer: {math.degrees(self.steering):+.1f}° | '
                f'Speed: {self.speed:.2f}m/s'
            )

    # ================================================================
    #  Envoi commande moteur
    # ================================================================
    def _send(self, speed: float, steering: float):
        msg = AckermannDriveStamped()
        msg.header.stamp       = Clock().now().to_msg()
        msg.drive.speed        = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub.publish(msg)

    # ================================================================
    #  Sauvegarde CSV toutes les 5s
    # ================================================================
    def _save_log(self):
        if not self.log_buffer:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename  = os.path.join(self.log_dir, f"log_{timestamp}.csv")

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time", "center_offset", "heading_error",
                "lateral_corr", "delta_raw", "steering", "speed"
            ])
            writer.writerows(self.log_buffer)

        self.get_logger().info(f'Log sauvegardé : {len(self.log_buffer)} frames → {filename}')
        self.log_buffer = []


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # STOP propre à l'arrêt
        node._send(speed=0.0, steering=0.0)
        node.get_logger().info('Ctrl+C — STOP envoyé.')
        node._save_log()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# """
# Pure Pursuit Node — Exécution moteur

# RÔLE : Reçoit un point cible → calcule angle de braquage → envoie aux moteurs

#     /control/target_point ──→ [Pure Pursuit] ──→ /autonomous/ackermann_cmd

# Ce node ne sait RIEN de la caméra ni des lignes.
# Il reçoit juste "va vers ce point (x, y)" et calcule comment tourner.

# Formule Pure Pursuit :
#     δ = arctan(2 * L * sin(α) / ld)

#     L  = empattement (wheelbase)
#     α  = angle vers le point cible
#     ld = distance au point cible

# PLUS TARD : remplacer par Stanley Controller (même input, autre formule)
# """

# import rclpy
# from rclpy.node import Node
# from ackermann_msgs.msg import AckermannDriveStamped
# from std_msgs.msg import Float32MultiArray, Bool, Float32
# import math
# import numpy as np


# class PurePursuitNode(Node):

#     def __init__(self):
#         super().__init__('pure_pursuit_node')

#         # ── Paramètres véhicule ──
#         self.declare_parameter('wheelbase', 0.3)            # mètres
#         self.declare_parameter('max_steering_angle', 30.0)  # degrés

#         # ── Paramètres de contrôle ──
#         self.declare_parameter('steering_gain', 1.0)
#         self.declare_parameter('control_rate', 20.0)        # Hz
#         self.declare_parameter('no_data_timeout', 0.5)      # secondes

#         # ── Variables ──
#         self.wheelbase = self.get_parameter('wheelbase').value
#         self.target_point = None     # (x, y) en mètres
#         self.suggested_speed = 0.0
#         self.data_valid = False
#         self.last_data_time = None
#         self._log_counter = 0
#         self.obstacle_stop = False
#         self.obstacle_distance_m = -1.0

#         # ── Subscriber : point cible du Controller ──
#         self.target_sub = self.create_subscription(
#             Float32MultiArray,
#             '/control/target_point',
#             self.target_callback,
#             10
#         )
#         self.obstacle_sub = self.create_subscription(
#             Bool,
#             '/safety/obstacle_stop',
#             self.obstacle_stop_callback,
#             10
#         )
#         self.obstacle_dist_sub = self.create_subscription(
#             Float32,
#             '/safety/obstacle_distance_m',
#             self.obstacle_distance_callback,
#             10
#         )

#         # ── Publisher : commandes moteur ──
#         self.cmd_pub = self.create_publisher(
#             AckermannDriveStamped,
#             '/autonomous/ackermann_cmd',
#             10
#         )

#         # ── Timer ──
#         rate = self.get_parameter('control_rate').value
#         self.timer = self.create_timer(1.0 / rate, self.control_loop)

#         self.get_logger().info('=== Pure Pursuit Node ===')
#         self.get_logger().info(f'Wheelbase: {self.wheelbase}m')
#         self.get_logger().info('Écoute : /control/target_point')
#         self.get_logger().info('Écoute sécurité : /safety/obstacle_stop')
#         self.get_logger().info('Publie : /autonomous/ackermann_cmd')

#     # ================================================================
#     #  Callback : réception du point cible
#     # ================================================================
#     def target_callback(self, msg):
#         """
#         Reçoit [valid, target_x, target_y, center_offset, heading_error, speed].
#         """
#         data = msg.data

#         if len(data) < 6 or data[0] < 0.5:
#             self.data_valid = False
#             return

#         self.target_point = np.array([data[1], data[2]])
#         self.suggested_speed = data[5]
#         self.data_valid = True
#         self.last_data_time = self.get_clock().now()

#     def obstacle_stop_callback(self, msg):
#         self.obstacle_stop = bool(msg.data)

#     def obstacle_distance_callback(self, msg):
#         self.obstacle_distance_m = float(msg.data)

#     # ================================================================
#     #  Envoyer une commande Ackermann
#     # ================================================================
#     def send_command(self, speed, steering_angle):
#         """Publie une commande Ackermann."""
#         cmd = AckermannDriveStamped()
#         cmd.header.stamp = self.get_clock().now().to_msg()
#         cmd.header.frame_id = 'base_link'
#         cmd.drive.speed = float(speed)
#         cmd.drive.steering_angle = float(steering_angle)
#         self.cmd_pub.publish(cmd)

#     # ================================================================
#     #  Pure Pursuit : calcul du steering
#     # ================================================================
#     def compute_steering(self, target_x, target_y):
#         """
#         Formule Pure Pursuit.

#         Args:
#             target_x: distance avant (mètres)
#             target_y: décalage latéral (mètres, + = gauche)

#         Returns:
#             Angle de braquage (radians, + = gauche)
#         """
#         ld = math.sqrt(target_x**2 + target_y**2)

#         if ld < 1e-6:
#             return 0.0

#         # Angle vers le point cible
#         alpha = math.atan2(target_y, target_x)

#         # Formule Pure Pursuit
#         steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)

#         return steering

#     # ================================================================
#     #  Boucle de contrôle (20 Hz)
#     # ================================================================
#     def control_loop(self):
#         """Boucle principale : calcule et envoie la commande."""

#         # Timeout ?
#         timeout = self.get_parameter('no_data_timeout').value
#         if self.last_data_time is not None:
#             elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
#             if elapsed > timeout:
#                 self.data_valid = False

#         # Pas de données → STOP
#         if not self.data_valid or self.target_point is None:
#             self.send_command(0.0, 0.0)
#             return

#         # Sécurité obstacle → STOP prioritaire
#         if self.obstacle_stop:
#             self.send_command(0.0, 0.0)
#             self._log_counter += 1
#             if self._log_counter % 20 == 0:
#                 self.get_logger().warn(
#                     f'STOP obstacle actif (distance={self.obstacle_distance_m:.2f}m)'
#                 )
#             return

#         # Calcul du steering
#         steering = self.compute_steering(self.target_point[0], self.target_point[1])

#         # Appliquer le gain
#         gain = self.get_parameter('steering_gain').value
#         steering *= gain

#         # Limiter l'angle
#         max_steer = math.radians(self.get_parameter('max_steering_angle').value)
#         steering = float(np.clip(steering, -max_steer, max_steer))

#         # Vitesse = suggestion du controller
#         speed = self.suggested_speed

#         # Envoyer
#         self.send_command(speed, steering)

#         # Log
#         self._log_counter += 1
#         if self._log_counter % 50 == 0:
#             self.get_logger().info(
#                 f'Target: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})m | '
#                 f'Steering: {math.degrees(steering):+.1f}° | '
#                 f'Speed: {speed:.2f} m/s'
#             )


# def main(args=None):
#     rclpy.init(args=args)
#     node = PurePursuitNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.send_command(0.0, 0.0)
#         node.get_logger().info('STOP envoyé')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


# ## Résumé du contrôleur

# # Toutes les 50ms (20 Hz) :

# # 1. Est-ce que j'ai des données de voie ?
# #    NON → STOP
# #    OUI → continue

# # 2. Je suis décalé de combien ? (center_offset)

# # 3. Je calcule l'angle pour corriger

# # 4. Je calcule la vitesse (plus lent si virage)

# # 5. J'envoie la commande aux moteurs via /autonomous/ackermann_cmd




# ## Résumé du contrôleur

# #Toutes les 50ms (20 Hz) :

# #1. Est-ce que j'ai des données de voie ?
# #   NON → STOP
# #   OUI → continue

# #2. Je suis décalé de combien ? (center_offset)

# #3. Je calcule l'angle pour corriger

# #4. Je calcule la vitesse (plus lent si virage)

# #5. J'envoie la commande aux moteurs

#!/usr/bin/env python3

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.clock import Clock
# from ackermann_msgs.msg import AckermannDriveStamped
# from std_msgs.msg import Float32MultiArray, Bool

# class PurePursuitNode(Node):

#     def __init__(self):
#         super().__init__('pure_pursuit_node')

#         self.declare_parameter('v_max', 0.5)     # maximale Geschwindigkeit
#         self.declare_parameter('v_min', 0.1)     # minimale Geschwindigkeit
#         self.declare_parameter('gamma', 2.5)     # Geschwindigkeitsskalierung
#         self.declare_parameter('wheel_base', 0.36)
#         self.declare_parameter('steer_smoothing', 0.1)  # Glättungsfaktor Lenkwinkel
#         self.declare_parameter('steering_gain', 1.0)
#         self.declare_parameter('max_steer', 0.4)  # rad
#         self.declare_parameter('no_data_timeout', 0.5)      # secondes
#         self.load_params()
#         self.pub = self.create_publisher( AckermannDriveStamped, '/autonomous/ackermann_cmd', 10)
#         self.lane_data_sub = self.create_subscription(Float32MultiArray,'/control/target_point',self.updateParameter, 10)
#         self.obstacle_sub = self.create_subscription(Bool,'/safety/obstacle_stop',self.obstacle_stop_callback,10)
#         # self.lane_data_sub = self.create_subscription(Float32MultiArray,'/lane_detection/lane_data',self.updateParameter, 10)
#         # Fahrparameter
#         self.speed = 0.2         # m/s  (ANPASSEN!)
#         self.steering = 0.0       # geradeaus

#         self.center_offset = None
#         self.angle = None

        
#         # ── Variables ──
#         self.wheelbase = self.get_parameter('wheel_base').value
#         self.target_point = None     # (x, y) en mètres
#         self.data_valid = False
#         self.last_data_time = None
#         self._log_counter = 0
#         self.obstacle_stop = False
#         self.obstacle_distance_m = -1.0

#         # 20 Hz senden (wichtig: kontinuierlich!)
#         #self.timer = self.create_timer(0.05, self.send_cmd)

#         self.get_logger().info("Autonomous Drive Node  is started. Ensure to be in Autonous Mode. Use Joystick or RC")
        
#         self.get_logger().info('=== Pure Pursuit Node ===')
#         self.get_logger().info(f'Wheelbase: {self.wheelbase}m')
#         self.get_logger().info('Écoute : /control/target_point')
#         self.get_logger().info('Écoute sécurité : /safety/obstacle_stop')


#     def load_params(self):
#         self.v_max = self.get_parameter('v_max').value
#         self.v_min = self.get_parameter('v_min').value
#         self.gamma = self.get_parameter('gamma').value
#         self.wheel_base = self.get_parameter('wheel_base').value
#         self.steer_smoothing = self.get_parameter('steer_smoothing').value
#         self.max_steer = self.get_parameter('max_steer').value

#     def stanley_adaptive(self, v):
#         eps = 0.1
#         K_eff = (abs(self.center_offset) + abs(self.angle)) /(v + eps)
#         steering_angle = self.angle + np.arctan2(K_eff*self.center_offset, v+eps)
            
#         # Application du gain manuel
#         steering_gain = self.get_parameter('steering_gain').value
#         steering_angle = steering_angle * steering_gain     

#         # Clamp
#         steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
        
#         # Smooth
#         self.steering = ((1 - self.steer_smoothing) * self.steering +
#                          self.steer_smoothing * steering_angle)
#         return self.steering

#     def adaptive_speed(self):
#         """
#         Geschwindigkeit passt sich automatisch der Situation an
#         """
#         # "Schwierigkeitsindex" basierend auf center_offset und angle
#         R = np.clip(abs(self.center_offset) + abs(self.angle), 0.0, 1.0)
#         # Geschwindigkeit nimmt ab bei großem Fehler
#         v_ref = self.v_max * np.exp(-self.gamma * R)
#         return max(self.v_min, min(self.v_max, v_ref))

#     def updateParameter(self,msg):
#         if len(msg.data) < 6 or msg.data[0] < 0.5:
#             self.get_logger().info("Error by updating value : No value was received")
#             return
        
#         self.target_point = np.array([msg.data[1], msg.data[2]])
#         self.speed = msg.data[5]
#         self.data_valid = True
#         self.last_data_time = self.get_clock().now()

#         # J'ai modifie a partir d'ici
#         new_offset = -msg.data[3]
#         new_angle = -msg.data[4]
        
#         # Ne recalculer que si les données ont changé
#         if (self.center_offset is not None and 
#             abs(new_offset - self.center_offset) < 1e-6 and 
#             abs(new_angle - self.angle) < 1e-6):
#             # Mêmes données → juste republier le dernier steering
#             self.send_cmd_cached()
#             return
        
#         self.center_offset = new_offset
#         self.angle = new_angle
#         self.send_cmd()
#         # Jusqu'ici

#     def obstacle_stop_callback(self, msg):
#         self.obstacle_stop = bool(msg.data)

#     def obstacle_distance_callback(self, msg):
#         self.obstacle_distance_m = float(msg.data)

#     def send_cmd(self):
#         msg = AckermannDriveStamped()
#         msg.header.stamp = Clock().now().to_msg()
        
#         if self.center_offset is None or self.angle is None:
#             msg.drive.speed = 0.0
#             msg.drive.steering_angle = 0.0
#             self.pub.publish(msg)
#             return

#         # Timeout check
#         timeout = self.get_parameter('no_data_timeout').value
#         if self.last_data_time is not None:
#             elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
#             if elapsed > timeout:
#                 self.data_valid = False
#                 msg.drive.speed = 0.0
#                 msg.drive.steering_angle = 0.0
#                 self.pub.publish(msg)
#                 return

#         # Obstacle safety stop
#         if self.obstacle_stop:
#             msg.drive.speed = 0.0
#             msg.drive.steering_angle = 0.0
#             self.pub.publish(msg)
#             return

#         # ── Compute steering FIRST, then publish ──
#         #self.steering = - self.stanley_adaptive(self.speed)
#         self.stanley_adaptive(self.speed)
#         v_ref = self.adaptive_speed()
#         self.speed += 0.4 * (v_ref - self.speed)

#         msg.drive.speed = float(self.speed)
#         msg.drive.steering_angle = float(self.steering)
#         self.get_logger().info(f"vitesse actuelle : {msg.drive.speed } | angle de braquage : {msg.drive.steering_angle} ")
#         self.pub.publish(msg)

#     def send_cmd_cached(self):
#             """Republier la dernière commande sans recalculer le steering."""
#             msg = AckermannDriveStamped()
#             msg.header.stamp = Clock().now().to_msg()
#             msg.drive.speed = float(self.speed)
#             msg.drive.steering_angle = float(self.steering)
#             self.pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = PurePursuitNode()
#         rclpy.spin(node)

#     except KeyboardInterrupt:
#         pass
#     finally:
#         if 'node' in locals():
#             node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
