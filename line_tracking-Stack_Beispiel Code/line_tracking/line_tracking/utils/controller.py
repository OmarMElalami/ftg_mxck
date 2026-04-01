#!/usr/bin/env python3
"""
Module de contrôle Pure Pursuit.
Implémente l'algorithme de suivi de trajectoire.
"""

import math
import numpy as np


class PurePursuitController:
    """
    Contrôleur Pure Pursuit pour le suivi de trajectoire.
    """
    
    def __init__(self, lookahead_distance=1.0, wheelbase=0.3):
        """
        Initialisation du contrôleur.
        
        Args:
            lookahead_distance: Distance de lookahead (mètres)
            wheelbase: Empattement du véhicule (mètres)
        """
        self.lookahead_distance = lookahead_distance  # Distance de regard (mètres)
        self.wheelbase = wheelbase                     # Empattement (mètres)
    
    def calculate_steering_angle(self, current_pose, target_point):
        """
        Calcule l'angle de braquage nécessaire.
        
        Args:
            current_pose: Pose actuelle (x, y, theta)
            target_point: Point cible (x, y)
            
        Returns:
            Angle de braquage en radians
        """
        # Calcul de l'angle vers le point cible
        # Distance en X et Y vers le point cible
        dx = target_point[0] - current_pose[0]
        dy = target_point[1] - current_pose[1]
        
        # Angle absolu vers le target
        target_angle = math.atan2(dy, dx)
        
        # Différence d'angle (où je regarde vs où je dois aller)
        alpha = target_angle - current_pose[2]
        
        # Normalisation de l'angle entre -pi et pi
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        
        # Formule de Pure Pursuit
        steering_angle = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            self.lookahead_distance
        )
        
        return steering_angle
    
    def find_lookahead_point(self, path, current_pose):
        """
        Trouve le point de lookahead sur la trajectoire.
        
        Args:
            path: Trajectoire (liste de points)
            current_pose: Pose actuelle
            
        Returns:
            Point de lookahead ou None
        """
        if path is None or len(path) == 0:
            return None
        
        # Recherche du point le plus proche à lookahead_distance
        min_dist_diff = float('inf')
        lookahead_point = None
        
        for point in path:
            # Distance entre robot et ce point
            dx = point[0] - current_pose[0]
            dy = point[1] - current_pose[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            # Est-ce proche de la distance de lookahead ?
            dist_diff = abs(dist - self.lookahead_distance)
            
            if dist_diff < min_dist_diff:
                min_dist_diff = dist_diff
                lookahead_point = point
        
        return lookahead_point
    
    def calculate_curvature(self, steering_angle):
        """
        Calcule la courbure à partir de l'angle de braquage.
        
        Args:
            steering_angle: Angle de braquage (radians)
            
        Returns:
            Courbure (1/rayon)
        """
        if abs(steering_angle) < 1e-6:
            return 0.0                                      # Ligne droite
        return math.tan(steering_angle) / self.wheelbase
    
    def adaptive_lookahead(self, velocity, min_distance=0.5, max_distance=2.0):
        """
        Ajuste dynamiquement la distance de lookahead en fonction de la vitesse.
        
        Args:
            velocity: Vitesse actuelle du véhicule
            min_distance: Distance minimale
            max_distance: Distance maximale
            
        Returns:
            Distance de lookahead ajustée
        """
        # Augmente le lookahead avec la vitesse
        lookahead = min_distance + velocity * 0.5
        return np.clip(lookahead, min_distance, max_distance)
    
    def calculate_turn_radius(self, steering_angle):
        """
        Calcule le rayon de braquage.
        
        Args:
            steering_angle: Angle de braquage (radians)
            
        Returns:
            Rayon de braquage (mètres)
        """
        if abs(steering_angle) < 1e-6:
            return float('inf')
        return self.wheelbase / math.tan(steering_angle)


## Résumé de Pure Pursuit

#1. Où suis-je ?          → current_pose (x, y, angle)
#2. Où dois-je aller ?    → trouver lookahead_point sur la trajectoire
#3. Combien tourner ?     → calculer steering_angle avec la formule
#4. Envoyer la commande   → steering_angle → moteurs