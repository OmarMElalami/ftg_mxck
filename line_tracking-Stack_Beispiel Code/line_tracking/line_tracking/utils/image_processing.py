#!/usr/bin/env python3
"""
Boite a outils OpenCV pour la detection de voie.
Utilise par lane_detector_node.

Pipeline (v5 — Gauss + CLAHE + HSV + Contours) :
  Image BGR -> GaussianBlur -> CLAHE -> HSV mask -> Morpho -> Contours -> Selection -> Points

  Sortie par ligne : 4 points echantillonnes de bas en haut (proche -> loin du robot)
  Point 1 = bas du blob  (plus proche robot)
  Point 2 = 1/3 hauteur
  Point 3 = 2/3 hauteur
  Point 4 = haut du blob (plus loin robot)
"""

import cv2
import numpy as np


class ImageProcessor:

    def __init__(self, image_width=640, image_height=360, logger=None):
        self.image_width  = image_width
        self.image_height = image_height
        self.logger       = logger
        self.camera_matrix = None
        self.dist_coeffs   = None

        # ── ROI ──
        self.roi_top_ratio        = 0.55
        self.roi_bottom_cut_ratio = 0.22

        # ── Gauss + CLAHE ──
        self.gauss_kernel     = (3, 3)
        self.clahe_clip_limit = 2.0
        self.clahe_tile_grid  = (8, 8)

        # ── Masque HSV blanc ──
        self.hsv_lower      = np.array([0,   0, 155])
        self.hsv_upper      = np.array([180, 60, 255])
        self.gray_threshold = 200

        # ── Morphologie ──
        self.min_blob_area = 400

        # ── Selection intelligente ──
        self.lane_width_min_px = 80
        self.lane_width_max_px = 610

        # ── Nb points par ligne ──
        self.n_points = 4

        # ── Largeur estimee piste (fallback mode dominante) ──
        self.expected_lane_width_px = 381.0

        # ── Lissage temporel du centre ──
        self.cx_history    = []
        self.history_length = 5

        # ── Compteurs ──
        self.consecutive_failures  = 0
        self.max_consecutive_failures = 20

    # ================================================================
    #  Pipeline principal
    # ================================================================
    def detect_lanes(self, img_bgr):
        """
        Pipeline complet.

        Retourne :
            left_pts   : np.array (4,2) points ligne gauche [(x,y),...] ou None
            right_pts  : np.array (4,2) points ligne droite ou None
            mode       : 'paire' | 'dominante' | 'aucune'
            cx_lane    : int, centre du couloir en pixels (lisse)
            debug_img  : image BGR annotee
        """
        img = cv2.resize(img_bgr, (self.image_width, self.image_height))
        H, W = self.image_height, self.image_width

        # 1. ROI
        roi_top    = int(H * self.roi_top_ratio)
        roi_bottom = int(H * (1.0 - self.roi_bottom_cut_ratio))
        roi = img[roi_top:roi_bottom, :]

        # 2. Gauss
        blurred = cv2.GaussianBlur(roi, self.gauss_kernel, 0)

        # 3. CLAHE
        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=self.clahe_clip_limit,
                                 tileGridSize=self.clahe_tile_grid)
        l   = clahe.apply(l)
        lab = cv2.merge((l, a, b))
        blurred = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # 4. Masque HSV blanc
        hsv      = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # 5. Masque gris backup
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        _, mask_gray = cv2.threshold(gray, self.gray_threshold, 255, cv2.THRESH_BINARY)

        # 6. Fusion
        combined = cv2.bitwise_or(mask_hsv, mask_gray)

        # 7. Nettoyage morphologique
        k3 = np.ones((3, 3), np.uint8)
        k5 = np.ones((5, 5), np.uint8)
        cleaned = cv2.morphologyEx(combined, cv2.MORPH_OPEN,  k3, iterations=1)
        cleaned = cv2.morphologyEx(cleaned,  cv2.MORPH_CLOSE, k5, iterations=3)

        # 8. Extraction blobs valides
        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_mask  = np.zeros_like(cleaned)
        valid_cnts  = []
        for cnt in contours:
            if cv2.contourArea(cnt) > self.min_blob_area:
                cv2.drawContours(white_mask, [cnt], -1, 255, -1)
                valid_cnts.append(cnt)

        # 9. Selection intelligente
        blobs, left_blob, right_blob, mode = self._select_lane_lines(valid_cnts, roi_top)

        # 10. Extraire 4 points par ligne
        left_pts  = self._extract_line_points(left_blob,  roi_top, roi_bottom)
        right_pts = self._extract_line_points(right_blob, roi_top, roi_bottom)

        # 11. Mode dominante : estimer la ligne manquante
        # if mode == 'dominante' and left_pts is not None:
        #     right_pts = self._estimate_parallel(left_pts, offset=+self.expected_lane_width_px)
        if mode == 'dominante' and left_pts is not None:
            cx_blob = float(left_pts[0, 0])   # x du point bas de la ligne détectée
            if cx_blob < self.image_width / 2:
                # Ligne gauche détectée → estimer la droite à droite
                right_pts = self._estimate_parallel(left_pts, offset=+self.expected_lane_width_px)
            else:
                # Ligne droite détectée → elle est dans left_pts par erreur → corriger
                right_pts = left_pts.copy()
                left_pts  = self._estimate_parallel(right_pts, offset=-self.expected_lane_width_px)
        
        # 12. Centre du couloir
        cx_lane = self._compute_center(left_pts, right_pts, mode, W)

        # 13. Image debug
        debug_img = self._draw_debug(
            img, roi_top, roi_bottom, white_mask, valid_cnts,
            left_pts, right_pts, left_blob, right_blob, mode, cx_lane, W, H
        )

        if self.logger:
            if mode == 'aucune':
                self.consecutive_failures += 1
                if self.consecutive_failures % 10 == 1:
                    self.logger.warn(
                        f'detect_lanes: aucune ligne ({self.consecutive_failures} echecs)')
            else:
                self.consecutive_failures = 0

        return left_pts, right_pts, mode, cx_lane, debug_img

    # ================================================================
    #  Selection intelligente des lignes
    # ================================================================
    def _select_lane_lines(self, valid_cnts, roi_top):
        if len(valid_cnts) == 0:
            return [], None, None, 'aucune'

        blobs = []
        for cnt in valid_cnts:
            area      = cv2.contourArea(cnt)
            bottom_pt = tuple(cnt[cnt[:, :, 1].argmax()][0])
            bottom_pt = (bottom_pt[0], bottom_pt[1] + roi_top)
            M  = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00']) if M['m00'] > 0 else bottom_pt[0]
            cy = int(M['m01'] / M['m00']) + roi_top if M['m00'] > 0 else bottom_pt[1]
            blobs.append({
                'cnt':       cnt,
                'area':      area,
                'bottom_pt': bottom_pt,
                'centroid':  (cx, cy),
            })

        blobs.sort(key=lambda b: b['bottom_pt'][0])

        best_pair  = None
        best_score = -1
        for i in range(len(blobs)):
            for j in range(i + 1, len(blobs)):
                dist = abs(blobs[j]['bottom_pt'][0] - blobs[i]['bottom_pt'][0])
                if self.lane_width_min_px <= dist <= self.lane_width_max_px:
                    #score = blobs[i]['area'] + blobs[j]['area']
                    # APRÈS (favoriser les paires dont les blobs sont proches du bas de la ROI)
                    # Les vraies lignes de piste sont toujours proches du bas de l'image
                    # Les reflets/murs sont en haut
                    score = (blobs[i]['area'] + blobs[j]['area']) * \
                            (blobs[i]['bottom_pt'][1] + blobs[j]['bottom_pt'][1]) / (2 * self.image_height)
                    if score > best_score:
                        best_score = score
                        best_pair  = (blobs[i], blobs[j])

        # if best_pair is not None:
        #     # Mettre a jour la largeur attendue
        #     dist = abs(best_pair[1]['bottom_pt'][0] - best_pair[0]['bottom_pt'][0])
        #     self.expected_lane_width_px = 0.9 * self.expected_lane_width_px + 0.1 * dist
        #     return blobs, best_pair[0], best_pair[1], 'paire'
        if best_pair is not None:
            dist = abs(best_pair[1]['bottom_pt'][0] - best_pair[0]['bottom_pt'][0])
            # ── Cohérence temporelle ──
            # Calculer le centre de la paire candidate
            cx_pair = (best_pair[0]['bottom_pt'][0] + best_pair[1]['bottom_pt'][0]) / 2.0
            # Si on a un historique et que le saut est trop grand -> mauvais appariement
            if self.cx_history and abs(cx_pair - self.cx_history[-1]) > 120:
                # Rejeter la paire — basculer en dominante
                # Prendre le blob dont le cx est le plus proche du centre précédent
                dominant = min(blobs, key=lambda b: abs(b['bottom_pt'][0] - self.cx_history[-1]))
                return blobs, dominant, None, 'dominante'
            # Paire validée
            self.expected_lane_width_px = 0.9 * self.expected_lane_width_px + 0.1 * dist
            return blobs, best_pair[0], best_pair[1], 'paire'

        dominant = max(blobs, key=lambda b: b['area'])
        return blobs, dominant, None, 'dominante'

    # ================================================================
    #  Extraction de 4 points par ligne (bas -> haut)
    # ================================================================
    def _extract_line_points(self, blob, roi_top, roi_bottom):
        """
        Echantillonne n_points points sur le contour du blob,
        de bas en haut (proche robot -> loin robot).

        Retourne np.array shape (n_points, 2) en coordonnees image complete,
        ou None si blob est None.
        """
        if blob is None:
            return None

        cnt = blob['cnt']

        # Recuperer tous les points du contour en coords image complete
        pts = cnt.reshape(-1, 2)
        pts[:, 1] += roi_top  # shift ROI -> image complète

        # Trier par y decroissant (y grand = bas image = proche robot)
        pts = pts[pts[:, 1].argsort()[::-1]]

        # Echantillonner n_points indices regulierement espaces
        indices = np.linspace(0, len(pts) - 1, self.n_points, dtype=int)
        sampled = pts[indices]

        # Pour chaque tranche de hauteur, prendre le x median
        # -> plus robuste que le premier point du contour
        result = []
        y_min = pts[:, 1].min()
        y_max = pts[:, 1].max()
        y_levels = np.linspace(y_max, y_min, self.n_points)  # bas -> haut

        for y_target in y_levels:
            band = 15  # pixels de tolerance
            mask = np.abs(pts[:, 1] - y_target) < band
            if mask.sum() > 0:
                x_median = int(np.median(pts[mask, 0]))
                result.append([x_median, int(y_target)])
            else:
                # Fallback : point le plus proche en y
                idx = np.argmin(np.abs(pts[:, 1] - y_target))
                result.append([int(pts[idx, 0]), int(y_target)])

        return np.array(result, dtype=np.float32)

    # ================================================================
    #  Estimation ligne parallele (mode dominante)
    # ================================================================
    def _estimate_parallel(self, pts, offset):
        """
        Decale les points d'une ligne de 'offset' pixels en X
        pour estimer la ligne opposee.
        offset > 0 = vers la droite, offset < 0 = vers la gauche
        """
        estimated = pts.copy().astype(np.float32)
        estimated[:, 0] = np.clip(
            estimated[:, 0] + int(offset),
            0, self.image_width - 1
        )
        return estimated

    # ================================================================
    #  Centre du couloir (lisse temporellement)
    # ================================================================
    def _compute_center(self, left_pts, right_pts, mode, W):
        if left_pts is not None and right_pts is not None:
            # Moyenne des x du point bas (index 0 = plus proche robot)
            cx_raw = int((left_pts[0, 0] + right_pts[0, 0]) / 2)
        elif left_pts is not None:
            cx_raw = int(left_pts[0, 0])
        elif right_pts is not None:
            cx_raw = int(right_pts[0, 0])
        else:
            return self.cx_history[-1] if self.cx_history else W // 2

        self.cx_history.append(cx_raw)
        if len(self.cx_history) > self.history_length:
            self.cx_history.pop(0)
        return int(np.mean(self.cx_history))

    # ================================================================
    #  Visualisation debug
    # ================================================================
    def _draw_debug(self, img, roi_top, roi_bottom, white_mask, valid_cnts,
                    left_pts, right_pts, left_blob, right_blob,
                    mode, cx_lane, W, H):
        out = img.copy()

        # Zone ROI
        ov = out.copy()
        cv2.rectangle(ov, (0, roi_top), (W, roi_bottom), (0, 40, 0), -1)
        out = cv2.addWeighted(ov, 0.12, out, 0.88, 0)

        # Overlay bleu zones blanches
        mask_full = np.zeros((H, W), dtype=np.uint8)
        mask_full[roi_top:roi_bottom, :] = white_mask
        blue_ov = np.zeros_like(out)
        blue_ov[mask_full > 0] = (255, 80, 0)
        out = cv2.addWeighted(out, 1.0, blue_ov, 0.35, 0)

        # Contours cyan tous les blobs
        for cnt in valid_cnts:
            cnt_s = cnt + np.array([[[0, roi_top]]])
            cv2.drawContours(out, [cnt_s], -1, (0, 255, 255), 1)

        # Contours epais lignes selectionnees
        if left_blob:
            cnt_s = left_blob['cnt'] + np.array([[[0, roi_top]]])
            cv2.drawContours(out, [cnt_s], -1, (0, 255, 255), 3)
        if right_blob:
            cnt_s = right_blob['cnt'] + np.array([[[0, roi_top]]])
            cv2.drawContours(out, [cnt_s], -1, (255, 255, 0), 3)

        # Points echantillonnes — ligne gauche (cyan) ligne droite (jaune)
        for pts, color, label in [
            (left_pts,  (0, 255, 100),   'G'),
            (right_pts, (255, 200,   0), 'D'),
        ]:
            if pts is None:
                continue
            for i, (px, py) in enumerate(pts):
                px, py = int(px), int(py)
                cv2.circle(out, (px, py), 6, color, -1)
                cv2.circle(out, (px, py), 6, (255, 255, 255), 1)
                cv2.putText(out, f'{label}{i+1}', (px + 8, py - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            # Relier les points
            for i in range(len(pts) - 1):
                p1 = (int(pts[i][0]),   int(pts[i][1]))
                p2 = (int(pts[i+1][0]), int(pts[i+1][1]))
                cv2.line(out, p1, p2, color, 2)

        # Centre du couloir
        cy_center = (roi_top + roi_bottom) // 2
        cv2.circle(out, (cx_lane, cy_center), 10, (0, 165, 255), -1)
        cv2.putText(out, f'cx={cx_lane}', (cx_lane + 12, cy_center),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)

        # Lignes ROI
        cv2.line(out, (0, roi_top),    (W, roi_top),    (0, 200, 255), 1)
        cv2.line(out, (0, roi_bottom), (W, roi_bottom), (0, 100, 255), 1)

        # Banner
        mode_color = (0, 255, 100) if mode == 'paire' else (0, 130, 255)
        cv2.rectangle(out, (0, 0), (W, 50), (20, 20, 20), -1)
        cv2.putText(out,
                    f'Gauss+CLAHE+HSV | {len(valid_cnts)} blobs | MODE: {mode.upper()}',
                    (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.55, mode_color, 1)

        return out



# #!/usr/bin/env python3
# """
# Boîte à outils OpenCV pour la détection de voie.
# Utilisé par lane_detector_node.

# Pipeline (v4 — hsv_canny + mono-ligne robuste) :
#   Image BGR → GaussianBlur → Canny AND HSV_mask → Bird's eye → Sliding window → Poly fit

#   Mode hsv_canny (DÉFAUT) :
#     - Canny détecte les bords nets des lignes blanches
#     - Masque HSV confirme que ces bords appartiennent à du blanc
#     - bitwise_and → seuls les bords de lignes blanches passent
#     - Élimine les rayures du sol (bords Canny mais mauvais HSV)
#     - Élimine les reflets colorés (bon HSV mais pas de bord net)

#   Gestion mono-ligne renforcée :
#     - expected_lane_width_px calibrée depuis params (fixe, fiable)
#     - Détection du côté visible via histogram complet (pas demi-image)
#     - Fallback parallèle robuste basé sur la ligne détectée
# """

# import cv2
# import numpy as np


# class ImageProcessor:
#     """
#     Traitement d'images : détecte les 2 lignes blanches
#     et retourne leurs polynômes (supporte les courbes).
#     """

#     def __init__(self, image_width=640, image_height=360, logger=None):
#         self.image_width = image_width
#         self.image_height = image_height
#         self.logger = logger
#         self.camera_matrix = None
#         self.dist_coeffs = None

#         # ── Perspective transform (bird's eye view) ──
#         # Points calibrés sur la piste réelle (à ajuster via params.yaml)
#         self.src_points = np.float32([
#             [91,  250],
#             [249, 181],
#             [420, 181],
#             [573, 250],
#         ])
#         self.dst_points = np.float32([
#             [100, 360],
#             [100, 0],
#             [540, 0],
#             [540, 360],
#         ])
#         self._compute_warp_matrices()

#         # ── Sliding window ──
#         self.n_windows = 12
#         self.window_margin = 50
#         self.window_margin_max = 100
#         self.min_pixels = 20
#         # Largeur piste ~80cm, pixels_per_meter=905 → ~380px en bird's eye
#         # Sur dst [100→540] = 440px utiles, ~380px = ~86% des 440px
#         # Valeur initiale fixe — NE PAS mettre à jour si une seule ligne visible
#         self.expected_lane_width_px = 340.0   # calibrer sur image statique
#         self.lane_width_tolerance_ratio = 0.25
#         self.min_lane_width_px = image_width * 0.10
#         self.max_lane_width_px = image_width * 0.92
#         # Compteur de mises à jour de la largeur (seulement quand 2 lignes visibles)
#         self._width_update_count = 0

#         # ── ROI (appliquée AVANT le warp) ──
#         self.roi_top_ratio = 0.0       # 0.0 = pas de coupure (le warp fait déjà le travail)
#         self.roi_bottom_cut_ratio = 0.0
#         self.roi_side_cut_ratio = 0.0

#         # ── Canny thresholds ──
#         self.canny_low = 50
#         self.canny_high = 150

#         # ── Mode de prétraitement ──
#         # 'hsv_canny' = Canny ET masque HSV blanc → recommandé pour cette piste
#         # 'canny_only' = Canny seul (plus de bruit)
#         # 'hsv_only'   = HSV seul (moins précis sur les bords)
#         self.preprocess_mode = 'hsv_canny'

#         # ── CUDA init ──
#         self.use_cuda = False
#         self.gpu_blur = None
#         self.gpu_canny = None
#         self._cuda_preprocess_warned = False
#         try:
#             self.use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
#         except Exception:
#             self.use_cuda = False

#         if self.use_cuda:
#             try:
#                 self.gpu_blur = cv2.cuda.createGaussianFilter(
#                     cv2.CV_8UC3, cv2.CV_8UC3, (5, 5), 0)
#                 self.gpu_canny = cv2.cuda.createCannyEdgeDetector(
#                     low_thresh=self.canny_low, high_thresh=self.canny_high)
#             except Exception:
#                 self.use_cuda = False
#                 self.gpu_blur = None
#                 self.gpu_canny = None

#         # ── Nettoyage morphologique ──
#         self.min_component_area = 120

#         # ── Perspective adaptative ──
#         self.enable_dynamic_warp = False
#         self.dynamic_warp_period = 5
#         self._frame_counter = 0

#         # ── Lissage temporel ──
#         self.left_fit_history = []
#         self.right_fit_history = []
#         self.history_length = 5

#         # ── Dernier fit valide (fallback temporel) ──
#         self.last_left_fit = None
#         self.last_right_fit = None
#         self.allow_single_line_fallback = True
#         self.consecutive_failures = 0
#         self.max_consecutive_failures = 20

#     def _compute_warp_matrices(self):
#         self.M_warp = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
#         self.M_unwarp = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

#     def set_warp_points(self, src, dst):
#         self.src_points = np.float32(src)
#         self.dst_points = np.float32(dst)
#         self.initial_src_points = self.src_points.copy()
#         self.initial_dst_points = self.dst_points.copy()
#         self.consecutive_failures = 0
#         self._compute_warp_matrices()

#     # ================================================================
#     #  Pré-traitement
#     # ================================================================
#     def preprocess(self, image, hsv_lower=None, hsv_upper=None):
#         """
#         Mode 'hsv_canny' (DÉFAUT) : Canny AND masque HSV blanc
#         Mode 'canny_only'         : Canny seul
#         Mode 'hsv_only'           : masque HSV seul
#         """
#         mask = None

#         if self.use_cuda and self.gpu_blur is not None:
#             try:
#                 mask = self._preprocess_gpu(image, hsv_lower, hsv_upper)
#             except Exception as e:
#                 if self.logger and not self._cuda_preprocess_warned:
#                     self.logger.warn(f'CUDA preprocess fail ({str(e)}), fallback CPU')
#                 self._cuda_preprocess_warned = True
#                 self.use_cuda = False

#         if mask is None:
#             mask = self._preprocess_cpu(image, hsv_lower, hsv_upper)

#         mask = self._apply_roi_mask(mask)

#         # Morphologie adaptée au mode
#         if self.preprocess_mode == 'canny_only':
#             # Canny = contours fins → dilater légèrement pour le sliding window
#             dilate_kernel = np.ones((3, 3), np.uint8)
#             mask = cv2.dilate(mask, dilate_kernel, iterations=1)
#             close_kernel = np.ones((5, 3), np.uint8)
#             mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
#         elif self.preprocess_mode == 'hsv_canny':
#             # hsv_canny = bords fins et propres → dilater pour épaissir les contours
#             dilate_kernel = np.ones((3, 3), np.uint8)
#             mask = cv2.dilate(mask, dilate_kernel, iterations=1)
#             close_kernel = np.ones((5, 3), np.uint8)
#             mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
#         else:
#             # hsv_only = zones remplies → ouvrir pour supprimer bruit, fermer pour remplir trous
#             open_kernel = np.ones((3, 3), np.uint8)
#             mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
#             close_kernel = np.ones((7, 5), np.uint8)
#             mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

#         mask = self._remove_small_components(mask, min_area=self.min_component_area)

#         if self.logger:
#             white_pixels = cv2.countNonZero(mask)
#             if white_pixels == 0:
#                 self.logger.warn('preprocess: AUCUN pixel blanc')
#             elif white_pixels < 300:
#                 self.logger.warn(f'preprocess: peu de pixels ({white_pixels})')

#         return mask

#     def _preprocess_gpu(self, image, hsv_lower, hsv_upper):
#         gpu_img = cv2.cuda_GpuMat()
#         gpu_img.upload(image)
#         gpu_blurred = self.gpu_blur.apply(gpu_img)

#         if self.preprocess_mode == 'canny_only':
#             gpu_gray = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2GRAY)
#             return self.gpu_canny.detect(gpu_gray).download()

#         elif self.preprocess_mode == 'hsv_only':
#             lower = np.array(hsv_lower, dtype=np.uint8)
#             upper = np.array(hsv_upper, dtype=np.uint8)
#             gpu_hsv = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2HSV)
#             return cv2.cuda.inRange(gpu_hsv, lower, upper).download()

#         else:  # hsv_canny
#             lower = np.array(hsv_lower, dtype=np.uint8)
#             upper = np.array(hsv_upper, dtype=np.uint8)
#             gpu_gray = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2GRAY)
#             gpu_canny = self.gpu_canny.detect(gpu_gray)
#             gpu_hsv = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2HSV)
#             gpu_mask = cv2.cuda.inRange(gpu_hsv, lower, upper)
#             return cv2.cuda.bitwise_and(gpu_canny, gpu_mask).download()

#     def _preprocess_cpu(self, image, hsv_lower, hsv_upper):
#         blurred = cv2.GaussianBlur(image, (5, 5), 0)

#         if self.preprocess_mode == 'canny_only':
#             gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
#             return cv2.Canny(gray, self.canny_low, self.canny_high)

#         elif self.preprocess_mode == 'hsv_only':
#             hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#             lower = np.array(hsv_lower, dtype=np.uint8)
#             upper = np.array(hsv_upper, dtype=np.uint8)
#             return cv2.inRange(hsv, lower, upper)

#         else:  # hsv_canny — MODE RECOMMANDÉ POUR CETTE PISTE
#             gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
#             hsv  = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#             lower = np.array(hsv_lower, dtype=np.uint8)
#             upper = np.array(hsv_upper, dtype=np.uint8)
#             canny    = cv2.Canny(gray, self.canny_low, self.canny_high)
#             mask_hsv = cv2.inRange(hsv, lower, upper)
#             # Dilater le masque HSV pour "couvrir" les bords Canny qui tombent
#             # légèrement à l'extérieur de la zone blanche pure
#             mask_hsv_dilated = cv2.dilate(mask_hsv, np.ones((5, 5), np.uint8), iterations=1)
#             return cv2.bitwise_and(canny, mask_hsv_dilated)

#     def _apply_roi_mask(self, mask):
#         h, w = mask.shape[:2]
#         y_top    = int(h * self.roi_top_ratio)
#         y_bottom = int(h * (1.0 - self.roi_bottom_cut_ratio))
#         x_left   = int(w * self.roi_side_cut_ratio)
#         x_right  = int(w * (1.0 - self.roi_side_cut_ratio))
#         y_top    = np.clip(y_top, 0, h - 1)
#         y_bottom = np.clip(y_bottom, y_top + 1, h)
#         x_left   = np.clip(x_left, 0, w - 1)
#         x_right  = np.clip(x_right, x_left + 1, w)
#         roi = np.zeros((h, w), dtype=np.uint8)
#         roi[y_top:y_bottom, x_left:x_right] = 255
#         return cv2.bitwise_and(mask, roi)

#     def _remove_small_components(self, mask, min_area=100):
#         n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
#         cleaned = np.zeros_like(mask)
#         for i in range(1, n_labels):
#             if stats[i, cv2.CC_STAT_AREA] >= min_area:
#                 cleaned[labels == i] = 255
#         return cleaned

#     # ================================================================
#     #  Bird's eye view
#     # ================================================================
#     def warp_to_birdseye(self, image):
#         self._frame_counter += 1
#         should_adjust = (
#             self.enable_dynamic_warp
#             and (self._frame_counter % self.dynamic_warp_period == 0)
#             and not self._is_in_curve()
#         )
#         if should_adjust:
#             self.auto_adjust_warp_points(image)

#         result = cv2.warpPerspective(image, self.M_warp,
#                                      (self.image_width, self.image_height))
#         if self.logger:
#             if cv2.countNonZero(result) == 0:
#                 self.logger.warn('warp: AUCUN pixel après warp')
#         return result

#     def _is_in_curve(self):
#         if self.last_left_fit is None or self.last_right_fit is None:
#             return False
#         avg_curv = (abs(2.0 * self.last_left_fit[0]) + abs(2.0 * self.last_right_fit[0])) / 2.0
#         return avg_curv > 0.0005

#     def unwarp_from_birdseye(self, image):
#         return cv2.warpPerspective(image, self.M_unwarp,
#                                    (self.image_width, self.image_height))

#     def auto_adjust_warp_points(self, binary_mask):
#         if binary_mask is None or binary_mask.ndim != 2:
#             return False
#         h, w = binary_mask.shape[:2]
#         histogram = np.sum(binary_mask[int(h * 0.55):, :], axis=0)
#         midpoint = w // 2
#         left_base  = int(np.argmax(histogram[:midpoint]))
#         right_base = int(np.argmax(histogram[midpoint:]) + midpoint)
#         lane_width = right_base - left_base
#         if lane_width < self.min_lane_width_px or lane_width > self.max_lane_width_px:
#             return False
#         top_y    = int(h * 0.58)
#         bottom_y = int(h * 0.92)
#         band = 12
#         top_band    = binary_mask[max(0, top_y - band):min(h, top_y + band), :]
#         bottom_band = binary_mask[max(0, bottom_y - band):min(h, bottom_y + band), :]
#         top_nz = np.where(top_band > 0)
#         bot_nz = np.where(bottom_band > 0)
#         if len(top_nz[1]) < 60 or len(bot_nz[1]) < 60:
#             return False
#         top_x = top_nz[1]
#         bot_x = bot_nz[1]
#         left_top     = np.percentile(top_x[top_x < midpoint], 60) if np.any(top_x < midpoint) else left_base + lane_width * 0.25
#         right_top    = np.percentile(top_x[top_x > midpoint], 40) if np.any(top_x > midpoint) else right_base - lane_width * 0.25
#         left_bottom  = np.percentile(bot_x[bot_x < midpoint], 70) if np.any(bot_x < midpoint) else left_base
#         right_bottom = np.percentile(bot_x[bot_x > midpoint], 30) if np.any(bot_x > midpoint) else right_base
#         new_src = np.float32([
#             [np.clip(left_bottom,  0, w - 1), bottom_y],
#             [np.clip(left_top,     0, w - 1), top_y],
#             [np.clip(right_top,    0, w - 1), top_y],
#             [np.clip(right_bottom, 0, w - 1), bottom_y],
#         ])
#         top_width    = new_src[2, 0] - new_src[1, 0]
#         bottom_width = new_src[3, 0] - new_src[0, 0]
#         if bottom_width <= 0 or top_width <= 0:
#             return False
#         ratio = top_width / bottom_width
#         if ratio < 0.30 or ratio > 0.95:
#             return False
#         alpha = 0.12
#         self.src_points = (1.0 - alpha) * self.src_points + alpha * new_src
#         self._compute_warp_matrices()
#         return True

#     # ================================================================
#     #  Sliding window — version robuste mono-ligne
#     # ================================================================
#     def find_lane_pixels(self, binary_warped):
#         """
#         Sliding window robuste pour piste où une seule ligne est souvent visible.

#         Améliorations vs version précédente :
#         1. Histogram sur toute la largeur → détecte les 2 meilleurs pics
#            sans contrainte gauche/droite stricte
#         2. Si un seul pic fort → on sait quel côté est visible
#         3. Pas de calcul de width_samples si une seule ligne → pas de
#            largeur quasi-nulle qui bloque tout
#         4. Retourne quand même les pixels trouvés pour le fit mono-ligne
#         """
#         h, w = binary_warped.shape[:2]

#         # Histogram sur les 35% bas de l'image (zone la plus fiable)
#         histogram = np.sum(binary_warped[int(h * 0.65):, :], axis=0)

#         # ── Trouver les 2 meilleurs pics dans l'histogramme complet ──
#         left_base, right_base, left_strong, right_strong = \
#             self._find_two_peaks(histogram, w)

#         nonzero   = binary_warped.nonzero()
#         nonzero_y = np.array(nonzero[0])
#         nonzero_x = np.array(nonzero[1])

#         window_height  = h // self.n_windows
#         leftx_current  = left_base
#         rightx_current = right_base

#         left_lane_inds  = []
#         right_lane_inds = []
#         window_img = np.zeros((*binary_warped.shape, 3), dtype=np.uint8)
#         width_samples = []

#         left_margin  = self.window_margin
#         right_margin = self.window_margin

#         for win in range(self.n_windows):
#             win_y_low  = h - (win + 1) * window_height
#             win_y_high = h - win * window_height

#             win_xl_low  = max(leftx_current  - left_margin,  0)
#             win_xl_high = min(leftx_current  + left_margin,  w)
#             win_xr_low  = max(rightx_current - right_margin, 0)
#             win_xr_high = min(rightx_current + right_margin, w)

#             cv2.rectangle(window_img, (win_xl_low,  win_y_low),
#                           (win_xl_high, win_y_high), (0, 255, 0), 2)
#             cv2.rectangle(window_img, (win_xr_low,  win_y_low),
#                           (win_xr_high, win_y_high), (0, 0, 255), 2)

#             good_left = (
#                 (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
#                 (nonzero_x >= win_xl_low) & (nonzero_x < win_xl_high)
#             ).nonzero()[0]

#             good_right = (
#                 (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
#                 (nonzero_x >= win_xr_low) & (nonzero_x < win_xr_high)
#             ).nonzero()[0]

#             left_lane_inds.append(good_left)
#             right_lane_inds.append(good_right)

#             if len(good_left) > self.min_pixels:
#                 leftx_current = int(np.mean(nonzero_x[good_left]))
#                 left_margin   = self.window_margin
#             else:
#                 left_margin = min(left_margin + 15, self.window_margin_max)

#             if len(good_right) > self.min_pixels:
#                 rightx_current = int(np.mean(nonzero_x[good_right]))
#                 right_margin   = self.window_margin
#             else:
#                 right_margin = min(right_margin + 15, self.window_margin_max)

#             # width_samples : seulement si LES DEUX pics de départ étaient forts
#             if (left_strong and right_strong and
#                     len(good_left) > self.min_pixels and
#                     len(good_right) > self.min_pixels):
#                 width = rightx_current - leftx_current
#                 if width > 10:
#                     width_samples.append(width)

#         left_lane_inds  = np.concatenate(left_lane_inds)
#         right_lane_inds = np.concatenate(right_lane_inds)

#         # ── Mise à jour de expected_lane_width_px ──
#         # SEULEMENT si les deux lignes étaient vraiment détectées
#         if len(width_samples) >= 3 and left_strong and right_strong:
#             width_samples  = np.array(width_samples, dtype=np.float32)
#             median_width   = float(np.median(width_samples))
#             if self.min_lane_width_px <= median_width <= self.max_lane_width_px:
#                 # EMA conservative — ne pas laisser dériver trop vite
#                 self.expected_lane_width_px = (
#                     0.95 * self.expected_lane_width_px + 0.05 * median_width
#                 )
#                 self._width_update_count += 1
#                 if self.logger and self._width_update_count % 50 == 1:
#                     self.logger.info(
#                         f'Largeur piste calibrée : {self.expected_lane_width_px:.1f}px '
#                         f'(mise à jour #{self._width_update_count})'
#                     )
#             elif median_width < 10:
#                 # Largeur quasi-nulle = les deux fenêtres se chevauchent
#                 # → on ignore ce résultat mais ON NE BLOQUE PAS le pipeline
#                 if self.logger:
#                     self.logger.warn(
#                         f'sliding_window: largeur quasi-nulle ({median_width:.1f}px) '
#                         f'— ignorée, pipeline continue'
#                     )
#                 # On retourne quand même les pixels (mono-ligne possible)

#         # ── Log état détection ──
#         if self.logger:
#             nl = len(left_lane_inds)
#             nr = len(right_lane_inds)
#             if nl < 50 and nr < 50:
#                 self.logger.warn(f'sliding_window: G={nl} D={nr} pixels — faible')
#             elif nl < 50:
#                 self.logger.info(f'sliding_window: seule ligne DROITE ({nr}px)')
#             elif nr < 50:
#                 self.logger.info(f'sliding_window: seule ligne GAUCHE ({nl}px)')

#         return (nonzero_x[left_lane_inds], nonzero_y[left_lane_inds],
#                 nonzero_x[right_lane_inds], nonzero_y[right_lane_inds],
#                 window_img)

#     def _find_two_peaks(self, histogram, w):
#         """
#         Trouve les deux meilleurs pics dans l'histogramme complet.

#         Retourne (left_base, right_base, left_strong, right_strong)

#         Stratégie :
#           1. Chercher le pic global (ligne la plus forte)
#           2. Chercher le meilleur pic de l'autre côté (excluant une zone
#              autour du premier pic pour éviter de détecter le même bord)
#           3. Si un seul pic fort → positionner l'autre côté à ±expected_width

#         'strong' = True si le pic dépasse le seuil min (ligne vraiment détectée)
#         'strong' = False si position estimée (pas de ligne visible de ce côté)
#         """
#         MIN_PEAK_STRENGTH = 50   # seuil minimum pour considérer un pic comme valide

#         midpoint   = w // 2
#         left_half  = histogram[:midpoint]
#         right_half = histogram[midpoint:]

#         # Pic dans chaque moitié
#         l_idx = int(np.argmax(left_half))
#         r_idx = int(np.argmax(right_half)) + midpoint

#         l_strength = left_half[l_idx]
#         r_strength = right_half[r_idx - midpoint]

#         left_strong  = bool(l_strength  >= MIN_PEAK_STRENGTH)
#         right_strong = bool(r_strength >= MIN_PEAK_STRENGTH)

#         if left_strong and right_strong:
#             # Cas normal : deux lignes visibles
#             return l_idx, r_idx, True, True

#         elif left_strong and not right_strong:
#             # Seule ligne gauche visible → droite estimée à +expected_width
#             r_estimated = int(np.clip(
#                 l_idx + self.expected_lane_width_px, 0, w - 1
#             ))
#             if self.logger:
#                 self.logger.info(
#                     f'_find_two_peaks: seule gauche ({l_idx}px, force={l_strength:.0f}) '
#                     f'→ droite estimée à {r_estimated}px'
#                 )
#             return l_idx, r_estimated, True, False

#         elif right_strong and not left_strong:
#             # Seule ligne droite visible → gauche estimée à -expected_width
#             l_estimated = int(np.clip(
#                 r_idx - self.expected_lane_width_px, 0, w - 1
#             ))
#             if self.logger:
#                 self.logger.info(
#                     f'_find_two_peaks: seule droite ({r_idx}px, force={r_strength:.0f}) '
#                     f'→ gauche estimée à {l_estimated}px'
#                 )
#             return l_estimated, r_idx, False, True

#         else:
#             # Aucun pic fort — utiliser les positions par défaut
#             if self.logger:
#                 self.logger.warn(
#                     f'_find_two_peaks: aucun pic (G={l_strength:.0f} D={r_strength:.0f}) '
#                     f'→ positions par défaut'
#                 )
#             return w // 4, 3 * w // 4, False, False

#     def _find_peak(self, histogram, start, end):
#         """Conservé pour compatibilité mais remplacé par _find_two_peaks."""
#         region = histogram[start:end]
#         if len(region) == 0:
#             return None
#         peak_idx = np.argmax(region)
#         if region[peak_idx] < 50:
#             return None
#         return int(peak_idx + start)

#     # ================================================================
#     #  Polynomial fit — robuste mono-ligne
#     # ================================================================
#     def fit_polynomials(self, left_x, left_y, right_x, right_y):
#         """
#         Fit les polynômes des deux lignes.

#         Amélioration mono-ligne :
#         - Si une seule ligne détectée, l'autre est estimée par décalage parallèle
#         - La ligne estimée a les MÊMES coefficients a et b (même courbure)
#         - Seul le terme c (offset horizontal) change de ±expected_lane_width_px
#         - Le lissage temporel s'applique aux deux (stabilité)
#         """
#         min_points = 50
#         left_valid  = len(left_x) > min_points
#         right_valid = len(right_x) > min_points

#         left_fit  = None
#         right_fit = None

#         if left_valid:
#             weights_l = left_y / (np.max(left_y) + 1e-6)
#             left_fit  = np.polyfit(left_y, left_x, 2, w=weights_l)
#         if right_valid:
#             weights_r = right_y / (np.max(right_y) + 1e-6)
#             right_fit = np.polyfit(right_y, right_x, 2, w=weights_r)

#         lane_width_px = self.expected_lane_width_px

#         if right_valid and not left_valid:
#             # Droite détectée → gauche parallèle
#             left_fit = right_fit.copy()
#             left_fit[2] -= lane_width_px
#             if self.logger:
#                 self.logger.info(
#                     f'fit: ligne DROITE seule → gauche estimée '
#                     f'(offset -{lane_width_px:.0f}px)'
#                 )
#         elif left_valid and not right_valid:
#             # Gauche détectée → droite parallèle
#             right_fit = left_fit.copy()
#             right_fit[2] += lane_width_px
#             if self.logger:
#                 self.logger.info(
#                     f'fit: ligne GAUCHE seule → droite estimée '
#                     f'(offset +{lane_width_px:.0f}px)'
#                 )
#         elif not left_valid and not right_valid:
#             # Aucune ligne → fallback temporel
#             return self._temporal_fallback()

#         # Reset compteur d'échecs
#         self.consecutive_failures = 0

#         # Lissage temporel
#         left_fit  = self._smooth_fit(left_fit,  self.left_fit_history)
#         right_fit = self._smooth_fit(right_fit, self.right_fit_history)

#         self.last_left_fit  = left_fit
#         self.last_right_fit = right_fit

#         return left_fit, right_fit, True

#     def _temporal_fallback(self):
#         self.consecutive_failures += 1

#         if (hasattr(self, 'initial_src_points') and
#                 self.consecutive_failures >= self.max_consecutive_failures):
#             if self.logger:
#                 self.logger.warn(
#                     f'fit: {self.consecutive_failures} échecs consécutifs — RESET warp'
#                 )
#             self.src_points = self.initial_src_points.copy()
#             self._compute_warp_matrices()
#             self.consecutive_failures = 0
#             self.left_fit_history.clear()
#             self.right_fit_history.clear()

#         if self.last_left_fit is not None:
#             if self.logger and self.consecutive_failures % 20 == 1:
#                 self.logger.warn(
#                     f'fit: échec #{self.consecutive_failures} — fallback temporel'
#                 )
#             # valid=True seulement pour les 8 premières frames de fallback
#             return self.last_left_fit, self.last_right_fit, self.consecutive_failures <= 8

#         if self.logger:
#             self.logger.error('fit: AUCUNE ligne et pas de fallback disponible')
#         return None, None, False

#     def _smooth_fit(self, new_fit, history):
#         history.append(new_fit)
#         if len(history) > self.history_length:
#             history.pop(0)
#         if len(history) == 1:
#             return history[0]
#         weights = np.exp(np.linspace(-1, 0, len(history)))
#         weights /= weights.sum()
#         return np.average(history, axis=0, weights=weights)

#     # ================================================================
#     #  Visualisation debug
#     # ================================================================
#     def draw_lane_overlay(self, original_image, left_fit, right_fit, mid_fit=None):
#         if left_fit is None or right_fit is None:
#             return original_image

#         overlay  = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
#         y_values = np.linspace(0, self.image_height - 1, self.image_height)
#         left_x   = np.clip(np.polyval(left_fit,  y_values), 0, self.image_width - 1)
#         right_x  = np.clip(np.polyval(right_fit, y_values), 0, self.image_width - 1)

#         pts_left  = np.array([np.flipud(np.column_stack((left_x,  y_values)))], dtype=np.int32)
#         pts_right = np.array([np.column_stack((right_x, y_values))],            dtype=np.int32)
#         pts = np.hstack((pts_left, pts_right))
#         cv2.fillPoly(overlay, pts, (0, 100, 0))

#         for i in range(len(y_values) - 1):
#             y1, y2 = int(y_values[i]), int(y_values[i + 1])
#             cv2.line(overlay, (int(left_x[i]),  y1), (int(left_x[i + 1]),  y2), (255, 200, 0), 3)
#             cv2.line(overlay, (int(right_x[i]), y1), (int(right_x[i + 1]), y2), (255, 200, 0), 3)

#         if mid_fit is not None:
#             mid_x = np.clip(np.polyval(mid_fit, y_values), 0, self.image_width - 1)
#             for i in range(len(y_values) - 1):
#                 y1, y2 = int(y_values[i]), int(y_values[i + 1])
#                 cv2.line(overlay, (int(mid_x[i]), y1), (int(mid_x[i + 1]), y2), (0, 0, 255), 3)

#         overlay_cam = self.unwarp_from_birdseye(overlay)

#         oh, ow = original_image.shape[:2]
#         if overlay_cam.shape[0] != oh or overlay_cam.shape[1] != ow:
#             overlay_cam = cv2.resize(overlay_cam, (ow, oh), interpolation=cv2.INTER_LINEAR)

#         oc = overlay_cam.shape[2]     if overlay_cam.ndim     == 3 else 1
#         ic = original_image.shape[2] if original_image.ndim == 3 else 1
#         if oc != ic:
#             if oc == 1 and ic == 3:
#                 overlay_cam = cv2.cvtColor(overlay_cam, cv2.COLOR_GRAY2BGR)
#             elif oc == 3 and ic == 1:
#                 overlay_cam = cv2.cvtColor(overlay_cam, cv2.COLOR_BGR2GRAY)

#         original_image = np.ascontiguousarray(original_image)
#         overlay_cam    = np.ascontiguousarray(overlay_cam)
#         return cv2.addWeighted(original_image, 1.0, overlay_cam, 0.5, 0.0)

#     def draw_birdseye_debug(self, binary_warped, window_img, left_fit, right_fit):
#         debug = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
#         if window_img.shape[0] != debug.shape[0] or window_img.shape[1] != debug.shape[1]:
#             window_img = cv2.resize(window_img, (debug.shape[1], debug.shape[0]),
#                                     interpolation=cv2.INTER_NEAREST)
#         debug      = np.ascontiguousarray(debug)
#         window_img = np.ascontiguousarray(window_img)
#         debug = cv2.addWeighted(debug, 0.5, window_img, 1.0, 0.0)

#         if left_fit is not None:
#             y_vals = np.linspace(0, self.image_height - 1, 100)
#             lx = np.clip(np.polyval(left_fit,  y_vals), 0, self.image_width - 1).astype(int)
#             rx = np.clip(np.polyval(right_fit, y_vals), 0, self.image_width - 1).astype(int)
#             for i in range(len(y_vals) - 1):
#                 y1, y2 = int(y_vals[i]), int(y_vals[i + 1])
#                 cv2.line(debug, (lx[i], y1), (lx[i + 1], y2), (255, 100, 0), 2)
#                 cv2.line(debug, (rx[i], y1), (rx[i + 1], y2), (255, 100, 0), 2)

#         return debug



# # #!/usr/bin/env python3
# # """
# # Boîte à outils OpenCV pour la détection de voie.
# # Utilisé par lane_detector_node.

# # Ce fichier n'est PAS un node ROS. C'est juste une classe Python
# # avec des fonctions de traitement d'image, pour garder le node lisible.

# # Pipeline (v3 — approche Canny-centrage) :
# #   Image BGR → GaussianBlur → Canny → Bird's eye → Sliding window → Poly fit

# #   Le sliding window tombe sur les DEUX bords Canny de chaque ligne blanche.
# #   np.mean donne automatiquement le CENTRE de la ligne.
# #   → Pas besoin de HSV, pas de seuillage couleur.

# #   Mode alternatif 'hsv_canny' disponible si Canny seul est trop bruité.
# # """

# # import cv2
# # import numpy as np


# # class ImageProcessor:
# #     """
# #     Traitement d'images : détecte les 2 lignes blanches
# #     et retourne leurs polynômes (supporte les courbes).
# #     """

# #     def __init__(self, image_width=640, image_height=360, logger=None):
# #         self.image_width = image_width
# #         self.image_height = image_height
# #         self.logger = logger
# #         self.camera_matrix = None
# #         self.dist_coeffs = None

# #         # ── Perspective transform (bird's eye view) ──
# #         self.src_points = np.float32([
# #             [int(image_width * 0.1), image_height],
# #             [int(image_width * 0.40), int(image_height * 0.55)],
# #             [int(image_width * 0.60), int(image_height * 0.55)],
# #             [int(image_width * 0.9), image_height],
# #         ])
# #         self.dst_points = np.float32([
# #             [int(image_width * 0.2), image_height],
# #             [int(image_width * 0.2), 0],
# #             [int(image_width * 0.8), 0],
# #             [int(image_width * 0.8), image_height],
# #         ])
# #         self._compute_warp_matrices()

# #         # ── Sliding window ──
# #         self.n_windows = 15
# #         self.window_margin = 40
# #         self.window_margin_max = 80
# #         self.min_pixels = 15
# #         self.expected_lane_width_px = self.image_width * 0.59
# #         self.lane_width_tolerance_ratio = 0.30
# #         self.min_lane_width_px = self.image_width * 0.10
# #         self.max_lane_width_px = self.image_width * 0.85

# #         # ── ROI ──
# #         self.roi_top_ratio = 0.50
# #         self.roi_bottom_cut_ratio = 0.0
# #         self.roi_side_cut_ratio = 0.0

# #         # ── Canny thresholds ──
# #         self.canny_low = 75
# #         self.canny_high = 175

# #         # ── Mode de prétraitement ──
# #         self.preprocess_mode = 'canny_only'

# #         # ── CUDA init ──
# #         self.use_cuda = False
# #         self.gpu_blur = None
# #         self.gpu_canny = None
# #         self._cuda_preprocess_warned = False
# #         try:
# #             self.use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
# #         except Exception:
# #             self.use_cuda = False

# #         if self.use_cuda:
# #             try:
# #                 self.gpu_blur = cv2.cuda.createGaussianFilter(
# #                     cv2.CV_8UC3, cv2.CV_8UC3, (5, 5), 0)
# #                 self.gpu_canny = cv2.cuda.createCannyEdgeDetector(
# #                     low_thresh=self.canny_low, high_thresh=self.canny_high)
# #             except Exception:
# #                 self.use_cuda = False
# #                 self.gpu_blur = None
# #                 self.gpu_canny = None

# #         # ── Nettoyage morphologique ──
# #         self.min_component_area = 30

# #         # ── Perspective adaptative ──
# #         self.enable_dynamic_warp = False
# #         self.dynamic_warp_period = 5
# #         self._frame_counter = 0

# #         # ── Lissage temporel ──
# #         self.left_fit_history = []
# #         self.right_fit_history = []
# #         self.history_length = 5

# #         # ── Dernier fit valide (fallback) ──
# #         self.last_left_fit = None
# #         self.last_right_fit = None
# #         self.allow_single_line_fallback = True

# #     def _compute_warp_matrices(self):
# #         self.M_warp = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
# #         self.M_unwarp = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

# #     def set_warp_points(self, src, dst):
# #         self.src_points = np.float32(src)
# #         self.dst_points = np.float32(dst)
# #         self.initial_src_points = self.src_points.copy()
# #         self.initial_dst_points = self.dst_points.copy()
# #         self.consecutive_failures = 0
# #         self.max_consecutive_failures = 15
# #         self._compute_warp_matrices()

# #     # ================================================================
# #     #  Pré-traitement
# #     # ================================================================
# #     def preprocess(self, image, hsv_lower=None, hsv_upper=None):
# #         """
# #         Mode 'canny_only' (DÉFAUT) : BGR → Blur → Gray → Canny
# #         Mode 'hsv_canny'           : HSV mask AND Canny
# #         Mode 'hsv_only'            : HSV mask seul
# #         """
# #         mask = None

# #         if self.use_cuda and self.gpu_blur is not None:
# #             try:
# #                 mask = self._preprocess_gpu(image, hsv_lower, hsv_upper)
# #             except Exception as e:
# #                 if self.logger and not self._cuda_preprocess_warned:
# #                     self.logger.warn(f'CUDA preprocess fail ({str(e)}), fallback CPU')
# #                 self._cuda_preprocess_warned = True
# #                 self.use_cuda = False

# #         if mask is None:
# #             mask = self._preprocess_cpu(image, hsv_lower, hsv_upper)

# #         mask = self._apply_roi_mask(mask)

# #         if self.preprocess_mode == 'canny_only':
# #             dilate_kernel = np.ones((3, 3), np.uint8)
# #             mask = cv2.dilate(mask, dilate_kernel, iterations=1)
# #             close_kernel = np.ones((5, 3), np.uint8)
# #             mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
# #         else:
# #             open_kernel = np.ones((3, 3), np.uint8)
# #             mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
# #             close_kernel = np.ones((7, 5), np.uint8)
# #             mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

# #         mask = self._remove_small_components(mask, min_area=self.min_component_area)

# #         if self.logger:
# #             white_pixels = cv2.countNonZero(mask)
# #             if white_pixels == 0:
# #                 self.logger.warn('preprocess: AUCUN pixel blanc')
# #             elif white_pixels < 300:
# #                 self.logger.warn(f'preprocess: peu de pixels ({white_pixels})')

# #         return mask

# #     def _preprocess_gpu(self, image, hsv_lower, hsv_upper):
# #         gpu_img = cv2.cuda_GpuMat()
# #         gpu_img.upload(image)
# #         gpu_blurred = self.gpu_blur.apply(gpu_img)

# #         if self.preprocess_mode == 'canny_only':
# #             gpu_gray = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2GRAY)
# #             return self.gpu_canny.detect(gpu_gray).download()

# #         elif self.preprocess_mode == 'hsv_only':
# #             lower = np.array(hsv_lower, dtype=np.uint8)
# #             upper = np.array(hsv_upper, dtype=np.uint8)
# #             gpu_hsv = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2HSV)
# #             return cv2.cuda.inRange(gpu_hsv, lower, upper).download()

# #         else:
# #             lower = np.array(hsv_lower, dtype=np.uint8)
# #             upper = np.array(hsv_upper, dtype=np.uint8)
# #             gpu_gray = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2GRAY)
# #             gpu_canny = self.gpu_canny.detect(gpu_gray)
# #             # filter_dilate = cv2.cuda.createMorphologyFilter(
# #             #     cv2.MORPH_DILATE, cv2.CV_8U, np.ones((3, 3), np.uint8))
# #             # gpu_canny = filter_dilate.apply(gpu_canny)
# #             gpu_hsv = cv2.cuda.cvtColor(gpu_blurred, cv2.COLOR_BGR2HSV)
# #             gpu_mask = cv2.cuda.inRange(gpu_hsv, lower, upper)
# #             return cv2.cuda.bitwise_and(gpu_canny, gpu_mask).download()

# #     def _preprocess_cpu(self, image, hsv_lower, hsv_upper):
# #         blurred = cv2.GaussianBlur(image, (5, 5), 0)

# #         if self.preprocess_mode == 'canny_only':
# #             gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
# #             return cv2.Canny(gray, self.canny_low, self.canny_high)

# #         elif self.preprocess_mode == 'hsv_only':
# #             hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
# #             lower = np.array(hsv_lower, dtype=np.uint8)
# #             upper = np.array(hsv_upper, dtype=np.uint8)
# #             return cv2.inRange(hsv, lower, upper)

# #         else:
# #             gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
# #             hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
# #             lower = np.array(hsv_lower, dtype=np.uint8)
# #             upper = np.array(hsv_upper, dtype=np.uint8)
# #             mask_hsv = cv2.inRange(hsv, lower, upper)
# #             canny = cv2.Canny(gray, self.canny_low, self.canny_high)
# #             #canny = cv2.dilate(canny, np.ones((3, 3), np.uint8), iterations=1)
# #             return cv2.bitwise_and(canny, mask_hsv)

# #     def _apply_roi_mask(self, mask):
# #         h, w = mask.shape[:2]
# #         y_top = int(h * self.roi_top_ratio)
# #         y_bottom = int(h * (1.0 - self.roi_bottom_cut_ratio))
# #         x_left = int(w * self.roi_side_cut_ratio)
# #         x_right = int(w * (1.0 - self.roi_side_cut_ratio))
# #         y_top = np.clip(y_top, 0, h - 1)
# #         y_bottom = np.clip(y_bottom, y_top + 1, h)
# #         x_left = np.clip(x_left, 0, w - 1)
# #         x_right = np.clip(x_right, x_left + 1, w)
# #         roi = np.zeros((h, w), dtype=np.uint8)
# #         roi[y_top:y_bottom, x_left:x_right] = 255
# #         return cv2.bitwise_and(mask, roi)

# #     def _remove_small_components(self, mask, min_area=100):
# #         n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
# #         cleaned = np.zeros_like(mask)
# #         for i in range(1, n_labels):
# #             if stats[i, cv2.CC_STAT_AREA] >= min_area:
# #                 cleaned[labels == i] = 255
# #         return cleaned

# #     # ================================================================
# #     #  Bird's eye view
# #     # ================================================================
# #     def warp_to_birdseye(self, image):
# #         self._frame_counter += 1
# #         should_adjust = (
# #             self.enable_dynamic_warp
# #             and (self._frame_counter % self.dynamic_warp_period == 0)
# #             and not self._is_in_curve()
# #         )
# #         if should_adjust:
# #             self.auto_adjust_warp_points(image)

# #         result = cv2.warpPerspective(image, self.M_warp,
# #                                      (self.image_width, self.image_height))
# #         if self.logger:
# #             if cv2.countNonZero(result) == 0:
# #                 self.logger.warn('warp: AUCUN pixel après warp')
# #         return result

# #     def _is_in_curve(self):
# #         if self.last_left_fit is None or self.last_right_fit is None:
# #             return False
# #         avg_curv = (abs(2.0 * self.last_left_fit[0]) + abs(2.0 * self.last_right_fit[0])) / 2.0
# #         return avg_curv > 0.0005

# #     def unwarp_from_birdseye(self, image):
# #         return cv2.warpPerspective(image, self.M_unwarp,
# #                                    (self.image_width, self.image_height))

# #     def auto_adjust_warp_points(self, binary_mask):
# #         if binary_mask is None or binary_mask.ndim != 2:
# #             return False
# #         h, w = binary_mask.shape[:2]
# #         histogram = np.sum(binary_mask[int(h * 0.55):, :], axis=0)
# #         midpoint = w // 2
# #         left_base = int(np.argmax(histogram[:midpoint]))
# #         right_base = int(np.argmax(histogram[midpoint:]) + midpoint)
# #         lane_width = right_base - left_base
# #         if lane_width < self.min_lane_width_px or lane_width > self.max_lane_width_px:
# #             return False
# #         top_y = int(h * 0.58)
# #         bottom_y = int(h * 0.92)
# #         band = 12
# #         top_band = binary_mask[max(0, top_y - band):min(h, top_y + band), :]
# #         bottom_band = binary_mask[max(0, bottom_y - band):min(h, bottom_y + band), :]
# #         top_nz = np.where(top_band > 0)
# #         bot_nz = np.where(bottom_band > 0)
# #         if len(top_nz[1]) < 60 or len(bot_nz[1]) < 60:
# #             return False
# #         top_x = top_nz[1]
# #         bot_x = bot_nz[1]
# #         left_top = np.percentile(top_x[top_x < midpoint], 60) if np.any(top_x < midpoint) else left_base + lane_width * 0.25
# #         right_top = np.percentile(top_x[top_x > midpoint], 40) if np.any(top_x > midpoint) else right_base - lane_width * 0.25
# #         left_bottom = np.percentile(bot_x[bot_x < midpoint], 70) if np.any(bot_x < midpoint) else left_base
# #         right_bottom = np.percentile(bot_x[bot_x > midpoint], 30) if np.any(bot_x > midpoint) else right_base
# #         new_src = np.float32([
# #             [np.clip(left_bottom, 0, w - 1), bottom_y],
# #             [np.clip(left_top, 0, w - 1), top_y],
# #             [np.clip(right_top, 0, w - 1), top_y],
# #             [np.clip(right_bottom, 0, w - 1), bottom_y],
# #         ])
# #         top_width = new_src[2, 0] - new_src[1, 0]
# #         bottom_width = new_src[3, 0] - new_src[0, 0]
# #         if bottom_width <= 0 or top_width <= 0:
# #             return False
# #         ratio = top_width / bottom_width
# #         if ratio < 0.30 or ratio > 0.95:
# #             return False
# #         alpha = 0.12
# #         self.src_points = (1.0 - alpha) * self.src_points + alpha * new_src
# #         self._compute_warp_matrices()
# #         return True

# #     # ================================================================
# #     #  Sliding window
# #     # ================================================================
# #     def find_lane_pixels(self, binary_warped):
# #         """
# #         Sliding window avec marges adaptatives.
# #         np.mean sur les 2 bords Canny → centre automatique de la ligne.
# #         """
# #         h, w = binary_warped.shape[:2]
# #         histogram = np.sum(binary_warped[int(h * 0.65):, :], axis=0)
# #         midpoint = len(histogram) // 2

# #         left_base = self._find_peak(histogram, 0, midpoint)
# #         right_base = self._find_peak(histogram, midpoint, w)

# #         if left_base is None:
# #             left_base = self.image_width // 4
# #             if self.logger:
# #                 self.logger.warn('sliding_window: pas de pic gauche')
# #         if right_base is None:
# #             right_base = 3 * self.image_width // 4
# #             if self.logger:
# #                 self.logger.warn('sliding_window: pas de pic droite')

# #         nonzero = binary_warped.nonzero()
# #         nonzero_y = np.array(nonzero[0])
# #         nonzero_x = np.array(nonzero[1])

# #         window_height = h // self.n_windows
# #         leftx_current = left_base
# #         rightx_current = right_base

# #         left_lane_inds = []
# #         right_lane_inds = []
# #         window_img = np.zeros((*binary_warped.shape, 3), dtype=np.uint8)
# #         width_samples = []

# #         left_margin = self.window_margin
# #         right_margin = self.window_margin

# #         for win in range(self.n_windows):
# #             win_y_low = h - (win + 1) * window_height
# #             win_y_high = h - win * window_height

# #             win_xl_low = max(leftx_current - left_margin, 0)
# #             win_xl_high = min(leftx_current + left_margin, w)
# #             win_xr_low = max(rightx_current - right_margin, 0)
# #             win_xr_high = min(rightx_current + right_margin, w)

# #             cv2.rectangle(window_img, (win_xl_low, win_y_low),
# #                           (win_xl_high, win_y_high), (0, 255, 0), 2)
# #             cv2.rectangle(window_img, (win_xr_low, win_y_low),
# #                           (win_xr_high, win_y_high), (0, 255, 0), 2)

# #             good_left = (
# #                 (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
# #                 (nonzero_x >= win_xl_low) & (nonzero_x < win_xl_high)
# #             ).nonzero()[0]

# #             good_right = (
# #                 (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
# #                 (nonzero_x >= win_xr_low) & (nonzero_x < win_xr_high)
# #             ).nonzero()[0]

# #             left_lane_inds.append(good_left)
# #             right_lane_inds.append(good_right)

# #             # np.mean = centre entre les 2 bords Canny
# #             if len(good_left) > self.min_pixels:
# #                 leftx_current = int(np.mean(nonzero_x[good_left]))
# #                 left_margin = self.window_margin
# #             else:
# #                 left_margin = min(left_margin + 15, self.window_margin_max)

# #             if len(good_right) > self.min_pixels:
# #                 rightx_current = int(np.mean(nonzero_x[good_right]))
# #                 right_margin = self.window_margin
# #             else:
# #                 right_margin = min(right_margin + 15, self.window_margin_max)

# #             if len(good_left) > self.min_pixels and len(good_right) > self.min_pixels:
# #                 width_samples.append(rightx_current - leftx_current)

# #         left_lane_inds = np.concatenate(left_lane_inds)
# #         right_lane_inds = np.concatenate(right_lane_inds)

# #         if len(width_samples) >= 2:
# #             width_samples = np.array(width_samples, dtype=np.float32)
# #             median_width = float(np.median(width_samples))
# #             if self.min_lane_width_px <= median_width <= self.max_lane_width_px:
# #                 self.expected_lane_width_px = 0.92 * self.expected_lane_width_px + 0.08 * median_width
# #             elif median_width < 10:
# #                 if self.logger:
# #                     self.logger.warn(f'sliding_window: largeur quasi-nulle ({median_width:.1f}px)')
# #                 return (np.array([]), np.array([]), np.array([]), np.array([]), window_img)

# #         if self.logger:
# #             nl = len(left_lane_inds)
# #             nr = len(right_lane_inds)
# #             if nl < 50 and nr < 50:
# #                 self.logger.error(f'sliding_window: G={nl} D={nr} pixels (faible)')
# #             elif nl < 50:
# #                 self.logger.warn(f'sliding_window: GAUCHE faible ({nl})')
# #             elif nr < 50:
# #                 self.logger.warn(f'sliding_window: DROITE faible ({nr})')

# #         return (nonzero_x[left_lane_inds], nonzero_y[left_lane_inds],
# #                 nonzero_x[right_lane_inds], nonzero_y[right_lane_inds],
# #                 window_img)

# #     def _find_peak(self, histogram, start, end):
# #         region = histogram[start:end]
# #         if len(region) == 0:
# #             return None
# #         peak_idx = np.argmax(region)
# #         if region[peak_idx] < 50:
# #             return None
# #         return int(peak_idx + start)

# #     # ================================================================
# #     #  Polynomial fit
# #     # ================================================================
# #     def fit_polynomials(self, left_x, left_y, right_x, right_y):
# #         min_points = 50
# #         left_valid = len(left_x) > min_points
# #         right_valid = len(right_x) > min_points

# #         left_fit = None
# #         right_fit = None

# #         if left_valid:
# #             weights_l = left_y / np.max(left_y)
# #             left_fit = np.polyfit(left_y, left_x, 2, w=weights_l)
# #         if right_valid:
# #             weights_r = right_y / np.max(right_y)
# #             right_fit = np.polyfit(right_y, right_x, 2, w=weights_r)

# #         lane_width_px = self.expected_lane_width_px

# #         if right_valid and not left_valid:
# #             # Condition A : droite seule → gauche forcée parallèle
# #             left_fit = right_fit.copy()
# #             left_fit[2] -= lane_width_px
# #             if self.logger:
# #                 self.logger.warn('fit: GAUCHE perdue — estimée depuis droite')
# #         elif left_valid and not right_valid:
# #             # Condition B : gauche seule → droite forcée parallèle
# #             right_fit = left_fit.copy()
# #             right_fit[2] += lane_width_px
# #             if self.logger:
# #                 self.logger.warn('fit: DROITE perdue — estimée depuis gauche')
# #         elif not left_valid and not right_valid:
# #             # Condition C : rien → fallback temporel
# #             return self._temporal_fallback()

# #         if not hasattr(self, 'consecutive_failures'):
# #             self.consecutive_failures = 0
# #         self.consecutive_failures = 0

# #         left_fit = self._smooth_fit(left_fit, self.left_fit_history)
# #         right_fit = self._smooth_fit(right_fit, self.right_fit_history)

# #         self.last_left_fit = left_fit
# #         self.last_right_fit = right_fit

# #         return left_fit, right_fit, True

# #     def _temporal_fallback(self):
# #         if not hasattr(self, 'consecutive_failures'):
# #             self.consecutive_failures = 0
# #         if not hasattr(self, 'max_consecutive_failures'):
# #             self.max_consecutive_failures = 15
# #         self.consecutive_failures += 1

# #         if hasattr(self, 'initial_src_points') and \
# #            self.consecutive_failures >= self.max_consecutive_failures:
# #             if self.logger:
# #                 self.logger.warn(f'fit: {self.consecutive_failures} échecs — RESET warp')
# #             self.src_points = self.initial_src_points.copy()
# #             self._compute_warp_matrices()
# #             self.consecutive_failures = 0
# #             self.left_fit_history.clear()
# #             self.right_fit_history.clear()

# #         if self.last_left_fit is not None:
# #             if self.logger and self.consecutive_failures % 20 == 1:
# #                 self.logger.error(f'fit: échec #{self.consecutive_failures} — fallback')
# #             return self.last_left_fit, self.last_right_fit, self.consecutive_failures <= 5

# #         if self.logger:
# #             self.logger.error('fit: AUCUNE ligne — ÉCHOUÉE')
# #         return None, None, False

# #     def _smooth_fit(self, new_fit, history):
# #         history.append(new_fit)
# #         if len(history) > self.history_length:
# #             history.pop(0)
# #         if len(history) == 1:
# #             return history[0]
# #         weights = np.exp(np.linspace(-1, 0, len(history)))
# #         weights /= weights.sum()
# #         return np.average(history, axis=0, weights=weights)

# #     # ================================================================
# #     #  Visualisation debug
# #     # ================================================================
# #     def draw_lane_overlay(self, original_image, left_fit, right_fit, mid_fit=None):
# #         if left_fit is None or right_fit is None:
# #             return original_image

# #         overlay = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
# #         y_values = np.linspace(0, self.image_height - 1, self.image_height)
# #         left_x = np.clip(np.polyval(left_fit, y_values), 0, self.image_width - 1)
# #         right_x = np.clip(np.polyval(right_fit, y_values), 0, self.image_width - 1)

# #         pts_left = np.array([np.flipud(np.column_stack((left_x, y_values)))], dtype=np.int32)
# #         pts_right = np.array([np.column_stack((right_x, y_values))], dtype=np.int32)
# #         pts = np.hstack((pts_left, pts_right))
# #         cv2.fillPoly(overlay, pts, (0, 100, 0))

# #         for i in range(len(y_values) - 1):
# #             y1, y2 = int(y_values[i]), int(y_values[i + 1])
# #             cv2.line(overlay, (int(left_x[i]), y1), (int(left_x[i + 1]), y2), (255, 200, 0), 3)
# #             cv2.line(overlay, (int(right_x[i]), y1), (int(right_x[i + 1]), y2), (255, 200, 0), 3)

# #         if mid_fit is not None:
# #             mid_x = np.clip(np.polyval(mid_fit, y_values), 0, self.image_width - 1)
# #             for i in range(len(y_values) - 1):
# #                 y1, y2 = int(y_values[i]), int(y_values[i + 1])
# #                 cv2.line(overlay, (int(mid_x[i]), y1), (int(mid_x[i + 1]), y2), (0, 0, 255), 3)

# #         overlay_cam = self.unwarp_from_birdseye(overlay)

# #         oh, ow = original_image.shape[:2]
# #         if overlay_cam.shape[0] != oh or overlay_cam.shape[1] != ow:
# #             overlay_cam = cv2.resize(overlay_cam, (ow, oh), interpolation=cv2.INTER_LINEAR)

# #         oc = overlay_cam.shape[2] if overlay_cam.ndim == 3 else 1
# #         ic = original_image.shape[2] if original_image.ndim == 3 else 1
# #         if oc != ic:
# #             if oc == 1 and ic == 3:
# #                 overlay_cam = cv2.cvtColor(overlay_cam, cv2.COLOR_GRAY2BGR)
# #             elif oc == 3 and ic == 1:
# #                 overlay_cam = cv2.cvtColor(overlay_cam, cv2.COLOR_BGR2GRAY)

# #         original_image = np.ascontiguousarray(original_image)
# #         overlay_cam = np.ascontiguousarray(overlay_cam)
# #         return cv2.addWeighted(original_image, 1.0, overlay_cam, 0.5, 0.0)

# #     def draw_birdseye_debug(self, binary_warped, window_img, left_fit, right_fit):
# #         debug = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
# #         if window_img.shape[0] != debug.shape[0] or window_img.shape[1] != debug.shape[1]:
# #             window_img = cv2.resize(window_img, (debug.shape[1], debug.shape[0]),
# #                                     interpolation=cv2.INTER_NEAREST)
# #         debug = np.ascontiguousarray(debug)
# #         window_img = np.ascontiguousarray(window_img)
# #         debug = cv2.addWeighted(debug, 0.5, window_img, 1.0, 0.0)

# #         if left_fit is not None:
# #             y_vals = np.linspace(0, self.image_height - 1, 100)
# #             lx = np.clip(np.polyval(left_fit, y_vals), 0, self.image_width - 1).astype(int)
# #             rx = np.clip(np.polyval(right_fit, y_vals), 0, self.image_width - 1).astype(int)
# #             for i in range(len(y_vals) - 1):
# #                 y1, y2 = int(y_vals[i]), int(y_vals[i + 1])
# #                 cv2.line(debug, (lx[i], y1), (lx[i + 1], y2), (255, 100, 0), 2)
# #                 cv2.line(debug, (rx[i], y1), (rx[i + 1], y2), (255, 100, 0), 2)

# #         return debug