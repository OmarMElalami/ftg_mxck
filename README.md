# MXCK Follow-The-Gap Stack – Dokumentation

> **Projektdokumentation:** [PDF herunterladen](docs/pdf/dokumentation.pdf)
- LaTeX-/Ausarbeitungsquellen: `docs/Projektarbeit_MXCK_FTG`

## Inhaltsverzeichnis

1. [Systemübersicht](#1-systemübersicht)
2. [Architektur & Datenfluss](#2-architektur--datenfluss)
3. [Pakete & Nodes im Detail](#3-pakete--nodes-im-detail)
4. [Topic-Referenz](#4-topic-referenz)
5. [Konfiguration](#5-konfiguration)
6. [Deployment auf dem Jetson](#6-deployment-auf-dem-jetson)
7. [Testen – Schritt für Schritt](#7-testen--schritt-für-schritt)
8. [Visualisierung mit Foxglove](#8-visualisierung-mit-foxglove)
9. [Troubleshooting](#9-troubleshooting)
10. [Optimierungsvorschläge](#10-optimierungsvorschläge)

---

## 1. Systemübersicht

Der **ftg_mxck** Stack implementiert einen reaktiven Follow-The-Gap (FTG) Algorithmus
für das MXCarkit (MXCK) – ein Jetson-basiertes autonomes Modellfahrzeug.

**Zielplattform:**
- NVIDIA Jetson (Tegra, aarch64), Ubuntu 22.04
- ROS 2 Foxy (mxck2_development) / Humble (mxck2_control)
- RPLidar (2D-LaserScan, ~10 Hz)
- Ackermann-Lenkung via VESC

**Was der Stack tut:**
1. Empfängt rohe LiDAR-Daten (`/scan`)
2. Filtert und rezentriert den Scan auf das Fahrzeug-Frontfenster
3. Erkennt die größte Lücke (Gap) in den Hindernissen
4. Berechnet Lenkwinkel und Geschwindigkeit basierend auf Gap-Richtung und Abstand
5. Gibt Ackermann-Fahrbefehle aus (`/autonomous/ackermann_cmd`)

---

## 2. Architektur & Datenfluss

```
/scan (LaserScan, frame: laser)
  │
  ▼
┌─────────────────────────┐
│  scan_preprocessor_node │  Paket: mxck_ftg_perception
│  TF: base_link ← laser  │  Rezentrierung auf Fahrzeug-Front
│  FOV: ±60° (120°)       │  Clipping: 0.25–10.0 m
└────────┬────────────────┘
         │
         ├──▶ /autonomous/ftg/scan_filtered  (LaserScan, frame: base_link)
         └──▶ /autonomous/ftg/front_clearance (Float32, Meter)
                │                                    │
                ▼                                    │
┌─────────────────────────┐                          │
│    follow_the_gap_v0    │  Paket: follow_the_gap_v0│
│    (C++, input_mode=scan)│  CTU FTG-Algorithmus     │
└────────┬────────────────┘                          │
         │                                           │
         ├──▶ /final_heading_angle (Float32, rad)    │
         ├──▶ /gap_found           (Bool)            │
         ├──▶ /visualize_obstacles (Marker)          │
         ├──▶ /visualize_largest_gap (PointStamped)  │
         └──▶ /visualize_final_heading_angle (PoseStamped)
                │                                    │
                ▼                                    ▼
┌─────────────────────────────────────────────────────┐
│              ftg_planner_node                        │
│  Paket: mxck_ftg_planner                            │
│  Speed-Policy: clearance + steering → Geschwindigkeit│
│  Heading-Smoothing + Speed-Smoothing                 │
└────────┬────────────────────────────────────────────┘
         │
         ├──▶ /autonomous/ftg/gap_angle      (Float32, rad)
         ├──▶ /autonomous/ftg/target_speed   (Float32, m/s)
         └──▶ /autonomous/ftg/planner_status (String)
                │
                ▼
┌─────────────────────────┐
│    ftg_command_node     │  Paket: mxck_ftg_control
│    → AckermannDriveStamped
└────────┬────────────────┘
         │
         ├──▶ /autonomous/ackermann_cmd       (AckermannDriveStamped)
         └──▶ /autonomous/ftg/control_status  (String)
                │
                ▼
         [vehicle_control]  (extern, nicht Teil dieses Stacks)
```

---

## 3. Pakete & Nodes im Detail

### 3.1 mxck_ftg_perception

**Zweck:** LiDAR-Vorverarbeitung – rezentriert den Scan auf das Fahrzeug-Frontfenster.

**Nodes:**

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `scan_preprocessor_node` | `scan_preprocessor_node` | Hauptnode: TF-basierte Rezentrierung, FOV-Filter, Front-Clearance |
| `scan_front_window_check` | `scan_front_window_check` | Diagnose-Node: zeigt nächstes Hindernis im Frontfenster (optional) |

**scan_preprocessor_node – Was er macht:**
1. Liest `/scan` (roher LiDAR-Scan im `laser`-Frame)
2. Holt die TF `base_link ← laser` um die Montagerotation zu kennen
3. Addiert `front_center_deg` (normalerweise 0.0) als „was ist vorne"
4. Filtert nur Beams im konfigurierten Frontfenster (±60° bei 120° FOV)
5. Clippt Ranges auf `clip_min` bis `clip_max`
6. Optional: Moving-Average-Glättung auf die Ranges
7. Publiziert den rezentrierten Scan als `/autonomous/ftg/scan_filtered` im `base_link`-Frame
8. Publiziert die minimale Frontdistanz als `/autonomous/ftg/front_clearance`

**scan_front_window_check – Was er macht:**
- Diagnosetool: zeigt alle N Scans den nächsten Punkt im Frontfenster
- Publiziert RViz-Marker (Kugel + Pfeil) für visuelles Debugging
- Aktivierung: `run_scan_check:=true` in der Launch-Datei

### 3.2 follow_the_gap_v0

**Zweck:** CTU Follow-The-Gap Algorithmus (C++). Findet die größte hindernisfreie Lücke.

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `follow_the_gap` | `follow_the_gap` | FTG-Algorithmus, Scan-Modus |

**Was er macht:**
1. Empfängt `/autonomous/ftg/scan_filtered` (bereits rezentriert, frame: `base_link`)
2. Erkennt Hindernisse, findet Gaps
3. Berechnet den besten Heading-Winkel zur größten Lücke
4. Publiziert Ergebnis-Topics und Visualisierungsdaten

**Publizierte Topics (alle hardcoded, nicht per Parameter änderbar):**

| Topic | Typ | Beschreibung | Wann publiziert |
|---|---|---|---|
| `/final_heading_angle` | `Float32` | Winkel zur besten Lücke (rad) | Nur wenn Gap gefunden |
| `/gap_found` | `Bool` | Ob eine Lücke existiert | Immer (bei jedem Scan) |
| `/visualize_obstacles` | `Marker` (POINTS) | Erkannte Hindernisse als grüne Punkte | Immer (bei jedem Scan) |
| `/visualize_largest_gap` | `PointStamped` | 3 Punkte: Roboterposition + linker/rechter Gap-Rand | Nur wenn Gap gefunden |
| `/visualize_final_heading_angle` | `PoseStamped` | Orientierung (Quaternion) der gewählten Fahrtrichtung | Nur wenn Gap gefunden |

**Subscriber (per Parameter konfigurierbar):**

| Parameter | Default | Beschreibung |
|---|---|---|
| `input_mode` | `"scan"` | `"scan"` oder `"obstacles"` |
| `scan_topic` | `"/autonomous/ftg/scan_filtered"` | LaserScan-Input (wenn input_mode=scan) |
| `obstacles_topic` | `"/obstacles"` | ObstaclesStamped-Input (wenn input_mode=obstacles) |
| `goal_angle_topic` | `"/lsr/angle"` | Externer Zielwinkel (optional) |

**Interne C++-Konstanten (nicht per YAML konfigurierbar):**

`follow_the_gap_v0` verwendet intern hardcodierte Geometrie-Konstanten, die das Gap-Finding-Verhalten bestimmen. Diese sind aktuell nicht als ROS-Parameter verfügbar.

| Konstante | Wert | Beschreibung |
|---|---|---|
| `kCarRadius` | 0.40 m | Sicherheitsradius – Hindernisse werden um diesen Wert aufgeblasen |
| `kTurnRadius` | 0.30 m | Angenommener Wendekreis |
| `kTrackMinWidth` | 0.35 m | Mindestbreite für Corner-Erkennung |
| `kDistanceToCorner` | 0.22 m | Sicherheitsabstand bei Corner-Following |
| `kGapWeightCoefficient` | 100.0 | Gewichtung Gap-Richtung vs. Zielrichtung |
| `kCornerWeightCoefficient` | 100.0 | Gewichtung im Corner-Fall |

> **Hinweis zur Fahrzeuggröße:** Das MXCK ist ca. 30 cm breit. Der aktuelle `kCarRadius = 0.4` behandelt
> das Fahrzeug als wäre es 80 cm breit (2 × Radius). Dies führt dazu, dass passierbare Lücken
> in engen Korridoren als blockiert gewertet werden. Siehe [Optimierungsvorschläge](#10-optimierungsvorschläge).

**Frame-ID der Visualisierungen:** Wird aus dem eingehenden Scan übernommen.
Beim Einsatz von `scan_preprocessor_node` entspricht diese dem konfigurierten `base_frame` des Output-Scans (Default: `base_link`).

### 3.3 mxck_ftg_planner

**Zweck:** Übersetzt FTG-Outputs in stabile Planner-Topics mit Speed-Policy.

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `ftg_planner_node` | `ftg_planner_node` | Unified Planner mit Speed-Policy, Heading- und Speed-Smoothing |

**Was er macht:**
1. Subscribes: `/final_heading_angle`, `/gap_found`, `/autonomous/ftg/front_clearance`
2. Prüft Freshness aller Inputs (Timeout → Stopp)
3. Glättet den Heading-Winkel (Exponential-Filter, konfigurierbar via `heading_smoothing_alpha`)
4. Begrenzt den Lenkwinkel auf `±max_abs_gap_angle_rad`
5. Berechnet Geschwindigkeit basierend auf:
   - **Clearance-Faktor:** Linear von `stop_clearance_m` (0%) bis `caution_clearance_m` (100%)
   - **Steering-Faktor:** Linear von `steering_slowdown_start_rad` (100%) bis `steering_slowdown_full_rad` (0%)
   - Ergebnis: `cruise_speed * min(clearance_faktor, steering_faktor)`, mindestens `min_speed`
6. Glättet die Geschwindigkeit (asymmetrisch: sofort bremsen, langsam beschleunigen)
7. Publiziert: `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`, `/autonomous/ftg/planner_status`

### 3.4 mxck_ftg_control

**Zweck:** Konvertiert Planner-Outputs in AckermannDriveStamped.

| Node | Executable | Beschreibung |
|------|-----------|--------------|
| `ftg_command_node` | `ftg_command_node` | Gap-Angle + Speed → Ackermann |

**Was er macht:**
1. Subscribes: `/autonomous/ftg/gap_angle`, `/autonomous/ftg/target_speed`
2. Prüft Freshness (Timeout → publiziert speed=0, steering=0)
3. Wendet `angle_to_steering_gain` und `max_steering_angle_rad` an
4. Begrenzt speed auf `[min_speed_mps, max_speed_mps]`
5. Publiziert: `/autonomous/ackermann_cmd` (AckermannDriveStamped)

### 3.5 mxck_ftg_bringup

**Zweck:** Launch-Dateien die den gesamten Stack starten.

| Launch-Datei | Beschreibung |
|---|---|
| `ftg_full_system.launch.py` | **Empfohlen.** Startet alle 4 Stages mit YAML-Konfiguration. |
| `ftg_scan_path.launch.py` | Vereinfachte Variante, gleiche Pipeline. |

### 3.6 Hilfs-Pakete (CTU-Legacy)

| Paket | Zweck | Wird aktiv genutzt? |
|---|---|---|
| `obstacle_msgs` | Custom Messages für Hindernis-Formate | Nein (nur für Legacy-Pfad) |
| `obstacle_substitution` | Konvertiert LaserScan→Obstacles | Nein (nur für Legacy-Pfad) |

---

## 4. Topic-Referenz

### Inputs (extern)

| Topic | Typ | Quelle | Beschreibung |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR-Treiber | Roher 360° LiDAR-Scan |
| `/tf` | TF | `mxck_run` | Transform `base_link → laser` |

### Interne Topics (Datenfluss)

| Topic | Typ | Publisher | Subscriber |
|---|---|---|---|
| `/autonomous/ftg/scan_filtered` | `LaserScan` | scan_preprocessor | follow_the_gap |
| `/autonomous/ftg/front_clearance` | `Float32` | scan_preprocessor | ftg_planner |
| `/autonomous/ftg/status` | `String` | scan_preprocessor | (Diagnose) |
| `/final_heading_angle` | `Float32` | follow_the_gap | ftg_planner |
| `/gap_found` | `Bool` | follow_the_gap | ftg_planner |
| `/autonomous/ftg/gap_angle` | `Float32` | ftg_planner | ftg_command |
| `/autonomous/ftg/target_speed` | `Float32` | ftg_planner | ftg_command |
| `/autonomous/ftg/planner_status` | `String` | ftg_planner | (Diagnose) |

### Outputs (extern)

| Topic | Typ | Beschreibung |
|---|---|---|
| `/autonomous/ackermann_cmd` | `AckermannDriveStamped` | Fahrbefehl für vehicle_control |
| `/autonomous/ftg/control_status` | `String` | Diagnose-Status |

### Visualisierungstopics (follow_the_gap_v0)

Diese Topics werden von `follow_the_gap_v0` automatisch publiziert (hardcoded).
Frame: `base_link` (übernommen aus dem eingehenden Scan).

| Topic | Typ | Beschreibung | Wann aktiv |
|---|---|---|---|
| `/visualize_obstacles` | `Marker` (POINTS) | Erkannte Hindernisse als grüne Punkte | Immer |
| `/visualize_largest_gap` | `PointStamped` | Roboterposition + linker/rechter Gap-Rand (3 Messages pro Callback) | Nur wenn Gap gefunden |
| `/visualize_final_heading_angle` | `PoseStamped` | Orientierung der gewählten Fahrtrichtung als Quaternion | Nur wenn Gap gefunden |

### Diagnose-Topics (optional)

| Topic | Typ | Quelle | Beschreibung |
|---|---|---|---|
| `/autonomous/ftg/scan_check` | `String` | scan_front_window_check | Nächster Punkt im Frontfenster |
| `/autonomous/ftg/scan_check_markers` | `MarkerArray` | scan_front_window_check | RViz/Foxglove Marker (Kugel + Pfeil) |

---

## 5. Konfiguration

### 5.1 scan_preprocessor.yaml

```yaml
front_center_deg: 0.0       # 0° = TF bestimmt allein wo vorne ist
front_fov_deg: 120.0         # ±60° Sichtfeld
clip_min_range_m: 0.25       # Punkte näher als 25cm ignorieren (Chassis-Reflexionen)
clip_max_range_m: 10.0       # Punkte weiter als 10m clippen
enable_moving_average: true  # Glättet Scan-Rauschen
moving_average_window: 3     # Fenstergröße der Mittelung
```

| Parameter | Wirkung |
|---|---|
| `front_center_deg` | Definiert „vorne" im `base_link`-Frame. Bei TF-basierter Rezentrierung bleibt der Wert `0.0`. |
| `front_fov_deg` | Breite des FTG-Frontfensters. Größer = mehr Umgebung, aber auch mehr Seitenwände im Scan. |
| `clip_min_range_m` | Untere Distanzgrenze. Filtert Chassis-Eigenreflexionen und zu nahe Punkte heraus. |
| `clip_max_range_m` | Obere Distanzgrenze des verwendeten Scans. |
| `enable_moving_average` | Glättet einzelne Ausreißer im Scan, stabilisiert Gap-Erkennung. |
| `moving_average_window` | Fenstergröße für die Mittelung. Größer = glatter, aber mehr Latenz. |

### 5.2 ftg_planner.yaml

```yaml
input_timeout_sec: 0.50          # Bei Datenverlust → Stopp
max_abs_gap_angle_rad: 0.45      # ±25.8° max Lenkwinkel

cruise_speed_mps: 0.45           # Maximalgeschwindigkeit bei freier Fahrt
min_speed_mps: 0.20              # Minimale Fahrgeschwindigkeit (wenn nicht Stopp)
stop_speed_mps: 0.00             # Geschwindigkeit im Stoppfall

stop_clearance_m: 0.25           # Stopp wenn Hindernis näher als 25cm
caution_clearance_m: 0.50        # Ab 50cm volle Geschwindigkeit

steering_slowdown_start_rad: 0.40  # Bis hier volle Geschwindigkeit
steering_slowdown_full_rad: 0.70   # Ab hier maximale Abbremsung (muss > max_abs_gap_angle sein!)

heading_smoothing_alpha: 0.4     # Heading-Glättung (0=sehr glatt, 1=kein Filter)
speed_smoothing_alpha: 0.3       # Speed-Glättung beim Beschleunigen (Bremsen ist immer sofort)
```

| Parameter | Wirkung |
|---|---|
| `input_timeout_sec` | Wenn Heading, Gap oder Clearance zu alt sind, stoppt der Planner. |
| `max_abs_gap_angle_rad` | Begrenzung des Lenkwinkels. |
| `cruise_speed_mps` | Maximale Wunschgeschwindigkeit bei freiem Weg. |
| `min_speed_mps` | Mindestgeschwindigkeit, solange noch gefahren wird. |
| `stop_clearance_m` | Unterhalb dieses Frontabstands wird gestoppt. |
| `caution_clearance_m` | Ab hier ist der Clearance-Faktor 1.0 (volle Geschwindigkeit). |
| `steering_slowdown_start_rad` | Bis hier wird durch Lenkwinkel noch nicht abgebremst. |
| `steering_slowdown_full_rad` | Ab hier ist der Steering-Faktor 0. **Muss größer sein als `max_abs_gap_angle_rad`**, sonst blockiert das Auto bei maximalem Ausweichen. |
| `heading_smoothing_alpha` | Glättet Sprünge im Heading-Winkel zwischen Scans. |
| `speed_smoothing_alpha` | Glättet Beschleunigung. Bremsen bleibt immer sofort. |

### 5.3 ftg_control.yaml

```yaml
input_timeout_sec: 0.50          # Bei zu alten Planner-Daten → Nullkommando
angle_to_steering_gain: -1.00    # Invertiert den Lenkwinkel (Hardware-spezifisch)
max_steering_angle_rad: 0.45     # Hardware-Limit Lenkung

min_speed_mps: 0.18              # Untergrenze für Speed-Commands
max_speed_mps: 1.80              # Absolute Geschwindigkeitsbegrenzung
```

| Parameter | Wirkung |
|---|---|
| `angle_to_steering_gain` | Skaliert/invertiert den Lenkwinkel. `-1.0` wenn die physische Lenkung invertiert ist. |
| `max_steering_angle_rad` | Hardware-Limit der Lenkung. |
| `min_speed_mps` | Untergrenze für Speed-Commands. |
| `max_speed_mps` | Absolute Obergrenze für Geschwindigkeit. |

### 5.4 Interne Konstanten von follow_the_gap_v0

Diese Werte sind im C++-Code fest definiert und **nicht** per YAML änderbar.
Eine Änderung erfordert Bearbeitung der Quelldateien und Neukompilierung.

| Konstante | Aktueller Wert | Datei | Beschreibung |
|---|---|---|---|
| `kCarRadius` | 0.40 m | `follow_the_gap.hpp` | Sicherheitsradius für Hindernis-Aufblähung |
| `kTurnRadius` | 0.30 m | `follow_the_gap.hpp` | Angenommener Wendekreis |
| `kTrackMinWidth` | 0.35 m | `follow_the_gap.hpp` | Mindestbreite für Corner-Erkennung |
| `kDistanceToCorner` | 0.22 m | `follow_the_gap.hpp` | Sicherheitsabstand bei Corner-Following |
| `kGapWeightCoefficient` | 100.0 | `follow_the_gap.hpp` | Gewichtung Gap-Richtung vs. Zielrichtung |
| `kCornerWeightCoefficient` | 100.0 | `follow_the_gap.hpp` | Gewichtung im Corner-Fall |

> **Wichtig:** Die Fahrzeugbreite wird indirekt über `kCarRadius` berücksichtigt.
> Der FTG-Algorithmus bläst jedes Hindernis um diesen Radius auf und prüft dann,
> ob die verbleibende Lücke passierbar ist.

---

## 6. Deployment auf dem Jetson

### 6.1 Verbindung zum Jetson

Auf deinem PC:
```bash
ssh mxck@192.168.0.100
```

### 6.2 Wichtige Container-Rollen

- **Jetson-Host:** SSH, Dateien kopieren, Bags transferieren
- **mxck2_control:** Vehicle-Control, TF, LiDAR, MXCK-Plattformfunktionen
- **mxck2_development:** FTG-Workspace bauen, testen, starten, Bags aufnehmen

### 6.3 Dateien auf den Jetson kopieren

Auf deinem PC:
```bash
scp -r ftg_mxck/ mxck@192.168.0.100:/home/mxck/
```

Auf dem Jetson-Host:
```bash
mkdir -p /home/mxck/mxck2_ws/src/ftg_mxck
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_perception/   /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_perception/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_planner/      /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_planner/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_control/      /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_control/
rsync -a --delete /home/mxck/ftg_mxck/mxck_ftg_bringup/      /home/mxck/mxck2_ws/src/ftg_mxck/mxck_ftg_bringup/
rsync -a --delete /home/mxck/ftg_mxck/follow_the_gap_v0/     /home/mxck/mxck2_ws/src/ftg_mxck/follow_the_gap_v0/
rsync -a --delete /home/mxck/ftg_mxck/obstacle_msgs/         /home/mxck/mxck2_ws/src/ftg_mxck/obstacle_msgs/
rsync -a --delete /home/mxck/ftg_mxck/obstacle_substitution/ /home/mxck/mxck2_ws/src/ftg_mxck/obstacle_substitution/
```

### 6.4 FTG-Workspace bauen

```bash
sudo docker exec -it mxck2_development bash
source /opt/ros/foxy/setup.bash
cd /mxck2_ws
colcon build --symlink-install --packages-select obstacle_msgs
source install/setup.bash
colcon build --symlink-install --packages-select \
  mxck_ftg_perception \
  follow_the_gap_v0 \
  mxck_ftg_planner \
  mxck_ftg_control \
  mxck_ftg_bringup
source /mxck2_ws/install/setup.bash
```

> **Hinweis:** Bei Änderungen an Python-Dateien oder YAML-Configs reicht ein Neustart des Stacks
> (da mit `--symlink-install` gebaut). Bei C++-Änderungen (`follow_the_gap_v0`) ist ein
> erneuter `colcon build` nötig.

### 6.5 BETAFPV LiteRadio 3 – Kalibrierung und Verbindung

Verwendeter Sender: **BETAFPV LiteRadio 3 Radio Transmitter**

**Kalibrierung:**
1. Alle Joysticks in Neutralstellung, Sender aus
2. Einmal auf Setup drücken → LED leuchtet rot
3. Joysticks in der Mitte lassen, nochmal Setup → Sender piept zweimal
4. Direction Joystick nacheinander: oben, unten, links, rechts
5. Setup drücken
6. Throttle Joystick nach unten → LED leuchtet blau → verbunden

### 6.6 Vehicle-Control starten

```bash
sudo docker exec -it mxck2_control bash
ros2 launch vehicle_control manual_control_launch.py
```

Warten auf `Calibration complete!`, dann **Switch C**:
- oben = Manual, Mitte = Autonomous, unten = Deadman

### 6.7 TF und LiDAR starten

```bash
sudo docker exec -it mxck2_control bash
ros2 launch mxck_run mxck_run_launch.py broadcast_tf:=true run_lidar:=true
```

---

## 7. Testen – Schritt für Schritt

### 7.1 Terminal-Aufteilung

- **Terminal 1** – `mxck2_control`: `manual_control_launch.py`
- **Terminal 2** – `mxck2_control`: TF + LiDAR
- **Terminal 3** – `mxck2_development`: FTG-Tests und Full Stack

### 7.2 Schritt 1 – Vehicle-Control und Remote prüfen

In Terminal 1:
```bash
sudo docker exec -it mxck2_control bash
ros2 launch vehicle_control manual_control_launch.py
```

### 7.3 Schritt 2 – TF und LiDAR prüfen

In Terminal 2:
```bash
sudo docker exec -it mxck2_control bash
ros2 launch mxck_run mxck_run_launch.py broadcast_tf:=true run_lidar:=true
```

Prüfen:
```bash
ros2 topic hz /scan                             # ✅ ~10 Hz
ros2 run tf2_ros tf2_echo base_link laser       # ✅ Gültige Translation + Rotation
```

### 7.4 Schritt 3 – FTG-Container vorbereiten

In Terminal 3:
```bash
sudo docker exec -it mxck2_development bash
source /opt/ros/foxy/setup.bash
source /mxck2_ws/install/setup.bash
cd /mxck2_ws
```

### 7.5 Schritt 4 – Preprocessor testen

```bash
ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
```

| Prüfung | Befehl | Erwartung |
|---|---|---|
| Frame-ID | `ros2 topic echo /autonomous/ftg/scan_filtered --once` | `frame_id: base_link` |
| Winkel | (gleicher Befehl) | `angle_min/max` symmetrisch um 0 |
| Clearance | `ros2 topic echo /autonomous/ftg/front_clearance --once` | Realistischer Wert (0.3–10 m) |
| Status | `ros2 topic echo /autonomous/ftg/status --once` | `recentered_front_scan=true` |

### 7.6 Schritt 5 – follow_the_gap_v0 testen

Voraussetzung: Preprocessor läuft.

```bash
ros2 run follow_the_gap_v0 follow_the_gap \
  --ros-args -p input_mode:=scan -p scan_topic:=/autonomous/ftg/scan_filtered
```

| Prüfung | Befehl | Erwartung |
|---|---|---|
| Heading | `ros2 topic echo /final_heading_angle` | Werte ≠ 0 |
| Gap | `ros2 topic echo /gap_found` | `true` bei freiem Raum |
| Viz-Rate | `ros2 topic hz /visualize_obstacles` | ~10 Hz |

### 7.7 Schritt 6 – Planner testen

Voraussetzung: Preprocessor + FTG laufen.

```bash
ros2 launch mxck_ftg_planner ftg_planner.launch.py
```

| Prüfung | Erwartung |
|---|---|
| `planner_status` | `gap_angle=..., target_speed=..., front_clearance=...` |
| `gap_angle` | Werte zwischen -0.45 und +0.45 rad |
| `target_speed` | 0.20–0.45 m/s bei freier Fahrt |

### 7.8 Schritt 7 – Control-Node testen

```bash
ros2 launch mxck_ftg_control ftg_command.launch.py
```

| Prüfung | Erwartung |
|---|---|
| `ackermann_cmd` | `speed` und `steering_angle` werden publiziert |
| `control_status` | `[CONTROL] speed=..., steering=...` |

### 7.9 Schritt 8 – Full Stack

```bash
ros2 launch mxck_ftg_bringup ftg_full_system.launch.py
```

```bash
ros2 topic hz /autonomous/ftg/scan_filtered   # ✅ ~10 Hz
ros2 topic hz /autonomous/ackermann_cmd        # ✅ ~20 Hz
ros2 topic echo /autonomous/ftg/planner_status # ✅ Keine "waiting" Meldungen
```

### 7.10 Schritt 9 – Fahrtest

- Vehicle-Control läuft bereits in `mxck2_control`
- Auf Deadman bleiben, bis alles stabil ist
- Danach auf Autonomous umschalten
- Zuerst in freiem Bereich testen, dann Hindernisse

### 7.11 Schritt 10 – Bag aufnehmen

```bash
ros2 bag record -o /mxck2_ws/bags/ftg_test \
  /scan /tf /tf_static \
  /autonomous/ftg/scan_filtered \
  /autonomous/ftg/front_clearance \
  /final_heading_angle /gap_found \
  /visualize_obstacles \
  /visualize_largest_gap \
  /visualize_final_heading_angle \
  /autonomous/ftg/gap_angle \
  /autonomous/ftg/target_speed \
  /autonomous/ftg/planner_status \
  /autonomous/ackermann_cmd \
  /autonomous/ftg/control_status
```

### 7.12 Bag vom Jetson kopieren

```bash
# Vom Jetson-Host:
sudo docker cp mxck2_development:/mxck2_ws/bags/ftg_test /home/mxck/ftg_test_bag

# Vom PC:
scp -r mxck@192.168.0.100:/home/mxck/ftg_test_bag .
```

---

## 8. Visualisierung mit Foxglove

### 8.1 Verbindung herstellen

Foxglove Studio → „Open Connection" →
**Rosbridge WebSocket**: `ws://<JETSON_IP>:8765`

(Der `mxck2_foxglove` Container stellt die Bridge bereit.)

> **Hinweis:** FTG-Topics sind nur sichtbar, wenn der FTG-Stack im selben ROS-Domain
> wie die Foxglove-Bridge läuft. Für Live-Visualisierung entweder den FTG-Stack in
> `mxck2_control` bauen oder ein aufgenommenes Bag in Foxglove öffnen.

### 8.2 Alle verfügbaren Topics

| Topic | Typ | Quelle | Panel-Typ |
|---|---|---|---|
| `/scan` | LaserScan | LiDAR-Treiber | 3D |
| `/autonomous/ftg/scan_filtered` | LaserScan | scan_preprocessor | 3D |
| `/autonomous/ftg/front_clearance` | Float32 | scan_preprocessor | Plot |
| `/autonomous/ftg/status` | String | scan_preprocessor | Log |
| `/final_heading_angle` | Float32 | follow_the_gap_v0 | Plot |
| `/gap_found` | Bool | follow_the_gap_v0 | Log |
| `/visualize_obstacles` | Marker (POINTS) | follow_the_gap_v0 | 3D |
| `/visualize_largest_gap` | PointStamped | follow_the_gap_v0 | 3D |
| `/visualize_final_heading_angle` | PoseStamped | follow_the_gap_v0 | 3D |
| `/autonomous/ftg/gap_angle` | Float32 | ftg_planner | Plot |
| `/autonomous/ftg/target_speed` | Float32 | ftg_planner | Plot |
| `/autonomous/ftg/planner_status` | String | ftg_planner | Log |
| `/autonomous/ackermann_cmd` | AckermannDriveStamped | ftg_command | Log / Plot |
| `/autonomous/ftg/control_status` | String | ftg_command | Log |
| `/autonomous/ftg/scan_check_markers` | MarkerArray | scan_front_window_check | 3D |
| `/autonomous/ftg/scan_check` | String | scan_front_window_check | Log |

### 8.3 Panel-Konfiguration

#### Panel 1: 3D-Ansicht

Fixed Frame: `base_link`.

| Topic | Was man sieht |
|---|---|
| `/scan` | Roher 360° LiDAR-Scan (weiß) |
| `/autonomous/ftg/scan_filtered` | Gefiltertes Frontfenster (grün) |
| `/visualize_obstacles` | Erkannte Hindernisse (grün, hardcoded) |
| `/visualize_largest_gap` | Gap-Ränder + Roboterposition |
| `/visualize_final_heading_angle` | Gewählte Fahrtrichtung als Pfeil |

#### Panel 2: Plot – Steering & Speed

| Serie | Topic-Pfad |
|---|---|
| Gap Angle | `/autonomous/ftg/gap_angle.data` |
| Target Speed | `/autonomous/ftg/target_speed.data` |
| Front Clearance | `/autonomous/ftg/front_clearance.data` |

#### Panel 3: Plot – Ackermann-Output

| Serie | Topic-Pfad |
|---|---|
| Actual Speed | `/autonomous/ackermann_cmd.drive.speed` |
| Actual Steering | `/autonomous/ackermann_cmd.drive.steering_angle` |

#### Panel 4: Plot – Heading vs. Gap Angle

| Serie | Topic-Pfad |
|---|---|
| Raw Heading | `/final_heading_angle.data` |
| Gap Angle (nach Planner) | `/autonomous/ftg/gap_angle.data` |

#### Panel 5: Log – Status

| Topic | Inhalt |
|---|---|
| `/autonomous/ftg/planner_status` | Planner-Entscheidungen |
| `/autonomous/ftg/control_status` | Control-Status |
| `/autonomous/ftg/status` | Preprocessor-Status |

### 8.4 Debugging-Szenarien

| Problem | Was prüfen |
|---|---|
| Auto fährt nicht | Panel 5: `waiting for fresh inputs`? |
| Keine Lücke | 3D: `/autonomous/ftg/scan_filtered` sichtbar? |
| Falsche Richtung | 3D: `/visualize_final_heading_angle` Pfeil richtig? |
| Zu langsam | Panel 2: `front_clearance` niedrig oder `gap_angle` groß? |
| Steering = 0 | Panel 4: `/final_heading_angle` Werte ≠ 0? |

### 8.5 Layout speichern

File → Export Layout → `ftg_debug_layout.json`

---

## 9. Troubleshooting

| Symptom | Ursache | Lösung |
|---|---|---|
| `[PREPROCESSOR] TF lookup failed` | TF nicht gestartet | `mxck_run_launch.py broadcast_tf:=true` starten |
| `[PLANNER] waiting for fresh inputs` | Ein Input fehlt oder zu alt | Prüfe ob alle 3 Topics publizieren |
| `[PLANNER] no valid gap found` | FTG sieht keine Lücke | Prüfe `scan_filtered` Ranges |
| `[CONTROL] stale planner inputs` | Planner publiziert nicht mehr | Prüfe ob ftg_planner_node läuft |
| `scan_filtered` alle Ranges 0.05 | Clippt zu aggressiv oder TF falsch | Prüfe TF und `clip_min_range_m` |
| Speed immer 0 | Clearance < `stop_clearance` | Hindernis zu nah |
| Falsche Lenkrichtung | Steering-Vorzeichen falsch | `angle_to_steering_gain: -1.0` |
| Auto stoppt an Corners | FTG-Heading am Limit + Steering-Slowdown | Siehe Optimierungsvorschläge |

---

## 10. Optimierungsvorschläge

### 10.1 follow_the_gap_v0 Geometrie parametrisierbar machen

Der wichtigste Engpass für enge Korridore und Corners ist `kCarRadius = 0.4 m`.
Für das 30 cm breite MXCK sollte dieser Wert ca. **0.18–0.20 m** betragen.

**Empfohlene Änderung:** Die C++-Konstanten `kCarRadius`, `kTurnRadius`, `kTrackMinWidth`,
`kDistanceToCorner`, `kGapWeightCoefficient` und `kCornerWeightCoefficient` als ROS-Parameter
verfügbar machen, damit sie per YAML konfiguriert werden können.

Dazu müssen in `follow_the_gap.hpp` die `static constexpr`-Deklarationen durch `extern`-Variablen
ersetzt werden, die in `main.cpp` via `declare_parameter` / `get_parameter` geladen werden.

**Empfohlene Startwerte für MXCK (30 cm Breite):**

| Konstante | CTU-Default | Empfohlen (MXCK) | Begründung |
|---|---|---|---|
| `kCarRadius` | 0.40 m | **0.20 m** | 15 cm halbe Breite + 5 cm Sicherheitsmarge |
| `kTurnRadius` | 0.30 m | **0.25 m** | Ackermann-Wendekreis |
| `kTrackMinWidth` | 0.35 m | **0.30 m** | Fahrzeugbreite |
| `kDistanceToCorner` | 0.22 m | **0.18 m** | Engerer Corner-Approach |

### 10.2 Clearance-FOV vom FTG-FOV trennen

Aktuell wird `front_clearance` über das gesamte `front_fov_deg` berechnet. In engen Korridoren
sieht der Preprocessor Seitenwände und meldet niedrige Clearance, obwohl der Weg geradeaus frei ist.

**Empfehlung:** Einen separaten Parameter `clearance_fov_deg` (z.B. 30°) einführen, der nur für die
`front_clearance`-Berechnung verwendet wird, während `front_fov_deg` für den FTG-Scan-Output bleibt.

### 10.3 Adaptive Speed-Policy

Die aktuelle Speed-Policy kennt nur zwei lineare Faktoren (Clearance und Steering). In bestimmten
Situationen (z.B. enger Korridor mit Kurve) können sich beide Faktoren gegenseitig zu stark abbremsen.

**Mögliche Verbesserung:** Statt `min(clearance_factor, steering_factor)` könnte ein gewichteter
Durchschnitt oder eine situationsabhängige Logik verwendet werden, z.B.:
- Wenn Clearance hoch aber Steering groß → trotzdem fahren (Kurvenfahrt)
- Wenn Clearance niedrig aber Steering klein → langsamer (enger Korridor geradeaus)
