# ftg_mxck — Follow-The-Gap für MXCarkit (ROS 2 Humble)

Dieses Repository enthält eine vollständige ROS 2-Implementierung des **Follow-The-Gap (FTG)**-Algorithmus für das autonome Fahrzeug **MXCarkit (MXCK)**. Das System läuft auf einem **NVIDIA Jetson Xavier NX** unter **Ubuntu 20.04 / JetPack 5.1.5** in einem Docker-Workspace (`mxck2_ws`) mit ROS 2 Humble.

---

## Inhaltsverzeichnis

1. [Systemübersicht](#1-systemübersicht)
2. [Repository-Struktur](#2-repository-struktur)
3. [Pakete im Überblick](#3-pakete-im-überblick)
4. [Nodes, Topics und Verbindungen](#4-nodes-topics-und-verbindungen)
5. [Konfigurationsparameter](#5-konfigurationsparameter)
6. [Abhängigkeiten](#6-abhängigkeiten)
7. [Installation und Build](#7-installation-und-build)
8. [Deployment auf den Jetson](#8-deployment-auf-den-jetson)
9. [Starten des Systems](#9-starten-des-systems)
10. [Einzelne Pakete separat starten](#10-einzelne-pakete-separat-starten)
11. [Diagnose und Debugging](#11-diagnose-und-debugging)
12. [Rosbag aufnehmen](#12-rosbag-aufnehmen)
13. [Algorithmik — wie FTG funktioniert](#13-algorithmik--wie-ftg-funktioniert)
14. [Architektur-Entscheidungen](#14-architektur-entscheidungen)

---

## 1. Systemübersicht

Der FTG-Stack setzt sich aus vier Stufen zusammen, die als eigene ROS 2-Pakete realisiert sind:

```
[LiDAR]
   │  /scan  (sensor_msgs/LaserScan)
   ▼
┌──────────────────────────────┐
│  mxck_ftg_perception         │  Stage 2
│  scan_preprocessor_node      │──► /autonomous/ftg/scan_filtered
│                              │──► /autonomous/ftg/front_clearance
│  scan_front_window_check     │──► /autonomous/ftg/scan_check  (Debug)
└──────────────────────────────┘
           │
           ▼ /autonomous/ftg/scan_filtered
           ▼ /autonomous/ftg/front_clearance
┌──────────────────────────────┐
│  mxck_ftg_planner            │  Stage 3 / 4
│  ftg_planner_node            │──► /autonomous/ftg/gap_angle
│                              │──► /autonomous/ftg/target_speed
│                              │──► /autonomous/ftg/planner_status
│                              │──► /autonomous/ftg/planner_markers
└──────────────────────────────┘
           │
           ▼ /autonomous/ftg/gap_angle
           ▼ /autonomous/ftg/target_speed
┌──────────────────────────────┐
│  mxck_ftg_control            │  Stage 5
│  ftg_command_node            │──► /autonomous/ackermann_cmd
│                              │──► /autonomous/ftg/control_status
└──────────────────────────────┘
           │
           ▼ /autonomous/ackermann_cmd  (ackermann_msgs/AckermannDriveStamped)
        [Vehicle Control / VESC]
```

Das Fahrzeug-Backend (VESC-Treiber, RC-Weiche, TF-Publisher) kommt aus dem separaten `mxck2_ws`-System (`mxck_run`, `vehicle_control`). Der FTG-Stack gibt seinen finalen Fahrbefehl auf dem Topic `/autonomous/ackermann_cmd` aus — genau dem Topic, das die bestehende Fahrzeugsteuerung im Autonomous-Modus übernimmt.

---

## 2. Repository-Struktur

```
ftg_mxck/
└── ftg_mxck/                         # gemeinsamer Quell-Ordner (in mxck2_ws/src/ ablegen)
    ├── mxck_ftg_perception/           # Stage 2: Scan-Vorverarbeitung
    │   ├── config/
    │   │   ├── scan_preprocessor.yaml
    │   │   └── scan_front_window_check.yaml
    │   ├── launch/
    │   │   ├── scan_preprocessor.launch.py
    │   │   ├── scan_front_window_check.launch.py
    │   │   └── stage2_perception.launch.py
    │   └── mxck_ftg_perception/
    │       ├── common.py
    │       ├── scan_preprocessor_node.py
    │       └── scan_front_window_check.py
    ├── mxck_ftg_planner/              # Stage 3 / 4: FTG-Algorithmus
    │   ├── config/
    │   │   └── ftg_planner.yaml
    │   ├── launch/
    │   │   └── ftg_planner.launch.py
    │   └── mxck_ftg_planner/
    │       ├── common.py
    │       └── ftg_planner_node.py
    ├── mxck_ftg_control/              # Stage 5: Fahrbefehl-Ausgabe
    │   ├── config/
    │   │   └── ftg_control.yaml
    │   ├── launch/
    │   │   └── ftg_command.launch.py
    │   └── mxck_ftg_control/
    │       └── ftg_command_node.py
    └── mxck_ftg_bringup/              # Stage 6: System-Bringup
        └── launch/
            ├── ftg_stack.launch.py    # kompakter Stack-Launch
            └── ftg_full_system.launch.py  # vollständiges System inkl. TF + Rosbag
```

---

## 3. Pakete im Überblick

### 3.1 `mxck_ftg_perception` — Scan-Vorverarbeitung (Stage 2)

Dieses Paket liest den Roh-LiDAR-Scan und bereitet ihn für den FTG-Algorithmus auf. Es enthält zwei Nodes:

#### `scan_preprocessor_node`

Der **Haupt-Preprocessing-Node** des FTG-Stacks:

- Abonniert `/scan` (roher LiDAR-Scan)
- Nutzt TF, um den Scan-Frame nach `base_link` zu transformieren
- Filtert alle Strahlen außerhalb des konfigurierbaren **Front-FOV-Fensters** heraus
- Clippt Messwerte auf `[clip_min_range_m, clip_max_range_m]`
- Setzt Strahlen außerhalb des FOV optional auf `clip_min` (als Hindernis) oder `inf`
- Berechnet die **minimale Frontdistanz** (`front_clearance`) aus allen gültigen Front-Strahlen
- Optionaler gleitender Mittelwert zur Rauschunterdrückung
- Publiziert den gefilterten Scan und die Frontfreiheit

#### `scan_front_window_check`

Ein **Diagnose-Node** zur visuellen Verifikation der Scan-Orientierung:

- Liest `/scan` und transformiert Punkte nach `base_link`
- Findet den nächsten Punkt im Front-Sichtfenster
- Gibt Richtungsinformation aus (LEFT / CENTER / RIGHT)
- Publiziert optional RViz-Marker (Sphere + Pfeil)
- Läuft nur alle N Scans, um die Konsole zu schonen (`log_every_n_scans`)

---

### 3.2 `mxck_ftg_planner` — FTG-Algorithmus (Stage 3 / 4)

Implementiert den eigentlichen **Follow-The-Gap**-Algorithmus nach Sezer & Gökasan:

#### `ftg_planner_node`

1. Empfängt den vorverarbeiteten Scan von `/autonomous/ftg/scan_filtered`
2. Ermittelt den nächsten Hindernis-Strahl
3. **Safety-Bubble**: Setzt alle Strahlen im Radius `safety_bubble_radius_m` um das nächste Hindernis auf 0
4. **Gap-Suche**: Findet zusammenhängende freie Segmente (`range >= free_space_threshold_m`)
5. **Gap-Selektion**: Wählt das beste Gap gewichtet nach Breite und Zentriertheit (`center_bias_weight`)
6. **Zielpunkt**: Findet den besten Punkt im Gap (maximale Reichweite, minimale Abweichung von geradeaus)
7. **Speed-Policy**: Leitet die Zielgeschwindigkeit aus der Frontfreiheit (`front_clearance`) ab
8. Publiziert `gap_angle` (Lenkwinkel in Radiant) und `target_speed` (m/s)
9. Publiziert RViz-Marker (Zielpunkt + Richtungspfeil)

---

### 3.3 `mxck_ftg_control` — Fahrbefehl-Node (Stage 5)

Wandelt die Planner-Ausgabe in einen robusten **AckermannDriveStamped**-Befehl um:

#### `ftg_command_node`

- Empfängt `gap_angle` und `target_speed` vom Planner
- **Timeout-Überwachung**: Stoppt das Fahrzeug, wenn keine frischen Daten vorliegen (`command_timeout_sec`)
- **Lenkwinkel-Verarbeitung**:
  - Optionale Invertierung (`invert_steering`)
  - Multiplikation mit `steering_gain`
  - Clamp auf `±steering_limit_deg`
  - Optionales **exponentielles Glätten** (IIR-Filter, `steering_smoothing_alpha`)
- **Geschwindigkeits-Verarbeitung**:
  - Clamp auf `[0, speed_limit_mps]`
  - Optionale **Kurvengeschwindigkeits-Reduktion** (Turn-Speed-Scaling): Verringert die Geschwindigkeit proportional bei großem Lenkwinkel
- Publiziert mit fester Rate (`publish_rate_hz` Hz) auf `/autonomous/ackermann_cmd`
- Publiziert Statustext auf `/autonomous/ftg/control_status`

---

### 3.4 `mxck_ftg_bringup` — System-Bringup (Stage 6)

Enthält zwei Launch-Dateien, die alle Komponenten koordiniert starten:

#### `ftg_stack.launch.py`
Kompakter Launch mit direkten Paketpfaden. Startet:
- Optional: TF-Broadcast (`mxck_run/broadcast_tf_launch.py`)
- Optional: `scan_front_window_check` (Debug-Node)
- `scan_preprocessor_node`
- `ftg_planner_node`
- `ftg_command_node`

Konfigurationsdateien werden direkt aus den jeweiligen Paket-Shares geladen.

#### `ftg_full_system.launch.py`
Erweiterter Launch mit Laufzeit-Argumenten. Startet zusätzlich:
- Optional: `vehicle_control/manual_control_launch.py` (Low-Level-Fahrzeugsteuerung)
- Optional: **Rosbag-Aufnahme** aller wichtigen FTG-Topics

---

## 4. Nodes, Topics und Verbindungen

### Vollständige Topic-Tabelle

| Topic | Typ | Publisher | Subscriber | Beschreibung |
|-------|-----|-----------|------------|--------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR-Driver (`mxck_run`) | `scan_preprocessor_node`, `scan_front_window_check` | Roh-LiDAR-Scan |
| `/tf` / `/tf_static` | TF-Baum | `mxck_run/broadcast_tf_launch.py` | alle Nodes (via `tf2_ros`) | Koordinatensystem-Transformationen |
| `/autonomous/ftg/scan_filtered` | `sensor_msgs/LaserScan` | `scan_preprocessor_node` | `ftg_planner_node` | Vorverarbeiteter, auf FOV-Fenster reduzierter Scan |
| `/autonomous/ftg/front_clearance` | `std_msgs/Float32` | `scan_preprocessor_node` | `ftg_planner_node` | Minimale Frontdistanz in Metern |
| `/autonomous/ftg/status` | `std_msgs/String` | `scan_preprocessor_node` | — | Statusmeldung der Vorverarbeitung |
| `/autonomous/ftg/gap_angle` | `std_msgs/Float32` | `ftg_planner_node` | `ftg_command_node` | Ziel-Lenkwinkel in Radiant |
| `/autonomous/ftg/target_speed` | `std_msgs/Float32` | `ftg_planner_node` | `ftg_command_node` | Ziel-Geschwindigkeit in m/s |
| `/autonomous/ftg/planner_status` | `std_msgs/String` | `ftg_planner_node` | — | Statusmeldung des Planers |
| `/autonomous/ftg/planner_markers` | `visualization_msgs/MarkerArray` | `ftg_planner_node` | — | RViz-Visualisierung (Zielpunkt + Richtungspfeil) |
| `/autonomous/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | `ftg_command_node` | Vehicle Control (VESC) | **Finaler autonomer Fahrbefehl** |
| `/autonomous/ftg/control_status` | `std_msgs/String` | `ftg_command_node` | — | Statusmeldung des Control-Nodes |
| `/autonomous/ftg/scan_check` | `std_msgs/String` | `scan_front_window_check` | — | Diagnose-Ausgabe (Frontbereich) |
| `/autonomous/ftg/scan_check_markers` | `visualization_msgs/MarkerArray` | `scan_front_window_check` | — | RViz-Marker (Diagnose) |

### Node-Verbindungsdiagramm

```
mxck_run (LiDAR)
   └─[/scan]──────────────────────────────────────────────────────┐
                                                                   │
                                              ┌────────────────────▼──────────────────────────┐
                                              │          scan_preprocessor_node                │
                                              │  sub: /scan                                    │
                                              │  pub: /autonomous/ftg/scan_filtered            │
                                              │  pub: /autonomous/ftg/front_clearance          │
                                              │  pub: /autonomous/ftg/status                   │
                                              └──────────┬──────────────────┬──────────────────┘
                                                         │                  │
                                         [scan_filtered] │    [front_clearance]
                                                         │                  │
                                              ┌──────────▼──────────────────▼──────────────────┐
                                              │            ftg_planner_node                     │
                                              │  sub: /autonomous/ftg/scan_filtered             │
                                              │  sub: /autonomous/ftg/front_clearance           │
                                              │  pub: /autonomous/ftg/gap_angle                 │
                                              │  pub: /autonomous/ftg/target_speed              │
                                              │  pub: /autonomous/ftg/planner_status            │
                                              │  pub: /autonomous/ftg/planner_markers           │
                                              └──────────┬──────────────────┬──────────────────┘
                                                         │                  │
                                             [gap_angle] │     [target_speed]
                                                         │                  │
                                              ┌──────────▼──────────────────▼──────────────────┐
                                              │            ftg_command_node                     │
                                              │  sub: /autonomous/ftg/gap_angle                 │
                                              │  sub: /autonomous/ftg/target_speed              │
                                              │  pub: /autonomous/ackermann_cmd ◄── FAHRZEUG    │
                                              │  pub: /autonomous/ftg/control_status            │
                                              └────────────────────────────────────────────────┘
```

---

## 5. Konfigurationsparameter

### 5.1 `mxck_ftg_perception` — `scan_preprocessor.yaml`

```yaml
scan_topic: "/scan"                  # Input: Roh-LiDAR-Topic
base_frame: "base_link"              # Ziel-Frame für TF-Lookup
filtered_scan_topic: "/autonomous/ftg/scan_filtered"
front_clearance_topic: "/autonomous/ftg/front_clearance"
status_topic: "/autonomous/ftg/status"

front_center_deg: 0.0                # Zentrum des Frontfensters relativ zu base_link [Grad]
front_fov_deg: 140.0                 # Gesamtbreite des Frontfensters [Grad]

clip_min_range_m: 0.05               # Minimale gültige Messung [m]
clip_max_range_m: 8.0                # Maximale gültige Messung [m]
outside_window_as_obstacle: true     # Strahlen außerhalb FOV auf clip_min setzen (= Hindernis)

enable_moving_average: false         # Gleitender Mittelwert aktivieren
moving_average_window: 3             # Fenstergröße für gleitenden Mittelwert
```

### 5.2 `mxck_ftg_perception` — `scan_front_window_check.yaml`

```yaml
scan_topic: "/scan"
base_frame: "base_link"
diagnostic_topic: "/autonomous/ftg/scan_check"
marker_topic: "/autonomous/ftg/scan_check_markers"

front_center_deg: 0.0                # Zentrum des Prüffensters [Grad]
front_fov_deg: 120.0                 # Breite des Prüffensters [Grad]

clip_min_range_m: 0.05
clip_max_range_m: 8.0
beam_stride: 2                       # Jeden 2. Strahl verarbeiten (Performance)
log_every_n_scans: 10                # Nur jeden 10. Scan in Konsole ausgeben
publish_markers: true                # RViz-Marker publizieren
```

### 5.3 `mxck_ftg_planner` — `ftg_planner.yaml`

```yaml
scan_topic: "/autonomous/ftg/scan_filtered"
front_clearance_topic: "/autonomous/ftg/front_clearance"
base_frame: "base_link"

gap_angle_topic: "/autonomous/ftg/gap_angle"
target_speed_topic: "/autonomous/ftg/target_speed"
status_topic: "/autonomous/ftg/planner_status"
marker_topic: "/autonomous/ftg/planner_markers"

# Gap-Erkennung
free_space_threshold_m: 1.00         # Mindestdistanz für "freien" Strahl [m]
min_gap_beams: 10                    # Mindestanzahl Strahlen für ein gültiges Gap
safety_bubble_radius_m: 0.35         # Radius der Sicherheitsblase [m]
steering_limit_deg: 22.0             # Maximaler Lenkwinkel für Planer [Grad]
center_bias_weight: 0.8              # Gewichtung der Geradeaus-Präferenz [0..1]

# Speed-Policy (basierend auf front_clearance)
stop_distance_m: 0.35                # Unterhalb: Stopp
slow_distance_m: 0.60                # Unterhalb: Langsamfahrt
cruise_distance_m: 1.20              # Unterhalb: Mittlere Geschwindigkeit; darüber: Vollgas

speed_stop_mps: 0.0
speed_slow_mps: 0.15
speed_medium_mps: 0.25
speed_fast_mps: 0.35
```

### 5.4 `mxck_ftg_control` — `ftg_control.yaml`

```yaml
gap_angle_topic: "/autonomous/ftg/gap_angle"
target_speed_topic: "/autonomous/ftg/target_speed"
ackermann_topic: "/autonomous/ackermann_cmd"
status_topic: "/autonomous/ftg/control_status"
frame_id: "base_link"

publish_rate_hz: 15.0                # Ausgabe-Rate [Hz]
command_timeout_sec: 0.5             # Maximales Alter der Planner-Daten [s]

# Lenkung
invert_steering: false               # Lenkrichtung umkehren
steering_gain: 1.0                   # Multiplikator für Lenkwinkel
steering_limit_deg: 22.0             # Maximaler Lenkwinkel [Grad]
enable_steering_smoothing: true      # IIR-Glättungsfilter aktivieren
steering_smoothing_alpha: 0.35       # Glättungskoeffizient (0=starr, 1=kein Filter)

# Geschwindigkeit
speed_limit_mps: 0.35                # Absolute Geschwindigkeitsbegrenzung [m/s]
stop_on_timeout: true                # Bei Timeout stoppen

# Kurvengeschwindigkeits-Reduktion
enable_turn_speed_scaling: true      # Aktivieren
slowdown_start_deg: 10.0             # Ab diesem Lenkwinkel beginnt Reduktion [Grad]
slowdown_full_deg: 22.0              # Bei diesem Lenkwinkel maximale Reduktion [Grad]
min_turn_speed_mps: 0.15             # Mindestgeschwindigkeit in scharfer Kurve [m/s]
```

---

## 6. Abhängigkeiten

| Paket | ROS 2 Dependencies |
|-------|--------------------|
| `mxck_ftg_perception` | `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros` |
| `mxck_ftg_planner` | `rclpy`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `tf2_ros` |
| `mxck_ftg_control` | `rclpy`, `std_msgs`, `ackermann_msgs` |
| `mxck_ftg_bringup` | `launch`, `launch_ros`, `ament_index_python`, `mxck_run`*, `vehicle_control`* |

> **\*Hinweis:** `mxck_run` und `vehicle_control` sind Pakete aus dem bestehenden `mxck2_ws`-System und müssen bereits installiert sein.

**System-Voraussetzungen:**
- Ubuntu 20.04 (auf Jetson: JetPack 5.1.5)
- ROS 2 Humble
- Python 3.10+
- `ros-humble-ackermann-msgs`
- `ros-humble-tf2-ros`
- `ros-humble-visualization-msgs`

---

## 7. Installation und Build

### 7.1 Pakete in den Workspace kopieren

Die Pakete müssen im `mxck2_ws`-Source-Verzeichnis liegen. Entweder direkt im Container oder via `scp` (siehe Abschnitt 8).

```bash
# Prüfen, ob die Pakete korrekt liegen:
ls /mxck2_ws/src/mxck_ftg_perception
ls /mxck2_ws/src/mxck_ftg_planner
ls /mxck2_ws/src/mxck_ftg_control
ls /mxck2_ws/src/mxck_ftg_bringup
```

### 7.2 In den Container einloggen

```bash
sudo docker exec -it mxck2_control bash
```

### 7.3 ROS-Umgebung sourcen

```bash
source /opt/ros/humble/setup.bash
source /mxck2_ws/install/setup.bash
```

### 7.4 Pakete bauen

```bash
cd /mxck2_ws
colcon build --symlink-install --packages-select \
    mxck_ftg_perception \
    mxck_ftg_planner \
    mxck_ftg_control \
    mxck_ftg_bringup
```

### 7.5 Workspace nach dem Build sourcen

```bash
source /mxck2_ws/install/setup.bash
```

### 7.6 Build prüfen

```bash
# Prüfen, ob alle ausführbaren Nodes registriert sind:
ros2 pkg executables mxck_ftg_perception
ros2 pkg executables mxck_ftg_planner
ros2 pkg executables mxck_ftg_control

# Erwartete Ausgabe:
# mxck_ftg_perception scan_front_window_check
# mxck_ftg_perception scan_preprocessor_node
# mxck_ftg_planner ftg_planner_node
# mxck_ftg_control ftg_command_node
```

### 7.7 Verfügbare Pakete im Workspace auflisten

```bash
colcon list
```

---

## 8. Deployment auf den Jetson

### 8.1 Alle Pakete auf einmal kopieren (vom Entwicklungs-PC)

```bash
# Gesamten FTG-Ordner kopieren:
scp -r ftg_mxck mxck@192.168.0.100:/home/mxck/mxck2_ws/src/

# Alternativ einzelne Pakete:
scp -r mxck_ftg_perception mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_planner    mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_control    mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
scp -r mxck_ftg_bringup    mxck@192.168.0.100:/home/mxck/mxck2_ws/src/
```

### 8.2 Docker-Container auf dem Jetson starten

```bash
cd ~/mxck2_ws/.devcontainer
sudo docker compose -f docker-compose.all.yml up -d lidar foxglove control kickstart
```

### 8.3 Build auf dem Jetson (im Container)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    cd /mxck2_ws &&
    colcon build --symlink-install --packages-select \
        mxck_ftg_perception \
        mxck_ftg_planner \
        mxck_ftg_control \
        mxck_ftg_bringup &&
    source /mxck2_ws/install/setup.bash
"
```

### 8.4 Prüfen, ob TF bereits läuft (nur einmal nötig!)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 node list | grep robot_state_publisher
"
# Wenn bereits ein robot_state_publisher läuft, keinen zweiten starten.
```

---

## 9. Starten des Systems

### 9.1 Vollständiges System mit einem Befehl (empfohlen)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_bringup ftg_stack.launch.py
"
```

**Mit optionalem Debug-Node:**

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_bringup ftg_stack.launch.py start_scan_check:=true
"
```

**Ohne TF-Broadcast** (wenn `robot_state_publisher` bereits läuft):

```bash
ros2 launch mxck_ftg_bringup ftg_stack.launch.py start_tf:=false
```

### 9.2 Vollständiges System mit Rosbag-Aufnahme

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_bringup ftg_full_system.launch.py \
        record_bag:=true \
        bag_dir:=/mxck2_ws/bags \
        bag_name:=ftg_run_$(date +%Y%m%d_%H%M%S)
"
```

### 9.3 Launch-Argumente von `ftg_stack.launch.py`

| Argument | Standard | Beschreibung |
|----------|----------|--------------|
| `start_tf` | `true` | TF-Broadcast via `mxck_run` starten |
| `start_scan_check` | `false` | Debug-Node `scan_front_window_check` mitstart |
| `scan_check_config` | Paket-Default | Pfad zur YAML-Config für `scan_front_window_check` |
| `perception_config` | Paket-Default | Pfad zur YAML-Config für `scan_preprocessor_node` |
| `planner_config` | Paket-Default | Pfad zur YAML-Config für `ftg_planner_node` |
| `control_config` | Paket-Default | Pfad zur YAML-Config für `ftg_command_node` |

### 9.4 Launch-Argumente von `ftg_full_system.launch.py`

| Argument | Standard | Beschreibung |
|----------|----------|--------------|
| `use_tf` | `true` | TF-Broadcast starten |
| `use_vehicle_control` | `false` | `vehicle_control` mitstart (i.d.R. schon aktiv) |
| `use_perception` | `true` | Perception-Paket starten |
| `use_planner` | `true` | Planner-Paket starten |
| `use_control` | `true` | Control-Paket starten |
| `record_bag` | `false` | Rosbag-Aufnahme aktivieren |
| `bag_dir` | `/mxck2_ws/bags` | Verzeichnis für Bag-Dateien |
| `bag_name` | `ftg_run` | Name der Bag-Session |

---

## 10. Einzelne Pakete separat starten

Für stufenweise Tests oder Debugging kann jedes Paket unabhängig gestartet werden.

### Stage 2: Scan-Vorverarbeitung

```bash
# Nur Preprocessor:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_perception scan_preprocessor.launch.py
"

# Nur Diagnose-Node:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_perception scan_front_window_check.launch.py
"

# Beide zusammen (Stage 2 komplett):
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_perception stage2_perception.launch.py
"
```

### Stage 3/4: Planner

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_planner ftg_planner.launch.py
"
```

### Stage 5: Control-Node

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 launch mxck_ftg_control ftg_command.launch.py
"
```

### Startreihenfolge für manuelle Tests (4 Terminals)

```bash
# Terminal A — TF
sudo docker exec -it mxck2_control bash -lc "source /opt/ros/humble/setup.bash && source /mxck2_ws/install/setup.bash && ros2 launch mxck_run broadcast_tf_launch.py"

# Terminal B — Perception
sudo docker exec -it mxck2_control bash -lc "source /opt/ros/humble/setup.bash && source /mxck2_ws/install/setup.bash && ros2 launch mxck_ftg_perception scan_preprocessor.launch.py"

# Terminal C — Planner
sudo docker exec -it mxck2_control bash -lc "source /opt/ros/humble/setup.bash && source /mxck2_ws/install/setup.bash && ros2 launch mxck_ftg_planner ftg_planner.launch.py"

# Terminal D — Control
sudo docker exec -it mxck2_control bash -lc "source /opt/ros/humble/setup.bash && source /mxck2_ws/install/setup.bash && ros2 launch mxck_ftg_control ftg_command.launch.py"
```

---

## 11. Diagnose und Debugging

### 11.1 Laufende Nodes prüfen

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 node list | sort
"
# Erwartete Nodes:
# /ftg_command_node
# /ftg_planner_node
# /scan_preprocessor_node
# (/scan_front_window_check — falls gestartet)
```

### 11.2 Alle aktiven Topics anzeigen

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic list
"
```

### 11.3 LiDAR-Scan prüfen

```bash
# Einmalig Scan ausgeben:
sudo docker exec -it mxck2_lidar bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /scan --once
"

# Scan-Frequenz messen:
sudo docker exec -it mxck2_lidar bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic hz /scan
"
```

### 11.4 Vorverarbeiteten Scan prüfen

```bash
# Frequenz des gefilterten Scans:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic hz /autonomous/ftg/scan_filtered
"

# Frontdistanz ausgeben:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ftg/front_clearance
"

# Preprocessor-Status:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ftg/status
"
```

### 11.5 Planner-Ausgabe prüfen

```bash
# Zielwinkel (in Radiant):
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ftg/gap_angle
"

# Zielgeschwindigkeit:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ftg/target_speed
"

# Planner-Statusmeldung:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ftg/planner_status
"
```

### 11.6 Finalen Fahrbefehl prüfen

```bash
# AckermannDriveStamped ausgeben:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ackermann_cmd
"

# Frequenz des Fahrbefehls:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic hz /autonomous/ackermann_cmd
"

# Control-Statusmeldung (Lenkwinkel, Geschwindigkeit, Timeout):
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /autonomous/ftg/control_status
"
```

### 11.7 Topic-Publisher prüfen (Mehrfach-Publisher erkennen)

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic info /autonomous/ftg/gap_angle -v &&
    ros2 topic info /autonomous/ftg/target_speed -v &&
    ros2 topic info /autonomous/ackermann_cmd -v
"
# Pro Topic sollte jeweils genau 1 Publisher vorhanden sein.
```

### 11.8 TF-Transformationen prüfen

```bash
# Alle statischen Transformationen ausgeben:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 topic echo /tf_static --qos-durability transient_local --once
"

# TF-Baum im Terminal anzeigen:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 run tf2_tools view_frames
"
```

### 11.9 Laufende Parameter eines Nodes einsehen

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 param list /ftg_planner_node &&
    ros2 param get /ftg_planner_node free_space_threshold_m
"
```

### 11.10 Laufende Parameter zur Laufzeit ändern

```bash
# Beispiel: Maximale Geschwindigkeit erhöhen:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 param set /ftg_command_node speed_limit_mps 0.5
"

# Beispiel: Sicherheitsblase vergrößern:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 param set /ftg_planner_node safety_bubble_radius_m 0.5
"
```

---

## 12. Rosbag aufnehmen

### 12.1 Manuelle Aufnahme der wichtigsten Topics

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    mkdir -p /mxck2_ws/bags &&
    ros2 bag record -s mcap -o /mxck2_ws/bags/ftg_run \
        /scan \
        /tf \
        /tf_static \
        /autonomous/ftg/scan_filtered \
        /autonomous/ftg/front_clearance \
        /autonomous/ftg/gap_angle \
        /autonomous/ftg/target_speed \
        /autonomous/ackermann_cmd \
        /autonomous/ftg/control_status \
        /autonomous/ftg/planner_status
"
```

### 12.2 Bag-Datei offline abspielen (für Tests ohne Fahrzeug)

```bash
# Bag-Datei abspielen:
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    source /mxck2_ws/install/setup.bash &&
    ros2 bag play /mxck2_ws/bags/ftg_run --clock
"

# Parallel FTG-Stack gegen die Bag-Daten laufen lassen:
ros2 launch mxck_ftg_bringup ftg_stack.launch.py start_tf:=false
```

### 12.3 Bag-Inhalt anzeigen

```bash
sudo docker exec -it mxck2_control bash -lc "
    source /opt/ros/humble/setup.bash &&
    ros2 bag info /mxck2_ws/bags/ftg_run
"
```

---

## 13. Algorithmik — wie FTG funktioniert

Der **Follow-The-Gap**-Algorithmus ist ein reaktives lokales Planungsverfahren. Die Implementierung folgt dem Paper von Sezer & Gökasan (2012) und arbeitet in folgenden Schritten:

### Schritt 1 — Scan empfangen und validieren

Der `ftg_planner_node` erhält den vorverarbeiteten Scan von `scan_preprocessor_node`. Ungültige Strahlen (NaN, Inf, ≤ 0) werden ignoriert.

### Schritt 2 — Nächsten Hindernis-Strahl finden

```
closest_idx = argmin(ranges[i] for valid i)
```

### Schritt 3 — Safety-Bubble setzen

Um das nächste Hindernis wird ein Sicherheitskreis mit Radius `safety_bubble_radius_m` gelegt. Alle Strahlen, die in diesen Kreis fallen, werden auf 0 gesetzt:

```
half_angle = atan2(safety_bubble_radius, distance_to_closest)
→ alle Strahlen in [closest_idx - bubble_beams, closest_idx + bubble_beams] = 0
```

### Schritt 4 — Gaps suchen

Zusammenhängende Folgen von Strahlen mit `range >= free_space_threshold_m` und Mindestlänge `min_gap_beams` werden als **Gaps** (freie Segmente) identifiziert.

### Schritt 5 — Bestes Gap wählen

Jedes Gap wird bewertet nach:

```
score = width_score - center_bias_weight × center_penalty × width_score
```

- `width_score` = Anzahl der Strahlen im Gap
- `center_penalty` = absolute Abweichung der Gap-Mitte von der Fahrtrichtung (geradeaus)
- `center_bias_weight` steuert, wie stark Geradeausfahrt bevorzugt wird (0 = keine Präferenz, 1 = starke Präferenz)

### Schritt 6 — Zielpunkt im Gap wählen

Innerhalb des gewählten Gaps wird der Punkt mit dem besten Score ausgewählt:

```
score = range - center_bias_weight × |theta_base|
```

Der zugehörige Winkel (im `base_link`-Frame) ist der **Ziel-Lenkwinkel** `gap_angle`.

### Schritt 7 — Speed-Policy

Die Zielgeschwindigkeit wird aus `front_clearance` (Minimaldistanz im Frontbereich) abgeleitet:

| Bedingung | Geschwindigkeit |
|-----------|-----------------|
| `clearance <= stop_distance_m` (0,35 m) | 0,0 m/s (Stopp) |
| `clearance <= slow_distance_m` (0,60 m) | 0,15 m/s (langsam) |
| `clearance <= cruise_distance_m` (1,20 m) | 0,25 m/s (mittel) |
| `clearance > cruise_distance_m` | 0,35 m/s (schnell) |

### Schritt 8 — Fahrbefehl ausgeben

Der `ftg_command_node` wendet auf `gap_angle` und `target_speed` noch folgende Sicherheitslogik an:

- **Timeout-Check**: Sind die Daten älter als `command_timeout_sec`? → Sofortstopp
- **Lenkwinkel-Clamp**: `±steering_limit_deg`
- **IIR-Glättung**: `filtered = α × new + (1−α) × old`
- **Turn-Speed-Scaling**: Reduziert die Geschwindigkeit bei großen Lenkwinkeln linear von `slowdown_start_deg` bis `slowdown_full_deg`

---

## 14. Architektur-Entscheidungen

### Warum 4 separate Pakete?

Die Aufteilung in `perception`, `planner`, `control` und `bringup` entspricht dem **Separation-of-Concerns**-Prinzip:

- **`perception`**: Kann unabhängig getestet werden, bevor der Algorithmus läuft
- **`planner`**: Rein algorithmisch, keine Hardware-Abhängigkeit
- **`control`**: Kapselt alle Sicherheits- und Fahrzeugspezifika
- **`bringup`**: Koordiniert alles, enthält keine Logik

### Warum `/autonomous/ackermann_cmd`?

Dieses Topic ist der definierte Eingang der autonomen Fahrzeugsteuerung im `mxck2_ws`-System. Die RC-Weiche und der VESC-Treiber übernehmen Befehle aus diesem Topic im Autonomous-Modus. Der FTG-Stack ist damit vollständig in die bestehende Fahrzeugarchitektur integriert.

### Warum TF für die Scan-Transformation?

Der LiDAR-Sensor ist nicht zwingend zentriert auf `base_link`. Durch die TF-Transformation wird sichergestellt, dass Winkel und Abstände immer im Fahrzeugkoordinatensystem (`base_link`) berechnet werden — unabhängig von der physischen Montageposition des Sensors.

### Warum `scan_filtered` statt Roh-Scan im Planner?

Der FTG-Algorithmus ist empfindlich gegenüber Rauschen und Ausreißern. Die Vorverarbeitung (FOV-Beschränkung, Range-Clipping, optionale Glättung) stellt sicher, dass der Planner nur sinnvolle Daten erhält und nicht auf Artefakte reagiert.

---

*Dieses Repository wurde für das MXCarkit-Projekt entwickelt. ROS 2 Humble, Jetson Xavier NX, Docker-basiertes Deployment.*