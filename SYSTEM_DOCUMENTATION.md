# MXCarKit – Vollständige Systemdokumentation

> **Zielgruppe:** Entwickler, die das MXCarKit-Projekt verstehen, betreiben oder weiterentwickeln möchten.  
> Diese Dokumentation basiert vollständig auf den Dateien und Informationen in diesem Repository.

---

## Inhaltsverzeichnis

1. [Systemüberblick](#1-systemüberblick)
2. [Hardwarearchitektur](#2-hardwarearchitektur)
   - 2.1 [Komponenten und Verkabelung](#21-komponenten-und-verkabelung)
   - 2.2 [Sensor-Frames (URDF)](#22-sensor-frames-urdf)
3. [Softwarearchitektur](#3-softwarearchitektur)
   - 3.1 [Docker-Strategie](#31-docker-strategie)
   - 3.2 [Basis-Images und Dockerfiles](#32-basis-images-und-dockerfiles)
   - 3.3 [ROS-Workspaces](#33-ros-workspaces)
4. [ROS-Pakete](#4-ros-pakete)
   - 4.1 [mxck_run](#41-mxck_run)
   - 4.2 [vehicle_control](#42-vehicle_control)
5. [Nodes – Funktionen und Datenfluss](#5-nodes--funktionen-und-datenfluss)
   - 5.1 [rc_to_joy](#51-rc_to_joy)
   - 5.2 [joy_to_ackermann](#52-joy_to_ackermann)
   - 5.3 [ackermann_to_vesc](#53-ackermann_to_vesc)
   - 5.4 [Externe Nodes](#54-externe-nodes)
6. [Topics und Nachrichtentypen](#6-topics-und-nachrichtentypen)
7. [Fahrmodi und Sicherheitssystem](#7-fahrmodi-und-sicherheitssystem)
8. [Launch-Dateien](#8-launch-dateien)
9. [Konfigurationsdateien](#9-konfigurationsdateien)
10. [Containerstart mit Portainer](#10-containerstart-mit-portainer)
11. [Netzwerk und Fernzugriff](#11-netzwerk-und-fernzugriff)
12. [Weiterentwicklung und neue Projekte](#12-weiterentwicklung-und-neue-projekte)

---

## 1. Systemüberblick

Das **MXCarKit** ist ein autonomes Fahrzeug im RC-Maßstab, das auf einem NVIDIA Jetson-Rechner läuft. Die gesamte Software ist in **Docker-Container** verpackt und verwendet **ROS 2 Humble** als Middleware. Das System ist modular aufgebaut: Jeder Container übernimmt genau eine Aufgabe. Die Verwaltung der Container erfolgt über **Portainer** (Weboberfläche).

```
┌─────────────────────────────────────────────────────────┐
│                    NVIDIA Jetson                         │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │  mxck2_ws    │  │ development_ │  │   Portainer   │  │
│  │  Container   │  │  ws Container│  │   :9000       │  │
│  │ (ROS Humble) │  │ (ROS Foxy +  │  │               │  │
│  │              │  │  GPU/AI)     │  │               │  │
│  └──────────────┘  └──────────────┘  └───────────────┘  │
│                                                          │
│  Hardware-Anbindung (USB Hub):                          │
│  ├─ STM32 Nucleo (/dev/stm32_nucleo) → micro-ROS        │
│  ├─ VESC          (/dev/vesc)         → vesc_driver     │
│  ├─ RPLidar       (USB)              → rplidar_ros      │
│  └─ RealSense D435i (USB)            → realsense2_camera│
└─────────────────────────────────────────────────────────┘
```

---

## 2. Hardwarearchitektur

### 2.1 Komponenten und Verkabelung

| Komponente | Beschreibung | Verbindung |
|---|---|---|
| **NVIDIA Jetson** | Zentraler Rechner (primär Xavier NX, auch Orin NX möglich), läuft mit NVIDIA JetPack SDK (Ubuntu L4T). Die Dokumentation und der Cheatsheet beziehen sich auf den Xavier NX. Bei Orin NX können sich Befehle und Konfigurationen leicht unterscheiden. | — |
| **USB-Hub** | Erweitert die USB-Ports des Jetson | USB an Jetson |
| **Powerbank** | Versorgt Jetson und USB-Hub mit Strom | — |
| **STM32 Nucleo** | Mikrocontroller; liest RC-Empfänger, Ultraschallsensoren, IMU; steuert LED-Treiber; kommuniziert via micro-ROS | Micro-USB → Jetson (`/dev/stm32_nucleo`, Baudrate: 921600) |
| **RC-Empfänger** | Empfängt PWM-Signale der Fernsteuerung | Kabel → STM32 |
| **Ultraschallsensoren (USS)** | 10 Sensoren rund um das Fahrzeug für Hinderniserkennung | Kabel → STM32 |
| **IMU** | Inertial Measurement Unit für Lage- und Beschleunigungsmessung | Kabel → STM32 |
| **LED-Treiber** | Steuert Fahrzeugbeleuchtung | Kabel → STM32 |
| **VESC** | Elektronischer Motorregler, steuert BLDC-Motor und Servo | USB → Jetson (`/dev/vesc`) |
| **BLDC-Motor** | Antriebsmotor | Kabel → VESC |
| **Servo-Motor** | Lenkung | Kabel → VESC |
| **4S LiPo-Akku** | Versorgt VESC, BLDC-Motor und Servo | — |
| **RPLidar** | 2D-Laserscanner für Umgebungswahrnehmung und Navigation | USB → Jetson (USB-Hub) |
| **Intel RealSense D435i** | RGB-D-Kamera mit eingebautem IMU für Tiefenwahrnehmung | USB → Jetson (USB-Hub) |
| **Fernsteuerung (RC Remote)** | Sendet PWM-Signale an RC-Empfänger | Funk → RC-Empfänger |

**Stromversorgungsschema:**

```
Powerbank ──┬── Jetson
            └── USB-Hub

4S LiPo ────── VESC ──┬── BLDC-Motor
                      └── Servo-Motor
```

**Kommunikationsschema:**

```
RC-Remote ──(Funk)──► RC-Empfänger ──► STM32 Nucleo ──(USB/micro-ROS)──► Jetson
                                              │
                                     Ultraschallsensoren
                                              │
                                            IMU
                                              │
                                        LED-Treiber

Jetson ──(USB/vesc_driver)──► VESC ──► BLDC-Motor + Servo

Jetson ──(USB)──► RPLidar
Jetson ──(USB)──► RealSense D435i
```

### 2.2 Sensor-Frames (URDF)

Das Fahrzeug ist in `src/mxck_run/urdf/mxcarkit.urdf` als URDF-Modell beschrieben. Koordinatensystem: **x = vorwärts, y = links, z = oben** (ROS-Standard).

| Frame | Position (x, y, z) [m] | Orientierung (r, p, y) [rad] | Beschreibung |
|---|---|---|---|
| `base_link` | 0, 0, 0 | 0, 0, 0 | Hinterachse (Referenzrahmen) |
| `front_axle` | 0.36, 0, 0 | 0, 0, 0 | Vorderachse |
| `laser` | 0.1915, 0, 0.0865 | 0, 0, 3.927 | RPLidar (225° gedreht) |
| `camera_rgb` | 0.19, 0.06, 0.116 | -1.5708, 0, -1.5708 | RealSense Kamera |
| `USS_SRF` | 0.26, -0.149, 0.022 | 0, 0, -1.571 | Ultraschall rechts-vorne |
| `USS_SRB` | 0.115, -0.149, 0.022 | 0, 0, -1.571 | Ultraschall rechts-hinten |
| `USS_BR` | -0.074, -0.135, 0.046 | 0, 0, -2.53 | Ultraschall hinten-rechts |
| `USS_BC` | -0.16, 0, 0.064 | 0, 0, 3.14 | Ultraschall hinten-mitte |
| `USS_BL` | -0.074, 0.135, 0.046 | 0, 0, 2.53 | Ultraschall hinten-links |
| `USS_SLB` | 0.115, 0.149, 0.022 | 0, 0, 1.57 | Ultraschall links-hinten |
| `USS_SLF` | 0.26, 0.149, 0.022 | 0, 0, 1.57 | Ultraschall links-vorne |
| `USS_FL` | 0.442, 0.112, 0 | 0, 0, 0.436 | Ultraschall vorne-links |
| `USS_FC` | 0.5, 0, 0 | 0, 0, 0 | Ultraschall vorne-mitte |
| `USS_FR` | 0.422, -0.112, 0 | 0, 0, -0.436 | Ultraschall vorne-rechts |

Die TF-Baum-Hierarchie: `base_link` → `front_axle`, `laser`, `camera_rgb`, alle `USS_*`-Frames.

---

## 3. Softwarearchitektur

### 3.1 Docker-Strategie

Das Prinzip: **Kein Software-Install direkt auf dem Jetson-Host-System.** Alle Abhängigkeiten sind in Docker-Containern gekapselt. Jeder Container hat eine einzige Aufgabe ("Single Responsibility Principle").

- **Container-Orchestrierung:** Portainer (läuft automatisch beim Jetson-Start)
- **Container-Management:** Start/Stop/Monitor einzelner Container über Webbrowser unter `http://<ip-adresse>:9000`

### 3.2 Basis-Images und Dockerfiles

#### `Dockerfile.ubuntu` → Primärer Sensor-/Kontroll-Container

**Basis-Image:** `ros:humble-ros-base-jammy`  
**Zweck:** Kernsystem für Sensoren, Fahrzeugsteuerung, Visualisierung.  
**ROS-Distribution:** **ROS 2 Humble** (Ubuntu 22.04)  
**Keine native GPU-Unterstützung**, aber optimale Kompatibilität mit ROS 2 Humble-Tooling (z. B. Foxglove).

Installierte Werkzeuge und Pakete:

| Komponente | Version/Quelle |
|---|---|
| `ros-humble-librealsense2*` | apt |
| `ros-humble-realsense2-*` | apt |
| micro-ROS Agent | gebaut aus Quellcode (`micro_ros_setup`, Branch `humble`) |
| VESC-Treiber (`vesc`) | gebaut aus Quellcode (f1tenth/vesc, Branch `ros2`) |
| `ackermann_msgs` | gebaut aus Quellcode |
| `transport_drivers` | gebaut aus Quellcode |
| RPLidar ROS (`rplidar_ros`) | gebaut aus Quellcode (Branch `ros2`) |
| `ros-humble-foxglove-bridge` | apt |
| `ros-humble-joy` | apt |
| `ros-humble-robot-state-publisher` | apt |
| `ros-humble-joint-state-publisher` | apt |
| `ros-humble-rosbag2-storage-mcap` | apt |
| `ros-humble-rosbag2-py` | apt |
| Python: `setuptools==58.2.0`, `transforms3d`, `ultralytics` | pip |

#### `Dockerfile.l4t` → GPU/KI-Container

**Basis-Image:** `ultralytics/ultralytics:latest-jetson-jetpack5`  
**Zweck:** Neuronale Netze, GPU-beschleunigte Wahrnehmung (z. B. YOLO-Objekterkennung).  
**ROS-Distribution:** **ROS 2 Foxy** (Jetson JetPack 5 kompatibel)

Zusätzlich installiert:

| Komponente | Quelle |
|---|---|
| RealSense SDK (`librealsense`) | gebaut aus Quellcode |
| `ros-foxy-realsense2-*` | apt |
| micro-ROS Agent (Branch `foxy`) | aus Quellcode |
| VESC-Treiber (Branch `foxy`) | aus Quellcode |
| RPLidar ROS | aus Quellcode |
| `vision_msgs` (Branch `humble`) | aus Quellcode (neuere Nachrichtentypen) |
| Python: `pycuda`, `numpy==1.23`, `transforms3d` | pip |
| TensorRT (`trtexec`) | via symlink |

#### `Dockerfile` → Anpassungs-Layer

**Basis-Image:** `mxwilliam/mxck:mxck-humble-ubuntu-22.04` (vorgefertigtes Image auf Docker Hub)  
**Zweck:** Minimaler Layer für projektspezifische Anpassungen.  
Installiert zusätzlich:
- `ros2_numpy` (aus GitHub)

#### `ros_entrypoint.sh` – Container-Startskript

Beim Container-Start werden folgende Workspaces automatisch in dieser Reihenfolge gesourced:

```
/opt/ros/humble/setup.bash
/microros_ws/install/setup.bash
/vesc_ws/install/setup.bash
/rplidar_ws/install/setup.bash
/mxck2_ws/install/setup.bash    ← wird bei erstem Start automatisch gebaut
```

### 3.3 ROS-Workspaces

| Workspace | Speicherort | Inhalt |
|---|---|---|
| `mxck2_ws` | `/mxck2_ws` | Kernsystem: `mxck_run`, `vehicle_control` |
| `development_ws` | `/development_ws` | GPU-basierte Fahrfunktionen (separater Branch: `development_ws`) |
| `microros_ws` | `/microros_ws` | micro-ROS Agent |
| `vesc_ws` | `/vesc_ws` | VESC-Treiber |
| `rplidar_ws` | `/rplidar_ws` | RPLidar-Treiber |

```bash
# development_ws herunterladen:
git clone --recurse-submodules -b development_ws \
    https://github.com/william-mx/mxck2_ws.git ~/development_ws
```

---

## 4. ROS-Pakete

### 4.1 `mxck_run`

**Speicherort:** `src/mxck_run/`  
**Typ:** `ament_python`  
**Zweck:** Zentrale Steuerung aller Sensoren und Aktuatoren über eine einzige Launch-Datei. Enthält keine eigenen ausführbaren Nodes; stattdessen orchestriert es alle anderen Nodes.

**Verzeichnisstruktur:**

```
mxck_run/
├── launch/
│   ├── mxck_run_launch.py      ← Haupt-Launch-Datei
│   ├── broadcast_tf_launch.py  ← TF-Broadcaster starten
│   ├── realsense_launch.py     ← Detaillierter Kamerastart
│   └── record_launch.py        ← ROS-Bag aufzeichnen
├── mxck_run/
│   ├── __init__.py
│   └── message_utils.py        ← Hilfsfunktionen für ROS-Nachrichten
├── urdf/
│   └── mxcarkit.urdf           ← Roboterbeschreibung (URDF)
├── package.xml
├── setup.py
└── setup.cfg
```

**`message_utils.py` – verfügbare Hilfsfunktionen:**

| Funktion | Beschreibung |
|---|---|
| `create_ackermann_msg(speed, steering_angle)` | Erstellt `AckermannDriveStamped`-Nachricht |
| `create_compressed_image_message(cv_image)` | BGR OpenCV-Bild → JPEG `CompressedImage` |
| `create_compressed_grayscale_image_message(cv_image)` | Graustufenbild → JPEG `CompressedImage` |
| `create_ros_image(numpy_image)` | NumPy-Array → ROS `Image` (bgr8) |
| `angle_to_quaternion(angle)` | Yaw-Winkel [rad] → ROS `Quaternion` |
| `create_pose_message(point, angle)` | Punkt + Winkel → `PoseStamped` |
| `create_path_message(waypoints)` | Wegpunkte [[x, y, θ]] → `Path` |
| `image_msg_to_numpy(image_msg)` | ROS `Image` → NumPy-Array |
| `compressed_image_msg_to_numpy(compressed_msg)` | `CompressedImage` → NumPy-Array |
| `get_relative_transform(source_frame, target_frame)` | TF2-Transformation als 4×4-Matrix |
| `create_point_cloud_message(points)` | 2D/3D-Punkte → `PointCloud2` |

### 4.2 `vehicle_control`

**Speicherort:** `src/vehicle_control/`  
**Typ:** `ament_python`  
**Zweck:** Fahrzeugsteuerung auf niedrigstem Level – liest Signale der Fernsteuerung, konvertiert sie und sendet Befehle an den VESC.

**Verzeichnisstruktur:**

```
vehicle_control/
├── launch/
│   └── manual_control_launch.py  ← Startet die vollständige Steuerkette
├── config/
│   ├── control_config.yaml        ← PWM-Kalibrierung, Geschwindigkeiten, Lenkung
│   └── vesc_config.yaml           ← VESC-Treiber-Limits
├── vehicle_control/
│   ├── __init__.py
│   ├── rc_to_joy.py               ← Node: PWM → Joy-Nachricht
│   ├── joy_to_ackermann.py        ← Node: Joy → Ackermann-Befehl
│   └── ackermann_to_vesc.py       ← Node: Ackermann → VESC-Befehle
├── package.xml
├── setup.py
└── setup.cfg
```

**Ausführbare Nodes (Entry Points):**

| Executable | Node-Klasse | Funktion |
|---|---|---|
| `rc_to_joy` | `RCJoystick` | PWM-Signale → Joy-Nachricht |
| `joy_to_ackermann` | `JoyControl` | Joy → Ackermann-Lenkbefehl |
| `ackermann_to_vesc` | `AckermannToVesc` | Ackermann → VESC-Motorbefehle |

---

## 5. Nodes – Funktionen und Datenfluss

### Vollständiger Datenfluss

```
RC-Modus (control_type: "rc"):
───────────────────────────────
RC-Remote
  │ (PWM-Funk)
  ▼
RC-Empfänger
  │
  ▼
STM32 Nucleo (micro-ROS)
  │ /veh_remote_ctrl [Int16MultiArray]
  ▼
rc_to_joy ──────────────────────────────────────┐
  │ /rc/joy [Joy]                                │
  ▼                                              │
joy_to_ackermann                                 │ mode_button
  │ /rc/ackermann_cmd [AckermannDriveStamped]    │
  ▼                                              │
ackermann_to_vesc ◄─────────────────────────────┘
  ├── /commands/motor/speed [Float64]  ──► VESC ──► BLDC-Motor
  ├── /commands/servo/position [Float64] ──► VESC ──► Servo
  └── /commands/motor/brake [Float64]  ──► VESC (Bremse)

Autonomer Modus:
────────────────
Autonomie-Software (development_ws)
  │ /autonomous/ackermann_cmd [AckermannDriveStamped]
  ▼
ackermann_to_vesc
  ├── /commands/motor/speed  ──► VESC
  ├── /commands/servo/position ──► VESC
  └── /commands/motor/brake ──► VESC

Joy-Modus (control_type: "joy"):
─────────────────────────────────
USB-Gamepad
  │ /joy [Joy]
  ▼
joy_to_ackermann
  │ /rc/ackermann_cmd [AckermannDriveStamped]
  ▼
ackermann_to_vesc
  ...
```

### 5.1 `rc_to_joy`

**Datei:** `vehicle_control/rc_to_joy.py`  
**Node-Name:** `rc_to_joy`

**Aufgabe:** Liest rohe PWM-Werte vom STM32-Mikrocontroller und wandelt sie in eine standardisierte ROS `Joy`-Nachricht um.

| | Topic | Nachrichtentyp |
|---|---|---|
| **Subscriber** | `/veh_remote_ctrl` | `std_msgs/Int16MultiArray` |
| **Publisher** | `/rc/joy` | `sensor_msgs/Joy` |

**Parameter (aus `control_config.yaml`):**

| Parameter | Typ | Beschreibung |
|---|---|---|
| `steering_min_pwm` | int | Minimaler PWM-Wert für Lenkung (Standard: 1000) |
| `steering_mid_pwm` | int | Mittlerer PWM-Wert für Lenkung (Standard: 1500) |
| `steering_max_pwm` | int | Maximaler PWM-Wert für Lenkung (Standard: 2000) |
| `speed_min_pwm` | int | Minimaler PWM-Wert für Geschwindigkeit (Standard: 1000) |
| `speed_mid_pwm` | int | Mittlerer PWM-Wert (Standard: 1500) |
| `speed_max_pwm` | int | Maximaler PWM-Wert (Standard: 2000) |
| `mode_min_pwm` | int | PWM für Modus-Kanal MIN (Standard: 1000) |
| `mode_mid_pwm` | int | PWM für Modus-Kanal MID (Standard: 1500) |
| `mode_max_pwm` | int | PWM für Modus-Kanal MAX (Standard: 2000) |
| `rc_steering_axis` | int | Achsenindex in Joy-Nachricht für Lenkung (Standard: 0) |
| `rc_speed_axis` | int | Achsenindex in Joy-Nachricht für Geschwindigkeit (Standard: 1) |
| `rc_mode_button` | int | Button-Index in Joy-Nachricht für Modus (Standard: 0) |

**Logik:**
- PWM 0 → Verbindung unterbrochen → alle Werte auf 0
- PWM wird linear auf den Bereich [-1.0, 1.0] abgebildet (mit `np.interp`)
- Modus wird durch Vergleich mit `mode_values` ([mode_min, mode_mid, mode_max]) bestimmt → `argmin` → Wert 0, 1 oder 2

### 5.2 `joy_to_ackermann`

**Datei:** `vehicle_control/joy_to_ackermann.py`  
**Node-Name:** `joy_control`

**Aufgabe:** Wandelt Joy-Nachrichten in Ackermann-Lenkbefehle um. Unterstützt sowohl RC- als auch Gamepad-Modus.

| | Topic | Nachrichtentyp |
|---|---|---|
| **Subscriber (RC-Modus)** | `/rc/joy` | `sensor_msgs/Joy` |
| **Subscriber (Joy-Modus)** | `/joy` | `sensor_msgs/Joy` |
| **Publisher** | `/rc/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` |

**Parameter:**

| Parameter | Typ | Standard | Beschreibung |
|---|---|---|---|
| `control_type` | string | `"rc"` | `"rc"` oder `"joy"` |
| `steering_angle_max` | double | 0.44 | Maximaler Lenkwinkel [rad] |
| `max_forward_speed` | double | 2.0 | Maximale Vorwärtsgeschwindigkeit [m/s] |
| `max_backward_speed` | double | -2.0 | Maximale Rückwärtsgeschwindigkeit [m/s] |
| `erpm_min` | int | 700 | Mindest-ERPM (entspricht Mindestgeschwindigkeit) |
| `speed_to_erpm_gain` | int | 3786 | Konversionsfaktor ERPM/m·s⁻¹ |
| `joy_deadzone` | double | 0.07 | Totzone für Joystick-Achsen |
| `rc_steering_axis` | int | 0 | Joy-Achse für RC-Lenkung |
| `rc_speed_axis` | int | 1 | Joy-Achse für RC-Geschwindigkeit |
| `joy_steering_axis` | int | 3 | Joy-Achse für Gamepad-Lenkung |
| `joy_speed_axis` | int | 4 | Joy-Achse für Gamepad-Geschwindigkeit |

**Logik:**
- Lenkung: Joy-Wert [-1.0 .. 1.0] → Lenkwinkel [-steering_angle_max .. steering_angle_max] mit Totzone
- Geschwindigkeit: Joy-Wert [-1.0 .. 1.0] → Geschwindigkeit [m/s] mit Totzone und Mindest-ERPM-Berücksichtigung

### 5.3 `ackermann_to_vesc`

**Datei:** `vehicle_control/ackermann_to_vesc.py`  
**Node-Name:** `ackermann_to_vesc`

**Aufgabe:** Kernnode der Fahrzeugsteuerung. Empfängt Ackermann-Befehle (RC und autonom), verwaltet Fahrmodi, führt Sicherheitsprüfungen durch und sendet ERPM- und Servo-Befehle an den VESC-Treiber.

| | Topic | Nachrichtentyp |
|---|---|---|
| **Subscriber** | `/rc/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` |
| **Subscriber** | `/autonomous/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` |
| **Subscriber (Modus)** | `/rc/joy` oder `/joy` | `sensor_msgs/Joy` |
| **Publisher** | `/commands/motor/speed` | `std_msgs/Float64` |
| **Publisher** | `/commands/servo/position` | `std_msgs/Float64` |
| **Publisher** | `/commands/motor/brake` | `std_msgs/Float64` |

**Parameter:**

| Parameter | Typ | Standard | Beschreibung |
|---|---|---|---|
| `control_type` | string | `"rc"` | `"rc"` oder `"joy"` |
| `servo_mid` | double | 0.443 | Neutralposition des Servos [0..1] |
| `servo_max` | double | 0.9 | Maximalposition des Servos (rechts) |
| `servo_min` | double | 0.1 | Minimalposition des Servos (links) |
| `erpm_min` | int | 700 | Mindest-ERPM; unter diesem Wert wird gebremst |
| `brake_amps` | double | -20.0 | Bremsstroms [A] |
| `speed_to_erpm_gain` | int | 3786 | Konversionsfaktor ERPM/m·s⁻¹ |
| `steer_to_servo_gain` | double | 1.0 | Konversionsfaktor Servo-Position/Lenkwinkel[rad] |
| `rc_dead_value` | int | 0 | Modus-Wert für „Deadman" (RC) |
| `rc_auto_value` | int | 1 | Modus-Wert für „Autonom" (RC) |
| `rc_manu_value` | int | 2 | Modus-Wert für „Manuell" (RC) |
| `joy_dead_value` | int | 0 | Modus-Wert für „Deadman" (Joy) |
| `joy_manu_value` | int | 1 | Modus-Wert für „Manuell" (Joy) |
| `joy_auto_value` | int | 2 | Modus-Wert für „Autonom" (Joy) |
| `joy_mode_button` | int | 4 | Button-Index für Modus (Joy) |
| `joy_auto_button` | int | 5 | Button-Index für Auto-Aktivierung (Joy) |
| `invert_steering` | bool | true | Lenkrichtung umkehren |

**Formeln:**

```
ERPM = speed_to_erpm_gain × speed [m/s]
servo_value = servo_mid ± steering_sign × steering_angle × steer_to_servo_gain
servo_value = clamp(servo_value, servo_min, servo_max)
```

**Joy-Modus-Matrix** (2×2, für Gamepad):

```
                  auto_button=0    auto_button=1
mode_button=0  │    0 (Deadman)  │  2 (Autonom)  │
mode_button=1  │    1 (Manuell)  │  1 (Manuell)  │
```

### 5.4 Externe Nodes

Diese Nodes kommen aus installierten ROS-Paketen und werden über Launch-Dateien gestartet:

| Node | Paket | Zweck | Wichtige Topics |
|---|---|---|---|
| `vesc_driver_node` | `vesc_driver` | VESC-Kommunikation über USB | Sub: `/commands/motor/speed`, `/commands/servo/position`, `/commands/motor/brake` |
| `micro_ros_agent` | `micro_ros_agent` | Brücke STM32 ↔ ROS 2 | Pub: `/veh_remote_ctrl`, `/imu`, `/uss_sensors` |
| `camera/camera` | `realsense2_camera` | RealSense-Kameratreiber | Pub: `/camera/camera/color/image_raw`, `/camera/camera/depth/image_rect_raw`, `/camera/camera/imu` |
| `rplidar_node` | `rplidar_ros` | RPLidar-Treiber | Pub: `/scan` |
| `robot_state_publisher` | `robot_state_publisher` | TF-Baum aus URDF veröffentlichen | Pub: `/tf_static` |
| `foxglove_bridge` | `foxglove_bridge` | WebSocket-Brücke für Foxglove-Visualisierung | Port: 8765 |
| `joy_node` | `joy` | Gamepad-Eingabe lesen | Pub: `/joy` |

---

## 6. Topics und Nachrichtentypen

### Vollständige Topic-Übersicht

| Topic | Nachrichtentyp | Quelle → Ziel | Beschreibung |
|---|---|---|---|
| `/veh_remote_ctrl` | `std_msgs/Int16MultiArray` | STM32 (micro-ROS) → `rc_to_joy` | Rohe PWM-Werte [Lenkung, Speed, Modus] |
| `/imu` | `sensor_msgs/Imu` | STM32 (micro-ROS) → Autonomie | IMU-Daten vom Mikrocontroller |
| `/uss_sensors` | `std_msgs/Int16MultiArray` | STM32 (micro-ROS) → Autonomie | Ultraschallsensor-Abstände |
| `/rc/joy` | `sensor_msgs/Joy` | `rc_to_joy` → `joy_to_ackermann`, `ackermann_to_vesc` | Normalisierte RC-Joystick-Werte |
| `/joy` | `sensor_msgs/Joy` | `joy_node` → `joy_to_ackermann`, `ackermann_to_vesc` | Gamepad-Eingaben |
| `/rc/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | `joy_to_ackermann` → `ackermann_to_vesc` | Manuelle Fahrbefehle |
| `/autonomous/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Autonomie-Software → `ackermann_to_vesc` | Autonome Fahrbefehle |
| `/commands/motor/speed` | `std_msgs/Float64` | `ackermann_to_vesc` → `vesc_driver_node` | Motorgeschwindigkeit in ERPM |
| `/commands/servo/position` | `std_msgs/Float64` | `ackermann_to_vesc` → `vesc_driver_node` | Servo-Position [0.1 .. 0.9] |
| `/commands/motor/brake` | `std_msgs/Float64` | `ackermann_to_vesc` → `vesc_driver_node` | Bremsstrom in Ampere |
| `/scan` | `sensor_msgs/LaserScan` | `rplidar_node` → Autonomie, Foxglove | 2D-Laserscan-Daten |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | `camera/camera` → Autonomie, Foxglove | Farbbild (640×360, BGR8, 15fps) |
| `/camera/color/image_jpeg` | `sensor_msgs/CompressedImage` | Autonomie → Foxglove | JPEG-komprimiertes Farbbild |
| `/camera/camera/imu` | `sensor_msgs/Imu` | `camera/camera` → Autonomie | Kamera-IMU (Accel+Gyro, kombiniert) |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | `camera/camera` → Autonomie | Tiefenbild |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | `camera/camera` → Autonomie | Farbige Punktwolke |
| `/camera/camera/rgbd` | `realsense2_camera_msgs/RGBD` | `camera/camera` → Autonomie | RGBD-Composite |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` → alle | Statischer TF-Baum |
| `/pdc` | — | Autonomie → Foxglove | Park Distance Control |
| `/position` | — | Autonomie → Foxglove | Fahrzeugposition |
| `/path` | `nav_msgs/Path` | Autonomie → Foxglove | Geplanter Pfad |
| `/result` | — | Autonomie → Foxglove | Ergebnis (z. B. Klassifikation) |
| `/waypoint` | — | Autonomie → Foxglove | Nächster Wegpunkt |
| `/detections_2d` | — | Autonomie → Foxglove | 2D-Objekterkennung |
| `/detections_3d` | — | Autonomie → Foxglove | 3D-Objekterkennung |

### Topic-Whitelist für Foxglove

Die folgenden Topics werden über Foxglove Bridge gestreamt (konfiguriert in `mxck_run_launch.py`):

```
/tf_static, /rc/ackermann_cmd, /autonomous/ackermann_cmd, /scan,
/imu, /uss_sensors, /veh_remote_ctrl,
/camera/camera/color/image_raw, /camera/color/image_jpeg,
/camera/camera/imu, /camera/camera/depth/image_rect_raw,
/camera/camera/infra1/image_rect_raw, /camera/camera/infra2/image_rect_raw,
/camera/camera/depth/color/points, /camera/camera/rgbd,
/pdc, /position, /path, /result, /waypoint,
/detections_2d, /detections_3d
```

---

## 7. Fahrmodi und Sicherheitssystem

### Fahrmodi

| Modus | RC-Wert | Joy-Wert | Beschreibung |
|---|---|---|---|
| **Deadman** | 0 | 0 | Notaus; alle Fahrbefehle werden ignoriert |
| **Manuell** | 2 | 1 | RC/Gamepad steuert das Fahrzeug direkt |
| **Autonom** | 1 | 2 | Software auf `/autonomous/ackermann_cmd` steuert das Fahrzeug |

**Modusumschaltung:**
- **RC:** Der dritte Kanal der Fernsteuerung (Dreipositional-Schalter) bestimmt den Modus (0 = Deadman, 1 = Mitte = Autonom, 2 = Manuell)
- **Joy (Gamepad):** Zwei Buttons kombiniert (Mode-Button + Auto-Button, siehe Modus-Matrix)

### Sicherheitssystem beim Start

**Ablauf:**

1. System startet → Modus = `None`
2. `ackermann_to_vesc` informiert: _„Bitte Deadman-Modus aktivieren, 8 Sekunden lang Throttle und Lenkung nicht berühren."_
3. Safety-Check: Der Node überwacht `/rc/ackermann_cmd` und speichert Geschwindigkeitswerte der letzten **8 Sekunden** (bei ~40 Hz = 320 Werte)
4. Wenn während dieser 8 Sekunden **alle Geschwindigkeitswerte = 0**, gilt Kalibrierung als abgeschlossen
5. **Kalibrierungssignal:** Der Servo führt eine Winkelbewegung durch (Sinuswelle), um den Abschluss zu signalisieren
6. Erst **danach** werden Subscriber für `/rc/ackermann_cmd` und `/autonomous/ackermann_cmd` aktiviert
7. Moduswechsel triggern immer **sofort einen Notstopp** (Bremsstrompuls)

### Bremsverhalten

```python
# Notstopp: 420 Hz × 1 Sekunde = ~420 Bremsimpulse
brake_amps = -20.0  # Negativer Strom = regeneratives Bremsen
```

Bremsen wird ausgelöst bei:
- Moduswechsel
- Geschwindigkeit → 0 (wenn vorherige Geschwindigkeit ≠ 0)
- ERPM unter `erpm_min` (= Totzone nahe 0 m/s)

---

## 8. Launch-Dateien

### `mxck_run_launch.py` – Haupt-Launch-Datei

**Verwendung:**
```bash
ros2 launch mxck_run mxck_run_launch.py [Argumente]
```

| Argument | Standard | Beschreibung |
|---|---|---|
| `run_foxglove` | `false` | Foxglove WebSocket-Bridge starten (Port 8765) |
| `run_camera` | `false` | RGB-Kamera (RealSense) starten |
| `run_lidar` | `false` | RPLidar starten |
| `run_micro` | `false` | micro-ROS Agent starten (STM32-Kommunikation) |
| `broadcast_tf` | `false` | TF-Baum aus URDF veröffentlichen |
| `run_motors` | `false` | Fahrzeugsteuerungs-Chain starten |
| `run_rs_imu` | `false` | RealSense IMU aktivieren |

**Beispiel – Kamera + Lidar starten:**
```bash
ros2 launch mxck_run mxck_run_launch.py run_camera:=true run_lidar:=true
```

**Beispiel – Vollbetrieb:**
```bash
ros2 launch mxck_run mxck_run_launch.py \
    run_foxglove:=true \
    run_camera:=true \
    run_lidar:=true \
    run_micro:=true \
    broadcast_tf:=true \
    run_motors:=true
```

**Besonderheit:** Die Launch-Datei prüft vor dem Start mit `ros2 node list`, welche Nodes bereits laufen, und startet nur fehlende Nodes – kein doppeltes Starten!

---

### `manual_control_launch.py` – Fahrzeugsteuerung

Startet die vollständige Steuerkette für manuelles Fahren.

**Gestartete Nodes (abhängig von `control_type` in `control_config.yaml`):**

**RC-Modus:**
- `vesc_driver_node` (mit `vesc_config.yaml`)
- `micro_ros_agent` (serial, `/dev/stm32_nucleo`, 921600 Baud)
- `rc_to_joy` (mit `control_config.yaml`)
- `joy_to_ackermann` (mit `control_config.yaml`)
- `ackermann_to_vesc` (mit `control_config.yaml`)

**Joy-Modus:**
- `vesc_driver_node`
- `joy_node` (`/dev/input/js0`, deadzone: 0.05, autorepeat: 50 Hz)
- `joy_to_ackermann`
- `ackermann_to_vesc`

---

### `broadcast_tf_launch.py` – TF-Broadcaster

Startet `robot_state_publisher` mit dem URDF aus `mxck_run/urdf/mxcarkit.urdf`.  
Veröffentlicht alle Sensor-Frames auf `/tf_static` einmalig (statisch).

---

### `realsense_launch.py` – Erweiterte Kamera-Steuerung

Bietet mehr Konfigurationsmöglichkeiten als der einfache `run_camera`-Parameter:

| Argument | Standard | Beschreibung |
|---|---|---|
| `camera` | `false` | RGB-Farbbild aktivieren |
| `rgbd` | `false` | RGB-D-Stream aktivieren (Farbe + Tiefe kombiniert) |
| `rs_imu` | `false` | IMU aktivieren |
| `ir_left` | `false` | Linkes Infrarot-Bild |
| `ir_right` | `false` | Rechtes Infrarot-Bild |
| `ir_projector` | `false` | IR-Projektor-Muster (verbessert Tiefenmessung) |

**Kamera-Einstellungen (fest kodiert):**

```
RGB:   640×360 @ 15fps, BGR8, Auto-Exposure, Auto-White-Balance
Tiefe: 640×360 @ 15fps
IR:    640×360 @ 15fps
IMU:   Accel + Gyro kombiniert (unite_imu_method=2)
```

---

### `record_launch.py` – ROS-Bag aufzeichnen

Zeichnet ausgewählte Topics in eine Bag-Datei auf.

**Aufgezeichnete Topics (Standard):**
- `/rc/ackermann_cmd`
- `/camera/imu`
- `/imu`

**Verwendung:**
```bash
# Automatischer zufälliger Dateiname:
ros2 launch mxck_run record_launch.py

# Eigener Dateiname und Format:
ros2 launch mxck_run record_launch.py filename:=mein_test format:=mcap
# oder: format:=sqlite3
```

**Speicherort:** `/mxck2_ws/src/mxck_run/bagfiles/<filename>/`

---

## 9. Konfigurationsdateien

### `control_config.yaml`

**Speicherort:** `src/vehicle_control/config/control_config.yaml`

Diese Datei gilt für alle drei Nodes (`rc_to_joy`, `joy_to_ackermann`, `ackermann_to_vesc`) als gemeinsame Konfiguration.

**Kalibrierung der Fernsteuerung:**

Wenn sich das Fahrzeug mit der Fernsteuerung nicht korrekt verhält, müssen die PWM-Werte kalibriert werden:

```bash
# PWM-Werte der Fernsteuerung beobachten:
ros2 topic echo /veh_remote_ctrl

# Alle Joysticks bewegen und Extremwerte notieren:
# steering_min_pwm, steering_mid_pwm, steering_max_pwm
# speed_min_pwm, speed_mid_pwm, speed_max_pwm
# mode_min_pwm, mode_mid_pwm, mode_max_pwm
```

**Steuerungstyp umschalten (RC ↔ Gamepad):**

```yaml
# In control_config.yaml:
/**:
  ros__parameters:
    control_type: "rc"    # Fernsteuerung
    # oder
    control_type: "joy"   # USB-Gamepad
```

**Lenkrichtung korrigieren:**

```yaml
ackermann_to_vesc:
  ros__parameters:
    invert_steering: true   # oder false
```

### `vesc_config.yaml`

**Speicherort:** `src/vehicle_control/config/vesc_config.yaml`

| Parameter | Standard | Beschreibung |
|---|---|---|
| `port` | `/dev/vesc` | USB-Gerätepfad |
| `speed_max` | 23250.0 | Maximale ERPM |
| `speed_min` | -23250.0 | Minimale ERPM (Rückwärts) |
| `servo_max` | 0.9 | Maximale Servo-Position |
| `servo_min` | 0.1 | Minimale Servo-Position |
| `brake_max` | 200000.0 | Maximaler Bremswert |
| `current_max` | 100.0 | Maximaler Motorstrom [A] |

---

## 10. Containerstart mit Portainer

Portainer startet automatisch beim Jetson-Boot. Zugriff über:

```
http://<Jetson-IP-Adresse>:9000
```

**Typische IP-Adressen je nach Verbindung:**

| Verbindungstyp | IP-Adresse |
|---|---|
| USB (RNDIS/Gadget) | `192.168.55.1` |
| Ethernet | wird per DHCP vergeben |
| WiFi | wird per DHCP vergeben |
| Hotspot (Jetson erstellt) | `10.42.0.1` (typisch) |

**IP-Adresse auf dem Jetson ermitteln:**
```bash
ifconfig
```

**Container-Konzept:**

| Container-Name | Inhalt | Funktion |
|---|---|---|
| `mxck2_camera` | `realsense2_camera` | Kamera starten |
| `mxck2_lidar` | `rplidar_ros` | LiDAR starten |
| `mxck2_motors` | `vehicle_control` | Motorsteuerung |
| `mxck2_foxglove` | `foxglove_bridge` | Web-Visualisierung |
| `development_ws` | Autonomie-Pakete | KI/Fahrfunktionen |

Jeder Container repräsentiert eine einzelne Funktion und kann unabhängig gestartet/gestoppt werden.

---

## 11. Netzwerk und Fernzugriff

### SSH-Verbindung

```bash
# Verbindung mit Passwort:
ssh user@<jetson-ip>

# Empfohlen: SSH-Key-basierte Authentifizierung einrichten
# Template: ./ssh_config_template
```

**SSH-Konfiguration (`~/.ssh/config` auf dem Laptop):**
```
Host mxck
    HostName 192.168.55.1
    User <username>
    IdentityFile ~/.ssh/id_rsa
```

### WiFi-Hotspot auf dem Jetson erstellen

```bash
nmcli dev wifi hotspot ifname wlan0 ssid mxck0022 password mxck0022
```

### Empfohlener Entwicklungsworkflow

1. **VS Code Remote Development** – Direktes Entwickeln auf dem Jetson via SSH
2. **Foxglove Studio** – Visualisierung unter `ws://<jetson-ip>:8765`
3. **Portainer** – Container-Management unter `http://<jetson-ip>:9000`

---

## 12. Weiterentwicklung und neue Projekte

### Neuen ROS-Node hinzufügen

1. Python-Datei in `src/vehicle_control/vehicle_control/` oder `src/mxck_run/mxck_run/` erstellen
2. Entry Point in `setup.py` eintragen:
   ```python
   'console_scripts': [
       'my_new_node = my_package.my_module:main',
   ],
   ```
3. Node in eine Launch-Datei aufnehmen
4. Workspace neu bauen: `colcon build --symlink-install`

### Neue autonome Fahrfunktion implementieren

**Schnittstelle für autonomes Fahren:**

```python
# Mindestimplementierung für eine autonome Fahrfunktion
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from mxck_run.message_utils import create_ackermann_msg  # Hilfsfunktion

class MyAutonomousNode(Node):
    def __init__(self):
        super().__init__('my_autonomous_node')
        self.publisher = self.create_publisher(
            AckermannDriveStamped, 
            '/autonomous/ackermann_cmd',  # ← diese Topic steuert das Fahrzeug
            10
        )
    
    def drive(self, speed_ms, steering_rad):
        msg = create_ackermann_msg(speed_ms, steering_rad)
        self.publisher.publish(msg)
```

**Wichtig:** Der `ackermann_to_vesc`-Node muss im **autonomen Modus** sein (RC-Schalter auf Position 1), damit Befehle auf `/autonomous/ackermann_cmd` ausgeführt werden.

### Hilfsfunktionen aus `message_utils` nutzen

```python
from mxck_run.message_utils import (
    create_ackermann_msg,
    create_compressed_image_message,
    create_path_message,
    create_point_cloud_message,
    image_msg_to_numpy,
    get_relative_transform,
)
```

### Neue Sensordaten verarbeiten

```python
# Kamerabild abonnieren:
from sensor_msgs.msg import Image
from mxck_run.message_utils import image_msg_to_numpy

self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

def image_callback(self, msg):
    img = image_msg_to_numpy(msg)  # → NumPy BGR-Array (H, W, 3)
    # ... KI-Verarbeitung ...
```

```python
# LiDAR-Scan abonnieren:
from sensor_msgs.msg import LaserScan

self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

def scan_callback(self, msg):
    ranges = msg.ranges  # Liste von Abstandswerten in Metern
    angle_min = msg.angle_min   # Startwinkel [rad]
    angle_increment = msg.angle_increment  # Winkel pro Messung [rad]
```

### PWM-Kalibrierung der Fernsteuerung

```bash
# Schritt 1: Verbinden und Topic beobachten
ros2 topic echo /veh_remote_ctrl

# Schritt 2: Alle Kanäle bewegen, Extremwerte notieren
# Ausgabe: [steering_pwm, speed_pwm, mode_pwm]

# Schritt 3: Werte in control_config.yaml eintragen
# steering_min_pwm: <notierter Minimalwert>
# steering_mid_pwm: <notierter Mittelwert>
# steering_max_pwm: <notierter Maximalwert>
```

### Servo-Neutral-Position kalibrieren

```bash
# Servo-Position manuell setzen (Wertebereich 0.0 bis 1.0):
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.5"

# Prüfen, ob das Fahrzeug geradeaus fährt. Wert in control_config.yaml anpassen:
# servo_mid: <kalibrierter Wert>
```

### ROS-Bag aufzeichnen und abspielen

```bash
# Aufzeichnen (alle Topics):
ros2 bag record -a -o mein_test

# Aufzeichnen (spezifische Topics):
ros2 launch mxck_run record_launch.py filename:=mein_test

# Abspielen:
ros2 bag play /mxck2_ws/src/mxck_run/bagfiles/mein_test
```

### Nützliche Debug-Befehle

```bash
# Alle laufenden Nodes anzeigen:
ros2 node list

# Alle Topics anzeigen:
ros2 topic list

# Topic-Daten live anzeigen:
ros2 topic echo /rc/ackermann_cmd

# Topic-Frequenz messen:
ros2 topic hz /camera/camera/color/image_raw

# Node-Parameter anzeigen:
ros2 param list /ackermann_to_vesc
ros2 param get /ackermann_to_vesc servo_mid

# Parameter zur Laufzeit ändern:
ros2 param set /ackermann_to_vesc servo_mid 0.45

# TF-Baum anzeigen:
ros2 run tf2_tools view_frames

# Workspace neu bauen:
cd /mxck2_ws && colcon build --symlink-install
source install/setup.bash
```

### Jetson Performance-Modi (für Entwicklung)

> **Hinweis:** Die folgenden Modus-IDs beziehen sich auf den **Xavier NX**. Bei anderen Jetson-Modellen (z. B. Orin NX) können die IDs abweichen. Verfügbare Modi immer zuerst mit `cat /etc/nvpmodel.conf` oder `sudo nvpmodel -q` prüfen.

```bash
# Verfügbare Modi anzeigen:
cat /etc/nvpmodel.conf

# Aktuellen Modus prüfen:
sudo nvpmodel -q

# Maximale Leistung auf Xavier NX (20W, 6 Kerne, Modus 8):
sudo nvpmodel -m 8

# Leistungsaufnahme prüfen:
tegrastats
# oder (mit grafischer Oberfläche):
jtop
```

---

## Anhang: Schnellreferenz – Dateipfade

| Datei | Pfad |
|---|---|
| Haupt-Launch | `src/mxck_run/launch/mxck_run_launch.py` |
| Steuerungschain-Launch | `src/vehicle_control/launch/manual_control_launch.py` |
| TF-Broadcast-Launch | `src/mxck_run/launch/broadcast_tf_launch.py` |
| Kamera-Launch | `src/mxck_run/launch/realsense_launch.py` |
| Bag-Record-Launch | `src/mxck_run/launch/record_launch.py` |
| Steuerungskonfiguration | `src/vehicle_control/config/control_config.yaml` |
| VESC-Konfiguration | `src/vehicle_control/config/vesc_config.yaml` |
| URDF-Robotermodell | `src/mxck_run/urdf/mxcarkit.urdf` |
| Nachrichten-Hilfsfunktionen | `src/mxck_run/mxck_run/message_utils.py` |
| rc_to_joy Node | `src/vehicle_control/vehicle_control/rc_to_joy.py` |
| joy_to_ackermann Node | `src/vehicle_control/vehicle_control/joy_to_ackermann.py` |
| ackermann_to_vesc Node | `src/vehicle_control/vehicle_control/ackermann_to_vesc.py` |
| Container-Startskript | `ros_entrypoint.sh` |
| Basis-Dockerfile (Ubuntu) | `Dockerfile.ubuntu` |
| Basis-Dockerfile (L4T/GPU) | `Dockerfile.l4t` |
| Anpassungs-Dockerfile | `Dockerfile` |
| SSH-Template | `ssh_config_template` |
| Jetson-Cheatsheet | `CheatSheet.md` |
| ROS2-Best-Practices | `BestPractices.md` |
