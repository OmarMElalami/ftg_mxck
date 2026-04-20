# HowTo: Software Usage of the MXCK Carkit in SPEEDlab

> **Aktualisierte Version auf Basis der neuesten Projektstände, Workspace-Befunde, Container-/Compose-Struktur und Audit-Auswertung.**
>
> Dieses Dokument ersetzt die alte, knappe `HowTo.md` und dient als **praktische Betriebs-, Entwicklungs- und Übergabeanleitung** für das MXCK Carkit.

[[_TOC_]]

---


> **Hinweis zur Dokumentstruktur:**  
> Diese Version integriert die wesentlichen Inhalte der früheren Datei `MXCK_CARKIT_UEBERGABE_DOKU.md`.  
> Das vorliegende `HowTo.md` ist damit das **zentrale Hauptdokument** für Betrieb, Einarbeitung und technische Übergabe.  
> Stark operative Inhalte stehen in den Hauptkapiteln; zusätzliche tiefere Architektur- und Audit-Hinweise befinden sich im **technischen Anhang** am Ende.

---

## 1. Zweck dieses Dokuments

Diese Anleitung beschreibt die **aktuelle Nutzung der Software des MXCK Carkits** in einer Form, die für neue Bearbeiter direkt nutzbar ist.

Sie deckt ab:

- System- und Architekturüberblick
- sichere Inbetriebnahme
- Verbindungswege zum Fahrzeug
- aktuelle Docker-/ROS2-Workflows
- Startreihenfolge der Software
- Start einzelner Sensoren und Funktionen
- wichtige ROS2-Befehle
- Datenaufnahme und Visualisierung
- Entwicklung neuer ROS2-Pakete und Nodes
- FTG-/Autonomie-Kontext
- THM GitLab Setup via SSH unter Windows + VS Code
- typische Fehler und Best Practices

---

## 2. Kurzüberblick über das MXCK Carkit

Das MXCK Carkit ist ein **containerisiertes ROS2-Fahrzeugsystem** auf einem **NVIDIA Jetson**.

Typische Hardware des Fahrzeugs:

- **Jetson** als zentrale Recheneinheit
- **RPLiDAR** für 2D-Laserscans (`/scan`)
- **Intel RealSense D435i** für RGB / Depth / IMU
- **STM32 Nucleo** für micro-ROS / RC / Ultraschall / IMU-nahe Anbindung
- **VESC + Servo** für Motor und Lenkung
- **RC-Controller / Joystick** für sichere manuelle Kontrolle

Wichtige Software-Prinzipien:

- ROS2 läuft **in Docker-Containern**, nicht direkt „frei“ auf dem Host
- Sensorik und Fahrzeugfunktionen sind **in mehrere Container getrennt**
- es gibt einen **Basis-Fahrzeugstack** und einen **Entwicklungs-/Experimentierstack**
- neue autonome Funktionen sollen **in die bestehende Steuerkette integriert** werden, nicht direkt Motor-/Servoebenen umgehen

---

## 3. Sicherheitsregeln vor jedem Start

### 3.1 Fahrzeug sichern

Vor jedem Softwarestart muss sichergestellt sein, dass:

- das Fahrzeug stabil steht
- kein Hindernis direkt vor dem Fahrzeug ist
- die Räder bei Antriebstests möglichst frei drehen können
- eine Person jederzeit eingreifen kann

### 3.2 Manuelle Kontrolle verstehen

Bevor autonome Funktionen getestet werden, muss klar sein:

- wie das Fahrzeug **sofort gestoppt** werden kann
- wie man auf **manuellen Modus** zurückschaltet
- ob ein **Deadman / Safety-Mechanismus** aktiv ist
- wie notfalls der Container oder Node gestoppt wird

### 3.3 Grundregel

**Erst Sensorik prüfen, dann manuelle Steuerung prüfen, erst danach autonome Funktionen testen.**

---

## 4. Architekturübersicht

## 4.1 Host-System

Der Jetson ist die physische Plattform und übernimmt:

- Ubuntu / L4T / Tegra-Betriebssystem
- Docker
- USB-/Seriellinks zu LiDAR, STM32, VESC, Kamera
- Netzwerkzugang (WLAN / Ethernet / USB)
- GPU-nahe Workloads (z. B. YOLO im Development-Stack)

Das Fahrzeug ist also **kein einzelnes ROS2-Projekt**, sondern ein **mehrschichtiges Gesamtsystem**.

---

## 4.2 Die zwei wichtigsten Workspaces

### A) `mxck2_ws` = Basis-Fahrzeugstack

Dieser Workspace enthält die carkitnahen Kernpakete wie:

- `mxck_run`
- `vehicle_control`

Er ist der richtige Ort für:

- Sensorstart
- Foxglove / TF
- manuelle Steuerung
- stabile Fahrzeugbasis

### B) `development_ws` = Entwicklungs- und Experimentierstack

Dieser Workspace enthält zusätzliche Entwicklungsprojekte, z. B.:

- FTG / Hindernisvermeidung
- YOLO / Vision
- PDC / Ultraschall-Visualisierung
- Parking / Testumgebungen
- ältere oder parallele Forschungsstände

Typische Pakete/Strukturen:

- `ftg_mxck`
- `follow_the_gap_v0`
- `obstacle_substitution`
- `mxck_ftg_perception`
- `mxck_ftg_planner`
- `mxck_ftg_control`
- `mxck_ftg_bringup`
- `speedlab_carkit`
- `parking_detection`
- `pdc_visualization`
- `yolo_vision`

---

## 4.3 Kritisch: gleicher Containerpfad, aber unterschiedliche Host-Ordner

Ein sehr wichtiger Punkt:

In mehreren Containern heißt der Workspace im Container einfach:

```bash
/mxck2_ws
```

Aber je nach Container ist das **nicht derselbe Host-Ordner**.

### Basiscontainer

Hier zeigt `/mxck2_ws` typischerweise auf:

```bash
/home/mxck/mxck2_ws
```

### Development-Container

Hier zeigt `/mxck2_ws` typischerweise auf:

```bash
/home/mxck/development_ws
```

### Konsequenz

Der Pfadname allein reicht nicht aus, um zu wissen, in welchem Projektstand man arbeitet.

### Deshalb immer zuerst prüfen:

```bash
pwd
mount | grep mxck2_ws
ls -lah /mxck2_ws
```

Oder auf Host-Seite:

```bash
docker inspect <container_name> --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
```

---

## 4.4 Containerrollen

Typische Container im System:

- `mxck2_control`
- `mxck2_lidar`
- `mxck2_foxglove`
- `mxck2_micro`
- `mxck2_camera`
- `mxck2_kickstart`
- `mxck2_development`
- je nach Entwicklungsstand zusätzlich `mxck2_pdc`, `mxck2_yolo`
- `portainer`

### Bedeutung der wichtigsten Container

#### `mxck2_control`
Basis-Fahrzeugsteuerung, Fahrlogik, VESC-/Ackermann-nahe Steuerkette.

#### `mxck2_lidar`
LiDAR-Start und Publikation von `/scan`.

#### `mxck2_foxglove`
Foxglove Bridge / Visualisierung, meist zusammen mit TF.

#### `mxck2_micro`
micro-ROS-Anbindung des STM32-Controllers.

#### `mxck2_camera`
RealSense- bzw. Kamera-Topics.

#### `mxck2_kickstart`
Startpfad für manuelle Fahrzeugsteuerung.

#### `mxck2_development`
Entwicklungsumgebung für FTG, YOLO, Parking und weitere experimentelle Komponenten.

---

## 4.5 ROS2-Overlay-Prinzip

Der Basisstack arbeitet nicht nur mit einem einzelnen Workspace, sondern als **Overlay-Kette**.

Typischer Basisstart:

```bash
source /opt/ros/<distro>/setup.bash
source /microros_ws/install/setup.bash
source /vesc_ws/install/setup.bash
source /rplidar_ws/install/setup.bash
source /mxck2_ws/install/setup.bash
```

### Bedeutung

- `/opt/ros/...` = ROS2-Basis
- `/microros_ws` = STM32 / micro-ROS
- `/vesc_ws` = VESC-Treiber
- `/rplidar_ws` = LiDAR-Treiber
- `/mxck2_ws` = fahrzeugspezifische Pakete

Das erklärt, warum Sensor- oder Aktorikpakete verfügbar sein können, obwohl sie nicht direkt im gerade sichtbaren Quellbaum liegen.

---

## 4.6 ROS2-Distributionen

Ein wichtiger Praxispunkt:

- der **Basisstack** arbeitet typischerweise mit **ROS2 Humble**
- im **Development-Container** wurde in Audit-/Projektständen teilweise **ROS2 Foxy** gefunden

### Konsequenz

Vor jeder Arbeit im Container zuerst prüfen:

```bash
echo $ROS_DISTRO
cat /etc/os-release
```

Gerade bei Builds und Launches darf man keine Befehle blind von einem Stack in den anderen kopieren.

---

## 5. Datenfluss im Fahrzeug

## 5.1 Sensorpfade

### LiDAR-Datenfluss

```text
RPLiDAR -> mxck2_lidar -> /scan -> Wahrnehmung / FTG / Visualisierung
```

### Kamera-Datenfluss

```text
RealSense / Video -> mxck2_camera -> /camera/... -> Visualisierung / Vision / Logging
```

### STM32 / micro-ROS-Datenfluss

```text
STM32 / RC / Ultraschall / IMU -> mxck2_micro -> ROS2 Topics
```

---

## 5.2 Steuerpfad

Die Fahrzeugsteuerung soll grundsätzlich über die bestehende Kette laufen.

Gedachter Steuerpfad:

```text
Manuell oder Autonom -> Ackermann-Kommando -> vehicle_control / ackermann_to_vesc -> VESC / Servo / Motor
```

### Wichtige Regel

Autonome Funktionen wie FTG sollen **nicht direkt Motor- oder Servo-Kommandos hart überschreiben**, sondern sauber an die vorhandene Steuerkette angebunden werden.

---

## 5.3 FTG-Datenfluss

Die geplante / sinnvolle FTG-Kette ist modular:

```text
/scan -> Preprocessing / Hindernisaufbereitung -> Planner (Gap-Berechnung) -> Control (Ackermann) -> Autonomer Steuerpfad
```

Typische Module im Development-Stack:

- `mxck_ftg_perception`
- `mxck_ftg_planner`
- `mxck_ftg_control`
- `mxck_ftg_bringup`
- `follow_the_gap_v0`
- `obstacle_substitution`

### Ziel

- LiDAR auswerten
- größte Lücke bestimmen
- sichere Sollrichtung bestimmen
- konservative Geschwindigkeit wählen
- Ackermann-Kommando an die bestehende Fahrzeuglogik übergeben

---

## 6. Verbindungswege zum Fahrzeug

## 6.1 Direkt am Fahrzeug (Host-System)

Die direkte Arbeit am Host ist möglich, z. B. über:

- HDMI + Bildschirm
- Tastatur / Maus
- lokale Shell auf dem Jetson

### Host-Zugang

Benutzername und Passwort sind im alten Workflow typischerweise:

- User: `mxck`
- Passwort: `mxck`

### Wichtiger Hinweis

Die alte Arbeitsweise mit einem direkten Host-Skript wie `run_ros_docker.sh` ist als **Legacy-/Altweg** zu verstehen.

Die aktuelle, robustere Arbeitsweise ist:

- Container über **Portainer** oder **Compose** verwalten
- gezielt in den richtigen Container wechseln
- Sensoren und Funktionen modular starten

---

## 6.2 Netzwerkverbindung

Typische Optionen:

- WLAN
- Ethernet
- USB-Netzwerk
- Hotspot

Auf dem Jetson kann die aktuelle Adresse z. B. geprüft werden mit:

```bash
hostname
ip a
ifconfig
```

---

## 6.3 SSH-Zugriff

SSH ist der empfohlene Weg für die tägliche Arbeit.

### Verbinden

```bash
ssh mxck@<JETSON-IP>
```

### Vorteil

- kein lokaler Bildschirm nötig
- direktes Arbeiten vom Laptop aus
- kombinierbar mit VS Code Remote SSH
- Git / Entwicklung / Terminal an einem Ort

---

## 6.4 VS Code Remote Development

Empfohlener Workflow:

1. per SSH auf den Jetson verbinden
2. in VS Code den Remote-Zugriff nutzen
3. bei Bedarf Devcontainer / Docker-Container öffnen
4. dort entwickeln, bauen und testen

Das ist deutlich angenehmer und reproduzierbarer als direkte Arbeit nur per Host-Terminal.

---

## 7. Standard-Startreihenfolge im Betrieb

Die wichtigste Regel lautet:

**Nicht alles gleichzeitig blind starten. Erst die Basis prüfen, dann Sensoren einzeln, dann Steuerung, dann Autonomie.**

## 7.1 Schritt 1: Host und Containerstatus prüfen

Auf dem Jetson:

```bash
docker ps
docker ps --format "table {{.Names}}\t{{.Status}}"
```

Optional mit Portainer prüfen.

---

## 7.2 Schritt 2: in den richtigen Container wechseln

### Basisbetrieb

```bash
sudo docker exec -it mxck2_control bash
```

oder je nach Aufgabe:

```bash
sudo docker exec -it mxck2_lidar bash
sudo docker exec -it mxck2_foxglove bash
sudo docker exec -it mxck2_micro bash
```

### Entwicklung / FTG / YOLO / Parking

```bash
sudo docker exec -it mxck2_development bash
```

---

## 7.3 Schritt 3: Umgebung prüfen

Im Container:

```bash
echo $ROS_DISTRO
pwd
mount | grep mxck2_ws
```

Wenn nötig zusätzlich sourcen:

```bash
source /opt/ros/humble/setup.bash
source /mxck2_ws/install/setup.bash
```

oder im Development-Stack passend zur dortigen Distribution.

---

## 7.4 Schritt 4: ROS2-Grundzustand prüfen

```bash
ros2 topic list
ros2 node list
```

---

## 7.5 Schritt 5: Sensoren einzeln prüfen

### LiDAR

```bash
run_lidar
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

### Kamera

```bash
run_camera
ros2 topic list | grep camera
```

### RGB-D

```bash
run_rgbd
ros2 topic list | grep camera
```

### IMU

```bash
run_imu
ros2 topic list | grep imu
```

### Foxglove + TF

```bash
run_foxglove
ros2 topic list | grep tf
```

### micro-ROS

```bash
run_micro
ros2 topic list
```

---

## 7.6 Schritt 6: manuelle Steuerung prüfen

```bash
kickstart
```

oder direkt:

```bash
ros2 launch vehicle_control manual_control_launch.py
```

### Hinweise

- erst warten, bis die Systemchecks abgeschlossen sind
- Controller / Joystick korrekt einschalten
- nur dann manuell fahren, wenn der Notstopp-/Rückfallpfad klar ist

---

## 7.7 Schritt 7: erst danach Autonomie / FTG testen

Erst wenn folgende Punkte stimmen:

- `/scan` ist plausibel
- Kamera / TF sind bei Bedarf da
- manuelle Steuerung funktioniert
- die Safety-Kette ist verstanden

Dann erst FTG / autonome Module starten.

---

## 8. Wichtige Aliases und Launch-Befehle

Im Workspace existieren wichtige Shell-Aliases.

### 8.1 Aliases

```bash
run_foxglove
kickstart
run_micro
run_lidar
run_camera
run_rgbd
run_imu
run_ir_left
run_ir_right
run_projector
run_vio
```

### 8.2 Bedeutung

#### Foxglove + TF

```bash
run_foxglove
```

entspricht sinngemäß:

```bash
ros2 launch mxck_run mxck_run_launch.py run_foxglove:=true broadcast_tf:=true
```

#### Manuelle Steuerung

```bash
kickstart
```

entspricht sinngemäß:

```bash
ros2 launch vehicle_control manual_control_launch.py
```

#### LiDAR

```bash
run_lidar
```

entspricht sinngemäß:

```bash
ros2 launch mxck_run mxck_run_launch.py run_lidar:=true
```

#### Kamera

```bash
run_camera
```

#### RGB-D

```bash
run_rgbd
```

#### IMU

```bash
run_imu
```

#### Kamera + IMU (VIO)

```bash
run_vio
```

---

## 9. Aktuelle Bewertung der alten `run_ros_docker.sh`-Methode

Die alte Anleitung beschrieb ungefähr diesen Weg:

1. auf den Jetson gehen
2. `cd ~/mxck2_ws`
3. `bash run_ros_docker.sh`
4. dann im Container arbeiten

### Bewertung heute

Dieser Weg ist **nicht die bevorzugte Standardmethode mehr**.

### Warum?

Weil das aktuelle System:

- mehrere spezialisierte Container nutzt
- getrennte Workspaces besitzt
- Portainer / Compose / Devcontainer nutzt
- je nach Aufgabe unterschiedliche Container braucht

### Empfehlung

Die alte Methode höchstens als **Legacy-Hinweis** dokumentieren, aber nicht als Hauptworkflow verwenden.

Der neue Standardworkflow ist:

- Host prüfen
- laufende Container prüfen
- gezielt den richtigen Container wählen
- Sensorik modular starten
- Entwicklung im `mxck2_development`-Container

---

## 10. Running the Software Stack

## 10.1 Starten des Basisstacks über einzelne Dienste

Je nach gewünschter Funktion gezielt starten:

### LiDAR

```bash
run_lidar
```

### Kamera

```bash
run_camera
```

### Foxglove

```bash
run_foxglove
```

### micro-ROS

```bash
run_micro
```

### Manuelle Fahrzeugsteuerung

```bash
kickstart
```

---

## 10.2 Sinnvolle Startreihenfolge in der Praxis

Für typische LiDAR-/FTG-Arbeit:

1. `run_lidar`
2. `run_foxglove`
3. `kickstart`
4. danach FTG-/Development-Komponenten

Für Kamera-/Vision-Arbeit:

1. `run_camera` oder `run_rgbd`
2. `run_foxglove`
3. bei Bedarf `mxck2_yolo` / Development-Komponenten

---

## 10.3 Running Individual ROS 2 Nodes

Zur Diagnose und Entwicklung sind diese Standardbefehle wichtig:

```bash
ros2 topic list
ros2 node list
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic info /scan
ros2 param list
ros2 param get <node> <param_name>
```

Beispiele:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic list | grep camera
ros2 topic list | grep tf
```

---

## 10.4 Launch Files

Typische Launch-Aufrufe:

### Manuelle Steuerung

```bash
ros2 launch vehicle_control manual_control_launch.py
```

### Kamera über `mxck_run`

```bash
ros2 launch mxck_run mxck_run_launch.py run_camera:=true
```

### LiDAR über `mxck_run`

```bash
ros2 launch mxck_run mxck_run_launch.py run_lidar:=true
```

### Foxglove + TF

```bash
ros2 launch mxck_run mxck_run_launch.py run_foxglove:=true broadcast_tf:=true
```

---

## 11. Developing New ROS 2 Nodes

## 11.1 Wo entwickeln?

Neue experimentelle oder autonome Funktionen sollten in der Regel im:

```bash
mxck2_development
```

Container bzw. im dazugehörigen Entwicklungsworkspace entwickelt werden.

### Warum?

Weil dort:

- FTG-/Autonomie-Pakete liegen
- zusätzliche Python-/AI-Pakete vorhanden sind
- Entwicklungsarbeit vom Basis-Fahrzeugstack getrennt bleibt

---

## 11.2 Basisregel für neue Funktionen

Neue autonome Nodes sollen:

- sauber in die ROS2-Architektur passen
- nachvollziehbare Topics verwenden
- nicht direkt Low-Level-Fahrbefehle hart umgehen
- debugbar und modular sein
- bei Problemen leicht abschaltbar sein

---

## 11.3 Empfohlene Vorgehensweise

1. Eingangs-Topic klar definieren
2. Ausgangs-Topic klar definieren
3. Node isoliert testen
4. mit rosbag / Foxglove prüfen
5. erst danach im Gesamtsystem verwenden

---

## 11.4 Bauen eines Workspaces

Im passenden Container:

```bash
cd /mxck2_ws
colcon build --symlink-install
source install/setup.bash
```

### Nur ausgewählte Pakete

```bash
colcon build --symlink-install --packages-select <pkg1> <pkg2>
```

### Wichtig

Build-Befehle nur in der passenden ROS2-Umgebung und im passenden Container ausführen.

---

## 11.5 Typische Prüfung nach dem Build

```bash
source install/setup.bash
ros2 pkg list | grep <paketname>
ros2 launch <paketname> <launchfile>
```

---

## 12. FTG / Obstacle Avoidance im MXCK-System

## 12.1 Ziel

FTG (Follow-The-Gap) soll LiDAR-Daten auswerten, Hindernisse erkennen und eine sichere Fahrtrichtung bestimmen.

### Typischer fachlicher Ablauf

1. LiDAR-Scan empfangen
2. störende Messwerte filtern / Frontbereich prüfen
3. Hindernisse / belegte Bereiche erkennen
4. größte freie Lücke bestimmen
5. Sollrichtung berechnen
6. Ackermann-Steuerkommando ableiten

---

## 12.2 Warum FTG nicht direkt auf den VESC schreiben soll

Im MXCK-System existiert bereits eine Fahrzeugsteuerkette mit Safety / Mode-Logik.

Deshalb soll FTG:

- seine Ergebnisse als saubere ROS2-Kommandos publizieren
- sich in die bestehende Steuerarchitektur einfügen
- nicht an Safety vorbei direkt Servo-/Motor-Kommandos erzwingen

---

## 12.3 Typische FTG-Module im Development-Stack

- `mxck_ftg_perception`
- `mxck_ftg_planner`
- `mxck_ftg_control`
- `mxck_ftg_bringup`
- `follow_the_gap_v0`
- `obstacle_substitution`
- `obstacle_msgs`

---

## 12.4 FTG-Start nur nach erfolgreicher Basisprüfung

Vor FTG immer zuerst:

```bash
run_lidar
ros2 topic echo /scan --once
ros2 topic hz /scan
run_foxglove
kickstart
```

Erst wenn `/scan` plausibel ist und die manuelle Basis funktioniert, FTG starten.

---

## 13. Measurements (Data Recording and Replaying)

## 13.1 Recording ROS Topics (rosbag2)

Für Diagnosen, FTG-Analyse und Reproduzierbarkeit sollten wichtige Topics aufgenommen werden.

Typische Aufnahme:

```bash
ros2 bag record -o run01 /scan
```

Mehrere Topics:

```bash
ros2 bag record -o run01 /scan /tf /tf_static /camera/color/image_raw
```

Bei Bedarf auch:

```bash
ros2 bag record -o ftg_debug /scan /tf /tf_static /autonomous/ackermann_cmd
```

---

## 13.2 Visualisierung mit Foxglove

Foxglove ist das Standardtool für die Live-Visualisierung.

Start:

```bash
run_foxglove
```

Danach im Client prüfen:

- `/scan`
- `/tf`
- `/tf_static`
- Kamerathemen
- ggf. FTG- oder Ackermann-Themen

### Warum wichtig?

Foxglove ist eines der besten Mittel, um:

- falsche TF-Frames zu erkennen
- fehlende Topics zu sehen
- LiDAR- oder Kamera-Daten visuell zu prüfen
- Debugging schnell zu machen

---

## 13.3 Replaying Rosbags

Wiedergabe:

```bash
ros2 bag play <bag_ordner>
```

Wenn währenddessen Visualisierung läuft:

```bash
run_foxglove
ros2 bag play <bag_ordner>
```

---

## 14. Troubleshooting / typische Fehlerbilder

## 14.1 `ros2: command not found`

### Mögliche Ursache

Die Shell wurde im Container geöffnet, aber die Umgebung wurde nicht korrekt gesourct.

### Lösung

```bash
source /opt/ros/humble/setup.bash
source /mxck2_ws/install/setup.bash
```

Oder im Development-Container passend zur dortigen ROS2-Distribution.

---

## 14.2 Falscher Workspace im Container

### Symptom

- Pakete fehlen
- andere Dateien als erwartet
- Launches sind plötzlich nicht da

### Ursache

`/mxck2_ws` zeigt je nach Container auf unterschiedliche Host-Ordner.

### Lösung

```bash
mount | grep mxck2_ws
ls -lah /mxck2_ws
```

---

## 14.3 `ros2 topic echo /scan` zeigt nichts

### Prüfen

```bash
run_lidar
ros2 topic list | grep scan
ros2 topic info /scan
ros2 topic echo /scan --once
```

### Mögliche Ursachen

- LiDAR-Container nicht gestartet
- falscher Container
- Gerät nicht erkannt
- Treiberproblem

---

## 14.4 Foxglove zeigt keine sinnvolle Geometrie

### Mögliche Ursache

- TF fehlt
- `broadcast_tf` nicht aktiv
- falscher Frame-Name
- Sensorframe nicht korrekt eingebunden

### Lösung

```bash
run_foxglove
ros2 topic list | grep tf
```

---

## 14.5 Kamera-Container startet nicht stabil

### Mögliche Ursachen

- Workspace-/Build-/Python-Fehler
- Package Identification Problem
- Device-Zugriffsproblem

### Prüfen

```bash
docker logs mxck2_camera
ros2 topic list | grep camera
```

---

## 14.6 Kickstart / VESC-Startproblem

### Mögliche Ursache

- Serienkommunikation zum VESC fehlerhaft
- falsches Device
- Datenrahmenproblem

### Prüfen

```bash
docker logs mxck2_kickstart
ls -l /dev/vesc
```

---

## 15. Software Best Practices

## 15.1 Erst prüfen, dann fahren

Immer zuerst:

- Containerstatus
- ROS2-Umgebung
- Sensor-Topic
- Visualisierung
- manuelle Steuerung

---

## 15.2 Kleine Schritte

Nicht mehrere Baustellen gleichzeitig ändern.

Besser:

- eine Funktion ändern
- isoliert testen
- loggen / visualisieren
- dann integrieren

---

## 15.3 Debugging mit Daten

Nicht nur „gefühlt testen“, sondern:

- `ros2 topic echo`
- `ros2 topic hz`
- `ros2 topic info`
- `ros2 bag`
- Foxglove

verwenden.

---

## 15.4 Autonomie nur mit Sicherheitsrückfall

- niedrige Geschwindigkeit
- manuelle Übersteuerung bereit
- genug Platz
- klare Testumgebung

---

## 15.5 Basisstack nicht unnötig destabilisieren

Neue Features möglichst im Development-Stack entwickeln und erst dann in die stabile Fahrzeugbasis überführen.

---

## 16. THM GitLab Setup via SSH + Windows PowerShell + VS Code

Dieser Abschnitt integriert die bewährte Einrichtung für **THM GitLab (`git.thm.de`)**.

---

## 16.1 Wichtig: die richtige GitLab-Instanz

- ✅ THM GitLab: `https://git.thm.de/...`
- ❌ nicht `gitlab.com`

SSH-Zugang:

- Host: `git.thm.de`
- User: `git`
- Port: `22`

---

## 16.2 Voraussetzungen prüfen

### Git-Version

```powershell
git --version
```

### SSH verfügbar?

```powershell
ssh -V
```

---

## 16.3 SSH-Key erstellen

```powershell
ssh-keygen -t ed25519 -C "deine.mail@thm.de"
```

Empfehlung:

- Standardpfad verwenden:
  `C:\Users\<User>\.ssh\id_ed25519`
- Passphrase setzen

Public Key anzeigen:

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub
```

---

## 16.4 SSH-Agent aktivieren

In **Admin-PowerShell**:

```powershell
Set-Service -Name ssh-agent -StartupType Automatic
Start-Service ssh-agent
```

Key hinzufügen:

```powershell
ssh-add $env:USERPROFILE\.ssh\id_ed25519
```

Prüfen:

```powershell
ssh-add -l
```

Wichtig:

- `ssh-add -l` = kleines L
- nicht `-1`

---

## 16.5 Public Key in THM GitLab speichern

1. `https://git.thm.de` öffnen
2. einloggen
3. Profil → **Settings / Preferences**
4. **SSH Keys**
5. Public Key einfügen
6. Titel vergeben
7. speichern

---

## 16.6 Verbindung testen

```powershell
ssh -T git@git.thm.de
```

Erwartet:

```text
Welcome to GitLab, @<username>!
```

Beim ersten Verbindungsaufbau Hostabfrage mit `yes` bestätigen.

---

## 16.7 Im richtigen Repository-Ordner arbeiten

Beispiel:

```powershell
cd "C:\Users\Omar\Desktop\Studium\Master\Mxck carkit\Programm"
git status
git rev-parse --show-toplevel
```

Wenn `fatal: not a git repository ...` erscheint, bist du im falschen Ordner oder das Repo wurde noch nicht geklont.

---

## 16.8 Remote von HTTPS auf SSH umstellen

Remote prüfen:

```powershell
git remote -v
```

Wenn dort HTTPS steht, auf SSH ändern:

```powershell
git remote set-url origin git@git.thm.de:<gruppe>/<repo>.git
git remote -v
```

Beispiel:

```powershell
git remote set-url origin git@git.thm.de:bltk30/speedlab_carkit.git
git remote -v
```

---

## 16.9 Branches anzeigen und wechseln

Remote-Branches holen:

```powershell
git fetch --all --prune
git branch -a
```

Branch wechseln:

```powershell
git switch <branchname>
```

Wenn Branch nur remote existiert:

```powershell
git switch -c <branchname> --track origin/<branchname>
```

---

## 16.10 Standard-Workflow: Pull / Commit / Push

Änderungen holen:

```powershell
git pull --rebase
```

Status prüfen:

```powershell
git status
```

Stagen:

```powershell
git add .
```

Commit:

```powershell
git commit -m "kurze commit message"
```

Push:

```powershell
git push
```

Für neuen Branch:

```powershell
git push -u origin <branchname>
```

---

## 16.11 VS Code Workflow

1. Repo-Ordner in VS Code öffnen
2. **Source Control** öffnen (`Ctrl+Shift+G`)
3. Dateien ändern
4. Stage
5. Commit Message eingeben
6. Commit
7. Push / Sync Changes

### Wenn VS Code nach Passwort fragt

Dann ist häufig der Remote noch HTTPS statt SSH.

Prüfen:

```powershell
git remote -v
```

---

## 16.12 Häufige Git-/SSH-Probleme

### `Permission denied (publickey)`

Mögliche Ursachen:

- Key nicht in `git.thm.de` gespeichert
- falsche GitLab-Instanz
- falscher Host / User

Test:

```powershell
ssh -T git@git.thm.de
```

---

### Falscher Hostname

Richtig ist:

- ✅ `git.thm.de`
- ❌ `git.thm.com`

---

### VS Code fragt nach Passwort

Ursache meist: Remote ist noch HTTPS.

Lösung:

```powershell
git remote set-url origin git@git.thm.de:<gruppe>/<repo>.git
```

---

### Mehrere SSH-Keys

Dann SSH-Config anlegen:

Datei:

```text
C:\Users\<User>\.ssh\config
```

Inhalt:

```sshconfig
Host git.thm.de
  HostName git.thm.de
  User git
  Port 22
  IdentityFile ~/.ssh/id_ed25519
  IdentitiesOnly yes
```

---

## 16.13 Mini-Spickzettel GitLab-Alltag

```powershell
cd "C:\Users\Omar\Desktop\Studium\Master\Mxck carkit\Programm"

# Update
git fetch
git pull --rebase

# Work + Push
git status
git add .
git commit -m "message"
git push

# SSH Test
ssh -T git@git.thm.de
```

---

## 17. Useful Links / interne Orientierung

### Relevante Themen im Projekt

- ROS2 Grundlagen
- Docker / Devcontainer
- `mxck_run`
- `vehicle_control`
- LiDAR / `/scan`
- Foxglove / TF
- FTG / Obstacle Avoidance
- THM GitLab via SSH

### Praktische Orientierung

Für neue Bearbeiter gilt:

1. zuerst diese HowTo lesen
2. dann Container- und Workspace-Struktur prüfen
3. danach nur in kleinen, nachvollziehbaren Schritten arbeiten

---

## 18. Empfohlene erste Schritte für einen Nachfolger

### Schritt 1
Auf Jetson verbinden:

```bash
ssh mxck@<JETSON-IP>
```

### Schritt 2
Containerstatus prüfen:

```bash
docker ps --format "table {{.Names}}\t{{.Status}}"
```

### Schritt 3
LiDAR prüfen:

```bash
sudo docker exec -it mxck2_lidar bash
run_lidar
ros2 topic echo /scan --once
```

### Schritt 4
Foxglove / TF prüfen:

```bash
sudo docker exec -it mxck2_foxglove bash
run_foxglove
```

### Schritt 5
Manuelle Steuerung prüfen:

```bash
sudo docker exec -it mxck2_control bash
kickstart
```

### Schritt 6
Erst danach in den Development-Container gehen und FTG / neue Funktionen untersuchen.

---

## 19. Schlussbemerkung

Die wichtigste Erkenntnis dieser aktualisierten HowTo lautet:

**Das MXCK Carkit ist ein modular aufgebautes, containerisiertes ROS2-Fahrzeugsystem mit klarer Trennung zwischen stabiler Fahrzeugbasis und Entwicklungsumgebung.**

Wer das System sicher und produktiv nutzen will, muss immer zuerst verstehen:

- in welchem Container er arbeitet
- welcher Workspace dort wirklich gemountet ist
- welche ROS2-Distribution aktiv ist
- und ob er sich gerade im Basisbetrieb oder im Entwicklungsstack befindet

Dann lassen sich Sensorik, Steuerung und autonome Funktionen sauber und sicher weiterentwickeln.



---

## 20. Technischer Anhang: detaillierte Architektur- und Audit-Hinweise

Dieser Anhang verdichtet die früher separat geführte `MXCK_CARKIT_UEBERGABE_DOKU.md` in einer Form, die für Nachfolger, Systemanalyse und spätere Erweiterungen nützlich ist.

### 20.1 Fünf-Schichten-Modell des Systems

Das MXCK Carkit lässt sich am verständlichsten als **fünfschichtiges System** beschreiben:

1. **Hardware-Schicht**
   - Jetson
   - RPLiDAR
   - RealSense / weitere Videoquellen
   - STM32 Nucleo
   - VESC
   - RC / Fernsteuerung
   - ggf. Ultraschallsensoren

2. **Host-Schicht**
   - Ubuntu / L4T / Tegra
   - Docker
   - USB-/Seriellinks unter `/dev`
   - Netzwerk / WLAN / Portainer
   - GPU-nahe Laufzeit

3. **Container-Schicht**
   - `mxck2_control`
   - `mxck2_lidar`
   - `mxck2_foxglove`
   - `mxck2_micro`
   - `mxck2_camera`
   - `mxck2_kickstart`
   - `mxck2_development`
   - optional `mxck2_pdc`, `mxck2_yolo`

4. **ROS2-Overlay-Schicht**
   - `/opt/ros/<distro>`
   - `/microros_ws`
   - `/vesc_ws`
   - `/rplidar_ws`
   - `/mxck2_ws`

5. **Projekt-/Funktions-Schicht**
   - stabiler Fahrzeugbetrieb in `mxck2_ws`
   - FTG / YOLO / PDC / Parking / Experimente in `development_ws`

### 20.2 Reale Host-Rolle des Jetson

Der Jetson ist nicht bloß ein Rechner zum Ausführen einzelner Nodes, sondern die zentrale Plattform für das gesamte Fahrzeug. Er übernimmt gleichzeitig:

- Containerbetrieb
- USB-/Seriellinks zur Sensorik und Aktorik
- Netzwerkzugriff
- GPU-/Vision-Beschleunigung
- ROS2-Kommunikation
- Logging / Datenaufzeichnung

Praktisch bedeutet das: Fehler können auf unterschiedlichen Ebenen liegen – Hardware, USB-Link, Docker, Overlay, ROS2-Graph oder Projektcode.

### 20.3 Geräte- und `/dev`-Perspektive

Die Carkit-Architektur ist stark von den durchgereichten Linux-Geräten abhängig. Typischerweise existieren stabile symbolische Links für:

```bash
/dev/rplidar
/dev/stm32_nucleo
/dev/vesc
```

Die konkrete Gerätedatei im Hintergrund kann je nach Neustart wechseln (`ttyUSB0`, `ttyACM0`, `ttyACM1`, ...), aber der stabile Link soll die Fachfunktion beschreiben.

**Merksatz:**  
Nicht mit „welches tty ist es heute?“ arbeiten, sondern mit den stabilen Gerätenamen, wenn sie vorhanden sind.

Praktische Prüfung:

```bash
ls -l /dev/rplidar
ls -l /dev/stm32_nucleo
ls -l /dev/vesc
lsusb
```

### 20.4 Containerrollen im Detail

#### `mxck2_control`
Ort der fahrzeugnahen Basissteuerung. Typischer Ort für:

- `vehicle_control`
- VESC-/Ackermann-Kette
- Fahrlogik
- ggf. Safety- und Mode-Logik

#### `mxck2_lidar`
Sensorcontainer für den RPLiDAR. Aufgabe:

- LiDAR-Treiber starten
- `/scan` publizieren
- LiDAR isoliert testbar machen

#### `mxck2_foxglove`
Visualisierung und Topic-Bridge. Besonders nützlich für:

- `/scan`
- `/tf_static`
- Diagnose des ROS2-Graphs

#### `mxck2_micro`
Brücke zur STM32-/micro-ROS-Welt. Typische Themen:

- RC-Signale
- Ultraschall
- IMU-nahe Daten
- Zustands-/Hilfstopics

#### `mxck2_camera`
Kamerabezogener Container, z. B. für RealSense oder andere Videoquellen.

#### `mxck2_kickstart`
Startpfad für manuelle Fahrfunktion / Initialtest des Fahrpfads.

#### `mxck2_development`
Entwicklungscontainer für experimentelle, autonome und GPU-nahe Funktionen wie:

- FTG
- YOLO
- Parking / PDC
- weitere Forschungs- und Testprojekte

### 20.5 Warum im Projekt so leicht Verwirrung entsteht

Die Hauptgründe für Missverständnisse im MXCK-System sind:

1. gleicher Containerpfad `/mxck2_ws`, aber unterschiedliche Host-Ordner  
2. Basis- und Entwicklungswelt liegen fachlich nah beieinander  
3. es wurden unterschiedliche ROS2-Distributionen gefunden  
4. nicht jeder `docker exec` startet in einer vollständig gesourcten ROS-Umgebung  
5. im Projektbaum existieren aktive Pakete, historische Stände, Build- und Install-Artefakte nebeneinander

### 20.6 Interpretation typischer Audit-Befunde

#### A) `ros2: command not found`
Das bedeutet im MXCK-Kontext **nicht automatisch**, dass der Container oder Stack defekt ist. Häufige Ursache:

- der Audit- oder Shell-Aufruf lief **ohne korrekt gesourcte Umgebung**

Deshalb immer zuerst:

```bash
source /opt/ros/<distro>/setup.bash
source /mxck2_ws/install/setup.bash
echo $ROS_DISTRO
which ros2
```

#### B) Topic ist in Foxglove sichtbar, aber Shell-Tests schlagen fehl
Dann ist oft der **Service korrekt gestartet**, aber die **interaktive Shell** nicht vorbereitet.

#### C) Container startet, beendet sich aber direkt wieder
Dann prüfen:

```bash
docker ps -a
docker logs <container>
```

Häufige Ursachen sind:

- falscher Workspace
- fehlerhafte Python-/colcon-Umgebung
- fehlende Hardware
- Serien-/TF-/Frame-Probleme

### 20.7 Detaillierter Datenfluss des Systems

#### Sensorik

```text
RPLiDAR -> mxck2_lidar -> /scan
RealSense / Kamera -> mxck2_camera -> /camera/...
STM32 / micro-ROS -> mxck2_micro -> RC / IMU / Ultraschall / Zustände
```

#### Visualisierung

```text
ROS2 Topics -> mxck2_foxglove -> Foxglove Bridge -> externer Client
```

#### Steuerung

```text
Manual / Autonomous Command
    -> Ackermann-Kommando
    -> vehicle_control / ackermann_to_vesc
    -> VESC / Servo / Motor
```

#### FTG-Zielkette

```text
/scan
  -> Scan-Check / Frontfenster / Preprocessing
  -> Hindernisaufbereitung / obstacle_substitution
  -> Gap-Planung / FTG-Planner
  -> Ackermann-Kommando / FTG-Control
  -> bestehender autonomer Steuerpfad
```

### 20.8 Warum FTG modular integriert werden soll

FTG ist im Projekt nicht als einzelnes „Skript mit Lenkwinkel“ gedacht, sondern als **modulare Pipeline**:

- Wahrnehmung / Scanvorbereitung
- Planung / Gap-Berechnung
- Regelung / Ackermann-Kommando
- Bringup / Gesamtsystemstart

Das ist die richtige Architektur, weil dadurch:

- einzelne Teile testbar bleiben
- Debugging einfacher wird
- die bestehende Steuerkette respektiert wird
- Safety/Mode-Logik nicht umgangen wird

### 20.9 Was der Nachfolger immer zuerst prüfen soll

Vor jeder Fehlersuche oder Erweiterung:

```bash
docker ps
docker inspect <container> --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
echo $ROS_DISTRO
mount | grep mxck2_ws
ros2 topic list
ros2 node list
```

Dann gezielt:

```bash
ros2 topic echo /scan
ros2 topic list | grep tf
docker logs mxck2_lidar
docker logs mxck2_control
docker logs mxck2_foxglove
```

### 20.10 Klares Do / Don't für Nachfolger

#### Do
- zuerst Basisstack verstehen
- immer den aktiven Workspace prüfen
- Sensoren einzeln testen
- manuelle Steuerung vor Autonomie prüfen
- Logs und Bags zur Diagnose verwenden
- FTG über die bestehende Fahrzeugkette integrieren

#### Don't
- Humble- und Foxy-Kommandos blind mischen
- direkt auf Motor-/Servo-Themen schreiben, wenn schon eine Fahrkette existiert
- Build-, Install- und Quellbaum verwechseln
- alle Container gleichzeitig starten, ohne den Zustand zu verstehen
- aus einem zufälligen Container heraus annehmen, dass `/mxck2_ws` immer dasselbe Projekt ist

### 20.11 Empfohlener Lesepfad für neue Bearbeiter

Für schnelle Einarbeitung:

1. Kapitel 2 bis 8 dieses Dokuments  
2. Kapitel 10 bis 15 für Betrieb, Sensorik und Debugging  
3. Kapitel 12 für FTG/Obstacle Avoidance  
4. diesen technischen Anhang nur dann vertieft lesen, wenn:
   - Container/Workspace-Fragen auftreten
   - Sensor-/TF-/Overlay-Probleme auftreten
   - neue autonome Funktionen sauber integriert werden sollen
