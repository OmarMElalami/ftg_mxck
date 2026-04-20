# MXCK Carkit – Vollständige Übergabe-, Architektur- und Betriebsdokumentation

> Stand dieser Dokumentation: Konsolidierung aus Projekt-Chats, vorhandenen Workspace-Dateien, Compose-/Alias-Struktur und den ausgewerteten Audit-Informationen.
>
> Ziel: Ein Nachfolger soll das System verstehen, sicher starten, gezielt debuggen und neue Autonomiefunktionen – insbesondere LiDAR-basierte Hinderniserkennung und FTG – sauber in die bestehende Architektur integrieren können.

---

## 1. Zweck dieses Dokuments

Dieses Dokument erklärt das **MXCK Carkit** so, dass ein neuer Bearbeiter ohne Vorwissen die reale Systemarchitektur, die Betriebsweise und die korrekte Vorgehensweise versteht.

Es behandelt insbesondere:

- die **Gesamtarchitektur** aus Jetson, Docker, ROS2, Sensoren und Aktoren
- die Trennung zwischen **Basis-Fahrzeugstack** und **Entwicklungs-/Experimentierstack**
- die **Containerrollen** und ihre Aufgaben
- die **Datenflüsse** zwischen Sensorik, Verarbeitung, Visualisierung und Steuerung
- die **empfohlene Startreihenfolge**
- die **wichtigsten Kommandos**
- die **Einbindung von FTG (Follow-The-Gap)**
- typische **Fehlerbilder** und was sie bedeuten
- klare **Do/Don't-Regeln** für den sicheren Betrieb

---

## 2. Kernaussage zur Architektur

Das MXCK Carkit ist **kein einzelnes ROS2-Projekt**, sondern ein **mehrschichtiges, containerisiertes Fahrzeug-System** auf einem NVIDIA Jetson.

Die wichtigste Architektur-Trennung ist:

1. **`mxck2_ws`** = Basis-Fahrzeugstack
2. **`development_ws`** = Entwicklungs- und Experimentierstack

Diese Trennung ist entscheidend, weil in beiden Welten im Container oft derselbe Zielpfad verwendet wird (`/mxck2_ws`), obwohl auf dem Host **unterschiedliche Ordner** dahinter liegen.

Das führt schnell zu Verwechslungen, wenn man nicht bewusst prüft, **in welchem Container** und damit **in welchem Workspace** man arbeitet.

---

## 3. Gesamtübersicht des Systems

### 3.1 Hardware-Ebene

Das System besteht im Kern aus folgenden physischen Komponenten:

- **NVIDIA Jetson** als zentrale Recheneinheit
- **RPLiDAR A2M8 / A2M12** für 2D-Laserscans
- **Intel RealSense D435i** für RGB / Depth / IMU
- **STM32 Nucleo** mit micro-ROS bzw. serieller Anbindung
- **VESC** zur Motor- und Lenkungsansteuerung
- **RC-Empfänger / Remote Control** für manuelle Steuerung und Moduswechsel
- je nach Aufbau weitere Sensorik, z. B. **Ultraschallsensoren**

### 3.2 Host-Ebene (Jetson)

Der Jetson ist die Hardware-Plattform und übernimmt:

- Docker-Containerbetrieb
- USB-/Serial-Geräteverwaltung
- GPU-/Vision-Beschleunigung
- Netzwerkkommunikation
- Basis-Betriebssystem für ROS2-Container

Typische Eigenschaften aus dem Audit-Zustand:

- Ubuntu 20.04 L4T/Tegra
- aarch64
- Tegra-Kernel
- Root-Dateisystem auf NVMe
- Betrieb im lokalen Netzwerk mit bekannter IP, z. B. `192.168.0.100`

### 3.3 Software-Ebene

Die Software ist **funktionsorientiert in Docker-Container getrennt**. Das ist keine Nebensache, sondern ein Kernprinzip der MXCK-Architektur.

Ziel dieser Trennung:

- Sensorik separat starten und testen
- Visualisierung unabhängig betreiben
- Steuerung entkoppelt halten
- experimentelle Entwicklungen vom Basisbetrieb trennen
- reproduzierbare Umgebungen über Docker/Compose/Devcontainer nutzen

---

## 4. Die zwei zentralen Workspaces

## 4.1 Basis-Workspace: `mxck2_ws`

Dieser Workspace bildet den **stabilen Carkit-Basisbetrieb** ab.

Darin liegen insbesondere die Kernpakete für:

- Start-/Orchestrierungslogik
- Fahrzeugsteuerung
- Sensor-Launches
- Foxglove/TF/Bagging im Basisbetrieb

Wichtige Pakete:

- `mxck_run`
- `vehicle_control`

Dieser Workspace ist der richtige Ort für:

- regulären Fahrzeugbetrieb
- LiDAR/Kamera/Micro-ROS/TF-Start
- manuelle Steuerkette
- produktionsnahe Tests

---

## 4.2 Entwicklungs-Workspace: `development_ws`

Dieser Workspace enthält **zusätzliche Entwicklungs- und Forschungsfunktionen**.

Darin liegen u. a.:

- FTG-bezogene Pakete
- YOLO-/Vision-Komponenten
- PDC-/Ultraschall-Visualisierung
- Parking- und Testfunktionen
- ältere / experimentelle Projekte wie Lane Following / Pilotnet

Beispiele aus dem Projekt:

- `ftg_mxck`
  - `mxck_ftg_perception`
  - `mxck_ftg_planner`
  - `mxck_ftg_control`
  - `mxck_ftg_bringup`
  - `follow_the_gap_v0`
  - `obstacle_substitution`
  - `obstacle_msgs`
- `speedlab_carkit`
  - `parking_detection`
  - `pdc_visualization`
  - `drive_manager`
  - `yolo_vision`

Dieser Workspace ist der richtige Ort für:

- FTG-Weiterentwicklung
- autonome Experimentierfunktionen
- Sensorfusion- oder Wahrnehmungsprototypen
- Vision-/GPU-basierte Tests

---

## 5. Kritischer Architekturhinweis: gleicher Containerpfad, unterschiedliche Host-Ordner

Das ist einer der wichtigsten Punkte für den Nachfolger.

In mehreren Containern wird im Container **`/mxck2_ws`** verwendet.

Aber:

- in den **Basiscontainern** zeigt `/mxck2_ws` auf den Host-Ordner `~/mxck2_ws`
- im **Development-Container** zeigt `/mxck2_ws` auf den Host-Ordner `~/development_ws`

### Konsequenz

Der Pfadname im Container allein sagt **nicht**, in welchem Projektstand du gerade arbeitest.

### Deshalb immer prüfen:

```bash
pwd
mount | grep mxck2_ws
ls -lah /mxck2_ws
```

Oder außerhalb des Containers:

```bash
docker inspect <container_name> --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}'
```

### Merksatz

**Der Containerpfad `/mxck2_ws` ist nicht automatisch der gleiche Workspace. Entscheidend ist, welcher Host-Ordner dort gemountet wurde.**

---

## 6. ROS2- und Overlay-Architektur

## 6.1 Basisidee

Das System ist nicht nur ein einzelner ROS2-Workspace, sondern eine **Overlay-Kette**.

Typischer Basisstart im Carkit-Stack:

```bash
source /opt/ros/<distro>/setup.bash
source /microros_ws/install/setup.bash
source /vesc_ws/install/setup.bash
source /rplidar_ws/install/setup.bash
source /mxck2_ws/install/setup.bash
```

### Bedeutung der Schichten

1. **`/opt/ros/...`**
   - Basis-ROS2-Installation
2. **`/microros_ws`**
   - STM32 / micro-ROS-Integration
3. **`/vesc_ws`**
   - VESC-Treiber / fahrzeugnahe Ansteuerung
4. **`/rplidar_ws`**
   - RPLiDAR-Treiber
5. **`/mxck2_ws`**
   - carkitspezifische ROS2-Pakete

Diese Kette erklärt, warum das System auch dann funktionieren kann, wenn in einem einzelnen Workspace nicht alle Pakete im Quellbaum sichtbar sind.

---

## 6.2 Unterschiedliche ROS2-Distributionen

Im Projekt wurden unterschiedliche Umgebungen gefunden:

- **Basis-Carkit-Umgebung:** Humble-orientiert
- **Development-/GPU-/YOLO-/FTG-Umgebung:** teilweise Foxy-basiert

### Warum das wichtig ist

Dadurch entstehen Unterschiede bei:

- ROS2-CLI-Befehlen
- Build-Verhalten
- Python-Versionen
- Paketkompatibilitäten
- Overlays und Umgebungsvariablen

### Vor jedem Arbeiten deshalb zuerst prüfen

```bash
echo $ROS_DISTRO
cat /etc/os-release
printenv | sort | grep ROS
```

### Merksatz

**Vor einem Build oder Launch niemals annehmen, dass Humble und Foxy gleich behandelt werden können.**

---

## 7. Containerübersicht und Rollen

Die folgenden Container gehören zur realen Systemarchitektur:

| Container | Rolle |
|---|---|
| `mxck2_control` | Basis-Fahrzeugsteuerung / fahrzeugnahe ROS2-Logik |
| `mxck2_lidar` | RPLiDAR-Start und `/scan` |
| `mxck2_foxglove` | Foxglove Bridge / Visualisierung |
| `mxck2_micro` | micro-ROS-Agent / STM32-Anbindung |
| `mxck2_camera` | RealSense-/Kameralogik |
| `mxck2_kickstart` | manueller Fahrstart / Steuerkette |
| `mxck2_development` | Entwicklungscontainer für FTG, YOLO, Parking, Tests |
| `mxck2_pdc` | PDC-/Ultraschall-Visualisierung |
| `mxck2_yolo` | YOLO-/Vision-Pipeline |
| `portainer` | Weboberfläche zur Containerverwaltung |

---

## 8. Detaillierte Rollen der Hauptcontainer

## 8.1 `mxck2_control`

Dieser Container ist der zentrale **fahrzeugnahe Basiscontainer**.

Seine Aufgabe ist typischerweise:

- Start der Basis-ROS2-Umgebung
- Nutzung der Carkit-spezifischen Steuerpakete
- Brücke zwischen manueller/autonomer Kommandoebene und VESC
- Aufnahme in die Low-Level-Steuerkette

### Wichtig für Nachfolger

Wenn man verstehen will, wie das Fahrzeug **wirklich fährt**, ist `mxck2_control` einer der wichtigsten Container.

---

## 8.2 `mxck2_lidar`

Dieser Container startet den LiDAR-Zweig.

Aufgaben:

- RPLiDAR-Treiber starten
- `/scan` publizieren
- LiDAR im ROS2-Graph sichtbar machen

### Typische Nutzung

- LiDAR einzeln starten
- Scanformat prüfen
- FTG-/Perception-Tests vorbereiten

---

## 8.3 `mxck2_foxglove`

Dieser Container dient der **Live-Visualisierung**.

Aufgaben:

- Foxglove Bridge starten
- ROS2-Themen nach außen visualisieren
- TF, `/scan`, Kamera- oder Diagnose-Topics zugänglich machen

### Typische Nutzung

- Prüfen, ob `/scan` wirklich ankommt
- Prüfen, ob `/tf_static` / TF vorhanden ist
- spätere FTG-Diagnose-Topics darstellen

---

## 8.4 `mxck2_micro`

Dieser Container bindet den STM32-/micro-ROS-Zweig ein.

Mögliche Funktionen:

- RC-/Remote-Zustände
- Ultraschallsensoren
- IMU-Daten
- sonstige MCU-nahe Fahrzeugzustände

### Typische Aufgabe

Start des micro-ROS-Agenten und Herstellung der Kommunikation zum Mikrocontroller.

---

## 8.5 `mxck2_camera`

Dieser Container startet die Kamera-/RealSense-Logik.

Mögliche Topics:

- RGB
- Depth
- IMU
- IR
- RGBD

### Typische Nutzung

- visuelle Sensorik einzeln starten
- Kameratopics prüfen
- spätere Wahrnehmungspipelines vorbereiten

---

## 8.6 `mxck2_kickstart`

Dieser Container ist relevant für den **manuellen Steuerungsstart**.

Typischer Zweck:

- Low-Level-Steuerkette aktivieren
- manuelle Fahrkommandos einspeisen
- VESC-/Ackermann-Kette vorbereiten

### Wichtig

Vor jeder Autonomie sollte diese Kette separat verstanden und geprüft werden.

---

## 8.7 `mxck2_development`

Dies ist der **Entwicklungscontainer**.

Er enthält die erweiterte Entwicklungsumgebung für:

- FTG
- YOLO
- PDC
- Parking
- experimentelle Funktionen

### Wichtig

Dieser Container ist **nicht** automatisch gleichbedeutend mit dem stabilen Basisbetrieb.

Er ist vielmehr die Entwicklungsplattform, auf der autonome Funktionen vorbereitet, getestet und weiterentwickelt werden.

---

## 8.8 Zusatzcontainer wie `mxck2_pdc` und `mxck2_yolo`

Diese Container gehören zu spezialisierten Zusatzfunktionen:

- `mxck2_pdc` → Ultraschall-/PDC-Visualisierung
- `mxck2_yolo` → kamerabasierte Objekterkennung / Vision

Sie sind wichtig für erweiterte Funktionen, aber **nicht notwendig**, um zunächst LiDAR, Steuerkette und FTG-Grundintegration zu verstehen.

---

## 9. Wichtige Aliases und Startbefehle

Im Workspace sind alltagsrelevante Shell-Aliases hinterlegt.

### 9.1 Wichtige Standard-Aliases

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

### 9.2 Bedeutung der Aliases

#### Foxglove + TF

```bash
run_foxglove
```

Startet Foxglove Bridge plus TF-Broadcast.

#### manuelle Steuerung

```bash
kickstart
```

Startet die manuelle Fahrzeugsteuerkette.

#### micro-ROS

```bash
run_micro
```

Startet die STM32-/micro-ROS-Anbindung.

#### LiDAR

```bash
run_lidar
```

Startet den LiDAR-Zweig.

#### Kamera

```bash
run_camera
run_rgbd
run_imu
run_vio
```

Startet je nach Bedarf RGB, RGBD, IMU oder VIO-relevante Kombinationen.

---

## 10. Datenfluss im System

## 10.1 Überblick in Worten

Der Datenfluss des Carkits ist in Sensorik, Verarbeitung, Visualisierung und Steuerung gegliedert.

### Sensorikpfade

- **RPLiDAR** liefert 2D-Laserscans nach `/scan`
- **RealSense** liefert Kamera-/Depth-/IMU-Topics
- **STM32** liefert RC-, Ultraschall- und ggf. IMU-/Zustandsdaten

### Verarbeitungs-/Diagnosepfade

- LiDAR-Daten werden für Hinderniserkennung und FTG verarbeitet
- Kamera-/Visiondaten können durch YOLO oder andere Perception-Pipelines laufen
- PDC-/Ultraschalldaten können in Visualisierung oder Safety-Logik eingehen

### Steuerpfade

- manuelle Kommandos kommen aus RC / Steuercontainer
- autonome Kommandos werden künftig in die vorhandene Autonomiekette eingespeist
- am Ende wird über VESC / Servo / Motor die physische Fahrzeugreaktion erzeugt

---

## 10.2 Datenflussdiagramm (logisch)

```text
RPLiDAR ----------------------> /scan --------------------------> FTG / Hinderniserkennung
RealSense --------------------> /camera/... --------------------> Vision / Wahrnehmung
STM32 / micro-ROS -----------> RC / IMU / USS Topics ----------> Safety / Status / Zusatzlogik

FTG / Planner / Control -----> Ackermann-Autonomiekommando ----> vehicle_control / VESC-Kette
RC / manuelle Steuerung -----> manuelles Ackermannkommando -----> vehicle_control / VESC-Kette

ROS2 Topics -----------------> Foxglove Bridge -----------------> Visualisierung
```

---

## 10.3 Datenflussdiagramm (mit Schichten)

```text
[Sensoren]
  |-- RPLiDAR
  |-- RealSense
  |-- STM32 / RC / USS / IMU
  v
[ROS2 Sensor-Container]
  |-- mxck2_lidar
  |-- mxck2_camera
  |-- mxck2_micro
  v
[ROS2 Topics]
  |-- /scan
  |-- /camera/...
  |-- RC / IMU / USS Topics
  v
[Verarbeitung]
  |-- FTG Perception
  |-- FTG Planner
  |-- FTG Control
  |-- Vision / YOLO / PDC
  v
[Steuerkette]
  |-- manuell oder autonom
  |-- Ackermann / vehicle_control
  |-- VESC
  v
[Aktoren]
  |-- Motor
  |-- Servo
```

---

## 11. Steuerarchitektur und Sicherheitsprinzip

Die wichtigste Regel aus dem Projekt:

**Autonomie darf nicht die bestehende Steuerkette umgehen.**

Stattdessen soll sie sauber in die vorhandene Architektur eingehängt werden.

### Grundidee

- manuelle Steuerung bleibt erhalten
- Safety / Deadman / Moduslogik bleibt erhalten
- autonome Kommandos werden auf den vorgesehenen Pfad gelegt
- kein direkter „Bypass“ auf Servo/Motor-Topics

### Praktischer Grund

So bleibt:

- der manuelle Rückfallpfad erhalten
- Deadman/Not-Stopp wirksam
- der Fahrbetrieb nachvollziehbar und sicher
- die Architektur konsistent

### Sicherheitsregel

**Erst manuell stabil testen, dann Autonomie langsam und kontrolliert zuschalten.**

---

## 12. FTG (Follow-The-Gap) im MXCK-System

## 12.1 Fachliche Rolle

FTG ist in diesem Projekt die LiDAR-basierte reaktive Hindernisvermeidung.

Ziel:

- Hindernisse mit dem LiDAR erkennen
- freien Raum bestimmen
- größte sichere Lücke finden
- daraus Lenkwinkel und passende Geschwindigkeit ableiten
- das Fahrzeug sicher um Hindernisse herumführen

---

## 12.2 Warum FTG hier passt

Das Verfahren eignet sich für:

- Ackermann-Fahrzeuge
- lokale, reaktive Hindernisvermeidung
- begrenztes Sichtfeld
- strukturierte Testumgebungen ohne vollständige Karten

Im Projekt ist FTG deshalb ein sinnvoller erster Autonomieschritt.

---

## 12.3 Vorhandene FTG-Struktur im Projekt

Im Development-Stack existiert FTG bereits **modular aufgeteilt**.

Typische Funktionsblöcke:

- **Perception**
  - Scan prüfen
  - Frontfenster bestimmen
  - Vorverarbeitung
  - ggf. Hindernisersatz / Scan-Anpassung

- **Planner**
  - größte Lücke bestimmen
  - Sollrichtung / Zielwinkel berechnen

- **Control**
  - Lenkwinkel und Geschwindigkeit in fahrbare Befehle überführen

- **Bringup**
  - Gesamtkette gemeinsam starten

Diese Aufteilung ist korrekt und sinnvoll.

---

## 12.4 Richtige FTG-Integrationsstrategie

### Nicht richtig

- direkt auf Motor-/Servoebene schreiben
- bestehende Fahrzeuglogik umgehen
- ungeprüfte Scan-Richtung benutzen
- ohne manuelle Sicherheitskette testen

### Richtig

1. `/scan` verifizieren
2. Frontrichtung des LiDAR prüfen
3. Frontfenster / Scan-Vorverarbeitung definieren
4. FTG-Ausgang zunächst nur beobachten
5. Ackermann-Autonomiekommando in die vorhandene Steuerkette einspeisen
6. Testgeschwindigkeit klein halten
7. Deadman/RC immer verfügbar halten

---

## 13. Empfohlene Startreihenfolge

Dies ist die empfohlene und sichere Reihenfolge für den Nachfolger.

## 13.1 Schritt 1 – Verbindung zum Jetson

```bash
ssh mxck@<JETSON-IP>
```

Optional zur Netzprüfung:

```bash
ping <JETSON-IP>
```

Wenn die IP unbekannt ist, direkt am Jetson:

```bash
ifconfig
hostname -I
```

---

## 13.2 Schritt 2 – laufende Container prüfen

```bash
docker ps --format "table {{.Names}}\t{{.Status}}"
```

Oder mit Portainer im Browser, falls verwendet.

### Ziel

Zuerst sehen:

- welche Basiscontainer laufen
- welche Zusatzcontainer laufen
- ob etwas abgestürzt ist

---

## 13.3 Schritt 3 – in den passenden Container gehen

### Basiscontainer prüfen

```bash
sudo docker exec -it mxck2_control bash
```

### Entwicklungscontainer prüfen

```bash
sudo docker exec -it mxck2_development bash
```

### Dann immer sofort prüfen

```bash
pwd
mount | grep mxck2_ws
echo $ROS_DISTRO
ls -lah /mxck2_ws
```

---

## 13.4 Schritt 4 – ROS-Umgebung sauber laden

In interaktiven Shells nie blind davon ausgehen, dass alles bereits gesourct ist.

### Beispiel

```bash
source /opt/ros/foxy/setup.bash
source /mxck2_ws/install/setup.bash
```

oder je nach Container:

```bash
source /opt/ros/humble/setup.bash
source /microros_ws/install/setup.bash
source /vesc_ws/install/setup.bash
source /rplidar_ws/install/setup.bash
source /mxck2_ws/install/setup.bash
```

### Danach prüfen

```bash
echo $AMENT_PREFIX_PATH
echo $COLCON_PREFIX_PATH
which ros2
ros2 topic list
ros2 node list
```

---

## 13.5 Schritt 5 – Sensoren einzeln starten und prüfen

### LiDAR

```bash
run_lidar
ros2 topic list | grep scan
ros2 topic echo /scan
```

### Kamera

```bash
run_camera
ros2 topic list | grep camera
```

### RGBD

```bash
run_rgbd
ros2 topic list | grep camera
```

### IMU

```bash
run_imu
ros2 topic list | grep imu
```

### micro-ROS / STM32

```bash
run_micro
ros2 topic list
```

---

## 13.6 Schritt 6 – TF und Visualisierung prüfen

```bash
run_foxglove
ros2 topic list | grep tf
```

Wenn Foxglove läuft, dort prüfen:

- kommt `/scan` an?
- ist `/tf_static` vorhanden?
- sind Frames plausibel?

---

## 13.7 Schritt 7 – manuelle Steuerkette prüfen

```bash
kickstart
```

Dann prüfen:

- reagiert die manuelle Steuerung?
- funktioniert Deadman / Moduswechsel?
- werden Servo/Motor-Kommandos plausibel erzeugt?

Ohne stabilen manuellen Pfad keine Autonomie zuschalten.

---

## 13.8 Schritt 8 – FTG erst danach testen

Erst wenn `/scan`, TF und manuelle Steuerung stimmen.

Typische Reihenfolge:

1. Scan nur anzeigen
2. Scan-Vorverarbeitung / Frontfenster prüfen
3. Planner-Ausgabe beobachten
4. Control-Ausgabe beobachten
5. Integration in Ackermann-Autonomiepfad
6. langsame Fahrtests

---

## 14. Wichtige Diagnosebefehle

## 14.1 Allgemein

```bash
docker ps
docker logs <container>
docker inspect <container>
```

## 14.2 ROS2-Grunddiagnose

```bash
ros2 topic list
ros2 node list
ros2 interface list
ros2 param list
```

## 14.3 Topic-Prüfung

```bash
ros2 topic echo /scan
ros2 topic hz /scan
ros2 topic info /scan
```

## 14.4 TF-Prüfung

```bash
ros2 topic list | grep tf
```

## 14.5 Build-Diagnose

```bash
colcon build --symlink-install
```

Bei selektivem Build vorsichtig und distributionsabhängig arbeiten.

## 14.6 Dateisystem- und Mount-Prüfung

```bash
mount | grep mxck2_ws
df -h
ls -lah /mxck2_ws
```

---

## 15. Empfohlene Debug-Reihenfolge bei Problemen

Wenn etwas „nicht funktioniert“, dann nicht zufällig überall gleichzeitig suchen.

### 1. Läuft der richtige Container?

```bash
docker ps --format "table {{.Names}}\t{{.Status}}"
```

### 2. Ist der richtige Workspace gemountet?

```bash
mount | grep mxck2_ws
```

### 3. Ist ROS2 in der Shell wirklich verfügbar?

```bash
which ros2
echo $ROS_DISTRO
```

### 4. Wurde die Umgebung gesourct?

```bash
echo $AMENT_PREFIX_PATH
echo $COLCON_PREFIX_PATH
```

### 5. Existiert das erwartete Topic?

```bash
ros2 topic list | grep scan
```

### 6. Kommt auf dem Topic sinnvolle Information an?

```bash
ros2 topic echo /scan
```

### 7. Gibt es TF-Probleme?

```bash
ros2 topic list | grep tf
```

### 8. Was sagen die Containerlogs?

```bash
docker logs <container>
```

---

## 16. Typische Fehlerbilder und ihre Bedeutung

## 16.1 `ros2: command not found`

### Bedeutung

Das heißt in diesem Projekt **nicht automatisch**, dass der Container kaputt ist.

Oft bedeutet es:

- die Audit- oder Exec-Shell wurde ohne korrektes Sourcing geöffnet
- der Container läuft über einen Entry-Point korrekt, aber die manuell geöffnete Shell nicht

### Lösung

```bash
source /opt/ros/<distro>/setup.bash
source /mxck2_ws/install/setup.bash
which ros2
```

---

## 16.2 LiDAR läuft, aber `/scan` ist nicht sichtbar

### Mögliche Ursachen

- falscher Container
- RPLiDAR nicht gestartet
- Gerät `/dev/rplidar` nicht korrekt verfügbar
- Topic wird in anderem ROS-Graph gesucht

### Prüfen

```bash
ls -l /dev/rplidar
run_lidar
ros2 topic list | grep scan
```

---

## 16.3 Foxglove zeigt TF-Probleme

### Bedeutung

Die TF-Kette ist unvollständig oder nicht aktiv.

### Beispielhafte Auswirkungen

- Sensorvisualisierung passt nicht
- PDC-/Ultraschall-Frames fehlen
- spätere Sensorfusion wird instabil

### Prüfen

```bash
ros2 topic list | grep tf
run_foxglove
```

---

## 16.4 Kamera-Container beendet sich

### Bedeutung

Das muss nicht sofort ein Hardwareproblem sein.

Häufiger sind:

- Workspace-/Package-Probleme
- Python-/colcon-/Launch-Probleme
- Umgebungsfehler

### Erst prüfen

```bash
docker logs mxck2_camera
ros2 topic list | grep camera
```

---

## 16.5 VESC-/Kickstart-Probleme

Wenn bei der Steuerkette serielle Protokollfehler oder Frame-Fehler auftauchen, dann ist das eher ein Hinweis auf:

- Kommunikationsproblem mit dem VESC
- serielles Datenrahmenproblem
- falsches Device / Timing / Konfiguration

Nicht sofort auf ROS2 allgemein schließen.

---

## 16.6 `AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH` leer

### Bedeutung

Die Umgebung wurde nicht korrekt vorbereitet.

### Lösung

```bash
source /opt/ros/<distro>/setup.bash
source /mxck2_ws/install/setup.bash
```

Dann neu prüfen.

---

## 16.7 falscher `colcon`-Aufruf

Wenn `colcon` mit unbekannten Argumenten abbricht, dann wurde vermutlich ein nicht passender Build-Befehl aus einer anderen Umgebung übernommen.

### Regel

Nie ungeprüft Kommandos aus anderem ROS2-/colcon-Kontext übernehmen.

---

## 17. Bagging und Aufzeichnung

Im Projekt sind mehrere Bag-Testläufe vorhanden. Das zeigt, dass Diagnose und Reproduzierbarkeit wichtiger Bestandteil der Arbeit waren.

### Empfehlung

Bei FTG- oder Sensorproblemen immer Bag-Aufzeichnung mitdenken.

### Beispiel

```bash
ros2 bag record -o run01 /scan
```

Oder erweitert:

```bash
ros2 bag record -o run01 /scan /tf_static /camera/color/image_raw /camera/depth/image_rect_raw
```

### Nutzen

- spätere Offline-Auswertung
- Scan- und Topic-Verifikation
- Wiederholbarkeit von Tests
- Debugging ohne Live-Fahrt

---

## 18. Empfehlungen für FTG-Tests

## 18.1 Reihenfolge

1. `/scan` prüfen
2. Scan-Richtung / Frontfenster verstehen
3. Scan-Vorverarbeitung verifizieren
4. FTG-Planer-Ausgabe beobachten
5. Control-Ausgabe beobachten
6. Ackermann-Ausgabe in die Steuerkette hängen
7. langsame Fahrtests mit Deadman

## 18.2 Testumgebung

- große freie Fläche
- gut sichtbare, einfache Hindernisse
- geringe Geschwindigkeit
- genug Auslaufzone
- immer manuelle Eingriffsmöglichkeit

## 18.3 Nicht tun

- ohne Deadman testen
- direkt hohe Geschwindigkeit verwenden
- FTG im unbekannten Container starten
- Scan-Richtung nicht prüfen
- Servo/Motor direkt statt über Steuerkette ansteuern

---

## 19. Praktische Kommandosammlung für den Nachfolger

## 19.1 Verbindung und Überblick

```bash
ssh mxck@<JETSON-IP>
docker ps --format "table {{.Names}}\t{{.Status}}"
```

## 19.2 In Container einsteigen

```bash
sudo docker exec -it mxck2_control bash
sudo docker exec -it mxck2_development bash
```

## 19.3 Umgebung prüfen

```bash
pwd
mount | grep mxck2_ws
echo $ROS_DISTRO
which ros2
```

## 19.4 Umgebung laden

```bash
source /opt/ros/foxy/setup.bash
source /mxck2_ws/install/setup.bash
```

oder im Basisstack:

```bash
source /opt/ros/humble/setup.bash
source /microros_ws/install/setup.bash
source /vesc_ws/install/setup.bash
source /rplidar_ws/install/setup.bash
source /mxck2_ws/install/setup.bash
```

## 19.5 Sensoren starten

```bash
run_lidar
run_camera
run_rgbd
run_imu
run_micro
run_foxglove
```

## 19.6 Manuelle Steuerung

```bash
kickstart
```

## 19.7 ROS-Diagnose

```bash
ros2 topic list
ros2 node list
ros2 topic echo /scan
ros2 topic hz /scan
```

## 19.8 Logs

```bash
docker logs mxck2_lidar
docker logs mxck2_control
docker logs mxck2_camera
docker logs mxck2_micro
```

---

## 20. Do / Don’t für den Nachfolger

## 20.1 Do

- immer zuerst Container und Workspace prüfen
- ROS-Umgebung in der Shell bewusst sourcen
- Sensoren einzeln testen
- `/scan` und TF vor FTG prüfen
- manuelle Steuerkette vor Autonomie verifizieren
- Deadman aktiv halten
- Bags zur Diagnose aufnehmen
- FTG nur sauber in die bestehende Ackermann-/Autonomiekette integrieren

## 20.2 Don’t

- nicht blind Kommandos aus einem anderen Container übernehmen
- nicht davon ausgehen, dass `/mxck2_ws` immer derselbe Workspace ist
- nicht direkt Motor-/Servo-Topics überschreiben
- nicht gleichzeitig alles starten und dann raten, wo der Fehler liegt
- nicht ohne geringe Geschwindigkeit und manuellen Rückfallpfad testen
- Humble/Foxy nicht ungeprüft vermischen

---

## 21. Empfohlener Arbeitsmodus für neue Entwicklung

Wenn neue Funktionen entwickelt werden – insbesondere FTG, Scanfilter, Planner oder Safety-Erweiterungen – dann am besten so vorgehen:

### Phase 1 – Verstehen

- Basisarchitektur lesen
- Containerrollen verstehen
- vorhandene Aliases testen
- `/scan`, TF und Steuerkette verifizieren

### Phase 2 – Isoliertes Testen

- neue Nodes zunächst ohne Fahrzeugbewegung starten
- Outputs nur beobachten
- Diagnose-Topics sichtbar machen
- Bag-Aufnahmen erzeugen

### Phase 3 – Integration

- neue Funktion in bestehende Steuerkette einkoppeln
- Parameter konservativ setzen
- Ackermann-Ausgabe logisch prüfen

### Phase 4 – Reale Fahrtests

- niedrige Geschwindigkeit
- freie Testfläche
- einfache Hindernisse
- manuelle Rückfallebene verfügbar

---

## 22. Kurzfazit

Das MXCK Carkit ist ein **mehrschichtiges, modular containerisiertes ROS2-Fahrzeugsystem**.

Die wichtigste Struktur ist:

- **Basisbetrieb in `mxck2_ws`**
- **Entwicklung und autonome Erweiterungen in `development_ws`**

Die wichtigsten Erfolgsfaktoren für den Nachfolger sind:

1. **den richtigen Container wählen**
2. **den tatsächlich gemounteten Workspace prüfen**
3. **die ROS-Umgebung sauber laden**
4. **Sensoren einzeln prüfen**
5. **die manuelle Steuerkette verstehen**
6. **FTG und andere Autonomie nur über die bestehende Steuerarchitektur integrieren**

Wenn diese Regeln eingehalten werden, ist das System gut beherrschbar und kann sinnvoll erweitert werden.

---

## 23. Übergabesatz in einem Absatz

Das MXCK Carkit läuft auf einem NVIDIA Jetson als containerisiertes ROS2-System mit getrennten Funktionscontainern für LiDAR, Steuerung, micro-ROS, Kamera, Foxglove und Entwicklung. Für den stabilen Basisbetrieb ist vor allem der Workspace `mxck2_ws` relevant, während `development_ws` FTG, YOLO, Parking und weitere experimentelle Funktionen enthält. Da in verschiedenen Containern derselbe Zielpfad `/mxck2_ws` auf unterschiedliche Host-Ordner gemountet wird und zudem nicht überall dieselbe ROS2-Distribution verwendet wird, muss vor jedem Build oder Launch immer zuerst geprüft werden, in welchem Container und auf welchem Workspace gearbeitet wird. Sensoren sind einzeln zu starten und über Topics, TF und Logs zu verifizieren; erst danach darf die manuelle Steuerkette getestet und schließlich FTG oder andere autonome Funktionen kontrolliert in die bestehende Ackermann-/VESC-Kette integriert werden.
