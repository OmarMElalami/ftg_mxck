Create line_tracking-Stack_Beispiel Code
Diese Dateien sind als Vorbild am nützlichsten:
line_tracking_launch.py
→ zeigt sauberen Launch-Aufbau und Aktivierung alternativer Pfade
params.yaml
→ zeigt zentrale Parametrisierung und klare Controller-/Safety-/Speed-Struktur
controller_node.py
→ zeigt saubere Mittelschicht: Wahrnehmung → robotisches Zwischenformat
pure_pursuit_node.py
→ zeigt saubere letzte Stufe: Zwischenformat → /autonomous/ackermann_cmd
setup.py
package.xml
→ zeigen Paket-/Install-/Executable-Konsistenz
Optional zusätzlich
obstacle_detector_node.py
→ nützlich als Safety-Idee, aber nicht zwingend zentral für FTG
lane_detector_node.py oder lane_detector_cnn_node.py
→ hilfreich als Beispiel für dokumentierte Zwischenformate und Publish-Struktur, aber weniger wichtig als Controller/Launch/Params
data_recorder_node.py
→ gut als Beispiel für Logging/Datensammlung, aber nicht zwingend für FTG-Fix
