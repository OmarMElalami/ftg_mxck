# Projektarbeit_MXCK_FTG – aktualisierte Quellen

Diese Fassung ist auf den aktuellen scan-basierten FTG-Stack von
`ftg_mxck` abgestimmt.

Enthalten:
- `main.tex`
- `chapters/01_grundlagen.tex`
- `chapters/02_durchfuehrung.tex`
- `chapters/03_diskussion.tex`
- `chapters/04_fazit.tex`
- `.gitignore`
- `images/thm.png`

Hinweis:
Die Kapitel sind bewusst auf die aktuelle Primärpipeline ausgerichtet:

`/scan -> scan_preprocessor_node -> /autonomous/ftg/scan_filtered -> follow_the_gap_v0 -> ftg_planner_node -> ftg_command_node -> /autonomous/ackermann_cmd`

Legacy-Komponenten wie `obstacle_substitution` werden nur noch als
Kompatibilitäts-/Altpfad erwähnt.
