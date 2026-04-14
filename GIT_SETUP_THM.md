# GIT_SETUP_THM.md
**THM GitLab (git.thm.de) via SSH + Windows PowerShell + VS Code**

Diese Anleitung dokumentiert die vollständige Einrichtung und den Workflow, um ein GitLab-Repository auf **git.thm.de** per **SSH** mit **VS Code** zu nutzen (Clone / Pull / Commit / Push) – inkl. typischer Fehlerquellen.

---

## 0) Wichtig: Welche GitLab-Instanz?
- ✅ THM GitLab: `https://git.thm.de/...`
- ❌ Nicht `gitlab.com` (das ist eine andere Instanz)

**SSH Host (THM):**
- Host: `git.thm.de`
- User: `git`
- Port: `22`

---

## 1) Checkliste (Kurz)
A) Vorbereitung  
- [x] Git installiert  
- [x] VS Code installiert  
- [x] Zugriff auf THM GitLab-Projekt vorhanden (`git.thm.de`)  

B) SSH-Key erstellen  
- [x] `ssh-keygen -t ed25519 -C "mail@thm.de"`  

C) SSH-Agent einrichten  
- [x] ssh-agent aktivieren und Key hinzufügen  

D) GitLab konfigurieren  
- [x] Public Key in `git.thm.de` → Settings → SSH Keys speichern  

E) Verbindung testen  
- [x] `ssh -T git@git.thm.de` → “Welcome …”  

F) Repo lokal nutzen  
- [x] Remote auf SSH umstellen (statt HTTPS)  
- [x] Pull / Commit / Push erfolgreich  

---

## 2) Voraussetzungen prüfen

### 2.1 Git Version prüfen
```powershell
git --version
```

### 2.2 SSH verfügbar?
```powershell
ssh -V
```

---

## 3) SSH-Key erstellen (ed25519)

### 3.1 Key erzeugen
```powershell
ssh-keygen -t ed25519 -C "deine.mail@thm.de"
```

**Empfehlung:**
- Speicherort auf Standard lassen:  
  `C:\Users\<User>\.ssh\id_ed25519`
- Passphrase setzen und merken (sicherer)

### 3.2 Public Key anzeigen
```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub
```

---

## 4) SSH-Agent in Windows einrichten

### 4.1 ssh-agent aktivieren + starten (Admin-PowerShell)
```powershell
Set-Service -Name ssh-agent -StartupType Automatic
Start-Service ssh-agent
```

### 4.2 Key zum Agent hinzufügen
```powershell
ssh-add $env:USERPROFILE\.ssh\id_ed25519
```

### 4.3 Prüfen, ob Key geladen ist
```powershell
ssh-add -l
```

**Achtung:**  
`ssh-add -l` (kleines L) – nicht `-1` (Zahl 1).

---

## 5) GitLab (THM) konfigurieren

### 5.1 Key in GitLab speichern
1. `https://git.thm.de` öffnen und einloggen
2. Profil → **Settings / Preferences**
3. **SSH Keys**
4. Public Key einfügen (aus `id_ed25519.pub`)
5. Title vergeben (z. B. `Omar-Laptop`)
6. Speichern

---

## 6) Verbindung testen (THM GitLab)
```powershell
ssh -T git@git.thm.de
```

**Erwartet:**
- `Welcome to GitLab, @<username>!`

Beim ersten Mal kommt eine Host-Authentizitätsabfrage → `yes` bestätigen.

---

## 7) Repo / Ordner prüfen (wichtig: im richtigen Verzeichnis sein)

Wenn dein Repo z.B. hier liegt:  
`C:\Users\Omar\Desktop\Studium\Master\Mxck carkit\Programm`

Dann:
```powershell
cd "C:\Users\Omar\Desktop\Studium\Master\Mxck carkit\Programm"
git status
git rev-parse --show-toplevel
```

**Wenn du siehst:**
- `fatal: not a git repository ...`  
  → du bist im falschen Ordner oder das Repo wurde nicht geklont.

---

## 8) Remote-URL auf SSH umstellen (statt HTTPS)

### 8.1 Remote anzeigen
```powershell
git remote -v
```

Wenn dort HTTPS steht, z.B.:  
`https://git.thm.de/<gruppe>/<repo>.git`

Dann auf SSH ändern:
```powershell
git remote set-url origin git@git.thm.de:<gruppe>/<repo>.git
git remote -v
```

**Beispiel (realer erfolgreicher Fall):**
```powershell
git remote set-url origin git@git.thm.de:bltk30/speedlab_carkit.git
git remote -v
```

---

## 9) Branches (anzeigen / wechseln)

### 9.1 Remote Branches holen
```powershell
git fetch --all --prune
git branch -a
```

### 9.2 Branch wechseln
```powershell
git switch <branchname>
```

### 9.3 Wenn Branch nur remote existiert
```powershell
git switch -c <branchname> --track origin/<branchname>
```

---

## 10) Standard Workflow: Pull / Commit / Push

### 10.1 Änderungen holen (empfohlen mit Rebase)
```powershell
git pull --rebase
```

### 10.2 Änderungen prüfen
```powershell
git status
```

### 10.3 Änderungen stagen
```powershell
git add .
```

### 10.4 Commit erstellen
```powershell
git commit -m "kurze commit message"
```

### 10.5 Push zu GitLab
```powershell
git push
```

**Wenn es ein neuer Branch ist (erstes Push):**
```powershell
git push -u origin <branchname>
```

---

## 11) VS Code Schritte (UI)

1. Repo-Ordner in VS Code öffnen
2. **Source Control** (Ctrl+Shift+G) prüfen
3. Dateien ändern
4. Stage (Plus-Symbol) → Commit Message → Commit
5. Push / Sync Changes

**Hinweis:**  
Wenn VS Code nach Passwort fragt, ist häufig der Remote noch HTTPS.

---

## 12) Dokumentation: Was wurde erfolgreich gemacht? (dein tatsächlicher Verlauf)

### 12.1 SSH-Test erfolgreich
```powershell
ssh -T git@git.thm.de
# -> Welcome to GitLab, @omde271!
```

### 12.2 Repo erkannt, richtiger Branch
```powershell
git status
# On branch dev_obstacleAvoidance
# Your branch is up to date with 'origin/dev_obstacleAvoidance'.
# nothing to commit, working tree clean
```

### 12.3 Remote von HTTPS auf SSH geändert

Vorher:
- `https://git.thm.de/bltk30/speedlab_carkit.git`

Nachher:
```powershell
git remote set-url origin git@git.thm.de:bltk30/speedlab_carkit.git
git remote -v
```

### 12.4 Fetch / Pull erfolgreich
```powershell
git fetch
git pull --rebase
# -> Already up to date.
```

### 12.5 Commit & Push erfolgreich
```powershell
git add .
git commit -m "Omar_push"
git push
# -> dev_obstacleAvoidance -> dev_obstacleAvoidance
```

---

## 13) Häufige Probleme & Lösungen

### 13.1 `Permission denied (publickey)`

**Ursachen:**
- Key nicht in `git.thm.de` eingetragen
- falsche GitLab-Instanz (z.B. Key in gitlab.com gespeichert)
- falscher Host / falscher User

**Fix:**
```powershell
ssh -T git@git.thm.de
```

Wenn das nicht “Welcome …” zeigt → Key in `git.thm.de` hinzufügen.

### 13.2 Hostname falsch (`git.thm.com`)

**Fix:** THM ist:
- ✅ `git.thm.de`
- ❌ `git.thm.com`

### 13.3 VS Code fragt nach Passwort

Remote ist wahrscheinlich HTTPS.
```powershell
git remote -v
git remote set-url origin git@git.thm.de:<gruppe>/<repo>.git
```

### 13.4 Passphrase wird ständig gefragt

Normal, wenn Passphrase gesetzt ist. Komfort:
- sicherstellen, dass ssh-agent läuft und Key geladen ist:
```powershell
ssh-add -l
```

### 13.5 Mehrere SSH-Keys (falscher Key wird genutzt)

SSH Config anlegen:  
Datei: `C:\Users\<User>\.ssh\config`

```sshconfig
Host git.thm.de
  HostName git.thm.de
  User git
  Port 22
  IdentityFile ~/.ssh/id_ed25519
  IdentitiesOnly yes
```

---

## 14) Mini-Spickzettel (Alltag)

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

**Ende**