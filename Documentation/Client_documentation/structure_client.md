# Structure generale du client web_control

## 1) Role du package
Le package `web_control` fournit une interface web de teleoperation et de mission pour le robot, avec:
- un backend ROS2 en Python,
- un serveur HTTP local pour servir l'IHM,
- des pages web (controle, navigation, galerie, terminal),
- des passerelles ROS <-> UI (topics/services),
- la capture/synchronisation des medias (photo/video).

## 2) Arborescence principale

- `launch/`
  - `web_control_full.launch.py`: lance rosbridge, camera_publisher et backend_node.
- `web_control/` (backend Python)
  - `backend_node.py`: noeud principal (serveur web, topics/services, sync galerie, logs).
  - `camera_publisher.py`: publie le flux camera RTSP vers `/camera/clear`.
  - `capture_manager.py`: prise photo + enregistrement video (ffmpeg + fallback OpenCV).
  - `gallery_manager.py`: scan galerie et publication de `/ui/gallery_files`.
- `web/` (frontend)
  - `index.html` + `style.css`: dashboard principal.
  - `js/`
    - `01-core-ros.js`: connexions ROS/WebRTC + topics/services de base.
    - `02-PTZ.js`: controle robot/PTZ, clavier, sliders, arret d'urgence.
    - `03-communication-avec-robot.js`: mission, capture media, odometrie, feedback/resultats.
    - `04-media-gallery.js`: logique galerie cote dashboard.
    - `05-trajectoires.js`: chargement/rendu trajectoires, conversions carte/robot.
    - `06-UI-minimap.js`: gestion UI (dark mode, modal, panning/lock minimap).
  - `navig/`: editeur carte/trajectoires (page dediee navigation).
  - `galerie/`: page galerie complete (lightbox, suppression, download).
  - `terminal/`: page logs systeme en temps reel.
  - `accueil/`: page presentation du projet.
  - `configuration.json`: config video (host/port/stream).
- racine package
  - `setup.py`: packaging Python + installation recursive du dossier web.
  - `package.xml`: dependances ROS2.

## 3) Architecture logique

### A. Couche backend ROS2
Le backend `backend_node.py`:
- expose des services ROS (`/camera/take_photo`, `/camera/start_video`, `/camera/stop_video`),
- consomme/publie des topics UI/robot (cmd, PTZ, logs, missions, trajectoires),
- sert les fichiers web sur le port 8000,
- gere l'upload HTTP (`/upload_photo`, `/upload_video`),
- maintient des liens symboliques vers:
  - `~/robot_gallery` (medias),
  - `~/trajectories` (fichiers trajets),
- synchronise periodiquement les medias distants du robot vers le client.

### B. Couche capture media
- `camera_publisher.py` publie l'image camera en ROS2.
- `capture_manager.py` priorise ffmpeg (RTSP direct), puis fallback OpenCV/flux ROS si besoin.
- `gallery_manager.py` publie la liste ordonnee des fichiers media pour l'IHM.

### C. Couche frontend
Le dashboard principal combine:
- video live (WebRTC ou flux externe),
- commandes robot/PTZ,
- reglages,
- minimap mission,
- galerie rapide.

Les pages dediees completent les usages:
- navigation avancee,
- galerie plein ecran,
- terminal de logs.

## 4) Flux de donnees principaux

1. **Controle robot**  
   UI JS -> rosbridge -> topics robot (`/robot/cmd_vel`, `/camera/ptz`, etc.).

2. **Missions trajectoires**  
   UI charge un trajet -> publie `/ui/start_mission` -> recoit `/ui/mission_feedback` et `/ui/mission_result`.

3. **Photos/Videos**  
   - mode navigateur: capture locale puis upload HTTP vers backend,
   - mode ROS2: services capture backend.

4. **Galerie/Logs**  
   backend publie `/ui/gallery_files` et `/ui/system_logs`, consommes par dashboard/galerie/terminal.

## 5) Points forts de la structure
- Separation claire: backend ROS2 / frontend web / gestion media.
- Modules JS decoupes par responsabilite (core, commandes, mission, galerie, minimap).
- Robustesse runtime (reconnexion ROS, fallback capture, sync periodique, gestion offline).
- Facile a etendre (nouvelles pages web, nouveaux topics/services, nouvelles sources video).
