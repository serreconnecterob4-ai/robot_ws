# Resume du package camera

Ce document resume les fichiers principaux du package camera, avec un focus ROS2, reseau et robustesse.

## camera_control_node.py

### Role
Ce noeud est le pilote principal de la camera PTZ Reolink cote ROS2. Il recoit des commandes depuis des topics (mouvement, zoom, focus, lumiere, alerte, volume) puis envoie les requetes HTTP vers l'API de la camera. Il gere aussi la session (token), son rafraichissement et plusieurs strategies de repli selon les capacites du modele.

### Topics ROS2 publies et souscrits
- Publies: aucun.
- Souscrits:
  - /camera/ptz (geometry_msgs/msg/Point)
  - /camera/zoom (std_msgs/msg/Float32)
  - /camera/light (std_msgs/msg/Bool)
  - /camera/focus (std_msgs/msg/Float32)
  - /camera/autofocus (std_msgs/msg/Bool)
  - /camera/alert (std_msgs/msg/Bool)
  - /robot/volume (std_msgs/msg/Float32)

### Services ROS2 exposes ou appeles
- Aucun service ROS2 expose.
- Aucun service ROS2 appele.
- Declencheurs internes: callbacks de topics + timer de rafraichissement de token toutes les 15 s.

### Dependances externes
- requests (HTTP vers API Reolink)
- API camera Reolink (endpoints API CGI: Login, PtzCtrl, StartZoomFocus, SetWhiteLed, etc.)
- rclpy, geometry_msgs, std_msgs

### Ports reseau utilises
- HTTP camera Reolink sur 10.42.0.188:80 (implicitement via http://.../api.cgi).

### Comportement en cas d'erreur
- Reconnexion automatique si token absent/expire.
- Tentative de relogin si la camera repond avec une erreur de session.
- Multiples fallbacks pour certaines fonctions:
  - Alerte: AudioAlarm puis SetSiren puis SetAudioCfg(test).
  - Autofocus: SetAutoFocus puis StartZoomFocus(AutoFocus).
  - Focus manuel: FocusPos puis fallback focus.pos.
- Logs warn/error en cas d'echec HTTP, timeout ou commande non supportee.

### Points d'attention
- IP, identifiants et mot de passe sont en dur dans le code.
- En fermeture, tentative d'arret PTZ et extinction IR uniquement si etats actifs.
- Certaines commandes dependent fortement du firmware/modele Reolink (d'ou les fallbacks).

## camera_bridge.py

### Role
Ce noeud lit le flux RTSP de la camera avec OpenCV puis publie deux flux ROS2 image: un flux HD (clear) et un flux redimensionne (fluent) pour l'IHM. Le commentaire indique qu'il est devenu obsolete pour la capture photo/video, mais il reste utile pour la diffusion d'images ROS2. Il privilegie la faible latence en vidant le buffer avant publication.

### Topics ROS2 publies et souscrits
- Publies:
  - /camera/fluent (sensor_msgs/msg/Image)
  - /camera/clear (sensor_msgs/msg/Image)
- Souscrits: aucun.

### Services ROS2 exposes ou appeles
- Aucun service ROS2 expose.
- Aucun service ROS2 appele.
- Declencheur interne: timer a 0.04 s (~25 Hz) pour capturer/publier.

### Dependances externes
- OpenCV (cv2)
- cv_bridge
- FFmpeg via backend OpenCV CAP_FFMPEG
- Flux RTSP camera

### Ports reseau utilises
- RTSP camera sur 10.42.0.188:554.

### Comportement en cas d'erreur
- Si lecture frame echoue, tentative de reouverture du flux RTSP (cap.open).
- Pas de backoff explicite ni compteur d'echecs.

### Points d'attention
- URL RTSP et credentials en dur.
- Le script declare etre obsolete pour la capture (capture faite ailleurs via RTSP direct).
- La boucle de grab/retrieve peut reduire la latence mais augmente la charge decodeur.

## capture_manager.py

### Role
Cette classe gere la capture de medias de mission: prise de photos JPEG, enregistrement video MP4 et scan automatique de la serre avec deplacement PTZ. Elle lit directement le flux RTSP de la camera (sans topic Image ROS2 intermediaire). Elle pilote le PTZ en publiant des ordres ROS2 et synchronise ses acces video avec des verrous pour rester stable en multithread.

### Topics ROS2 publies et souscrits
- Publies:
  - /camera/ptz (geometry_msgs/msg/Point)
- Souscrits: aucun dans ce fichier.

### Services ROS2 exposes ou appeles
- Aucun service ROS2 expose.
- Aucun service ROS2 appele.
- Declencheurs internes: appels de methodes (run_auto_scan, take_photo, start_video, stop_video), potentiellement depuis un autre noeud applicatif.

### Dependances externes
- OpenCV (cv2) pour lecture RTSP et encodage
- Codec/stack multimedia (FFmpeg/GStreamer selon build OpenCV)
- threading, datetime, filesystem Python

### Ports reseau utilises
- RTSP camera sur 10.42.0.188:554.

### Comportement en cas d'erreur
- Si flux RTSP indisponible: logs d'erreur et annulation des operations dependantes.
- Protection contre scan concurrent (flag _scan_active + verrou).
- Arret propre en finally dans run_auto_scan: Stop PTZ + stop video meme en exception.
- stop_video attend le thread d'enregistrement avec timeout puis libere le writer.

### Points d'attention
- Publication PTZ suppose la presence d'un consumer (camera_control_node) pour agir reellement sur la camera.
- Le scan est temporel (sleep) et sensible a la latence mecanique/reseau.
- FourCC avc1 pour MP4 peut dependre des codecs disponibles sur la machine.

## gallery_manager.py

### Role
Cette classe publie regulierement la liste des medias de mission presents sur disque afin d'alimenter l'interface. Elle filtre les extensions image/video puis trie les fichiers du plus recent au plus ancien. Le resultat est envoye sous forme JSON dans un message ROS2 String.

### Topics ROS2 publies et souscrits
- Publies:
  - /ui/gallery_files (std_msgs/msg/String)
- Souscrits: aucun.

### Services ROS2 exposes ou appeles
- Aucun service ROS2 expose.
- Aucun service ROS2 appele.
- Declencheur interne: timer 1 Hz.

### Dependances externes
- os, json
- std_msgs/msg/String

### Ports reseau utilises
- Aucun port reseau direct dans ce fichier.

### Comportement en cas d'erreur
- Si dossier galerie absent: sortie silencieuse (pas de publication).
- Pas de try/except autour des acces disque; une erreur OS pourrait remonter.

### Points d'attention
- Le payload est une chaine JSON, pas un type message structure.
- Le scan disque est periodique; sur tres gros volumes de fichiers, impact potentiel performance.

## mission_gallery_http_server.py

### Role
Ce script expose les medias de mission via un serveur HTTP multi-thread. Il propose un endpoint de sante, un endpoint de liste et un endpoint de telechargement de fichier. Il applique des controles de securite simples (nom de fichier nettoye, extensions autorisees) pour limiter les acces non souhaites.

### Topics ROS2 publies et souscrits
- Aucun topic ROS2 publie.
- Aucun topic ROS2 souscrit.

### Services ROS2 exposes ou appeles
- Aucun service ROS2 expose.
- Aucun service ROS2 appele.
- Declencheurs: requetes HTTP entrantes.

### Dependances externes
- http.server (ThreadingHTTPServer)
- argparse, os, json, urllib.parse

### Ports reseau utilises
- Port HTTP 8092 par defaut (configurable via --port ou variable MISSION_GALLERY_HTTP_PORT).

### Comportement en cas d'erreur
- Reponses JSON explicites:
  - 400 pour nom de fichier invalide / extension non supportee
  - 404 pour fichier absent / route inconnue
  - 500 si lecture fichier impossible
- Logs applicatifs standards avec timestamp et IP client.
- Arret propre sur KeyboardInterrupt.

### Points d'attention
- Type MIME force a application/octet-stream pour tous les fichiers.
- Le service cree automatiquement le dossier galerie s'il n'existe pas.
- Pas d'authentification integree: exposition a proteger par reseau local, reverse proxy ou pare-feu.

## camera.launch.py

### Role
Ce fichier orchestre le demarrage des composants camera: MediaMTX, serveur HTTP de galerie et noeud camera_control_node. Il encapsule la logique de recherche du binaire mediamtx dans plusieurs emplacements possibles pour supporter differents layouts (source/install/workspace). Son objectif est d'assurer un demarrage reproductible de la chaine video et de controle.

### Topics ROS2 publies et souscrits
- Aucun topic publie directement (fichier de lancement).
- Aucun topic souscrit directement.

### Services ROS2 exposes ou appeles
- Aucun service ROS2 expose.
- Aucun service ROS2 appele.
- Declencheurs: lancement ROS2 (ros2 launch).

### Dependances externes
- Binaire MediaMTX (dossier mediamtx + executable mediamtx)
- Python3 module camera.mission_gallery_http_server
- launch, launch_ros, ament_index_python

### Ports reseau utilises
- Serveur galerie: 8092 (force dans la commande de lancement).
- MediaMTX (selon mediamtx.yml):
  - RTSP: 8554
  - RTSPS: 8322
  - RTP/RTCP UDP: 8000/8001
  - UDP multicast RTP/RTCP: 8002/8003
  - RTMP/RTMPS: 1935/1936
  - HLS: 8888

### Comportement en cas d'erreur
- Echec bloquant si dossier mediamtx introuvable: FileNotFoundError detaillee avec liste des chemins testes.
- Echec bloquant si executable mediamtx absent: FileNotFoundError explicite.
- Les processus lances ecrivent leur sortie sur ecran (output=screen).

### Points d'attention
- Ordre de demarrage: MediaMTX et serveur HTTP sont lances avant le noeud camera_control_node dans la description de launch.
- Recherche dynamique du binaire mediamtx (plusieurs chemins) utile mais sensible a la structure du workspace.
- Le launch ne demarre pas camera_bridge ni un noeud qui instancie explicitement CaptureManager/GalleryManager.
