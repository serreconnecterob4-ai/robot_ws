# Resume backend du package web_control

## backend_node.py

### Role
Ce fichier est le noeud principal cote client web. Il lance le serveur HTTP local sur le port 8000, relie l'interface web aux topics/services ROS2, et centralise les fonctions de log, de trajectoires et de gestion de galerie. Il orchestre aussi la synchronisation des medias du robot vers le client.

### Topics ROS2 publies et souscrits

| Nom | Type | Direction | Usage |
|---|---|---|---|
| /robot/cmd_vel_buttons | geometry_msgs/msg/Point | Publie | Envoie une commande d'arret (urgence). |
| /ui/trajectory_files | std_msgs/msg/String | Publie | Publie la liste JSON des trajectoires disponibles. |
| /ui/system_logs | std_msgs/msg/String | Publie | Publie des logs JSON pour l'IHM. |
| /ui/gallery_files | std_msgs/msg/String | Publie (via GalleryManager) | Met a jour la liste JSON des medias. |
| /robot/cmd_vel_buttons | geometry_msgs/msg/Point | Souscrit | Callback present (debug/pass). |
| /camera/ptz | geometry_msgs/msg/Point | Souscrit | Callback present (debug/pass). |
| /camera/delete_image | std_msgs/msg/String | Souscrit | Supprime un media local. |
| /ui/save_trajectory | std_msgs/msg/String | Souscrit | Sauvegarde un trajet JSON. |
| /ui/delete_trajectory | std_msgs/msg/String | Souscrit | Supprime un fichier trajet. |
| /robot/emergency_stop | std_msgs/msg/Bool | Souscrit | Declenche arret d'urgence robot/capture. |
| /ui/mission_result | std_msgs/msg/String | Souscrit | Declenche une synchro galerie robot->client. |

### Services ROS2 exposes ou appeles

| Nom | Type | Expose/Appel | Declencheur |
|---|---|---|---|
| /camera/take_photo | std_srvs/srv/Trigger | Expose | Appel UI/ROS pour capturer une photo. |
| /camera/start_video | std_srvs/srv/Trigger | Expose | Appel UI/ROS pour demarrer une video. |
| /camera/stop_video | std_srvs/srv/Trigger | Expose | Appel UI/ROS pour arreter/sauvegarder une video. |

Aucun client de service ROS2 explicite dans ce fichier.

### Parametres configurables

| Nom | Defaut | Effet |
|---|---|---|
| robot_gallery_sync_enabled | true | Active/desactive la synchro periodique avec la galerie du robot. |
| robot_gallery_host | localhost | Adresse du serveur galerie distant. |
| robot_gallery_port | 8092 | Port HTTP du serveur galerie distant. |
| robot_gallery_sync_period_sec | 10.0 | Periode de synchro automatique. |
| robot_gallery_timeout_sec | 3.0 | Timeout reseau pour listage/telechargement. |
| video.capture_url (configuration.json) | non defini | URL capture prioritaire pour ffmpeg. |
| video.rtsp_url (configuration.json) | non defini | URL RTSP alternative. |
| video.host (configuration.json) | localhost (normalise) | Hote utilise pour reconstruire l'URL RTSP. |
| video.stream (configuration.json) | mystream | Nom du stream RTSP. |
| video.rtsp_port (configuration.json) | 8554 | Port RTSP. |
| PORT_WEB (constante) | 8000 | Port du serveur web local. |

### Dependances externes
- Python: rclpy, ament_index_python, std_srvs, geometry_msgs, std_msgs, http.server, socketserver, urllib, threading, shutil, json, os, time.
- Modules internes: CaptureManager, GalleryManager.
- Outils systeme (indirect): ffmpeg via CaptureManager.

### Comportement en cas d'erreur
- Le serveur HTTP loggue les erreurs de demarrage (ex: port occupe).
- Les uploads HTTP repondent avec 400/404/500 selon le cas.
- La sync galerie distante est protegee par un lock anti-chevauchement et marque online/offline.
- Les telechargements utilisent un fichier temporaire .part puis remplacement atomique.
- Les erreurs de capture sont remontees dans la reponse de service et les logs UI.
- En arret d'urgence, la video est stoppee si active, puis une commande d'arret est publiee plusieurs fois.

### Points d'attention
- Le meme topic /robot/cmd_vel_buttons est a la fois souscrit et publie.
- cb_ptz_relay et cb_auto_scan existent mais ne sont pas relies explicitement dans l'initialisation.
- os.chdir(self.web_dir) modifie le repertoire courant du process entier.
- Validation securite basique sur les noms de fichiers upload/suppression.

---

## camera_publisher.py

### Role
Ce fichier lit un flux camera RTSP et publie des images ROS2 vers /camera/clear. Il sert de passerelle simple entre la camera reseau et le reste des composants (capture, IHM).

### Topics ROS2 publies et souscrits

| Nom | Type | Direction | Usage |
|---|---|---|---|
| /camera/clear | sensor_msgs/msg/Image | Publie | Publie les frames du flux RTSP. |

Aucun topic souscrit.

### Services ROS2 exposes ou appeles
Aucun.

### Parametres configurables

| Nom | Defaut | Effet |
|---|---|---|
| URL RTSP (hardcodee) | rtsp://localhost:8554/mystream | Source video lue par OpenCV. |
| Frequence timer (hardcodee) | 15 FPS | Cadence de publication sur /camera/clear. |

### Dependances externes
- Python: rclpy, sensor_msgs, cv2 (OpenCV), cv_bridge.

### Comportement en cas d'erreur
- Si la lecture frame echoue (ret=false), la frame est ignoree.
- Pas de mecanisme explicite de reconnexion ni de logs d'erreur detailles.

### Points d'attention
- URL RTSP en dur, non parametree en ROS2.
- Pas de verification explicite de l'ouverture du flux au demarrage.
- Pas de release explicite de VideoCapture a l'arret.

---

## capture_manager.py

### Role
Ce module gere la capture photo et video pour la galerie locale. Il privilegie ffmpeg pour capturer directement le flux RTSP (avec audio possible), et utilise OpenCV en fallback si ffmpeg echoue.

### Topics ROS2 publies et souscrits

| Nom | Type | Direction | Usage |
|---|---|---|---|
| /camera/clear | sensor_msgs/msg/Image | Souscrit | Recupere la derniere image pour fallback photo/video. |

Aucun topic publie.

### Services ROS2 exposes ou appeles
Aucun directement. Ce module est appele par les callbacks de services du backend.

### Parametres configurables

| Nom | Defaut | Effet |
|---|---|---|
| rtsp_url | rtsp://localhost:8554/mystream | Source ffmpeg pour photo/video. |
| timeout photo ffmpeg | 15 s | Limite d'attente pour une capture photo. |
| delai validation ffmpeg video | 0.35 s | Verifie que ffmpeg ne quitte pas immediatement. |
| timeout stop ffmpeg | 5 s | Attente avant terminate force. |
| fallback OpenCV FPS | 10.0 | Cadence du writer fallback (video seule). |
| fallback OpenCV codec | avc1 | Codec du fichier mp4 fallback. |

### Dependances externes
- Python: cv2, cv_bridge, sensor_msgs, subprocess, datetime, os, time.
- Outil systeme: ffmpeg (obligatoire pour le mode principal).

### Comportement en cas d'erreur
- Photo: tentative ffmpeg, puis fallback sur derniere image ROS si disponible.
- Video: tentative ffmpeg (audio+video), puis fallback OpenCV (video seule) si image disponible.
- Stop video: tentative arret propre ffmpeg via stdin, puis terminate en secours.
- Les erreurs sont logguees et renvoyees au backend appelant.

### Points d'attention
- Le fallback OpenCV depend d'une image deja recue sur /camera/clear.
- Sans ffmpeg operationnel, les fonctions media sont degradees.
- Le codec avc1 peut dependre des codecs disponibles sur la machine.
- ffmpeg_thread est defini mais non utilise.

---

## gallery_manager.py

### Role
Ce module maintient la liste des fichiers media visibles dans l'interface. Il scanne periodiquement le dossier galerie, trie les elements du plus recent au plus ancien, puis publie la liste au format JSON.

### Topics ROS2 publies et souscrits

| Nom | Type | Direction | Usage |
|---|---|---|---|
| /ui/gallery_files | std_msgs/msg/String | Publie | Envoie la liste JSON des medias a l'IHM. |

Aucun topic souscrit.

### Services ROS2 exposes ou appeles
Aucun.

### Parametres configurables

| Nom | Defaut | Effet |
|---|---|---|
| Periode timer (hardcodee) | 1.0 s | Frequence de publication de la galerie. |
| Extensions prises en compte | .jpg, .png, .avi, .mp4, .webm | Filtrage des fichiers publies. |

### Dependances externes
- Python: os, json, std_msgs.

### Comportement en cas d'erreur
- Si le dossier galerie n'existe pas, la publication est ignoree.
- Pas de try/except global dans publish_gallery: certaines erreurs filesystem peuvent remonter.

### Points d'attention
- Scan toutes les secondes: impact possible si dossier tres volumineux.
- Pas de pagination ni de limite de taille du payload JSON.
- Contrat implicite frontend/backend: JSON transporte dans std_msgs/String.
