# Resume frontend JS du dashboard web_control (W2)

## Ordre de chargement (critique)
Dans la page principale, les scripts sont charges dans cet ordre:
1. 01-core-ros.js
2. 02-PTZ.js
3. 03-communication-avec-robot.js
4. 04-media-gallery.js
5. 05-trajectoires.js
6. 06-UI-minimap.js

Cet ordre est important car plusieurs modules utilisent des variables/fonctions globales definies par les precedents.

---

## 01-core-ros.js

### Role
Ce module est le socle reseau du dashboard. Il connecte l'interface web a ROS2 via rosbridge, configure le flux video (WebRTC ou flux externe), et expose les publishers/subscribers/services utilises par les autres modules. Il gere aussi les reconnexions et un fallback local quand la connexion serveur est indisponible.

### Topics ROS2 via rosbridge (nom + type)
Publies:
- /robot/cmd_vel - geometry_msgs/Twist
- /camera/ptz - geometry_msgs/Point
- /mission/status - std_msgs/Bool
- /camera/delete_image - std_msgs/String
- /camera/zoom - std_msgs/Float32
- /camera/focus - std_msgs/Float32
- /camera/autofocus - std_msgs/Bool
- /camera/light - std_msgs/Bool
- /camera/alert - std_msgs/Bool
- /robot/volume - std_msgs/Float32
- /robot/arm_speed - std_msgs/Float32
- /robot/arm_position - std_msgs/Float32
- /ui/click - geometry_msgs/Point
- /ui/system_logs - std_msgs/String
- /robot/emergency_stop - std_msgs/Bool

Souscrits:
- /ui/trajectory_files - std_msgs/String
- /ui/gallery_files - std_msgs/String
- /robot/battery - std_msgs/Float32

### Services ROS2 appeles
- /camera/take_photo - std_srvs/Trigger: client expose pour la prise photo (utilise par module media)
- /camera/start_video - std_srvs/Trigger: client expose pour demarrer un enregistrement cote ROS2
- /camera/stop_video - std_srvs/Trigger: client expose pour arreter/sauvegarder l'enregistrement cote ROS2

### Interactions avec les autres modules JS
- Fournit les objets globaux utilises ensuite: ros, rosRobot, cmdVelPub, ptzPub, missionPub, deletePub, zoomPub, focusPub, autofocusPub, lightPub, alertPub, robotVolumePub, armSpeedPub, armPosPub, clickPub, emergencyPub, photoClient, startVideoClient, stopVideoClient.
- Appelle updateTrajectoryList(...) sur reception de /ui/trajectory_files (fonction implementee dans 05-trajectoires.js).
- Appelle updateGallery(...) sur reception de /ui/gallery_files (fonction implementee dans 04-media-gallery.js).

### Elements UI pilotes
- Etat connexion: #status
- Batterie: #battery
- Etat flux video: #webrtcStatus
- Flux video: #cameraFeed et conteneur #videoContainer

### Points d'attention
- Double connexion rosbridge: une vers serveur local, une vers robot.
- Fallback publish vers ws://localhost:9090 quand route serveur indisponible.
- WebRTC et flux externe peuvent se substituer selon configuration.json.
- Reconnexion automatique periodique et sur events navigateur (online/focus).

---

## 02-PTZ.js

### Role
Ce module gere les commandes de conduite manuelle (clavier + pad), le controle PTZ, et les reglages camera/bras/volume. Il applique aussi la persistance des sliders (localStorage) et les conversions hauteur bras <-> pourcentage.

### Topics ROS2 via rosbridge (nom + type)
Publies:
- /robot/cmd_vel - geometry_msgs/Twist
- /camera/ptz - geometry_msgs/Point
- /ui/cancel_mission - std_msgs/String (pause auto lors reprise manuelle)
- /camera/zoom - std_msgs/Float32
- /camera/focus - std_msgs/Float32
- /camera/autofocus - std_msgs/Bool
- /camera/light - std_msgs/Bool
- /camera/alert - std_msgs/Bool
- /robot/volume - std_msgs/Float32
- /robot/arm_speed - std_msgs/Float32
- /robot/arm_position - std_msgs/Float32

Souscrits:
- Aucun topic cree localement dans ce module.

### Services ROS2 appeles
- Aucun.

### Interactions avec les autres modules JS
- Depend des publishers et du logger fournis par 01-core-ros.js.
- Utilise missionActive/missionPaused + missionCancelPub definis par 03/05 pour la pause automatique sur commande manuelle.
- Expose des fonctions reutilisees ailleurs: sendCmd, sendPtz, updateSpeed, updateZoom, setAutofocusState, etc.

### Elements UI pilotes
- D-pad robot et camera (touch/mouse + clavier ZQSD / OKLM)
- Sliders: #speedSlider, #zoomSlider, #focusSliderModal, #armSpeedSlider, #armPosSlider, #robotVolumeSlider
- Affichages: #speedVal, #zoomVal, #zoomValModal, #focusValModal, #armSpeedVal, #armPosVal, #robotVolumeVal
- Boutons: #btnAutofocus, #btnLamp, #btnMic, #btnAlert

### Points d'attention
- Envoi PTZ maintenu en repetition tant que touche enfoncee, puis stop publie plusieurs fois.
- LUT de calibration bras (cm <-> %) a conserver coherente avec la mecanique reelle.
- Fait des effets de bord mission (pause) quand mouvement manuel detecte.

---

## 03-communication-avec-robot.js

### Role
Ce module gere le cycle de mission et le suivi robot en temps reel. Il traite les feedbacks/resultats mission, suit l'odometrie, et synchronise l'etat mission avec la minimap et les boutons de l'interface.

### Topics ROS2 via rosbridge (nom + type)
Publies:
- /ui/start_mission - std_msgs/String
- /ui/cancel_mission - std_msgs/String (pause/resume/cancel)
- /robot/emergency_stop - std_msgs/Bool
- /mission/status - std_msgs/Bool

Souscrits:
- /ui/mission_feedback - std_msgs/String
- /ui/mission_result - std_msgs/String
- /odometry/filtered - nav_msgs/Odometry

### Services ROS2 appeles
- Aucun.

### Interactions avec les autres modules JS
- Utilise sendCmd/sendPtz + setEmergencyButtonPausedState de 02-PTZ.js.
- Utilise draw/update/hide provenant de 05-trajectoires.js et 06-UI-minimap.js (updateTrajectoryDisplay, updateRobotDotOnMap, hideRobotDot, unloadCurrentTrajectory).
- Lit et met a jour des etats globaux partages: missionActive, missionPaused, currentTrajectoryData, currentWaypointIndex, lastKnownRobotPixel.

### Elements UI pilotes
- Bouton urgence: #btnEmergency
- Bouton mission: #btnMission
- Zone info mission: #trajInfo
- Badge offline robot (via helper): #robotOfflineBadge

### Points d'attention
- Deduplication feedback/resultat pour limiter les doublons UI.
- Gestion offline odometrie avec timeout et desactivation auto du mode lock carte.
- Restauration de trajet via cookie si feedback recu sans trajet charge.

---

## 04-media-gallery.js

### Role
Ce module gere la capture media depuis le dashboard: photo et video, puis publication/suppression dans la galerie rapide. Il privilegie la capture navigateur (WebRTC + upload HTTP) et bascule sur services ROS2 en fallback.

### Topics ROS2 via rosbridge (nom + type)
Publies:
- /ui/start_mission - std_msgs/String
- /ui/cancel_mission - std_msgs/String
- /mission/status - std_msgs/Bool
- /camera/delete_image - std_msgs/String

Souscrits:
- Aucun topic cree localement dans ce module (la maj galerie arrive via callback defini en 01).

### Services ROS2 appeles
- /camera/take_photo - std_srvs/Trigger: appele si capture WebRTC indisponible
- /camera/start_video - std_srvs/Trigger: appele si MediaRecorder indisponible
- /camera/stop_video - std_srvs/Trigger: appele pour arret video en mode fallback ROS2

### Interactions avec les autres modules JS
- Depend de 01-core-ros.js pour photoClient/startVideoClient/stopVideoClient/deletePub/logEvent.
- Depend de 03/05 pour etat mission et fonctions globales (missionActive, currentTrajectoryData, unloadCurrentTrajectory, updateTrajectoryDisplay).
- Rend l'affichage #galleryGrid qui est alimente par les messages recus dans 01.

### Elements UI pilotes
- Boutons: #btnMission, #btnRecord
- Infos mission: #trajInfo
- Selecteur trajet: #trajSelect
- Galerie rapide: #galleryGrid
- Video plein ecran: #videoContainer / #cameraFeed

### Points d'attention
- Upload HTTP direct vers /upload_photo et /upload_video (pas de retry natif).
- MediaRecorder stocke les chunks en memoire: attention aux videos longues.
- Si robot offline, certaines actions mission sont bloquees cote UI.

---

## 05-trajectoires.js

### Role
Ce module gere la logique trajectoire du dashboard: chargement JSON, rendu des chemins/waypoints sur la minimap, conversion coordonnees metres<->pixels, et etat mission partage avec les autres modules. Il sert de reference de donnees trajectoires pour le reste de l'IHM.

### Topics ROS2 via rosbridge (nom + type)
- Aucun topic ROS cree ou souscrit directement dans ce fichier.
- Ce module est invoque par callbacks ROS de 01 et 03 (ex: updateTrajectoryList, updateTrajectoryDisplay).

### Services ROS2 appeles
- Aucun.

### Interactions avec les autres modules JS
- Recoit updateTrajectoryList(...) depuis la souscription /ui/trajectory_files definie en 01.
- Est utilise par 03 et 04 pour charger/vider/rafraichir les trajectoires.
- Partage des variables globales critique: missionActive, missionPaused, currentTrajectoryData, currentWaypointIndex, lastKnownRobotPixel, lastKnownRobotPosition.
- Fournit updateRobotDotOnMap(...) et hideRobotDot(...) utilises par 03 et 06.

### Elements UI pilotes
- Couche SVG minimap: #displayPath, #passedPath, #startCircle, #waypointsGroup, #robotToMissionLink
- Widgets mission: #trajSelect, #trajInfo
- Carte: #mapArea

### Points d'attention
- Depend de constantes geo hardcodees (originPixel, metersPerPixel, thetaDegrees, etc.).
- Chargement trajectoire via fetch(trajectories/<fichier>) et cookies de restauration.
- Rendu fortement couple aux offsets de panning et aux dimensions affichees de la map.

---

## 06-UI-minimap.js

### Role
Ce module pilote l'ergonomie generale de l'UI (modale reglages, dark mode, toast) et surtout l'interaction minimap (pan, lock, recentrage, resize sync). Il maintient une experience fluide de navigation visuelle autour du robot.

### Topics ROS2 via rosbridge (nom + type)
- Aucun topic ROS cree ou souscrit directement dans ce fichier.

### Services ROS2 appeles
- Aucun.

### Interactions avec les autres modules JS
- Utilise logEvent et fonctions de commande de 01/02 (ex: updateSpeed, setAutofocusState).
- Depend des etats robot/mission calcules dans 03/05 (_odometryOffline, lastKnownRobotPixel, _currentlyTakingPhoto).
- Appelle updateTrajectoryDisplay(...) et updateRobotDotOnMap(...) fournis par 05.

### Elements UI pilotes
- Modale: #settingsModal
- Dark mode: #btnDarkMode + classe body.dark-mode
- Map mode: #mapModeBtn, #mapModeIcon
- Minimap: #mapArea
- Toast container dynamique: #toast-container

### Points d'attention
- Mode Lock bloque si odometrie offline.
- Panning par pointer events, avec synchronisation offsets map et overlays.
- Beaucoup de logique basee sur variables globales: necessite strictement l'ordre de chargement.
