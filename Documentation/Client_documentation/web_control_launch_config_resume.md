# Resume launch + configuration du package web_control (W4)

## web_control_full.launch.py

### Ce que fait ce fichier (langage simple)
Ce fichier demarre toute la pile minimale du client web_control en une seule commande. Il lance la passerelle websocket ROS (rosbridge), le noeud de publication camera, puis le backend web qui sert l interface et les services de capture.

### Parametres modifiables et leur effet (valeurs par defaut)
Parametres visibles dans ce launch:
- rosbridge_websocket.address: 0.0.0.0
  - Effet: autorise les connexions rosbridge depuis le reseau (pas seulement localhost).

Autres valeurs fixes dans ce launch:
- rosbridge port: 9090 (port par defaut de rosbridge_websocket)
- backend web port: 8000 (configure dans backend_node.py)

### Ordre de demarrage des noeuds
Ordre declare dans LaunchDescription:
1. rosbridge_server/rosbridge_websocket
2. web_control/camera_publisher
3. web_control/backend_node

Note pratique: l ordre est declare dans la liste de lancement, mais ROS2 peut initialiser les processus avec un leger chevauchement temporel.

### Dependances requises
- Packages ROS2:
  - rosbridge_server
  - web_control (ce package, avec executables camera_publisher et backend_node)
- Cote Python/ROS2 runtime:
  - rclpy
  - messages utilises par les noeuds (sensor_msgs, std_msgs, geometry_msgs, std_srvs)
- Outils externes indirects:
  - ffmpeg recommande pour la capture photo/video robuste (utilise par backend/capture manager)

### Commande de lancement typique (exemple complet)
Exemple complet apres build:
1. colcon build --packages-select web_control
2. source install/setup.bash
3. ros2 launch web_control web_control_full.launch.py

### Ce qui doit etre configure avant le premier lancement
- Avoir rosbridge_server installe et disponible.
- Verifier que les ports ne sont pas occupes:
  - 9090 pour rosbridge
  - 8000 pour le serveur web backend
- Verifier la source video dans le fichier configuration.json (host/port/stream).
- Verifier la disponibilite de ffmpeg si vous voulez les captures media fiables.
- Verifier que le navigateur client peut joindre ws://<hote>:9090 et http://<hote>:8000.

---

## configuration.json

### Ce que fait ce fichier (langage simple)
Ce fichier definit la source video utilisee par le frontend pour afficher le flux live. Il indique sur quel hote, port et nom de flux se connecter.

### Parametres modifiables et leur effet (valeurs par defaut)
Contenu actuel:
- video.host: 100.106.79.105
  - Effet: adresse IP ou nom du serveur media.
- video.port: 8889
  - Effet: port HTTP du serveur de stream.
- video.stream: mystream
  - Effet: chemin du flux.

Valeurs de repli dans le code frontend (si chargement config echoue):
- host: localhost
- port: 8889
- stream: mystream

URL construite cote frontend: http://<host>:<port>/<stream>/

### Dependances requises
- Pas de dependance ROS2 directe.
- Necessite un serveur de flux video joignable avec ces parametres.

### Ce qui doit etre configure avant le premier lancement
- Remplacer video.host par l hote reel du serveur media en production.
- Verifier que le port video.port est ouvert et accessible depuis le client web.
- Verifier que video.stream correspond exactement au nom publie par le serveur media.

---

## package.xml

### Ce que fait ce fichier (langage simple)
Ce fichier declare l identite ROS2 du package et ses dependances. C est lui qui permet a l environnement ROS2 de savoir quoi installer, quoi exporter, et ce qui est necessaire a l execution.

### Parametres modifiables et leur effet (valeurs par defaut)
Champs metadata:
- name: web_control
- version: 1.0.0
- description: Interface Web ROS2 Complete
- maintainer / maintainer_email
- license

Dependances declarees:
- depend:
  - rclpy
  - std_msgs
  - geometry_msgs
  - std_srvs
  - sensor_msgs
  - cv_bridge
- exec_depend:
  - rosbridge_server
  - web_video_server
- build_type exporte: ament_python

Effet: ces declarations pilotent build, execution et verification outillage ROS2.

### Dependances requises
- ROS2 Python runtime et messages listés ci-dessus.
- rosbridge_server pour la couche websocket du frontend.
- web_video_server est declare comme dependance d execution (meme si le code principal peut aussi fonctionner via flux externe selon configuration).

### Ce qui doit etre configure avant le premier lancement
- Completer les champs metadata encore generiques (maintainer/license).
- Verifier que toutes les dependances declarees sont installees dans l environnement cible.
- Confirmer que les dependances declarees correspondent bien au comportement reel du package pour eviter les ecarts build/runtime.

---

## setup.py

### Ce que fait ce fichier (langage simple)
Ce fichier definit comment le package Python est installe dans ROS2. Il enregistre les executables Python (backend_node, camera_publisher) et installe recursivement tout le dossier web pour que l interface soit disponible dans share/web_control.

### Parametres modifiables et leur effet (valeurs par defaut)
- package_name: web_control
- version: 1.0.0
- install_requires: setuptools
- entry_points console_scripts:
  - backend_node = web_control.backend_node:main
  - camera_publisher = web_control.camera_publisher:main
- data_files:
  - package.xml
  - launch/*.launch.py
  - tout le dossier web/ (scan recursif via package_files)

Effet important:
- Toute nouvelle ressource placee dans web/ est embarquee automatiquement a l installation, sans modifier setup.py.
- Les fichiers caches Mac commencant par ._ sont ignores volontairement.

### Dependances requises
- setuptools
- Environnement ament_python
- Arborescence web/ valide a l endroit attendu

### Ce qui doit etre configure avant le premier lancement
- Verifier que les points d entree console_scripts correspondent aux fichiers Python reels.
- Verifier que le dossier web/ contient bien tous les assets necessaires (js/css/images/pages).
- Verifier que launch/*.launch.py contient les launch files attendus a distribuer.
