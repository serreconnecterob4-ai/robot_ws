# Structure generale du repertoire robot_ws/src

## 1) Role global
Ce repertoire contient la stack ROS2 du robot (capteurs, navigation, action serveur, controle moteurs, camera, interfaces), avec un mix de packages Python (ament_python) et CMake (ament_cmake).

Objectif global:
- percevoir l'etat robot (IMU, GPS, odometrie, camera),
- fusionner/filtrer la localisation,
- executer des missions de waypoints,
- commander la base et les actionneurs,
- exposer les flux au client web via rosbridge.

## 2) Packages principaux

### A. camera/
Package Python pour acquisition video/capture et services medias.

Fichiers cles:
- `camera/launch/camera.launch.py`: lance MediaMTX, serveur HTTP de galerie de mission (port 8092), et `camera_control_node`.
- `camera/camera_control_node.py`: noeud de controle camera.
- `camera/camera_bridge.py`: pont camera vers ROS.
- `camera/capture_manager.py`: photo/video.
- `camera/gallery_manager.py`: gestion galerie.
- `camera/mission_gallery_http_server.py`: serveur HTTP des medias de mission.

Points importants:
- le launch cherche dynamiquement le binaire `mediamtx` dans plusieurs emplacements.
- ce package sert de source media pour le reste de la stack.

### B. navigation_pkg/
Package Python coeur navigation/missions.

Fichiers cles:
- `navigation_pkg/launch/global_launch.py`: orchestration complete (EKF, relay odom, rosbridge, Nav2, serveur waypoints, gate cmd_vel, bras).
- `navigation_pkg/launch/nav2_minimal.launch.py`: lancement Nav2 minimal.
- `navigation_pkg/waypoint_action_server.py`: logique mission waypoint (execution, feedback/result).
- `navigation_pkg/cmd_vel_gate.py`: controle/filtrage des commandes vitesse.
- `navigation_pkg/odom_rosbridge_relay.py`: relay ROS <-> rosbridge pour l'odometrie/commandes UI.

Points importants:
- demarrage echelonné via `TimerAction` pour reduire les courses au lancement.
- depend de `navigation_interfaces`, `camera`, `gps_package`, `curt_mini`.

### C. navigation_interfaces/
Package d'interfaces ROS2 (actions/messages partages).

Fichier cle:
- `navigation_interfaces/action/NavigateWaypoints.action`:
  - Goal: tableaux X/Y + flags photo.
  - Result: succes + message.
  - Feedback: index courant, distance restante, ETA, position robot, prise photo en cours.

### D. gps_package/
Package Python de localisation/fusion GPS + IMU + odometrie.

Fichiers cles:
- `gps_package/launch/ekf_launch.py`: lance TF statiques, EKF local, navsat_transform, EKF global.
- `gps_package/config/ekf_local.yaml`, `ekf_global.yaml`, `navsat.yaml`: parametrage robot_localization.
- `gps_package/README.md`: doc architecture (fusion multi-capteurs et depannage).

Points importants:
- separation local/global filter:
  - local = reactif,
  - global = stable et corrige par GPS.
- sortie utile pour navigation: `/odometry/filtered`.

### E. openzenros2/
Driver IMU/OpenZen (package CMake, nom runtime: openzen_driver).

Fichiers cles:
- `openzenros2/launch/openzen_lpms.launch.py`: lance le noeud capteur et un rqt_plot exemple.
- `openzenros2/README.md`: usage, parametres capteur, commandes calibration.

Role:
- publication des donnees inertielles (IMU, orientation, etc.) vers ROS2.

### F. candle_ros2/
Package C++ de pilotage des drives MD80 via CANdle.

Elements cles:
- services de setup/activation des drives,
- topics de commande mouvement/impedance/PID,
- publication etat articulations.

Role:
- interface bas niveau actionneurs (moteurs).

### G. curt_mini/ et ipa_ros2_control/
Packages lies a la plateforme mecanique/controle du robot (bringup, config, drivers, ros2_control).

Role:
- description robot, controle hardware, scripts utilitaires (ex: `arm_controller.py` utilise par `navigation_pkg/global_launch.py`).

## 3) Architecture logique (vue systeme)

1. **Capteurs**
- camera (flux video + captures),
- IMU (openzen),
- GPS (fix),
- odometrie roues.

2. **Fusion localisation**
- `gps_package` construit une odometrie stable `/odometry/filtered` via robot_localization.

3. **Navigation mission**
- `navigation_pkg` recoit des waypoints (action/interface),
- pilote Nav2 + publie feedback/resultat de mission,
- gere la securite de commande avec `cmd_vel_gate`.

4. **Actionneurs**
- `candle_ros2` et `curt_mini` pilotent moteurs/bras.

5. **Pont UI / reseau**
- rosbridge + relay odom permettent l'echange avec le client web.
- camera expose aussi la galerie mission en HTTP.

## 4) Launch d'orchestration principal
Le point d'entree global est:
- `navigation_pkg/launch/global_launch.py`

Il enchaine typiquement:
1. EKF/localisation,
2. relay odometrie vers bridge,
3. rosbridge websocket,
4. Nav2,
5. serveur waypoint,
6. gate cmd_vel,
7. controle bras.

## 5) Resume court
`robot_ws/src` est une stack robot complete:
- perception (camera, IMU, GPS),
- estimation d'etat (EKF),
- navigation mission (Nav2 + action serveur),
- controle bas niveau (drives/bras),
- exposition reseau pour l'IHM web.
