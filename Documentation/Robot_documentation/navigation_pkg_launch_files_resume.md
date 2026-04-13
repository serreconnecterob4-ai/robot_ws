# Resume des launch files de navigation_pkg

Ce document couvre:
- global_launch.py
- nav2_minimal.launch.py

## global_launch.py

### Ce que lance ce fichier (ordre de demarrage effectif)
1. EKF de gps_package via include de ekf_launch.py (a t=6s).
2. Relay odometrie/UI vers rosbridge: navigation_pkg.odom_rosbridge_relay (a t=8s).
3. rosbridge websocket server (a t=10s).
4. Serveur de mission waypoints: navigation_pkg.waypoint_action_server (a t=10s).
5. Stack Nav2 minimale via include de nav2_minimal.launch.py (a t=20s).
6. Filtre securite teleop: navigation_pkg.cmd_vel_gate (a t=41s).
7. Controle bras curt_mini (arm_controller.py) (a t=45s).

### Ordre et delais de demarrage (TimerAction)
- t=6s EKF en premier: prepare la chaine d'etat (odom fusionnee) avant la navigation.
- t=8s relay odom/web: commence la publication vers l'UI tot, meme avant Nav2.
- t=10s rosbridge + waypoint server: l'UI peut deja dialoguer mission/feedback.
- t=20s Nav2: laisse le temps aux briques de base (etat/bridge) de se stabiliser.
- t=41s cmd_vel_gate: demarrage volontairement tardif pour eviter un blocage prematuré de teleop pendant l'initialisation.
- t=45s controle bras: lance en dernier pour limiter la concurrence au demarrage.

Pourquoi cet ordre:
- Le fichier indique explicitement un demarrage echelonné pour eviter les race conditions.
- Le waypoint server depend de Nav2 a l'execution (il attend le serveur d'action), mais peut etre lance avant et rester en attente.
- Le relay vers rosbridge est lance avant rosbridge; il reessaie periodiquement jusqu'a connexion.

### Arguments configurables au lancement
- Aucun DeclareLaunchArgument dans ce fichier.
- Consequence: pas d'arguments launch exposes officiellement sur global_launch.py.
- Parametres actuellement figes dans le code:
  - bridge_host=100.92.193.85
  - bridge_port=9090
  - address rosbridge=0.0.0.0
  - delais TimerAction (6/8/10/10/20/41/45).

### Fichiers de config appeles et role
- Include: gps_package/launch/ekf_launch.py
  - Role: localisation/fusion capteurs (EKF).
- Include: navigation_pkg/launch/nav2_minimal.launch.py
  - Role: demarrage des serveurs Nav2 et lifecycle manager.
- Script externe curt_mini: arm_controller.py
  - Role: controle du bras robot.

### Dependances inter-packages requises
- gps_package (EKF).
- navigation_pkg (noeuds mission, gate, relay, launch Nav2).
- nav2_* packages via nav2_minimal.launch.py.
- rosbridge_server.
- curt_mini (arm_controller installe dans le prefix package).

### Commande de lancement typique (exemple complet)
```bash
source /opt/ros/jazzy/setup.bash
source ~/Bureau/robot_ws/robot_ws/install/setup.bash
ros2 launch navigation_pkg global_launch.py
```

### Ce qui ne demarre PAS automatiquement
- La simulation Gazebo, les bridges GZ et RViz de display.launch.py.
- Le package camera (camera.launch.py) et ses services medias.
- Les noeuds hors liste explicite dans ce launch (ex: autres outils UI, diagnostics, enregistrements).

### Points d'attention
- Ordre critique: odom_rosbridge_relay part avant rosbridge, mais c'est volontaire (reconnexion interne).
- Les valeurs de host/port rosbridge sont en dur dans la commande du relay.
- Les timings fixes peuvent etre trop courts ou trop longs selon la machine/reseau; si besoin, parametrer les TimerAction.
- waypoint_action_server peut recevoir une mission avant que Nav2 soit pret; il attend Nav2 mais abort si indisponible trop longtemps.

## nav2_minimal.launch.py

### Ce que lance ce fichier (liste ordonnee)
1. nav2_map_server/map_server
2. nav2_planner/planner_server
3. nav2_controller/controller_server
4. nav2_bt_navigator/bt_navigator
5. nav2_behaviors/behavior_server
6. nav2_lifecycle_manager/lifecycle_manager_navigation (autostart)

### Ordre et delais de demarrage
- Pas de TimerAction: les actions Node sont declarees directement.
- En pratique, lifecycle_manager_navigation avec autostart active et configure les transitions lifecycle des 5 serveurs.
- Dependance logique:
  - map_server doit avoir la map chargee,
  - planner/controller/bt/behavior doivent etre actifs pour accepter des goals,
  - lifecycle_manager orchestre cet ordre de mise en service.

### Arguments configurables au lancement
- Aucun DeclareLaunchArgument dans ce fichier.
- Valeurs internes definies dans le launch:
  - map_file = navigation_pkg/maps/map.yaml
  - nav2_params = navigation_pkg/config/nav2_params.yaml
  - BT custom:
    - navigate_to_pose_limited_recovery.xml
    - navigate_through_poses_limited_recovery.xml
  - always_reload_bt_xml = True
  - lifecycle manager: use_sim_time=True, autostart=True, node_names=[...].

### Fichiers de config appeles et role
- navigation_pkg/maps/map.yaml
  - Carte statique de navigation utilisee par map_server.
- navigation_pkg/config/nav2_params.yaml
  - Parametres planner/controller/costmaps/BT/behavior server.
- navigation_pkg/config/behavior_trees/navigate_to_pose_limited_recovery.xml
  - Arbre BT NavigateToPose avec recoveries limites.
- navigation_pkg/config/behavior_trees/navigate_through_poses_limited_recovery.xml
  - Arbre BT NavigateThroughPoses avec recoveries limites.

### Dependances inter-packages requises
- nav2_map_server
- nav2_planner
- nav2_controller
- nav2_bt_navigator
- nav2_behaviors
- nav2_lifecycle_manager
- map + TF + odometrie fournis par d'autres briques deja lancees (ex: EKF/robot model/simulation ou robot reel).

### Commande de lancement typique (exemple complet)
```bash
source /opt/ros/jazzy/setup.bash
source ~/Bureau/robot_ws/robot_ws/install/setup.bash
ros2 launch navigation_pkg nav2_minimal.launch.py
```

### Ce qui ne demarre PAS automatiquement
- EKF (gps_package), rosbridge, waypoint_action_server, cmd_vel_gate, arm_controller.
- Gazebo, ros_gz_bridge, robot_state_publisher, RViz.
- Les producteurs de /scan, /tf, /odometry/filtered si non fournis par ailleurs.

### Points d'attention
- Sans chaine TF/odom/lidar correcte, Nav2 demarre mais n'est pas exploitable.
- map_server depend du fichier map.yaml present et coherent avec la carte.
- Les BT custom reduisent les recoveries globales (choix volontaire anti-boucle): a verifier selon votre terrain.
- Les tolerances et vitesses de nav2_params.yaml impactent fortement la validation des waypoints et les faux succes/abandons.
