# Resume des noeuds Python de navigation_pkg

Ce document couvre les noeuds Python suivants:
- waypoint_action_server.py
- cmd_vel_gate.py
- odom_rosbridge_relay.py

## waypoint_action_server.py

### Role
Ce noeud est le chef d'orchestre des missions du robot: il recoit une liste de waypoints, pilote Nav2 pour les atteindre, et renvoie l'etat de mission vers l'interface web. Concretement, c'est lui qui decide quand avancer, quand faire une pause photo, quand reprendre, et quand abandonner en securite. Il gere aussi des protections anti-boucle pour eviter des faux succes de navigation.

### Topics ROS2 publies et souscrits
- Publies:
  - /ui/mission_feedback (std_msgs/msg/String, publication robot -> UI, JSON)
  - /ui/mission_result (std_msgs/msg/String, publication robot -> UI, JSON)
- Souscrits:
  - /ui/start_mission (std_msgs/msg/String, UI -> robot)
  - /ui/cancel_mission (std_msgs/msg/String, UI -> robot; commandes cancel/pause/resume)
  - /robot/cmd_vel (geometry_msgs/msg/Twist, teleop -> robot; observe pendant pause)
  - /rosout (rcl_interfaces/msg/Log, systeme -> robot; surveille planner_server)

### Actions ROS2 servies ou appelees
- Action servie:
  - navigate_waypoints (navigation_interfaces/action/NavigateWaypoints)
  - Goal:
    - float64[] waypoints_x
    - float64[] waypoints_y
    - bool[] take_photo
  - Feedback:
    - current_waypoint_index
    - waypoints_remaining
    - distance_remaining
    - estimated_time_remaining
    - robot_x, robot_y
    - is_taking_photo
  - Result:
    - success
    - message

- Actions appelees:
  - navigate_through_poses (nav2_msgs/action/NavigateThroughPoses)
    - Goal: sequence de PoseStamped generee depuis waypoints
    - Feedback utilise: pose courante, nombre de poses restantes, distance restante, ETA
    - Result exploite: statut (SUCCEEDED/CANCELED/ABORTED)
  - navigate_to_pose (nav2_msgs/action/NavigateToPose)
    - Utilise pour le repli maison et le mode anti-boucle waypoint unique
    - Result exploite: succes/echec + annulation possible

### Services ROS2 exposes ou appeles
- Services exposes: aucun.
- Services appeles:
  - /controller_server/set_parameters (rcl_interfaces/srv/SetParameters)
  - Usage: activer/desactiver dynamiquement FollowPath.allow_reversing pendant degagement.

### Logique principale
- Recoit une mission (waypoints + indicateurs photo), puis attend Nav2.
- Envoie les waypoints restants a Nav2 et suit la progression via feedback.
- Si waypoint photo detecte: annule temporairement Nav2, lance un scan photo/video via CaptureManager, puis reprend au waypoint suivant.
- Si pause demandee: annule Nav2, passe en etat pause, publie un feedback de pause, puis reprend sur resume ou reprise auto watchdog.
- Si echec de navigation:
  - tente un degagement (marche arriere temporaire),
  - sinon tente un repli vers la position home,
  - si trop d'echecs, termine en abort.
- Sur succes suspect (faux positif Nav2), reprend a l'index probable restant; en cas de repetition, bascule en navigation waypoint par waypoint (anti-boucle).

### Lien avec les autres packages
- Nav2: coeur de deplacement (NavigateThroughPoses, NavigateToPose, parametres controller).
- navigation_interfaces: type d'action NavigateWaypoints expose aux clients.
- camera: utilise camera.capture_manager.CaptureManager pour scan photo aux waypoints marques.
- gps_package: la chaine complete lance EKF/GPS dans global_launch; ce noeud charge aussi config_gps.json (origine/home) pour le repli.
- Interface web: demarrage/annulation/pause via /ui/start_mission et /ui/cancel_mission, retour etat/resultat via /ui/mission_feedback et /ui/mission_result.

### Comportement en cas d'erreur ou d'arret d'urgence
- Arret d'urgence operateur: commande cancel -> annulation du goal courant + annulation Nav2.
- Pause operateur: annulation Nav2 controlee puis etat pause (sans perdre la mission).
- Nav2 indisponible: abort immediat.
- Planner no valid path found repete: compteur de seuil, annulation du goal et abort mission.
- Echecs consecutifs: sequence de degagement puis repli maison; si impossible, log fatal et abort.
- Nettoyage: le cycle ROS2 se ferme proprement via executor et shutdown.

### Points d'attention
- Fichier sensible et dense: combine action server, client Nav2, pont UI et capture camera dans le meme noeud.
- Plusieurs mecanismes anti-doublon (anti-echo UI) evitent les boucles avec rosbridge/web.
- Le watchdog de reprise auto en pause depend de l'activite cmd_vel et de temporisations parametrees.
- Le repli home depend des coordonnees chargees dans config_gps.json.

## cmd_vel_gate.py

### Role
Ce noeud est un garde-barriere de securite entre la teleoperation et les moteurs. Il laisse passer les commandes manuelles quand aucune mission n'est active (ou en pause), et les bloque pendant une mission active. Son but concret: eviter qu'une commande teleop parasite interfere avec la navigation automatique.

### Topics ROS2 publies et souscrits
- Publies:
  - /cmd_vel (geometry_msgs/msg/TwistStamped, robot -> base mobile)
- Souscrits:
  - /robot/cmd_vel (geometry_msgs/msg/Twist, teleop/UI -> gate)
  - /ui/start_mission (std_msgs/msg/String, UI -> gate)
  - /ui/cancel_mission (std_msgs/msg/String, UI -> gate)
  - /ui/mission_result (std_msgs/msg/String, systeme mission -> gate)

### Actions ROS2 servies ou appelees
- Aucune action servie.
- Aucune action appelee.

### Services ROS2 exposes ou appeles
- Aucun service expose.
- Aucun service appele.

### Logique principale
- Maintient un etat mission local (active/paused) a partir des topics UI.
- Si teleop autorisee: republie la commande en TwistStamped avec horodatage/frame_id.
- Si teleop interdite: bloque la commande; optionnellement envoie une commande nulle unique pour stopper proprement le mouvement.
- Interprete /ui/cancel_mission comme mini protocole de commande: cancel, pause, resume.

### Lien avec les autres packages
- navigation_pkg: suit l'etat diffuse par waypoint_action_server via topics UI.
- Nav2 / controle base: protege la sortie /cmd_vel pendant l'autonomie.
- Interface web: depend directement des messages UI mission pour ouvrir/fermer la barriere teleop.
- Simulation/stack mobile: /cmd_vel est ensuite bridge vers le plugin mobile (via ros_gz_bridge dans la stack globale).

### Comportement en cas d'erreur ou d'arret d'urgence
- Si commande UI inconnue: warning et pas de changement d'etat.
- Sur cancel mission: repasse immediatement en mode teleop autorise.
- En mode blocage, emission optionnelle d'un zero cmd pour limiter les mouvements residuels.
- Pas de mecanisme interne de timeout: depend de la validite des topics d'etat mission.

### Points d'attention
- Le noeud ne valide pas le contenu JSON des messages UI, seulement la commande texte cancel/pause/resume.
- Si les topics UI ne sont pas emis correctement, l'etat de gating peut devenir incoherent.
- La sortie est en TwistStamped (pas Twist): verifier la compatibilite des consommateurs aval.

## odom_rosbridge_relay.py

### Role
Ce noeud relie ROS2 au monde web via rosbridge WebSocket. Il envoie l'odometrie et les etats de mission vers le serveur rosbridge pour l'interface distante. Il gere lui-meme la connexion WebSocket (client minimal RFC6455), la reconnexion automatique et le controle du debit de publication.

### Topics ROS2 publies et souscrits
- Publies ROS2: aucun (ce noeud publie vers rosbridge, pas vers un topic ROS2 local).
- Souscrits ROS2:
  - /odometry/filtered (nav_msgs/msg/Odometry, robot -> relay)
  - /ui/mission_result (std_msgs/msg/String, robot -> relay)
  - /ui/mission_feedback (std_msgs/msg/String, robot -> relay)
- Cote rosbridge (WebSocket publish):
  - /odometry/filtered (nav_msgs/msg/Odometry)
  - /ui/mission_result (std_msgs/msg/String)
  - /ui/mission_feedback (std_msgs/msg/String)

### Actions ROS2 servies ou appelees
- Aucune action servie.
- Aucune action appelee.

### Services ROS2 exposes ou appeles
- Aucun service expose.
- Aucun service appele.

### Logique principale
- Etablit une connexion WebSocket vers rosbridge (ws://bridge_host:bridge_port).
- Advertise automatiquement les topics cibles puis publie les messages convertis en dictionnaires.
- Applique un rate limit configurable pour les flux continus (odometrie/feedback), mais laisse le resultat mission partir immediatement.
- Sur erreur d'envoi ou de socket: ferme la connexion et laisse le timer de reconnexion restaurer le lien.

### Lien avec les autres packages
- Interface web / rosbridge_server: passerelle principale des donnees robot vers UI distante.
- navigation_pkg: transporte vers le web les sorties de waypoint_action_server (feedback/result).
- gps_package/ekf/Nav2: l'odometrie /odometry/filtered provient de la chaine localisation/navigation.

### Comportement en cas d'erreur ou d'arret d'urgence
- Echec handshake/socket: warning + retry periodique.
- Echec d'envoi: fermeture WebSocket immediate puis reconnexion ulterieure.
- Shutdown: fermeture propre de la socket dans destroy_node.
- Pas de reception de commandes UI entrantes dans cette version (inbound vide).

### Points d'attention
- Le client WebSocket est implemente maison (sans librairie externe): robuste mais plus sensible aux evolutions protocole qu'une lib dediee.
- Parametres bridge_host/bridge_port critiques pour la liaison web; verifier la coherence avec global_launch.
- Le noeud est principalement outbound; si vous attendez des commandes web -> ROS2 via ce relay, il faut ajouter une voie inbound explicite.
