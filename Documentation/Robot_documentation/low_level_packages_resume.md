# Resume des packages bas niveau

Packages couverts:
- openzenros2
- candle_ros2
- curt_mini
- ipa_ros2_control

## 1) openzenros2 (package ROS: openzen_driver)

### Fichiers principaux
- robot_ws/src/openzenros2/src/OpenZenNode.cpp
- robot_ws/src/openzenros2/launch/openzen_lpms.launch.py
- robot_ws/src/openzenros2/README.md
- robot_ws/src/openzenros2/package.xml
- robot_ws/src/openzenros2/CMakeLists.txt

### Role du package
Ce package pilote physiquement le capteur inertiel LP-Research (IMU, et GNSS si present) via la librairie OpenZen. Il transforme les donnees brutes capteur en messages ROS2 standard pour la localisation et la navigation. Il expose aussi des commandes de maintenance IMU (autocalibration, reset cap, calibration gyro).

### Topics publies et souscrits
- Publies:
  - data (sensor_msgs/msg/Imu)
  - mag (sensor_msgs/msg/MagneticField)
  - nav (sensor_msgs/msg/NavSatFix) si composant GNSS detecte
  - is_autocalibration_active (std_msgs/msg/Bool)
- Souscrits:
  - Aucun topic utilisateur (lecture capteur via OpenZen, pas via subscription ROS2)

### Services exposes
- enable_gyro_autocalibration (std_srvs/srv/SetBool)
  - Quand l appeler: activer/desactiver l autocalibration gyro selon votre strategie (stabilite long terme vs reactivite).
- reset_heading (std_srvs/srv/Trigger)
  - Quand l appeler: re-aligner le cap (heading) a un instant choisi (robot immobile/oriente).
- calibrate_gyroscope (std_srvs/srv/Trigger)
  - Quand l appeler: au demarrage ou apres choc/derive; le code demande capteur stationnaire ~4s.

### Parametres de configuration importants
- sensor_name (string)
  - Vide: autodiscovery premier capteur; renseigne: connexion directe.
- sensor_interface (string, defaut LinuxDevice)
  - Type d I/O OpenZen.
- baudrate (int, defaut 0)
  - 0 = baudrate par defaut capteur; utile si capteur configure autrement.
- configure_gnss_output (bool)
  - Active les proprietes GNSS utiles (fix type, lon/lat, accuracies...).
- frame_id (defaut imu), frame_id_gnss (defaut gnss)
  - Frames ROS des messages IMU/GNSS.
- openzen_verbose (bool)
  - Active logs OpenZen debug.

### Dependances materielles
- IMU LP-Research compatible OpenZen (ex: LPMS series / IG1)
- Acces port serie USB (droits utilisateur sur /dev/ttyUSB*)
- GNSS LP-Research si vous voulez /nav

### Commandes de calibration / initialisation avant usage
- Donner les droits serie (README): utilisateur dans groupe dialout.
- Option recommandee au demarrage:
  1. lancer le node,
  2. verifier flux IMU,
  3. calibrate_gyroscope robot immobile,
  4. reset_heading si vous devez definir un cap de reference.

### Points d attention
- Sans droits serie, le node ne connecte pas le capteur.
- Si sensor_name est vide, il prend le premier capteur trouve (attention en cas de plusieurs capteurs branchés).
- Convention IMU en ENU; verifier coherence avec le reste de la stack.
- GNSS est optionnel: absence de composant GNSS = pas de topic nav.

## 2) candle_ros2

### Fichiers principaux
- robot_ws/src/candle_ros2/src/md80_node.cpp
- robot_ws/src/candle_ros2/include/md80_node.hpp
- robot_ws/src/candle_ros2/msg/MotionCommand.msg
- robot_ws/src/candle_ros2/msg/ImpedanceCommand.msg
- robot_ws/src/candle_ros2/msg/VelocityPidCommand.msg
- robot_ws/src/candle_ros2/msg/PositionPidCommand.msg
- robot_ws/src/candle_ros2/srv/AddMd80s.srv
- robot_ws/src/candle_ros2/srv/GenericMd80Msg.srv
- robot_ws/src/candle_ros2/srv/SetModeMd80s.srv
- robot_ws/src/candle_ros2/srv/SetLimitsMd80.srv
- robot_ws/src/candle_ros2/README.md

### Role du package
Ce package commande physiquement les drives MD80 sur bus CAN via l interface CANdle. Il gere l ajout des drives, leur mode de controle (impedance, PID vitesse/position...), l activation/desactivation et l envoi de consignes temps reel. Il publie aussi l etat articulaire (position/vitesse/couple) pour le reste du robot.

### Topics publies et souscrits
- Publies:
  - md80/joint_states (sensor_msgs/msg/JointState)
- Souscrits:
  - md80/motion_command (candle_ros2/msg/MotionCommand)
  - md80/impedance_command (candle_ros2/msg/ImpedanceCommand)
  - md80/velocity_pid_command (candle_ros2/msg/VelocityPidCommand)
  - md80/position_pid_command (candle_ros2/msg/PositionPidCommand)

### Services exposes
Services sont prefixes par le nom du node candle_ros2_node:
- candle_ros2_node/add_md80s (candle_ros2/srv/AddMd80s)
  - Quand l appeler: au debut, pour declarer les IDs drives accessibles.
- candle_ros2_node/zero_md80s (candle_ros2/srv/GenericMd80Msg)
  - Quand l appeler: apres ajout et avant mouvement, pour zero encodeurs.
- candle_ros2_node/set_mode_md80s (candle_ros2/srv/SetModeMd80s)
  - Quand l appeler: avant envoi consignes, pour definir le mode de chaque drive.
- candle_ros2_node/enable_md80s (candle_ros2/srv/GenericMd80Msg)
  - Quand l appeler: une fois mode/config ok, pour activer communication/motors.
- candle_ros2_node/disable_md80s (candle_ros2/srv/GenericMd80Msg)
  - Quand l appeler: arret securise systeme.

### Parametres de configuration importants
- Arguments process (pas des params ROS):
  - bus: SPI | USB | UART
  - baud CAN: 1M | 2M | 5M | 8M
  - device optionnel (SPI/UART)
- Publication joint states: timer 10 ms (~100 Hz).
- Modes supportes par service set_mode: IMPEDANCE, POSITION_PID, VELOCITY_PID, RAW_TORQUE.

### Dependances materielles
- Interface CANdle MAB
- Drives MD80 sur bus CAN
- Cablage/alimentation moteurs conforme
- Liaison USB/SPI/UART selon bus choisi

### Commandes de calibration / initialisation avant usage
- Sequence recommandee:
  1. add_md80s
  2. set_mode_md80s
  3. zero_md80s
  4. enable_md80s
  5. publier commandes motion/PID
- Pour configuration avancee firmware/param drives: utiliser MDtool (README), pas ce node.

### Points d attention
- Le node detecte IDs dupliques et avertit.
- Les tailles de tableaux commandes doivent matcher les drive_ids, sinon message ignore.
- Le service SetLimitsMd80 existe dans les interfaces mais n est pas implemente dans le .cpp actuel.
- Sur disable, la communication CANdle est stoppee avant desactivation drives.

## 3) curt_mini

### Fichiers principaux
- robot_ws/src/curt_mini/curt_mini/bringup/robot_base.launch.py
- robot_ws/src/curt_mini/curt_mini/bringup/start_controller.launch.py
- robot_ws/src/curt_mini/curt_mini/bringup/joystick.launch.py
- robot_ws/src/curt_mini/curt_mini/config/ros2_control.yaml
- robot_ws/src/curt_mini/curt_mini/config/ros2_control_simulation.yaml
- robot_ws/src/curt_mini/curt_mini/config/twist_mux.yaml
- robot_ws/src/curt_mini/curt_mini/config/joystick.yaml
- robot_ws/src/curt_mini/curt_mini/arm_controller.py
- robot_ws/src/curt_mini/curt_mini/package.xml

### Role du package
Ce package est le bringup mecanique de la plateforme Curt Mini: il assemble les launches, la description robot, la teleoperation, le multiplexage cmd_vel et le demarrage des controleurs. Il ne fait pas le controle moteur bas niveau lui-meme, mais orchestre les briques qui le font. Il contient aussi un pont serie vers Arduino pour l actionneur de bras.

### Topics publies et souscrits
Dans le code direct du package (arm_controller.py):
- Souscrits:
  - /robot/arm_speed (std_msgs/msg/Float32)
  - /robot/arm_position (std_msgs/msg/Float32)
- Publies:
  - Aucun topic ROS (envoi serie vers Arduino)

Via les noeuds lances (fichiers bringup/config):
- joystick.launch.py lance joy_linux + teleop_twist_joy (topics joy + joy_teleop/cmd_vel)
- twist_mux.yaml configure entree navigation, joystick et zero_twist puis sortie /cmd_vel
- start_controller.launch.py remappe diff_drive_controller/odom -> /odometry/wheel

### Services exposes
- Aucun service implemente directement dans curt_mini.
- Services utilises indirectement via controller_manager spawner dans start_controller.launch.py.

### Parametres de configuration importants
- ros2_control.yaml:
  - controller_manager update_rate: 100
  - diff_drive wheel_separation: 0.44, wheel_radius: 0.13
  - cmd_vel_timeout: 0.25
  - limites vitesse/acceleration lineaire et angulaire
- twist_mux.yaml:
  - priorites commandes: joystick > navigation > zero_twist
  - timeout 0.1s par source
- joystick.yaml:
  - mapping axes/boutons Logitech F710
  - deadzone et autorepeat
- arm_controller.py:
  - scan ports /dev/ttyACM* /dev/ttyUSB*
  - baudrate serie 115200
  - seuil anti-spam d envoi consignes

### Dependances materielles
- Base mecanique Curt Mini (4 roues/moteurs)
- Manette joystick (config F710)
- Arduino pour bras (serie USB)
- IMU LP-Research quand activee dans launch

### Commandes de calibration / initialisation avant usage
- En pratique:
  1. demarrer ros2_control + drivers moteurs,
  2. spawner joint_state_broadcaster puis diff_drive_controller,
  3. valider joystick/twist_mux,
  4. pour bras: verifier detection port serie Arduino.

### Points d attention
- Dans robot_base.launch.py actuel, plusieurs briques critiques sont commentees (hardware_interface, joystick, twist_mux, controller): le launch tel quel ne demarre pas tout le robot.
- zero_twist peut prendre la main selon priorites/timeout si mal configure.
- arm_controller depend du protocole serie Arduino specifique (commande P...,V...).

## 4) ipa_ros2_control

### Fichiers principaux
- robot_ws/src/curt_mini/ipa_ros2_control/src/curt_mini_hardware_interface.cpp
- robot_ws/src/curt_mini/ipa_ros2_control/include/ipa_ros2_control/curt_mini_hardware_interface.hpp
- robot_ws/src/curt_mini/ipa_ros2_control/launch/ros2_control.launch.py
- robot_ws/src/curt_mini/ipa_ros2_control/curt_mini_driver.xml
- robot_ws/src/curt_mini/ipa_ros2_control/package.xml

### Role du package
Ce package est la couche ros2_control hardware du robot. Il relie les commandes vitesse des controleurs ROS2 aux drives MD80 via candle_ros2, et remonte les etats roues vers les interfaces ros2_control. Concretement, c est le pont entre la logique controleur et le materiel moteur.

### Topics publies et souscrits
Dans CurtMiniHardwareInterface:
- Souscrits:
  - /md80/joint_states (sensor_msgs/msg/JointState)
- Publies:
  - /md80/motion_command (candle_ros2/msg/MotionCommand)
  - /md80/velocity_pid_command (candle_ros2/msg/VelocityPidCommand)

Et via ros2_control.launch.py:
- Lance controller_manager ros2_control_node
- Lance candle_ros2_node (arguments USB 1M)

### Services exposes
- Le package n expose pas de service public propre.
- Il appelle les services candle_ros2_node suivants pendant activation/desactivation:
  - candle_ros2_node/add_md80s (AddMd80s)
  - candle_ros2_node/set_mode_md80s (SetModeMd80s)
  - candle_ros2_node/zero_md80s (GenericMd80Msg)
  - candle_ros2_node/enable_md80s (GenericMd80Msg)
  - candle_ros2_node/disable_md80s (GenericMd80Msg)

### Parametres de configuration importants
- Parametres runtime declares dans le hardware interface:
  - pid_config.kp (defaut 8.0)
  - pid_config.ki (1.0)
  - pid_config.kd (0.0)
  - pid_config.i_windup (6.0)
  - pid_config.max_output (18.0)
  - standstill_thresh (0.01)
- IDs MD80 codes en dur dans la logique:
  - ordre service souvent {102, 100, 103, 101}
  - ordre commandes souvent {101, 100, 103, 102}
- Correction cinematique:
  - inversion signe cote droit pour vitesses moteurs.

### Dependances materielles
- Drives MD80 et interface CANdle operationnels
- Robot configure en ros2_control diff drive (4 joints moteurs)
- Bus CAN stable (ici preset USB, 1M dans ros2_control.launch.py)

### Commandes de calibration / initialisation avant usage
- Sequence automatisee dans on_activate:
  1. attendre services candle,
  2. add_md80s,
  3. set_mode_md80s en VELOCITY_PID,
  4. zero_md80s,
  5. enable_md80s,
  6. publier PID et commande zero.
- A l arret (on_deactivate): envoi zero puis disable_md80s.

### Points d attention
- IDs moteurs sont hardcodes: verifier correspondance physique exacte des 4 roues.
- Si un service candle echoue pour un seul moteur, activation echoue.
- Le package coupe le couple a l arret via reduction max_output a zero quand standstill detecte.
- Il depend fortement de la bonne disponibilite de candle_ros2_node au demarrage.
