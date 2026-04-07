
## Configuration simulation Gazebo pure

Commande de lancement:

```bash
ros2 launch curt_mini simulation.xml
```

Cette commande lance Gazebo, spawn le robot, charge les controllers, le bridge ROS<->Gazebo, RViz et la navigation.

## Fichiers a modifier

- `robot_ws/src/curt_mini/curt_mini/bringup/simulation.xml`
- `robot_ws/src/curt_mini/curt_mini/config/ros2_control_simulation.yaml`
- `robot_ws/src/curt_mini/curt_mini/config/gz_bridge.yaml`
- `robot_ws/src/curt_mini/curt_mini/models/curt_mini/worlds/world.sdf`
- `robot_ws/src/curt_mini/curt_mini/models/map_plane/model.sdf`

## 1) Parametres de lancement Gazebo

Fichier: `bringup/simulation.xml`

- Monde charge par Gazebo (argument `gz_args`):
    - Chemin du monde: `.../models/curt_mini/worlds/world.sdf`
    - `-r`: lance la simulation directement (run)
    - `-v 1`: niveau de verbosite
    - `-s`: mode serveur
- Ressources de modeles (`GZ_SIM_RESOURCE_PATH`):
    - Permet a Gazebo de resoudre `model://...`

## 2) Position initiale du robot (spawn)

Fichier: `bringup/simulation.xml` (node `ros_gz_sim create`)

- `-x`: position X initiale
- `-y`: position Y initiale
- `-z`: hauteur initiale
- `-yaw`: orientation initiale
- `-name`: nom du robot dans Gazebo

Valeurs actuelles:

```bash
-x 3.849 -y 13.486 -z 0.65 -yaw 0.0 -name curt_mini
```

## 3) Delais de demarrage (timers)

Fichier: `bringup/simulation.xml`

- Navigation: `period="6.0"`
- Spawn robot: `period="2.0"`
- `joint_state_broadcaster`: `period="8.0"`
- `diff_drive_controller`: `period="10.0"`
- RViz: `period="10.0"`

Si un module demarre trop tot (topics absents, TF manquantes), augmenter son timer.

## 4) Physique et environnement Gazebo

Fichier: `models/curt_mini/worlds/world.sdf`

- Physique:
    - `max_step_size`
    - `real_time_factor`
    - `real_time_update_rate`
- Gravite: `gravity`
- Champ magnetique: `magnetic_field`
- Lumiere et scene:
    - `light` (soleil)
    - `scene/ambient`
    - `scene/background`
    - `scene/shadows`
- Coordonnees geographiques (important pour GPS simule):
    - `latitude_deg`
    - `longitude_deg`
    - `elevation`
    - `heading_deg`

## 5) Carte au sol (map_plane)

Fichiers:

- Position/orientation de la carte dans le monde: `models/curt_mini/worlds/world.sdf`
- Taille/friction de la carte: `models/map_plane/model.sdf`

Parametres modifiables:

- Pose de la carte (`<include><pose> x y z roll pitch yaw </pose>`)
- Taille du plan (`<size> largeur hauteur </size>`) en metres
- Friction du sol (`mu`, `mu2`)
- Texture de la carte (`albedo_map`)

Attention: garder la meme taille pour collision et visual dans `model.sdf`.

## 6) Bridge ROS <-> Gazebo

Fichier: `config/gz_bridge.yaml`

Tu peux configurer:

- Les topics bridges
- Le sens du bridge (`GZ_TO_ROS` / `ROS_TO_GZ`)
- Les types de messages ROS et Gazebo

Topics deja relies:

- `clock`
- `odometry/wheel`
- `imu/data`
- `scan`
- `/ouster/points`
- `/gps/fix`

## 7) Parametres de controle du robot en simulation

Fichier: `config/ros2_control_simulation.yaml`

### Cinematique et odometrie

- `wheel_separation`
- `wheel_radius`
- `wheels_per_side`
- `odom_frame_id`
- `base_frame_id`
- `open_loop`
- `enable_odom_tf`
- `cmd_vel_timeout`

### Limites de vitesse et acceleration

```yaml
linear.x.has_velocity_limits: true
linear.x.min_velocity: -1.5
linear.x.max_velocity: 1.5
linear.x.has_acceleration_limits: true
linear.x.min_acceleration: -2.5
linear.x.max_acceleration: 2.5

angular.z.has_velocity_limits: true
angular.z.min_velocity: -12.0
angular.z.max_velocity: 12.0
angular.z.has_acceleration_limits: true
angular.z.min_acceleration: -15.0
angular.z.max_acceleration: 15.0
```

## Resume rapide

Pour une simulation Gazebo pure, les reglages les plus utilises sont:

1. Pose initiale du robot (`simulation.xml`)
2. Monde + physique (`world.sdf`)
3. Carte sol (pose + taille, `world.sdf` et `map_plane/model.sdf`, voir configuration générale.md)
4. Limites de vitesse (`ros2_control_simulation.yaml`)
5. Topics bridges (`gz_bridge.yaml`)