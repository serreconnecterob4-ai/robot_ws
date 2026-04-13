# Configuration générale

Certains de nos codes nécessitent une configuration lorsque vous les utiliserez en pratique, pour les adapter à votre propre situation. La quasi totalité du temps, cette configuration est à faire une fois pour toute. Voici les éléments de configuration que vous devez connaître.

## Position base gps fixe (coordonnées GPS)

**Représente la position de votre base GPS fixe (le module "BASE" du kit u-blox RTK).**

Unité : Coordonnées (latitude, longitude)

Valeur actuelle : `(48.80393950° , 2.07576810°)`

> Cette position va être utilisée comme point d'origine dans le repère Gazebo (point 0,0). Elle servira aussi de point d'origine à associer sur les maps.jpg & .pgm.

Code où changer la valeur : 
robot_ws/src/gps_package/config/navsat.yaml ---- Lignes 22

````yaml
datum: [48.7994, 2.0281, 0.0]
````

robot_ws/src/curt_mini/curt_mini/models/curt_mini/worlds/world.sdf ---- Lignes 4 et 5
````sdf
      <latitude_deg>48.7994</latitude_deg>
      <longitude_deg>2.0281</longitude_deg>
````

## Réglages vis à vis des cartes .jpg & .pgm

### Position base gps fixe (pixels) + Résolution

**Représente la position de votre base GPS fixe (le module "BASE" du kit u-blox RTK) sur les maps.jpg & .pgm.**

Unité : Coordonnées (x, y) en pixels

On considère que l'origine (0,0) est en haut à gauche de l'image, (que x augmente vers la droite et que y augmente vers le bas).

Valeur actuelle : `(x= 26.6833333333 , y= 184.133333333)` (pour map 1661x437)
        
robot_ws/src/curt_mini/curt_mini/maps/image_rotate.py ---- Lignes 5-7
````py
px_original = 26.6833333333
py_original = 184.133333333
origin_width = 9966
````

**Représente la résolution de la carte, c'est à dire le nombre de pixels correspondant à une distance réelle.**

Unité : mètre/pixels

> La résolution x est la même que la résolution y.

Pour st_cyr_vu_du_ciel_large_map.jpg (src/Documentation/Illustrations) 
    Valeur actuelle : `(x= 160.1 , y= 1104.8)`
    Valeur actuelle : `0.026617 m/px`
> cette carte est juste utile pour trouver le pixel PRÉCIS sur une image à grande résolution.
Elle fait 9966x2622 pixels.

Code où changer la valeur : 

client_ws/src/web_control/web/js/05-trajectoires.js ---- Lignes 6-8
````js
const originPixel = { x: 160.1, y: 1104.8 };
const metersPerPixel = 2.6617 / 100;
const origin_map_size = { width: 9966, height: 2622 };
````
client_ws/src/web_control/web/navig/navig.js ---- Lignes 647 - 650
````js
const originPixel = { x: 160.1, y: 1104.8 };
const metersPerPixel = 2.6617 / 100;
const origin_map_size = { width: 9966, height: 2622 };
````


Pour map.jpg et map.pgm (les maps utilisés pour les calculs et l'affichage gazebo x Rviz)
> robot_ws/src/navigation_pkg/maps/
> robot_ws/src/curt_mini/curt_mini/maps/
> client_ws/src/web_control/web/navig/ (map_costmap.jpg)
> robot_ws/src/curt_mini/curt_mini/models/curt_mini/map_plane/media/materials/textures
> robot_ws/src/navigation_pkg/maps/
> robot_ws/src/navigation_pkg/models/map_plane/media/materials/textures

Valeur actuelle : `taille de l'image : (1661 x 437 pixels)`
Valeur actuelle : `0.159702 m/px` (proportionnel à la carte précédente : ratio 9966/1661 = 0.026617/0.159702 = 6.0)

Code où changer la valeur :
client_ws/src/web_control/web/js/05-trajectoires.js ---- Lignes 9
````js
const map_size = { width: 1661, height: 437 };
````

robot_ws/src/curt_mini/curt_mini/maps/image_rotate.py ---- Ligne 8
````py
mini_width = 1661
````

robot_ws/src/curt_mini/curt_mini/maps/map.yaml ---- Lignes 2 et 3
````yaml
resolution: 0.159702
origin: [-29.5257857707, -13.3704269525, 0] # Rviz compte en mètres, et non en pixels, donc on convertit les coordonnées de l'origine en mètres
````
robot_ws/src/curt_mini/curt_mini/models/map_plane/model.sdf ---- Lignes 17,37
````xml
<size>265.265022 69.789774</size> <!-- resolution*taille(pixels)-->
<size>265.265022 69.789774</size>
````

robot_ws/src/navigation_pkg/models/map_plane/model.sdf ---- Lignes 17,37
````xml
<size>265.265022 69.789774</size> <!-- resolution*taille(pixels)-->
<size>265.265022 69.789774</size>
````


Code où changer la valeur : 

### Orientation de la carte

**Représente l'orientation de la carte, c'est à dire l'angle de rotation de la carte par rapport à la direction nord.**

Unité : degré

Valeur actuelle : `76.681°`

> Un angle de 0° signifie que la carte est orientée vers le nord.
>Important : on prend le point "origine" (voir plus haut) comme pivot de rotation.

Code où changer la valeur : 

client_ws/src/web_control/web/js/05-trajectoires.js ---- Lignes 11
````js
const thetaDegrees = 76.681;
````

src/client_ws/src/web_control/web/navig/navig.js ---- Lignes 647 - 650
````js
const thetaDegrees = 76.681;
````

src/robot_ws/src/curt_mini/curt_mini/maps/image_rotate.py ---- Lignes 4
````py
angle = 76.681
````




RESTE A COMPRENDRE :
* robot_ws/src/curt_mini/curt_mini/models/curt_mini/worlds/worlds.sdf ---- Lignes 73
````xml
<pose>34.9136417796 123.653828903 0.001 0 0 1.3383</pose>

````
* robot_ws/src/curt_mini/curt_mini/maps/map.yaml ---- Lignes 4
````yaml
origin: [-29.5257857707, -13.3704269525, 0] #38.3154525596, -13.4510749993, 1.3383 # -4.261, -29.406
````








## Réglages réseaux

### Adresse IP Caméra



## Adresses IP des 2 machines (robot & client)

### Adresse IP du robot

client_ws/src/web_control/web/js/01-core-ros.js ---- Lignes 5-6
````js
const robotIp = "100.113.93.106";
const videoHost = serverIp === "" ? "100.113.93.106" : serverIp;
````
client_ws/src/web_control/web/configuration.yaml 
````yaml
"host": "100.113.93.106",
````

client_ws/src/web_control/web_control/backend_node.py ---- Lignes 100
````py
self.declare_parameter('robot_gallery_host', '100.106.79.105')
````

### Adresse IP du client

robot_ws/src/navigation_pkg/launch/global_launch.py ---- Lignes 56
````py
'bridge_host:=100.123.147.56',
````

robot_ws/src/navigation_pkg/navigation_pkg/odom_rosbridge_relay.py ---- Lignes 146
````py
        self.declare_parameter('bridge_host', '100.92.193.85')
````

## Réglages caméra

Récuperez l'ip de la caméra (par défaut et en permanence: 10.42.0.188), veuillez bien vérifier que la caméra est branchée au robot en ethernet, et que le mode de configuration filaire est en "partagée avec d'autres ordinateurs".

Connectez-y vous dans un navigateur web :

http://10.42.0.188/

Entrez les identifiants (voir DOCUMENT D'AUTHENTIFICATION sur le drive Google).

Vous pouvez accéder dans les paramètres, aux réglages de la caméra, notamment à la résolution d'image, au taux de rafraîchissement, etc. Voir section > Stream.

Il peut être intéréssant de modifier :
* La résolution d'image (3 valeurs possibles)
* Le framerate
* Le débit
* Le format de compression (H264, OU h265)
* Le i-frame interval (nombre d'images entre 2 images clés, plus il est grand, plus la compression est efficace, mais plus la latence est grande)

pour trouver un équilibre entre qualité et latence selon votre situation.

## Emplacement des dossiers galerie et trajectoires

### Dossier galerie (robot)

robot_ws/src/navigation_pkg/navigation_pkg/waypoint_action_server.py ---- Lignes 193

````py
gallery_path = os.path.expanduser('~/mission_gallery')
````

### Dossier galerie + trajectoires (client) 

client_ws/src/web_control/backend_node.py ---- Lignes 132 - 133

````py
home_dir = os.path.expanduser('~')
self.gallery_dir = os.path.join(home_dir, 'robot_gallery')
self.trajectories_dir = os.path.join(home_dir, 'trajectories')
````

