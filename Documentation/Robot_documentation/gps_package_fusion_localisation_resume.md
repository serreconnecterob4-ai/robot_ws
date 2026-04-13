# Resume du package gps_package - Fusion localisation

Ce document resume les fichiers suivants:
- launch/ekf_launch.py
- config/ekf_local.yaml
- config/ekf_global.yaml
- config/navsat.yaml
- README.md

## launch/ekf_launch.py

### Role
Ce fichier sert a lancer toute la chaine de fusion de localisation. Il demarre les deux filtres EKF (local et global), le noeud de transformation GPS et les TF statiques IMU/GPS. En pratique, c'est le point d'entree pour obtenir une odometrie finale stable sur le robot.

### Topics consommes
- Par les noeuds qu'il lance:
  - /imu/data
  - /odometry/wheel (via EKF local)
  - /gps/fix (via navsat_transform)
  - /odometry/local (via navsat_transform et EKF global)
  - /odometry/gps (via EKF global)

### Topics produits
- /odometry/local (sortie EKF local, remap de odometry/filtered)
- /odometry/gps (sortie navsat_transform)
- /gps/filtered (si publish_filtered_gps actif)
- /odometry/filtered (sortie EKF global, sortie finale a utiliser)
- TF:
  - base_link -> curt_mini/base_link/imu_sensor (statique)
  - base_link -> gps_link (statique)
  - odom -> base_link (EKF local)
  - map -> odom (EKF global)

### Parametres cles et effet
- TimerAction 3.0 s avant navsat_transform: evite de lancer la conversion GPS avant initialisation des EKF/TF.
- Remapping EKF local: odometry/filtered -> /odometry/local pour separer proprement filtre local et global.
- Chargement de 3 YAML dedies: local, global, navsat.

### Difference local vs global (niveau lanceur)
- Local: rapide, basee odometrie+IMU court terme, dans le repere odom.
- Global: corrigee GPS, plus stable long terme, dans le repere map.

### Points d'attention
- Les frames des TF statiques doivent correspondre exactement aux frames publiees par les capteurs/simulation.
- Le topic odom roues doit exister sur /odometry/wheel (sinon EKF local n'alimente rien).
- L'ordre de demarrage est critique pour eviter une mauvaise initialisation de navsat_transform.

## config/ekf_local.yaml

### Role
Ce fichier sert a regler le filtre local, celui qui donne une estimation reactive du mouvement du robot. Il privilegie la fluidite court terme pour le pilotage, sans chercher la correction absolue GPS. C'est la base dynamique qui alimente ensuite la fusion globale.

### Topics consommes
- /imu/data
- /odometry/wheel

### Topics produits
- odometry/filtered (dans ce package, remappe en /odometry/local par ekf_launch.py)
- TF odom -> base_link (publish_tf: true)

### Parametres cles et effet
- frequency: 20.0
  - Filtre assez reactif pour le court terme.
- sensor_timeout: 0.5
  - Si un capteur se tait plus de 0.5 s, le filtre degrade cette source.
- two_d_mode: true
  - Ignore la dynamique 3D, adapte a robot mobile plan.
- world_frame/odom_frame/base_link_frame: odom/odom/base_link
  - Le filtre local vit dans le repere odom.
- imu0_config: tout a false
  - IMU desactivee dans cette config locale actuelle.
- odom0_config:
  - active yaw, vx, vy, yaw_rate depuis odometrie roues.
  - desactive position absolue (x,y): evite de traiter une pose roues comme verite absolue.

### Difference local vs global
- Le local suit vite les mouvements mais peut deriver.
- Il est pense comme entree du global, pas comme sortie finale de navigation long terme.

### Points d'attention
- IMU desactivee localement: si l'odom roues degrade, le local peut devenir fragile en rotation.
- Le choix vy=true suppose une cinematique compatible (a verifier pour robot non holonome).

## config/ekf_global.yaml

### Role
Ce fichier sert a regler le filtre global, qui corrige la derive avec le GPS. Il combine odometrie locale, odometrie GPS transformee et IMU pour produire la pose finale stable. C'est la sortie de reference pour la navigation mission.

### Topics consommes
- /odometry/local
- /odometry/gps
- /imu/data

### Topics produits
- /odometry/filtered (sortie finale recommandee)
- TF map -> odom (publish_tf: true)

### Parametres cles et effet
- frequency: 15.0
  - Adaptee a un GPS plus lent.
- sensor_timeout: 3.0
  - Plus tolerant aux trous GPS.
- world_frame/map_frame/odom_frame/base_link_frame: map/map/odom/base_link
  - Definit la chaine globale map -> odom -> base_link.
- odom0_config (/odometry/local): position XY + vitesses XY, yaw desactive
  - Evite de recycler la derive d'orientation du local.
- odom1_config (/odometry/gps): correction absolue XY active
  - Le GPS corrige la derive de position.
- imu0_config: yaw + yaw_rate actifs
  - L'orientation globale vient surtout de l'IMU, independamment du local.
- odom1_differential: false
  - Utilisation absolue du GPS (normal pour correction globale).

### Difference local vs global
- Local: continuité/reactivite du mouvement.
- Global: stabilite absolue (corrigee GPS) et reference map.

### Points d'attention
- Si /odometry/gps est absent ou bruit fort, la sortie globale devient instable ou peu corrigee.
- La coherence IMU (yaw) est determinante pour une trajectoire propre.

## config/navsat.yaml

### Role
Ce fichier sert a convertir les donnees GPS (lat/lon) en odometrie ROS exploitable en metres. Il aligne cette conversion avec l'IMU et l'odometrie locale pour rester coherent avec les frames robot. En clair, il transforme le GPS brut en /odometry/gps utilisable par l'EKF global.

### Topics consommes
- /gps/fix
- /imu/data
- /odometry/local

### Topics produits
- /odometry/gps
- /gps/filtered (si active)
- (pas de transform UTM car broadcast_utm_transform: false)

### Parametres cles et effet
- frequency: 10.0, sensor_timeout: 1.5
  - Cadence moyenne avec tolerance a pertes courtes.
- wait_for_datum: true
  - Le noeud attend un datum explicite (ici fourni dans le YAML).
- datum: [48.7994, 2.0281, 0.0]
  - Origine geographique fixe du repere local.
- magnetic_declination_radians: 0.0, yaw_offset: 0.0
  - Alignement cap geographique/simulation sans correction supplementaire.
- use_odometry_yaw: false
  - Orientation prise de l'IMU, pas du yaw odometrie.
- zero_altitude: true
  - Navigation 2D (altitude forcee a zero).

### Difference local vs global (impact navsat)
- navsat alimente le global avec une mesure absolue XY.
- le local sert de reference de mouvement mais ne remplace pas la correction GPS.

### Points d'attention
- Le datum doit correspondre exactement au monde/simulation (sinon decalages importants).
- Qualite GPS faible => /odometry/gps bruitee => corrections agressives possibles.

## README.md

### Role
Ce fichier sert a expliquer l'architecture de fusion, le raisonnement local/global et les procedures de debug. Il donne aussi des strategies de tuning (covariances, frequences, bruit capteurs) selon le contexte. C'est la reference operationnelle pour diagnostiquer les erreurs de localisation.

### Topics consommes / produits mis en avant
- Entrees de capteurs:
  - /wheel/odom (ou odom roues selon pipeline)
  - /imu/data
  - /gps/fix
- Sorties:
  - /odometry/local
  - /odometry/gps
  - /odometry/filtered (sortie recommandee)

### Parametres cles et effet (points pedagogiques du README)
- Covariances capteurs:
  - petite covariance = forte confiance,
  - grande covariance = faible confiance.
- Equilibre GPS vs odom:
  - GPS trop dominant peut provoquer des sauts,
  - GPS trop faible laisse deriver la pose.
- Frequence EKF:
  - trop faible = retard,
  - trop haute avec capteurs lents/bruites = instabilite possible.

### Difference local vs global (explication simple)
- Filtre local:
  - rapide, fluide, ideal pour reaction instantanee,
  - mais derive dans le temps.
- Filtre global:
  - corrige la derive grace au GPS,
  - sert de reference stable pour navigation.

### Problemes frequents mentionnes dans le README et solutions
- Datum GPS qui se reinitialise / robot qui saute:
  - verifier la config datum/wait_for_datum de navsat.
- /odometry/filtered qui derive:
  - augmenter la confiance GPS ou reduire la confiance odometrie.
- Filtre instable (oscillations):
  - reduire bruit capteurs, reequilibrer covariances, ajuster frequences.
- Erreur latitude invalide:
  - verifier bornes lat/lon du GPS simule/reel.
- Pas de TF map -> odom:
  - verifier EKF global actif, frames configurees et flux /odometry/gps present.

### Points d'attention
- Le README contient des exemples qui peuvent diverger de la config actuelle (ex: certaines frequences ou wait_for_datum).
- Toujours privilegier les YAML/launch reels comme source de verite runtime.
- Calibration IMU, qualite GPS et alignement des frames sont les trois causes majeures de mauvais comportement.

## Synthese local vs global (en une phrase)

Le filtre local donne une estimation rapide du mouvement instantane, tandis que le filtre global corrige cette estimation avec le GPS pour fournir une position fiable sur la duree via /odometry/filtered.
