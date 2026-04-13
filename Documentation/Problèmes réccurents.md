## Flux vidéo inexistant

0. Vérifiez que le caméra launch est lancé

0.1 Vérifier que la caméra est bien allumée (lumière bleue sur sa tête)

1. Vérifier sur l'ordinateur robot que le cable ethernet de la caméra est branché

1.1 Vérifier que sur l'ordinateur robot, la connexion filaire est activée, avec la bonne configuration (voir configuration générale)

2. Vérifier que les 2 ordinateurs sont connectés à internet et reliés par le réseau tailscale (voir console tailscale en ligne).

3. Essayer --> Ctrl + shift + r pour recharger le site et son cache.

## Flux vidéo avec Latence

0. Le flux vidéo peut être temporairement ralenti / interrompu si l'ordinateur de bord du robot est surchargé (exemple :  démarrage du système / nav2 ou surchauffe).

1. Votre connexion peut simplement être mauvaise, essayez de vous rapprocher du robot pour améliorer la connexion wifi.

2. Si cela ne suffit pas, re-configurer les paramètres du flux vidéo pour le rendre moins lourd et donc réduire la latence (balance qualité/latence). VOIR Configuration générale/ Réglages caméra. 

## Sur Rviz, la costmap n'apparait pas

0. Problème dans le lancement des différentes parties du programme (voir global_launch.py), les timers sont la pour que les parties du programmes / différents programmes aient le temps de charger complètement puisque certains codes sont dépendants du lancement d'autres en amont. Si des erreurs comme celle ci apparaissent, alors c'est que le lancement a été trop "rapide" et les codes ont étés trop rapidement lancés, tester de lancer les differentes parties avec plus de délai d'écart. problème dépendant de l'ordinateur utilisé.

## Sur Rviz, le robot apparait buggé : sans textures ou roues découplées de lui.

0. Problème dans le lancement des différentes parties du programme (voir global_launch.py), les timers sont la pour que les parties du programmes / différents programmes aient le temps de charger complètement puisque certains codes sont dépendants du lancement d'autres en amont. Si des erreurs comme celle ci apparaissent, alors c'est que le lancement a été trop "rapide" et les codes ont étés trop rapidement lancés, tester de lancer les differentes parties avec plus de délai d'écart. problème dépendant de l'ordinateur utilisé.

## Robot s'arrête en pleine trajectoire

0. Est il proche d'un bord ? afficher la costmap sur Rviz peut aider à savoir si il a percuté un bord "dur" de la costmap. Si c'est le cas, alors il s'arrête en prévention pour éviter d'abimer le robot plus.

## Robot incontrolable depuis le site web bien que affiché.

--> Ctrl + shift + r pour recharger le site et son cache.


