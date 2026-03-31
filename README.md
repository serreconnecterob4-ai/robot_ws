TODO : Ce fichier sera à transformer en PDF

# 0 - Introduction

Ce projet a pour objectif :

Deux parties composent ce projet :
- **client_ws** : contient le code source du client web, c'est à dire le site web qui affiche le flux vidéo de la caméra et les données de télémétrie du robot, à lancer sur un navigateur web, programme indépendant.
- **robot_ws** : contient le code source à lancer sur le robot, c'est à dire toute la structure logique du robot : caméra, navigation, motorisation, connexions avec l'interface web, etc.


# 1 - Configuration

## 1.1 Prérequis

## 1.2 Configuration récurrente (pouvant changer d'un lancement à l'autre)

### 1.2.a - IP De la caméra :

L'IP même de la caméra peut changer d'un lancement à l'autre, il est donc nécessaire de la configurer à chaque fois. Par défaut, elle est configurée sur `10.42.0.188`.

Cependant il est possible de configurer l'IP de la caméra dans le code source du projet.
changer l'IP de la caméra dans les fichiers suivants :

    - franhf_ws/src/camera/camera/camera_control_node.py -- ligne 24
    - franhf_ws/src/camera/camera/camera_bridge.py -- ligne 15
    - franhf_ws/src/camera/mediamtx/mediamtx.yml -- ligne 699
    - franhf_ws/src/camera/camera/camera_publisher.py -- ligne 12?
    - franhf_ws/src/camera/camera/capture_manager.py -- ligne 12

De même manière, on peut configurer le flux dans les fichiers ci-dessus, dans les lignes adjascentes.
Pour vérifier l'IP de la caméra :

#### **Étape 1 : Identifier l'interface réseau ethernet active**

Dans un terminal :
```bash
$ ip a
```
```
[..]
2: enp0s31f6: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP
    link/ether 10:e7:c6:78:17:d8 brd ff:ff:ff:ff:ff:ff
    inet 10.42.0.1/24 brd 10.42.0.255 scope global enp0s31f6
[..]
```

> 💡 **Note** : Cherchez l'interface possédant `link/ether` (ici : `enp0s31f6`)

---

#### Étape 2 : Scanner le réseau pour trouver la caméra


```bash
$ sudo arp-scan --interface=enp0s31f6 --localnet
```
```
Interface: enp0s31f6, type: EN10MB, MAC: 10:e7:c6:78:17:d8, IPv4: 10.42.0.1
Starting arp-scan 1.10.0 with 256 hosts
───────────────────────────────────────────────────
10.42.0.188  ec:71:db:2b:47:73  (Caméra)
───────────────────────────────────────────────────
1 host scanned in 1.801 seconds
```

✅ **IP de la caméra trouvée : `10.42.0.188`**

# 2 - Utilisation

# 3 - Contenu détaillé du projet

## Contenu - client_ws

## Contenu - robot_ws


# 4 - Problèmes possibles et solutions

--> Flux vidéo non visible sur le site web, vérifier que l'ip de la caméra est correcte, voir section 1.2.a.

