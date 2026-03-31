# 🛰️ GPS Package - Localisation Robot avec GPS RTK

## 📋 Table des matières
- [Vue d'ensemble](#vue-densemble)
- [Architecture du système](#architecture-du-système)
- [Installation](#installation)
- [Lancement rapide](#lancement-rapide)
- [Configuration détaillée](#configuration-détaillée)
- [Comprendre les covariances](#comprendre-les-covariances)
- [Dépannage](#dépannage)
- [Ajustements avancés](#ajustements-avancés)

---

## 🎯 Vue d'ensemble

Ce package permet à un robot mobile de se localiser précisément en fusionnant plusieurs capteurs :
- **GPS RTK** : Position absolue très précise (~5mm) mais lente (2 Hz)
- **Odomètrie roues** : Rapide (100 Hz) mais dérive avec le temps
- **IMU** : Orientation et vitesse angulaire

Le système utilise **deux filtres de Kalman** (EKF = Extended Kalman Filter) pour avoir :
- ✅ Une position **stable** sans dérive (grâce au GPS)
- ✅ Une estimation **rapide et fluide** (grâce à l'odométrie)
- ✅ Le meilleur des deux mondes !

### 🔍 Cas d'usage
- Navigation outdoor avec GPS
- Cartographie précise en extérieur
- Suivi de trajectoire avec correction GPS
- Applications agricoles, logistique, etc.

---

## 🏗️ Architecture du système

### Schéma de flux de données

```
┌─────────────┐      ┌─────────────┐      ┌─────────────┐
│  Capteurs   │      │   Nœuds     │      │  Sorties    │
└─────────────┘      └─────────────┘      └─────────────┘

  /wheel/odom  ─────┐
                     ├──► EKF Local ──────► /odometry/local
  /imu/data    ─────┤    (100 Hz)           (rapide, dérive)
                     │                             │
                     │                             │
  /gps/fix     ─────┼──► Navsat    ──────► /odometry/gps
                     │    Transform         (GPS en XY)
  /imu/data    ─────┘         │                   │
                               │                   │
  /odometry/local ─────────────┤                   │
                               ├──► EKF Global ───► /odometry/filtered
  /odometry/gps   ─────────────┤    (30 Hz)        (SORTIE FINALE)
                               │                    ↓
  /imu/data       ─────────────┘               Position stable
                                                + haute fréquence
```

### 📦 Trois nœuds principaux

#### 1️⃣ **EKF Local** (`ekf_filter_node` #1)
- **Rôle** : Fusion rapide odomètrie + IMU
- **Fréquence** : 100 Hz (très rapide)
- **Sortie** : `/odometry/local` (frame `odom` → `base_link`)
- **Avantages** : Réactivité instantanée
- **Inconvénient** : Dérive lentement dans le temps (normal !)

#### 2️⃣ **Navsat Transform** (`navsat_transform_node`)
- **Rôle** : Convertir GPS (latitude/longitude) en coordonnées cartésiennes (X, Y)
- **Fréquence** : 30 Hz
- **Sortie** : `/odometry/gps` (position GPS en mètres)
- **Astuce** : Initialise automatiquement le datum au premier message GPS

#### 3️⃣ **EKF Global** (`ekf_filter_node` #2)
- **Rôle** : Fusion GPS + odométrie locale + IMU
- **Fréquence** : 30 Hz
- **Sortie** : `/odometry/filtered` (frame `map` → `odom`)
- **C'EST CETTE SORTIE QU'IL FAUT UTILISER POUR LA NAVIGATION !**

---

## 🚀 Installation

### Prérequis
```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-tf2-tools
```

### Build du package
```bash
cd ~/ros2_ws
colcon build --packages-select gps_package
source install/setup.bash
```

---

## ⚡ Lancement rapide

### Démarrage de tout le système
```bash
# Dans un terminal
ros2 launch gps_package ekf_launch.py
```

### Avec le robot simulé
```bash
# Terminal 1 : Lancer le système de localisation
ros2 launch gps_package ekf_launch.py

# Terminal 2 : Lancer le robot simulé
ros2 run gps_package fake_robot
```

### Visualisation dans RViz
```bash
# Terminal 3
rviz2
```

**Dans RViz, ajouter :**
- Fixed Frame : `map`
- Odometry → `/odometry/filtered` (position finale filtrée)
- Odometry → `/odometry/local` (odomètrie locale qui dérive)
- TF (pour voir les frames)

---

## ⚙️ Configuration détaillée

### 📁 Fichiers de configuration

#### `launch/ekf_launch.py`
Orchestre les 3 nœuds avec leurs paramètres. **C'est ici qu'on configure les covariances.**

#### `config/navsat.yaml`
Configuration du Navsat Transform (conversion GPS → XY).

**Paramètres importants :**
```yaml
wait_for_datum: false      # Auto-initialisation au 1er message GPS
use_odometry_yaw: true     # Utilise l'orientation de l'odométrie
zero_altitude: true        # Ignore l'altitude (robot 2D)
```

#### `gps_package/fake_robot.py`
Simulateur de robot avec capteurs bruités (pour tests).

**Paramètres de bruit à ajuster :**
```python
self.gps_noise_std = 0.005      # Précision GPS (5mm = RTK)
self.odom_scale = 1.002         # Erreur d'échelle odomètrie
self.imu_gyro_bias_z = 0.0002   # Biais gyroscope IMU
```

---

## 🎛️ Comprendre les covariances

### C'est quoi une covariance ?

**En langage simple :** La covariance dit au filtre "à quel point je fais confiance à ce capteur".

- **Covariance PETITE** (ex: 0.0001) = "Ce capteur est TRÈS précis, écoute-le beaucoup !"
- **Covariance GRANDE** (ex: 10.0) = "Ce capteur est bruité, ne l'écoute pas trop"

### 📊 Matrice de covariance

Une matrice 6×6 pour chaque capteur :
```
[x, y, z, roll, pitch, yaw]
```

**Exemple dans le code :**
```python
'odom1_config': [True, True, False,  # x, y, z (on veut x et y du GPS)
                 False, False, False,  # roll, pitch, yaw
                 False, False, False,  # vx, vy, vz
                 False, False, False,  # vroll, vpitch, vyaw
                 False, False, False], # ax, ay, az
```

### 🎯 Configuration actuelle (EKF Global)

```python
# Odométrie locale (/odometry/local)
'odom0_differential': True,              # Mode différentiel (vitesse)
'odom0_relative': False,
'odom0_queue_size': 10,
'odom0_covariance_matrix': [1.0, 0.0, ... 1.0, ...]  # Confiance modérée

# GPS transformé (/odometry/gps)
'odom1_differential': False,             # Mode absolu (position)
'odom1_relative': False,
'odom1_queue_size': 10,
'odom1_covariance_matrix': [0.0001, 0.0, ... 0.0001, ...]  # Très haute confiance

# IMU (/imu/data)
'imu0_differential': False,
'imu0_relative': False,
'imu0_queue_size': 10,
'imu0_covariance_matrix': [0.0, ... 0.01, ...]  # Confiance moyenne sur le yaw
```

**Règle d'or :** GPS (0.0001) est **10 000× plus précis** que l'odomètrie (1.0) → il domine la position finale !

---

## 🔧 Dépannage

### ❌ Problème : Le datum GPS se réinitialise en permanence

**Symptômes :**
- Le robot "saute" de position régulièrement
- Messages `Setting datum` en boucle

**Solution :**
Vérifier dans `config/navsat.yaml` :
```yaml
wait_for_datum: false  # Doit être false pour auto-init
```

---

### ❌ Problème : La position filtrée dérive lentement

**Symptômes :**
- `/odometry/filtered` s'éloigne du GPS au fil du temps
- Le robot part en vrille lentement

**Causes possibles :**

1. **GPS pas assez écouté** → Augmenter sa confiance
   ```python
   # Dans ekf_launch.py, EKF Global
   'odom1_covariance_matrix': [0.00001, 0.0, ...]  # Plus petit = plus de confiance
   ```

2. **Odométrie trop dominante** → Réduire sa confiance
   ```python
   'odom0_covariance_matrix': [10.0, 0.0, ...]  # Plus grand = moins de confiance
   ```

3. **Vérifier que le GPS publie bien**
   ```bash
   ros2 topic hz /gps/fix        # Doit être ~2 Hz
   ros2 topic hz /odometry/gps   # Doit être ~30 Hz
   ```

---

### ❌ Problème : Le filtre est instable, il "saute"

**Symptômes :**
- Mouvements brusques dans `/odometry/filtered`
- Oscillations

**Causes possibles :**

1. **Capteurs trop bruités** → Réduire le bruit dans `fake_robot.py`
   ```python
   self.gps_noise_std = 0.003   # Réduire de 0.005 à 0.003
   self.odom_noise_std = 0.005  # Réduire de 0.008 à 0.005
   ```

2. **Covariances mal ajustées** → Équilibrer GPS vs odomètrie
   ```python
   # Ratio GPS/Odom ~ 10000 est bien, essayer 5000 si instable
   'odom1_covariance_matrix': [0.0002, ...]  # GPS moins dominant
   ```

3. **Fréquence EKF trop élevée/basse**
   ```python
   'frequency': 30.0,  # Essayer 20.0 ou 50.0
   ```

---

### ❌ Problème : Erreur "Latitude X more than 20d from N pole"

**Symptômes :**
- Le navsat_transform_node crash au démarrage

**Solution :**
Vérifier les coordonnées GPS dans `fake_robot.py` :
```python
self.lat0 = 48.8566  # Doit être entre -90 et 90 (ici Paris)
self.lon0 = 2.3522   # Doit être entre -180 et 180
```

---

### ❌ Problème : Pas de transform `map` → `odom`

**Symptômes :**
- RViz ne peut pas afficher les données
- Erreur "Could not transform from map to odom"

**Vérifications :**
```bash
# Vérifier que EKF Global tourne
ros2 node list  # Doit contenir ekf_filter_node_global

# Vérifier les frames TF
ros2 run tf2_ros tf2_echo map odom
```

**Solution :**
- S'assurer que `world_frame: map` dans la config EKF Global
- Vérifier que EKF Global reçoit bien les données GPS :
  ```bash
  ros2 topic echo /odometry/gps
  ```

---

### 🔍 Commandes de debug utiles

```bash
# Lister tous les topics actifs
ros2 topic list

# Voir la fréquence de publication
ros2 topic hz /odometry/filtered

# Afficher les données en temps réel
ros2 topic echo /odometry/filtered

# Visualiser l'arbre des frames TF
ros2 run tf2_tools view_frames
evince frames.pdf

# Info sur un nœud
ros2 node info /ekf_filter_node_global

# Paramètres d'un nœud
ros2 param list /ekf_filter_node_global
```

---

## 🎨 Ajustements avancés

### 🎯 Tuning pour différents scénarios

#### Scénario 1 : GPS très précis (RTK centimétrique)
→ **Faire confiance au GPS au maximum**
```python
# EKF Global
'odom1_covariance_matrix': [0.00001, ...]  # GPS ultra-dominant
'odom0_covariance_matrix': [5.0, ...]      # Odom secondaire
```

#### Scénario 2 : GPS standard (métrique)
→ **Équilibrer GPS et odométrie**
```python
'odom1_covariance_matrix': [0.1, ...]    # GPS modéré
'odom0_covariance_matrix': [0.5, ...]    # Odom aussi important
```

#### Scénario 3 : Environnement urbain (GPS dégradé)
→ **Se fier plus à l'odométrie**
```python
'odom1_covariance_matrix': [1.0, ...]    # GPS moins fiable
'odom0_covariance_matrix': [0.1, ...]    # Odom prioritaire
```

#### Scénario 4 : Robot rapide
→ **Augmenter la fréquence EKF Local**
```python
# EKF Local
'frequency': 200.0,  # Au lieu de 100 Hz
```

---

### 🔬 Ajuster les covariances étape par étape

**Méthode scientifique :**

1. **Tester le GPS seul**
   ```bash
   # Regarder /odometry/gps et évaluer sa stabilité
   ros2 topic echo /odometry/gps
   ```
   → Noter l'amplitude du bruit (ex: ±5mm)

2. **Définir la covariance GPS**
   ```python
   # Si bruit = ±5mm = ±0.005m
   # Covariance = écart-type² = 0.005² = 0.000025
   'odom1_covariance_matrix': [0.000025, ...]
   ```

3. **Tester l'odomètrie seule**
   ```bash
   ros2 topic echo /odometry/local
   ```
   → Noter la dérive sur 10 secondes

4. **Définir la covariance odométrie**
   ```python
   # Si dérive = ~0.1m en 10s
   # Covariance = 0.1² = 0.01
   'odom0_covariance_matrix': [0.01, ...]
   ```

5. **Affiner en observant le comportement**
   - GPS trop lent à corriger ? → Réduire sa covariance
   - Filtre instable ? → Augmenter covariance GPS ou réduire bruit capteurs

---

### 📏 État initial du robot

Le robot démarre à la position `(0, 3, 0)` :
```python
# Dans ekf_launch.py, EKF Local
'initial_state': [0.0, 3.0, 0.0,    # x, y, z
                  0.0, 0.0, 0.0,    # roll, pitch, yaw
                  0.0, 0.0, 0.0,    # vx, vy, vz
                  0.0, 0.0, 0.0,    # vroll, vpitch, vyaw
                  0.0, 0.0, 0.0],   # ax, ay, az
```

**⚠️ Important :** Si votre robot démarre ailleurs, **adaptez ces valeurs** sinon le filtre mettra du temps à converger !

---

## 📚 Ressources

- [Documentation robot_localization](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [REP-105 : Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Tutoriel navsat_transform](http://docs.ros.org/en/noetic/api/robot_localization/html/navsat_transform_node.html)

---

## 🤝 Contribution

Pour améliorer ce package :
1. Fork le projet
2. Créer une branche (`git checkout -b feature/amelioration`)
3. Commit (`git commit -m 'Ajout fonctionnalité X'`)
4. Push (`git push origin feature/amelioration`)
5. Ouvrir une Pull Request

---

## 📝 Licence

Ce package est sous licence MIT.

---

## ✨ Bon à savoir

### Pourquoi deux EKF ?

**Un seul EKF ne suffit pas !**
- Capteurs lents (GPS 2 Hz) + capteurs rapides (odom 100 Hz) = problèmes
- Solution : EKF Local (rapide) + EKF Global (stable)

### Quelle sortie utiliser ?

| Topic | Usage | Caractéristiques |
|-------|-------|------------------|
| `/odometry/local` | ❌ Ne PAS utiliser | Rapide mais dérive |
| `/odometry/gps` | ❌ Ne PAS utiliser | Stable mais lent et bruiteux |
| `/odometry/filtered` | ✅ À UTILISER | Le meilleur des deux ! |

### Le datum, c'est quoi ?

Le **datum** est l'origine du repère local :
- Au démarrage, le navsat_transform attend le 1er message GPS
- Il définit cette position comme origine (0, 0) du repère `map`
- Ensuite, tous les points GPS sont exprimés relativement à ce datum
- **Il ne change plus** (sauf si le nœud redémarre)

---

**🎉 Félicitations ! Vous êtes prêt à utiliser le système de localisation GPS + EKF !**

Pour toute question, ouvrir une issue sur le repo GitHub.
