# Robot ROS2 — Setup Guide

> `robot_ws` • `client_ws` • ROS2 Jazzy  
> https://github.com/serreconnecterob4-ai/robot_ws

---

## Prérequis

- Ubuntu **24.04 LTS** (requis pour ROS2 Jazzy)
- ROS2 **Jazzy Jalisco** installé
- Git installé
- `colcon` — outil de build ROS2
- `rosdep` — gestionnaire de dépendances ROS2

> ⚠️ ROS2 Jazzy requiert Ubuntu 24.04. Ne pas utiliser Ubuntu 22.04 pour cette distribution.

---

## 1. Installation de ROS2 Jazzy (si non installé)

## 2. Cloner le dépôt

### Créer le dossier de travail

```bash
mkdir -p ~/ros2_project
cd ~/ros2_project
```

### Cloner le dépôt GitHub

```bash
git clone https://github.com/serreconnecterob4-ai/robot_ws.git
cd robot_ws
```

### Structure attendue après le clone

```
~/ros2_project/
└── robot_ws/              ← dépôt GitHub cloné
    ├── Documentation/     ← documentation du projet
    ├── client_ws/         ← workspace client ROS2
    ├── robot_ws/          ← workspace robot ROS2
    ├── .gitignore
    └── rapport.txt
```

---

## 3. Setup de `robot_ws`

```bash
# Sourcer ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

cd ~/ros2_project/robot_ws/robot_ws

# Installer les dépendances
rosdep install --from-paths src --ignore-src -r -y

# Compiler
colcon build

# Sourcer l'environnement
source install/setup.bash
```

---

## 4. Setup de `client_ws`

```bash
cd ~/ros2_project/robot_ws/client_ws

# Installer les dépendances
rosdep install --from-paths src --ignore-src -r -y

# Compiler
colcon build

# Sourcer l'environnement
source install/setup.bash
```
---

## 5. Vérification

```bash
# Vérifier la distribution ROS2 active
echo $ROS_DISTRO   # doit afficher : jazzy

# Vérifier la version
ros2 --version

# Lister les packages disponibles
ros2 pkg list | grep -i robot
```

---

## Notes

> ⚠️ En cas de **sous-modules git**, ajouter `--recurse-submodules` au `git clone`.

> ⚠️ Si `colcon build` échoue, vérifier que ROS2 est sourcé : `echo $ROS_DISTRO` doit afficher `jazzy`.
