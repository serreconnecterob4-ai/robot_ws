#!/bin/bash

# Crée le dossier logs si nécessaire
mkdir -p "$(pwd)/sorted_logs"

# Nom du fichier avec timestamp
LOG_FILE="$(pwd)/sorted_logs/simulation_$(date +%Y-%m-%d_%H-%M-%S).log"

echo "Logs filtrés (WARN/ERROR) → $LOG_FILE"

colcon build
source install/setup.bash

# Lance simulation.xml et filtre les lignes WARN/ERROR en temps réel
ros2 launch curt_mini simulation.xml 2>&1 | tee >(grep -iE "warn|error" > "$LOG_FILE")