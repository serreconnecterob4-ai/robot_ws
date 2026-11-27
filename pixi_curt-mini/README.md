## Connexion au robot

### A faire sur le robot

- Il a fallu arrêter deux services qui lançaient automatiquement des noeuds ROS2 (avec une version différente de ROS2, `kilted`) :
```bash
systemctl stop ipa-ros-autostart.service 
systemctl stop ipa-tmux-master.service
```

- Lancement du conteneur :
```bash
curt@nuc-curt-mini-v2-2-b:~/ROB4_Fraunhofer/curtmini_piper_containers$ PUBLISH_BASE_DESCRIPTION=True docker compose up --build base-bringup
```

### A faire sur le PC

- Lancer l'environnement `pixi` :
```bash
# dans le répertoire $PIXI_ENV
pixi shell
```

- Tester si la communication est établie :
```bash
ros2 topic list
# quelque chose doit s'afficher
```
