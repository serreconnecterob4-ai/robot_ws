# Resume de l interface d action NavigateWaypoints

Fichier source de l interface:
- [robot_ws/src/navigation_interfaces/action/NavigateWaypoints.action](robot_ws/src/navigation_interfaces/action/NavigateWaypoints.action)

## Role de cette interface

Une action ROS2 sert a piloter une tache longue avec trois canaux:
- un Goal pour demarrer la tache,
- des Feedbacks pendant l execution,
- un Result final quand la tache se termine.

Ici, c est le bon mecanisme parce qu une mission waypoints peut durer longtemps, necessite du suivi en temps reel (distance restante, waypoint courant, photo en cours) et doit pouvoir etre annulee proprement.

## Champs du Goal (ce que le client envoie)

- waypoints_x (float64[])
  - Tableau des coordonnees X des waypoints dans le repere de navigation.
- waypoints_y (float64[])
  - Tableau des coordonnees Y des waypoints.
- take_photo (bool[])
  - Flags waypoint par waypoint: true si un arret/capture photo est requis a ce point, false sinon.

Interpretation pratique:
- Le waypoint i est defini par le couple (waypoints_x[i], waypoints_y[i]).
- take_photo[i] indique le comportement camera au passage du waypoint i.

## Champs du Result (retour fin de mission)

- success (bool)
  - true si la mission est terminee avec succes.
  - false si la mission est annulee ou echoue.
- message (string)
  - Message humain explicatif (ex: succes, annulation operateur, echec nav2, repli, etc.).

## Champs du Feedback (pendant la mission)

- current_waypoint_index (int32)
  - Index du waypoint en cours.
- waypoints_remaining (int32)
  - Nombre de waypoints restants.
- distance_remaining (float32)
  - Distance encore a parcourir selon la navigation en cours.
- estimated_time_remaining (float64)
  - Estimation du temps restant (ETA).
- robot_x (float64)
  - Position X courante du robot.
- robot_y (float64)
  - Position Y courante du robot.
- is_taking_photo (bool)
  - true quand la mission est temporairement en phase de prise photo/scan.

## Qui publie / qui consomme cette action dans le projet

Implementation trouvee dans:
- [robot_ws/src/navigation_pkg/navigation_pkg/waypoint_action_server.py](robot_ws/src/navigation_pkg/navigation_pkg/waypoint_action_server.py)

Roles:
- Serveur de l action:
  - waypoint_action_server expose navigate_waypoints (type NavigateWaypoints).
- Client principal dans le projet:
  - le meme waypoint_action_server cree aussi un ActionClient interne (self-client) pour transformer les commandes web en goals d action.
- Interface web:
  - n appelle pas directement l action ROS2 dans ce code; elle passe par les topics UI, puis le noeud serveur envoie le Goal.

## Cas d usage typique (mission de A a Z)

1. L interface web construit une mission:
   - listes X/Y des waypoints,
   - flags photo.
2. La mission est transmise au noeud navigation, qui cree un Goal NavigateWaypoints.
3. Le serveur accepte le Goal et lance la navigation (Nav2).
4. Pendant le trajet, le serveur envoie des Feedbacks:
   - waypoint courant,
   - distance/ETA,
   - position robot.
5. Si take_photo=true sur un waypoint, le serveur passe temporairement en mode photo, puis reprend la mission.
6. L operateur peut demander pause/cancel via l interface; la mission est suspendue ou annulee proprement.
7. En fin de parcours (ou en echec), le serveur renvoie le Result final:
   - success=true/false,
   - message explicatif.
