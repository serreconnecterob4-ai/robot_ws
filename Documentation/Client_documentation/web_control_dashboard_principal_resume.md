# Resume dashboard principal (index.html + style.css) (W5)

## Structure generale de la page
Le dashboard principal est organise en 4 zones operateur:
- Zone video live + commandes directes (colonne principale)
  - flux camera, pad robot, pad PTZ, boutons photo/video
- Zone reglages (sliders et bascules)
  - vitesse robot, zoom, focus, position/vitesse bras, volume, lampe, micro, alerte, urgence
- Zone minimap mission
  - carte, trajectoire, position robot, selection trajet, lancement/annulation mission, retour base
- Zone galerie rapide
  - vignettes recentes, ouverture galerie complete, suppression d un media

Le header centralise la navigation (presentation, terminal), le statut connexion et la batterie.

## Fonctionnalites accessibles a l utilisateur (non technique)
- Voir la video en direct du robot.
- Piloter le robot au clavier ou aux boutons directionnels.
- Piloter la camera (orientation PTZ, zoom/focus, autofocus).
- Demarrer/arreter une capture photo ou video.
- Lancer, mettre en pause, reprendre ou annuler une mission.
- Suivre la position du robot sur la minimap avec indicateur hors-ligne.
- Choisir et charger un trajet, puis lancer la mission associee.
- Consulter une galerie rapide et supprimer des medias.
- Ouvrir le terminal de logs et la page de navigation avancee.
- Basculer en mode sombre et ouvrir la modale de reglages avances.

## Dependances JS (modules charges et ordre)
Ordre de chargement dans index.html:
1. js/01-core-ros.js
2. js/02-PTZ.js
3. js/03-communication-avec-robot.js
4. js/04-media-gallery.js
5. js/05-trajectoires.js
6. js/06-UI-minimap.js

Dependances externes chargees dans head:
- eventemitter2 (CDN)
- roslib (CDN)

Point important: les modules suivants utilisent des variables/fonctions globales definies par les precedents, donc l ordre est obligatoire.

## Points de configuration visuelle (themes et CSS modifiables)
Dans style.css:
- Variables de theme en tete de fichier:
  - --bg-dark
  - --bg-panel
  - --accent
  - --text
  - --line-color
  - --mission-start
  - --mission-stop
- Theme sombre via body.dark-mode (couleurs header, panneaux, boutons, batterie, modal).
- Grille principale:
  - .container en 3 colonnes (2fr 1fr 1fr) et 9 lignes.
- Composants configurables:
  - tailles boutons dpad, boutons photo/rec, style urgence
  - aspect ratio minimap (16/9)
  - grille galerie (2 colonnes desktop)
  - style modal reglages

Conseil pratique: modifier d abord les variables de :root et body.dark-mode avant de toucher chaque composant unitairement.

## Comportement mobile / resolutions
Breakpoints declares:
- max-width 1200px:
  - passage a une grille 2 colonnes, video et galerie sur largeur etendue
- max-width 900px:
  - passage en colonne unique
  - header replie, controles plus compacts
  - galerie en 3 colonnes
- max-width 600px:
  - optimisation mobile renforcee
  - boutons et texte reduits
  - controles robot empiles
  - galerie en 2 colonnes

Le layout reste utilisable en ecran etroit, avec priorite a la lisibilite des controles critiques.

## Points d attention
- Une connexion ROS active est necessaire pour la majorite des fonctions temps reel:
  - statut connexion
  - batterie
  - missions
  - position robot
  - galerie synchronisee
- Sans flux video joignable, la zone live se degrade (fallback selon config).
- Les boutons mission/urgence dependent de l etat global partage entre modules.
- La map et la trajectoire reposent sur des donnees odometrie/mission; hors ligne, certaines actions sont limitees (mode lock indisponible).
- Les scripts sont relies au backend local web_control (port web et rosbridge), a verifier avant debug UI.
