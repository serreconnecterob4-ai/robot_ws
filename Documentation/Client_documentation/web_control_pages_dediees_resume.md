# Resume pages web dediees web_control (W3)

## navig (navig.html + navig.js)

### Role de la page
Page d'edition/navigation pour preparer et verifier des trajectoires. Elle cible l'operateur mission qui doit dessiner, sauvegarder, recharger et ajuster des points sur carte avant execution robot.

### Fonctionnalites utilisateur
- Ajouter des points de trajectoire par clic sur carte.
- Voir et modifier la liste des points du trajet en cours.
- Effacer le trajet ou supprimer le dernier point.
- Sauvegarder un trajet avec nom.
- Charger un trajet existant depuis la liste.
- Supprimer un trajet existant.
- Gerer des zones interdites (mode edition).
- Zoom +/- de la carte.
- Voir la position/orientation robot et l'etat offline.
- Activer/desactiver le dark mode.

### Topics ou services ROS2 utilises
Topics:
- /web_trajectory - geometry_msgs/PoseArray (publish)
- /ui/save_trajectory - std_msgs/String (publish)
- /ui/delete_trajectory - std_msgs/String (publish)
- /ui/trajectory_files - std_msgs/String (subscribe)
- /odometry/filtered - nav_msgs/Odometry (subscribe)
- /ui/mission_feedback - std_msgs/String (subscribe)

Services:
- Aucun service ROS2 appele depuis cette page.

### Communication avec le backend
- WebSocket rosbridge: ws://localhost:9090 (pub/sub ROS2).
- HTTP GET pour charger des trajectoires: ../trajectories/<filename>.
- HTTP GET pour zones de base: ./blank_area.json.
- Pas d'upload HTTP direct; la sauvegarde/suppression passe par topics ROS2.

### Donnees affichees et leur source
- Carte et overlay costmap: fichiers image locaux.
- Liste des trajectoires: topic /ui/trajectory_files.
- Position robot et orientation: topic /odometry/filtered.
- Feedback mission: topic /ui/mission_feedback.
- Points/segments courants: etat JS local (trajectoryPoints).

### Points d'attention
- Dependance forte a rosbridge localhost:9090.
- Format JSON de trajectoire attendu strict (image/startPoint/trajectory).
- Plusieurs etats persistes (cookies) peuvent restaurer des trajectoires automatiquement.
- Page volumineuse et multi-fonctions: risque de regressions si modifications non isolees.

---

## galerie (gallery.html)

### Role de la page
Page de consultation media plein ecran pour parcourir les photos/videos du robot. Cible les utilisateurs qui veulent verifier, telecharger ou supprimer rapidement des captures.

### Fonctionnalites utilisateur
- Affichage grille complete des medias.
- Ouverture lightbox image/video.
- Lecture video dans la lightbox.
- Telechargement du media courant.
- Suppression depuis carte ou lightbox.
- Affichage du nombre de fichiers.
- Dark mode.

### Topics ou services ROS2 utilises
Topics:
- /ui/gallery_files - std_msgs/String (subscribe)
- /camera/delete_image - std_msgs/String (publish)

Services:
- Aucun service ROS2 appele depuis cette page.

### Communication avec le backend
- WebSocket rosbridge: ws://<host>:9090 pour la liste et la suppression via topics.
- Chargement des medias via chemins HTTP statiques: ../gallery/<fichier>.
- Pas d'upload ici (upload fait depuis dashboard principal).

### Donnees affichees et leur source
- Liste des fichiers: topic /ui/gallery_files (JSON).
- Vignettes et medias: fichiers servis depuis le dossier gallery symlink.
- Etat connexion: events connexion/error/close ROS.

### Points d'attention
- Sans ROS, la liste ne se met pas a jour en temps reel.
- Suppression asynchrone via topic: pas de retour direct de succes echec cote UI.
- Aucune pagination: vigilance si galerie tres volumineuse.

---

## terminal (terminal.html + terminal.js)

### Role de la page
Console web de supervision pour voir les logs systeme en temps reel. Cible l'operateur ou l'integrateur pendant les tests/exploitation.

### Fonctionnalites utilisateur
- Afficher les logs entrants en flux continu.
- Coloration par niveau (info/warn/error/success).
- Horodatage local de chaque ligne.
- Pause/reprise d'affichage.
- Effacement du terminal.
- Compteur de lignes.
- Dark mode.

### Topics ou services ROS2 utilises
Topics:
- /ui/system_logs - std_msgs/String (subscribe)

Services:
- Aucun service ROS2 appele depuis cette page.

### Communication avec le backend
- WebSocket rosbridge: ws://<host>:9090.
- Pas d'appel HTTP metier.

### Donnees affichees et leur source
- Logs systeme: topic /ui/system_logs (JSON {message, level} ou string brut).
- Statut de connexion: callbacks ros.on(connection/error/close).
- Compteur: derive du nombre de lignes DOM (max 500).

### Points d'attention
- Des logs de demo sont injectes au demarrage (setTimeout), ce qui peut brouiller le diagnostic reel.
- Quand pause activee, les messages recus pendant la pause ne sont pas rejoues ensuite.

---

## accueil (page_presentation.html)

### Role de la page
Page vitrine du projet (mission, capacites, conception, equipe). Elle sert de point d'entree informatif pour les visiteurs/utilisateurs non techniques.

### Fonctionnalites utilisateur
- Lire la presentation du projet et de l'equipe.
- Consulter images et sections techniques de haut niveau.
- Naviguer vers les autres pages (dashboard, etc.).
- Activer/desactiver le dark mode.

### Topics ou services ROS2 utilises
- Aucun topic ROS2.
- Aucun service ROS2.

### Communication avec le backend
- Aucune communication websocket/HTTP metier.
- Page statique avec assets locaux.

### Donnees affichees et leur source
- Textes, tableau et medias statiques integres dans le HTML.
- Preference dark mode stockee en localStorage.

### Points d'attention
- Page purement statique: pas d'etat robot live.
- Les liens/images doivent rester valides dans l'arborescence web.
