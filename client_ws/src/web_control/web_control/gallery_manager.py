import os
import json
from std_msgs.msg import String

"""

Gallery Manager : Gère la publication des 
fichiers multimédias dans la galerie
Il scanne régulièrement le dossier de la galerie pour 
détecter les nouvelles photos/vidéos,
"""


class GalleryManager:
    def __init__(self, node, gallery_path):
        self.node = node
        self.gallery_path = gallery_path
        self.pub = self.node.create_publisher(String, '/ui/gallery_files', 10)
        
        # Timer pour mettre à jour la galerie (1Hz)
        self.timer = self.node.create_timer(1.0, self._timer_publish)

    def _timer_publish(self):
        """Callback interne du timer - ne loggue que silencieusement"""
        self.publish_gallery(reason='timer', log_update=False)

    def publish_gallery(self, reason='sync', log_update=True):
        if not os.path.exists(self.gallery_path):
            return

        # Liste des fichiers images/vidéos (jpg/png/avi/mp4)
        files = [f for f in os.listdir(self.gallery_path) if f.endswith(('.jpg', '.png', '.avi', '.mp4', '.webm'))]
        # Tri par date (plus récent en premier)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(self.gallery_path, x)), reverse=True)
        
        # Log: Galerie mise à jour (seulement si explicitement demandé)
        if log_update:
            self.node.get_logger().info(f"[GALLERY] Galerie mise à jour ({reason}): {len(files)} fichier(s)")
            if files:
                max_display = min(5, len(files))
                displayed = files[:max_display]
                more_text = f" +{len(files) - max_display} autres" if len(files) > max_display else ""
                self.node.get_logger().debug(f"[GALLERY] Fichiers: {', '.join(displayed)}{more_text}")
        
        msg = String()
        msg.data = json.dumps(files)
        self.pub.publish(msg)
