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

    def publish_gallery(self):
        if not os.path.exists(self.gallery_path):
            return

        # Liste des fichiers images/vidéos (jpg/png/avi/mp4)
        files = [f for f in os.listdir(self.gallery_path) if f.endswith(('.jpg', '.png', '.avi', '.mp4', '.webm'))]
        # Tri par date (plus récent en premier)
        files.sort(key=lambda x: os.path.getmtime(os.path.join(self.gallery_path, x)), reverse=True)
        
        msg = String()
        msg.data = json.dumps(files)
        self.pub.publish(msg)
