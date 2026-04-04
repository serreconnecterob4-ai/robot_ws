import cv2
import os
import time
import subprocess
import threading
from datetime import datetime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

'''
Capture manager : Gère la capture de photos et vidéos 
à partir du flux RTSP de la caméra.
Il s'abonne au topic /camera/clear pour recevoir 
les images en temps réel, et offre des fonctions 
pour prendre des photos
et enregistrer des vidéos. (CLEAR)
'''


class CaptureManager:
    def __init__(self, node, gallery_path):
        self.node = node
        self.gallery_path = gallery_path
        self.bridge = CvBridge()
        self.latest_image = None
        self.recording = False
        self.video_writer = None
        
        # Pour enregistrement FFmpeg (vidéo + audio)
        self.ffmpeg_process = None
        self.ffmpeg_thread = None
        self.rtsp_url = "rtsp://localhost:8554/mystream"
        
        # Abonnement au flux caméra pour capturer
        self.sub = self.node.create_subscription(
            Image, '/camera/clear', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            # Conversion ROS Image -> OpenCV
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Gestion enregistrement vidéo (fallback OpenCV si FFmpeg échoue)
            if self.recording and self.video_writer:
                self.video_writer.write(self.latest_image)
        except Exception as e:
            self.node.get_logger().error(f"Erreur conversion image: {e}")

    def take_photo(self):
        if self.latest_image is None:
            return False, "Pas d'image reçue"
        
        filename = f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        path = os.path.join(self.gallery_path, filename)
        cv2.imwrite(path, self.latest_image)
        self.node.get_logger().info(f"Photo sauvegardée: {path}")
        return True, path

    def start_video(self):
        if self.recording:
            return False, "Déjà en cours"
        if self.latest_image is None:
            return False, "Pas d'image pour initier la vidéo"
            
        filename = f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        path = os.path.join(self.gallery_path, filename)
        
        # Essayer d'abord avec FFmpeg (capture audio + vidéo du flux RTSP)
        try:
            self.node.get_logger().info(f"Tentative enregistrement FFmpeg avec audio: {path}")
            self._start_video_ffmpeg(path)
            self.recording = True
            self.node.get_logger().info(f"Début enregistrement FFmpeg: {path}")
            return True, "Enregistrement démarré (audio + vidéo)"
        except Exception as e:
            self.node.get_logger().warning(f"FFmpeg échoué, fallback OpenCV: {e}")
            # Fallback sur OpenCV (vidéo seulement)
            try:
                height, width, _ = self.latest_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'avc1')
                self.video_writer = cv2.VideoWriter(path, fourcc, 10.0, (width, height))
                
                self.recording = True
                self.node.get_logger().info(f"Début enregistrement OpenCV: {path}")
                return True, "Enregistrement démarré (vidéo uniquement)"
            except Exception as e2:
                return False, f"Erreur enregistrement: {e2}"

    def _start_video_ffmpeg(self, output_path):
        """Lance FFmpeg pour capturer vidéo + audio du flux RTSP"""
        # Commande FFmpeg : capture du flux RTSP avec audio
        cmd = [
            'ffmpeg',
            '-rtsp_transport', 'tcp',
            '-i', self.rtsp_url,
            '-c:v', 'avc1',           # Codec vidéo H.264
            '-c:a', 'aac',            # Codec audio AAC
            '-y',                      # Overwrite output file
            output_path
        ]
        
        self.ffmpeg_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.PIPE
        )

    def stop_video(self):
        if not self.recording:
            return False, "Pas d'enregistrement en cours"
        
        self.recording = False
        
        # Si FFmpeg est en cours
        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.stdin.write(b'q')
                self.ffmpeg_process.stdin.flush()
                self.ffmpeg_process.wait(timeout=5)
                self.node.get_logger().info("FFmpeg arrêté proprement")
            except Exception as e:
                self.node.get_logger().warning(f"Erreur arrêt FFmpeg: {e}")
                self.ffmpeg_process.terminate()
            finally:
                self.ffmpeg_process = None
        
        # Si OpenCV est en cours
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        self.node.get_logger().info("Fin enregistrement")
        return True, "Enregistrement arrêté"
