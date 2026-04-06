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

    def set_rtsp_url(self, stream_url):
        if not stream_url:
            return
        self.rtsp_url = stream_url
        self.node.get_logger().info(f"Source capture externe configurée: {self.rtsp_url}")

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
        filename = f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        path = os.path.join(self.gallery_path, filename)

        # Par défaut: capture directe du flux RTSP via ffmpeg.
        success, error = self._take_photo_ffmpeg(path)
        if success:
            self.node.get_logger().info(f"Photo sauvegardée (ffmpeg): {path}")
            return True, path

        # Secours uniquement: frame ROS si disponible.
        if self.latest_image is not None:
            cv2.imwrite(path, self.latest_image)
            self.node.get_logger().info(f"Photo sauvegardée (fallback ROS image): {path}")
            return True, path

        return False, f"Capture RTSP impossible: {error}"

    def _take_photo_ffmpeg(self, output_path):
        cmd = ['ffmpeg', '-y']
        if self.rtsp_url.startswith('rtsp://'):
            cmd += ['-rtsp_transport', 'tcp']
        cmd += [
            '-i', self.rtsp_url,
            '-frames:v', '1',
            '-q:v', '2',
            '-update', '1',
            output_path,
        ]

        try:
            proc = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=15,
                check=False,
            )
        except Exception as e:
            return False, str(e)

        if proc.returncode != 0:
            err = proc.stderr.decode(errors='ignore').strip().splitlines()
            return False, err[-1] if err else f"ffmpeg rc={proc.returncode}"

        if not os.path.exists(output_path) or os.path.getsize(output_path) == 0:
            return False, 'fichier photo vide'

        return True, None

    def start_video(self):
        if self.recording:
            return False, "Déjà en cours"
            
        filename = f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        path = os.path.join(self.gallery_path, filename)
        
        # Essayer d'abord avec FFmpeg (capture audio + vidéo du flux externe)
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
                if self.latest_image is None:
                    return False, f"Pas d'image pour fallback OpenCV et ffmpeg indisponible ({e})"
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
        cmd = ['ffmpeg', '-y']
        if self.rtsp_url.startswith('rtsp://'):
            cmd += ['-rtsp_transport', 'tcp']
        cmd += [
            '-i', self.rtsp_url,
            '-map', '0:v:0',
            '-map', '0:a?',
            '-c:v', 'copy',
            '-c:a', 'aac',
            '-movflags', '+faststart',
            output_path,
        ]
        
        self.ffmpeg_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.PIPE
        )

        # Validation rapide: si ffmpeg meurt immédiatement, remonter l'erreur.
        time.sleep(0.35)
        if self.ffmpeg_process.poll() is not None:
            err = self.ffmpeg_process.stderr.read().decode(errors='ignore')
            self.ffmpeg_process = None
            raise RuntimeError(err.splitlines()[-1] if err else 'ffmpeg a quitté prématurément')

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
