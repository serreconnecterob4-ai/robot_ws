import os
import time
import subprocess
from datetime import datetime

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
        self.recording = False

        # Enregistrement FFmpeg (video + audio)
        self.ffmpeg_process = None
        self.rtsp_url = "rtsp://localhost:8554/mystream"

    def set_rtsp_url(self, stream_url):
        if not stream_url:
            return
        self.rtsp_url = stream_url
        self.node.get_logger().info(f"Source capture externe configurée: {self.rtsp_url}")

    def take_photo(self):
        filename = f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        path = os.path.join(self.gallery_path, filename)

        # Par défaut: capture directe du flux RTSP via ffmpeg.
        success, error = self._take_photo_ffmpeg(path)
        if success:
            self.node.get_logger().info(f"Photo sauvegardée (ffmpeg): {path}")
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
        
        try:
            self.node.get_logger().info(f"Tentative enregistrement FFmpeg avec audio: {path}")
            self._start_video_ffmpeg(path)
            self.recording = True
            self.node.get_logger().info(f"Début enregistrement FFmpeg: {path}")
            return True, "Enregistrement démarré (audio + vidéo)"
        except Exception as e:
            return False, f"Erreur enregistrement FFmpeg: {e}"

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
        
        self.node.get_logger().info("Fin enregistrement")
        return True, "Enregistrement arrêté"
