import cv2
import os
import time
import threading
from datetime import datetime
from geometry_msgs.msg import Point

'''
CaptureManager : Gère le scan d'un point de vue
complet (haut/droite -> bas/gauche) 
en utilisant le flux RTSP direct de la caméra. (CLEAR)
'''

class CaptureManager:
    def __init__(self, node, gallery_path):
        self.node = node
        self.gallery_path = gallery_path
        self.rtsp_url = "rtsp://admin:ros2_2025@10.42.0.188:554/h264Preview_01_main"
        self.recording = False
        self.video_writer = None
        self._record_thread = None
        self._cap_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._scan_active = False

        # Connexion directe au flux RTSP (plus de topic ROS ni de CvBridge)
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            self.node.get_logger().error(f"❌ Impossible d'ouvrir le flux RTSP : {self.rtsp_url}")

        self.node.ptz_pub = self.node.create_publisher(Point, '/camera/ptz', 10)

    def _get_frame(self):
        """Lit une frame depuis le flux RTSP."""
        # OpenCV/FFmpeg n'est pas thread-safe sur un meme VideoCapture.
        with self._cap_lock:
            if not self.cap.isOpened():
                return None
            ret, frame = self.cap.read()
        return frame if ret else None

    def send_ptz_ros(self, op):
        """Envoie un ordre PTZ au ControlNode via le topic /camera/ptz."""
        msg = Point()
        if op == "Up":          msg.x = 1.0
        elif op == "Down":      msg.x = -1.0
        elif op == "Left":      msg.y = -1.0
        elif op == "Right":     msg.y = 1.0
        elif op == "LeftUp":    msg.x = 1.0;  msg.y = -1.0
        elif op == "RightDown": msg.x = -1.0; msg.y = 1.0
        elif op == "Stop":      msg.x = 0.0;  msg.y = 0.0

        self.node.ptz_pub.publish(msg)
        self.node.get_logger().info(f"👉 Ordre envoyé au ControlNode: {op}")

    def run_auto_scan(self):
        """Scan complet en Z : Aller (Haut/Droite) -> Descente -> Retour (Bas/Gauche)"""
        with self._state_lock:
            if self._scan_active:
                self.node.get_logger().warn('Scan deja en cours, commande ignoree.')
                return
            self._scan_active = True

        frame = self._get_frame()
        if frame is None:
            self.node.get_logger().error("❌ Scan Serre : Flux RTSP absent.")
            with self._state_lock:
                self._scan_active = False
            return

        self.start_video()
        self.node.get_logger().info("🌱 Début du scan complet de la serre...")

        try:
            # --- PHASE 1 : ALLER VERS LA DROITE (ZONE HAUTE) ---
            nb_steps = 4
            for i in range(nb_steps):
                self.node.get_logger().info(f"📸 Zone Haute - Position {i+1}/{nb_steps}")
                self.send_ptz_ros("Stop")
                time.sleep(2.0)  # Stabilisation autofocus
                self.take_photo()

                if i < nb_steps - 1:
                    self.send_ptz_ros("Right")
                    time.sleep(2.0)
                    self.send_ptz_ros("Stop")

            # --- PHASE 2 : CHANGEMENT DE TILT (DESCENTE) ---
            self.node.get_logger().info("📉 Descente du Tilt pour le rang inférieur...")
            self.send_ptz_ros("Down")
            time.sleep(1.5)
            self.send_ptz_ros("Stop")
            time.sleep(1.0)

            # --- PHASE 3 : RETOUR VERS LA GAUCHE (ZONE BASSE) ---
            for i in range(nb_steps):
                self.node.get_logger().info(f"📸 Zone Basse - Position {i+1}/{nb_steps}")
                self.send_ptz_ros("Stop")
                time.sleep(2.0)
                self.take_photo()

                if i < nb_steps - 1:
                    self.send_ptz_ros("Left")
                    time.sleep(2.0)
                    self.send_ptz_ros("Stop")

        except Exception as e:
            self.node.get_logger().error(f"⚠️ Erreur scan complet: {e}")
        finally:
            self.send_ptz_ros("Stop")
            self.stop_video()
            #self.node.gallery_mgr.publish_gallery()
            self.node.get_logger().info("✅ Scan complet terminé.")
            with self._state_lock:
                self._scan_active = False

    def take_photo(self):
        """Capture une frame depuis le flux RTSP et la sauvegarde en JPEG."""
        frame = self._get_frame()
        if frame is None:
            return False, "Pas d'image RTSP"
        filename = f"photo_{datetime.now().strftime('%H%M%S')}.jpg"
        path = os.path.join(self.gallery_path, filename)
        cv2.imwrite(path, frame)
        self.node.get_logger().info(f"📷 Photo sauvegardée : {path}")
        return True, path

    def start_video(self):
        """Démarre l'enregistrement vidéo depuis le flux RTSP."""
        with self._state_lock:
            if self.recording:
                return False, "Enregistrement déjà en cours"
        frame = self._get_frame()
        if frame is None:
            return False, "Flux RTSP indisponible"

        filename = f"mission_{datetime.now().strftime('%H%M%S')}.mp4"
        path = os.path.join(self.gallery_path, filename)
        h, w, _ = frame.shape
        self.video_writer = cv2.VideoWriter(
            path, cv2.VideoWriter_fourcc(*'avc1'), 15.0, (w, h)
        )
        with self._state_lock:
            self.recording = True
        self.node.get_logger().info(f"🎥 Enregistrement démarré : {path}")

        # Lance l'écriture des frames en continu dans un thread dédié
        self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._record_thread.start()

        return True, path

    def _record_loop(self):
        """Boucle d'écriture des frames vidéo (tourne dans un thread séparé)."""
        while True:
            with self._state_lock:
                if not self.recording:
                    break
            frame = self._get_frame()
            if frame is not None and self.video_writer is not None:
                self.video_writer.write(frame)
            time.sleep(1.0 / 15.0)  # ~15 fps

    def stop_video(self):
        """Arrête l'enregistrement vidéo."""
        with self._state_lock:
            self.recording = False
            thread = self._record_thread
            self._record_thread = None

        if thread is not None:
            thread.join(timeout=2.0)

        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        self.node.get_logger().info("⏹️ Enregistrement vidéo terminé.")
        return True, "Enregistrement terminé"

    def release(self):
        """Libère la connexion RTSP (à appeler à la destruction du nœud)."""
        self.stop_video()
        if self.cap.isOpened():
            self.cap.release()