import rclpy
from rclpy.node import Node
import os
import threading
import http.server
import socketserver
from urllib.parse import urlparse, parse_qs
from urllib import error as url_error
from urllib import parse as url_parse
from urllib import request as url_request
import shutil
import json
import time
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point # <--- Changement ici (Twist -> Point)
from std_msgs.msg import String, Bool     # <--- Pour la suppression et arrêt d'urgence

# Modules internes
from web_control.capture_manager import CaptureManager
from web_control.gallery_manager import GalleryManager

PORT_WEB = 8000

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    gallery_dir = None
    gallery_mgr = None

    def log_message(self, format, *args):
        pass

    def copyfile(self, source, outputfile):
        try:
            super().copyfile(source, outputfile)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path not in ("/upload_photo", "/upload_video"):
            self.send_response(404)
            self.end_headers()
            return

        if not QuietHandler.gallery_dir:
            self.send_response(500)
            self.end_headers()
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0

        data = self.rfile.read(length) if length > 0 else b""
        params = parse_qs(parsed.query)
        filename = params.get("filename", [""])[0]
        filename = os.path.basename(filename) if filename else ""

        if not filename:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            ext = ".jpg" if parsed.path == "/upload_photo" else ".webm"
            filename = f"capture_{stamp}{ext}"

        if not data:
            self.send_response(400)
            self.end_headers()
            return

        path = os.path.join(QuietHandler.gallery_dir, filename)
        try:
            with open(path, "wb") as f:
                f.write(data)
        except Exception:
            self.send_response(500)
            self.end_headers()
            return

        if QuietHandler.gallery_mgr:
            media_type = "vidéo" if parsed.path == "/upload_video" else "photo"
            file_size_kb = len(data) / 1024
            QuietHandler.gallery_mgr.node.get_logger().info(
                f"[UPLOAD] ✓ {media_type} uploadée localement: {filename} ({file_size_kb:.1f} KB)"
            )
            QuietHandler.gallery_mgr.publish_gallery(reason='local_upload', log_update=True)

        self.send_response(200)
        self.end_headers()

class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

class WebBackend(Node):
    def __init__(self):
        super().__init__('web_backend')

        self.declare_parameter('robot_gallery_sync_enabled', True)
        self.declare_parameter('robot_gallery_host', '100.106.79.105')
        self.declare_parameter('robot_gallery_port', 8092)
        self.declare_parameter('robot_gallery_sync_period_sec', 10.0)
        self.declare_parameter('robot_gallery_timeout_sec', 3.0)

        self.robot_gallery_sync_enabled = bool(self.get_parameter('robot_gallery_sync_enabled').value)
        self.robot_gallery_host = str(self.get_parameter('robot_gallery_host').value)
        self.robot_gallery_port = int(self.get_parameter('robot_gallery_port').value)
        self.robot_gallery_sync_period_sec = float(self.get_parameter('robot_gallery_sync_period_sec').value)
        self.robot_gallery_timeout_sec = float(self.get_parameter('robot_gallery_timeout_sec').value)

        self._sync_lock = threading.Lock()
        self._sync_in_progress = False
        self._robot_gallery_online = False
        self._last_mission_result_payload = ''
        self._last_mission_result_at = 0.0
        
        # 1. Chemin vers les fichiers du site Web (HTML/JS) - Dossier d'installation ROS
        package_share = get_package_share_directory('web_control')
        self.web_dir = os.path.join(package_share, 'web')
        
        # 2. Chemin de stockage SÉCURISÉ (Dans ton dossier Home)
        # Les fichiers iront dans /home/utilisateur/robot_gallery
        home_dir = os.path.expanduser('~')
        self.gallery_dir = os.path.join(home_dir, 'robot_gallery')
        self.trajectories_dir = os.path.join(home_dir, 'trajectories')
        
        # Création des dossiers physiques s'ils n'existent pas
        if not os.path.exists(self.gallery_dir):
            os.makedirs(self.gallery_dir)
            self.get_logger().info(f"Dossier de stockage créé: {self.gallery_dir}")
        
        if not os.path.exists(self.trajectories_dir):
            os.makedirs(self.trajectories_dir)
            self.get_logger().info(f"Dossier trajectoires créé: {self.trajectories_dir}")

        # 3. CRÉATION DU LIEN SYMBOLIQUE
        # Le serveur web cherche les images dans 'web/gallery'.
        # On crée un raccourci de 'web/gallery' vers '~/robot_gallery'.
        link_path = os.path.join(self.web_dir, 'gallery')

        # Nettoyage : si un dossier ou un vieux lien existe déjà à cet endroit, on l'enlève
        if os.path.exists(link_path):
            if os.path.islink(link_path):
                os.unlink(link_path)       # Supprime le lien existant
            else:
                shutil.rmtree(link_path)   # Supprime le dossier physique (s'il a été créé par erreur)

        # Création du nouveau lien pour gallery
        try:
            os.symlink(self.gallery_dir, link_path)
            self.get_logger().info(f"Lien symbolique créé de {link_path} vers {self.gallery_dir}")
        except OSError as e:
            self.get_logger().error(f"Erreur création lien symbolique gallery: {e}")
        
        # Création du lien symbolique pour trajectories
        traj_link_path = os.path.join(self.web_dir, 'trajectories')
        if os.path.exists(traj_link_path):
            if os.path.islink(traj_link_path):
                os.unlink(traj_link_path)
            else:
                shutil.rmtree(traj_link_path)
        
        try:
            os.symlink(self.trajectories_dir, traj_link_path)
            self.get_logger().info(f"Lien symbolique créé de {traj_link_path} vers {self.trajectories_dir}")
        except OSError as e:
            self.get_logger().error(f"Erreur création lien symbolique trajectories: {e}")

        # Initialisation des managers avec le dossier SÉCURISÉ
        self.capture_mgr = CaptureManager(self, self.gallery_dir)
        self.gallery_mgr = GalleryManager(self, self.gallery_dir)
        self.configure_capture_source()

        QuietHandler.gallery_dir = self.gallery_dir
        QuietHandler.gallery_mgr = self.gallery_mgr

        # Services
        self.srv_photo = self.create_service(Trigger, '/camera/take_photo', self.cb_take_photo)
        self.srv_start_vid = self.create_service(Trigger, '/camera/start_video', self.cb_start_video)
        self.srv_stop_vid = self.create_service(Trigger, '/camera/stop_video', self.cb_stop_video)

        # Subscribers
        self.create_subscription(Point, '/robot/cmd_vel_buttons', self.cb_cmd_vel, 10)
        self.create_subscription(Point, '/camera/ptz', self.cb_ptz, 10)
        
        # Publishers pour le contrôle du robot
        self.cmd_vel_pub = self.create_publisher(Point, '/robot/cmd_vel_buttons', 10)
        
        # Subscriber pour suppression
        self.create_subscription(String, '/camera/delete_image', self.cb_delete_image, 10)
        
        # Subscriber pour sauvegarde de trajectoire
        self.create_subscription(String, '/ui/save_trajectory', self.cb_save_trajectory, 10)
        
        # Subscriber pour suppression de trajectoire
        self.create_subscription(String, '/ui/delete_trajectory', self.cb_delete_trajectory, 10)
        
        # Subscriber pour arrêt d'urgence
        self.create_subscription(Bool, '/robot/emergency_stop', self.cb_emergency_stop, 10)

        # Déclenche une synchro immédiate des médias robot à la fin de mission
        self.create_subscription(String, '/ui/mission_result', self.cb_mission_result, 10)
        
        # Publisher pour la liste des trajectoires
        self.traj_list_pub = self.create_publisher(String, '/ui/trajectory_files', 10)
        
        # Publisher pour les logs système
        self.logs_pub = self.create_publisher(String, '/ui/system_logs', 10)
        
        # Timer pour publier la liste des trajectoires périodiquement
        self.create_timer(2.0, self.publish_trajectory_list)

        # Synchronisation périodique robot -> client (fichiers galerie)
        self.create_timer(max(1.0, self.robot_gallery_sync_period_sec), self._schedule_periodic_gallery_sync)

        self.httpd = None
        self.server_thread = None

        self.get_logger().info(f"Backend prêt. Web Port: {PORT_WEB}")
        self.publish_log("Backend initialisé", "success")
        self.start_web_server()
        self._schedule_gallery_sync('startup')

    def _robot_gallery_base_url(self):
        return f'http://{self.robot_gallery_host}:{self.robot_gallery_port}'

    def _schedule_periodic_gallery_sync(self):
        self._schedule_gallery_sync('periodic')

    def _schedule_gallery_sync(self, reason='manual'):
        if not self.robot_gallery_sync_enabled:
            return

        with self._sync_lock:
            if self._sync_in_progress:
                return
            self._sync_in_progress = True

        threading.Thread(
            target=self._sync_gallery_worker,
            args=(reason,),
            daemon=True,
        ).start()

    def _sync_gallery_worker(self, reason='manual'):
        try:
            self.get_logger().info(f"[SYNC] Démarrage synchronisation galerie ({reason})")
            remote_files = self._fetch_robot_gallery_list()
            self.get_logger().info(f"[SYNC] Récupéré {len(remote_files)} fichier(s) du robot")
            
            if not self._robot_gallery_online:
                self._robot_gallery_online = True
                self.get_logger().info(
                    f'[ROBOT] Robot gallery online ({self._robot_gallery_base_url()})'
                )

            local_files = set(
                f for f in os.listdir(self.gallery_dir)
                if os.path.isfile(os.path.join(self.gallery_dir, f))
            )
            self.get_logger().info(f"[SYNC] Fichiers locaux: {len(local_files)}")
            
            missing = [name for name in remote_files if name not in local_files]
            if missing:
                self.get_logger().info(f"[SYNC] ⚠️ Fichiers manquants ({len(missing)}): {missing}")
            else:
                self.get_logger().info(f"[SYNC] ✓ Tous les fichiers du robot sont présents localement")

            downloaded = 0
            failed = []
            for name in missing:
                self.get_logger().info(f"[SYNC] Téléchargement de {name}...")
                if self._download_robot_gallery_file(name):
                    downloaded += 1
                    self.get_logger().info(f"[SYNC] ✓ {name} téléchargé avec succès")
                else:
                    failed.append(name)
                    self.get_logger().warning(f"[SYNC] ✗ Erreur téléchargement de {name}")

            # Résumé final
            if len(missing) > 0:
                summary = f"[SYNC] Synchronisation ({reason}): {downloaded}/{len(missing)} fichier(s) téléchargé(s)"
                if failed:
                    summary += f" - Échecs: {failed}"
                self.get_logger().info(summary)
                
                if downloaded > 0:
                    self.gallery_mgr.publish_gallery(reason=f'sync_{reason}', log_update=True)
                    self.get_logger().info(f"[SYNC] Galerie mise à jour (ajout de {downloaded} nouveau/aux fichier(s))")
            else:
                self.get_logger().info(f"[SYNC] Synchronisation ({reason}): aucun nouveau fichier")

        except Exception as e:
            if self._robot_gallery_online:
                self.get_logger().warning(
                    f'[ROBOT] Robot gallery offline ({self._robot_gallery_base_url()}): {e}'
                )
            self._robot_gallery_online = False
        finally:
            with self._sync_lock:
                self._sync_in_progress = False

    def _fetch_robot_gallery_list(self):
        url = f'{self._robot_gallery_base_url()}/list'
        with url_request.urlopen(url, timeout=self.robot_gallery_timeout_sec) as response:
            if response.status != 200:
                raise RuntimeError(f'HTTP status {response.status} for {url}')
            payload = json.loads(response.read().decode('utf-8'))

        files = payload.get('files', []) if isinstance(payload, dict) else []
        if not isinstance(files, list):
            raise RuntimeError('Invalid robot gallery response format')

        safe = []
        for name in files:
            if not isinstance(name, str):
                continue
            base = os.path.basename(name)
            if base != name or not base:
                continue
            safe.append(base)
        return safe

    def _download_robot_gallery_file(self, filename):
        safe_name = os.path.basename(filename)
        if not safe_name or safe_name != filename:
            return False

        encoded = url_parse.quote(safe_name)
        url = f'{self._robot_gallery_base_url()}/files/{encoded}'
        target = os.path.join(self.gallery_dir, safe_name)
        tmp_target = f'{target}.part'

        try:
            with url_request.urlopen(url, timeout=self.robot_gallery_timeout_sec) as response:
                if response.status != 200:
                    self.get_logger().error(f"[DOWNLOAD] HTTP {response.status} pour {filename}")
                    return False
                with open(tmp_target, 'wb') as f:
                    shutil.copyfileobj(response, f)
            file_size = os.path.getsize(tmp_target)
            os.replace(tmp_target, target)
            self.get_logger().debug(f"[DOWNLOAD] {filename} ({file_size} bytes) sauvegardé localement")
            return True
        except (url_error.URLError, OSError) as e:
            self.get_logger().error(f"[DOWNLOAD] Erreur pour {filename}: {e}")
            try:
                if os.path.exists(tmp_target):
                    os.remove(tmp_target)
            except OSError:
                pass
            return False

    def cb_mission_result(self, msg):
        payload = (msg.data or '').strip()
        now = time.monotonic()
        if payload == self._last_mission_result_payload and (now - self._last_mission_result_at) < 3.0:
            return
        self._last_mission_result_payload = payload
        self._last_mission_result_at = now
        self._schedule_gallery_sync('mission_result')

    def configure_capture_source(self):
        """Configure la source de capture ffmpeg à partir de web/configuration.json."""
        config_path = os.path.join(self.web_dir, 'configuration.json')
        if not os.path.exists(config_path):
            self.get_logger().warning(f"Config vidéo introuvable: {config_path}")
            return

        try:
            with open(config_path, 'r') as f:
                cfg = json.load(f)

            video_cfg = cfg.get('video', {}) if isinstance(cfg, dict) else {}
            capture_url = video_cfg.get('capture_url') or video_cfg.get('rtsp_url')

            if not capture_url:
                host = str(video_cfg.get('host', '')).strip()
                stream = str(video_cfg.get('stream', 'mystream')).strip('/')
                rtsp_port = int(video_cfg.get('rtsp_port', 8554))

                if host.lower() in ('', 'auto', 'localhost', '127.0.0.1'):
                    host = 'localhost'

                capture_url = f"rtsp://{host}:{rtsp_port}/{stream}"

            self.capture_mgr.set_rtsp_url(capture_url)
            self.publish_log(f"Source capture configurée: {capture_url}", "info")
        except Exception as e:
            self.get_logger().warning(f"Impossible de configurer la source capture: {e}")
    
    def publish_log(self, message, level="info"):
        """Publie un message de log sur le topic /ui/system_logs"""
        log_data = {
            "message": message,
            "level": level
        }
        msg = String()
        msg.data = json.dumps(log_data)
        self.logs_pub.publish(msg)

    def start_web_server(self):
        try:
            os.chdir(self.web_dir)
            self.httpd = ThreadedHTTPServer(("", PORT_WEB), QuietHandler)
            self.server_thread = threading.Thread(target=self.httpd.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            self.publish_log(f"Serveur web démarré sur le port {PORT_WEB}", "success")
        except OSError as e:
            self.get_logger().error(f"Erreur serveur web: {e}")
            self.publish_log(f"Erreur serveur web: {e}", "error")

    # Callbacks (Juste pour debug ou hardware)
    def cb_cmd_vel(self, msg):
        # msg.x = Avant/Arrière, msg.y = Gauche/Droite
        pass

    def cb_ptz(self, msg):
        pass

    # Callback arrêt d'urgence
    def cb_emergency_stop(self, msg):
        if msg.data:
            self.get_logger().warn("🛑 ARRÊT D'URGENCE REÇU !")
            self.publish_log("🛑 ARRÊT D'URGENCE ACTIVÉ !", "error")
            
            # Arrête l'enregistrement vidéo si actif
            if self.capture_mgr.video_writer is not None:
                self.capture_mgr.stop_video()
                self.get_logger().info("Enregistrement vidéo arrêté")
                self.publish_log("Enregistrement vidéo arrêté", "warn")
            
            # Publie un arrêt complet sur le topic de commande du robot
            stop_msg = Point()
            stop_msg.x = 0.0  # Vitesse avant/arrière = 0
            stop_msg.y = 0.0  # Vitesse gauche/droite = 0
            stop_msg.z = 0.0  # Rotation = 0
            self.cmd_vel_pub.publish(stop_msg)
            self.get_logger().info("Commande d'arrêt envoyée au robot")
            self.publish_log("Robot arrêté", "warn")
            
            # Publier plusieurs fois pour s'assurer que le message passe
            for _ in range(5):
                self.cmd_vel_pub.publish(stop_msg)

    # Logique de suppression
    def cb_delete_image(self, msg):
        filename = msg.data
        # Sécurité simple pour ne pas sortir du dossier
        if ".." in filename or filename.startswith("/"):
            return
            
        path = os.path.join(self.gallery_dir, filename)
        if os.path.exists(path):
            try:
                os.remove(path)
                self.get_logger().info(f"[DELETE] ✓ Fichier supprimé: {filename}")
                self.publish_log(f"Fichier supprimé: {filename}", "success")
                # Force la mise à jour de la galerie pour le client
                self.gallery_mgr.publish_gallery(reason='file_deleted', log_update=True)
            except Exception as e:
                self.get_logger().error(f"[DELETE] ✗ Erreur suppression: {e}")
                self.publish_log(f"Erreur suppression: {e}", "error")
    
    # Sauvegarde de trajectoire
    def cb_save_trajectory(self, msg):
        try:
            # Décoder le JSON
            traj_data = json.loads(msg.data)
            
            # Récupérer le nom personnalisé ou générer un par défaut
            if 'meta' in traj_data and 'name' in traj_data['meta']:
                custom_name = traj_data['meta']['name']
            else:
                custom_name = "trajectory"
            
            # Nettoyer le nom (sécurité)
            custom_name = custom_name.replace('/', '_').replace('\\', '_')
            
            # Générer le nom de fichier
            filename = f"{custom_name}.json"
            filepath = os.path.join(self.trajectories_dir, filename)
            
            # Vérifier si le fichier existe déjà
            counter = 1
            while os.path.exists(filepath):
                filename = f"{custom_name}_{counter}.json"
                filepath = os.path.join(self.trajectories_dir, filename)
                counter += 1
            
            # Sauvegarder le fichier
            with open(filepath, 'w') as f:
                json.dump(traj_data, f, indent=2)
            
            self.get_logger().info(f"Trajectoire sauvegardée: {filename}")
            self.publish_log(f"Trajectoire sauvegardée: {filename}", "success")
        except Exception as e:
            self.get_logger().error(f"Erreur sauvegarde trajectoire: {e}")
            self.publish_log(f"Erreur sauvegarde trajectoire: {e}", "error")
    
    def publish_trajectory_list(self):
        """Publie la liste des fichiers de trajectoires disponibles"""
        try:
            if not os.path.exists(self.trajectories_dir):
                files = []
            else:
                files = [f for f in os.listdir(self.trajectories_dir) 
                        if f.endswith('.json')]
                files.sort()  # Trier par ordre alphabétique
            
            msg = String()
            msg.data = json.dumps(files)
            self.traj_list_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur publication liste trajectoires: {e}")
    
    def cb_delete_trajectory(self, msg):
        """Supprime un fichier de trajectoire"""
        try:
            filename = msg.data
            
            # Sécurité: vérifier que c'est bien un fichier .json et pas de chemin malveillant
            if not filename.endswith('.json') or '/' in filename or '\\' in filename or '..' in filename:
                self.get_logger().warning(f"Tentative de suppression refusée: {filename}")
                return
            
            filepath = os.path.join(self.trajectories_dir, filename)
            
            if os.path.exists(filepath):
                os.remove(filepath)
                self.get_logger().info(f"Trajectoire supprimée: {filename}")
                self.publish_log(f"Trajectoire supprimée: {filename}", "success")
            else:
                self.get_logger().warning(f"Fichier introuvable: {filename}")
        except Exception as e:
            self.get_logger().error(f"Erreur suppression trajectoire: {e}")
            self.publish_log(f"Erreur suppression trajectoire: {e}", "error")

    def cb_take_photo(self, request, response):
        success, path = self.capture_mgr.take_photo()
        response.success = success
        if success:
            filename = os.path.basename(path)
            response.message = f"Photo enregistrée : {filename}"
            self.get_logger().info(f"[PHOTO] ✓ Photo capturée: {filename}")
            self.gallery_mgr.publish_gallery(reason='local_photo', log_update=True) # Rafraîchir l'IHM
        else:
            response.message = f"Erreur capture : {path}"
            self.get_logger().error(f"[PHOTO] ✗ Erreur capture: {path}")
        
        self.publish_log(response.message, "success" if success else "error")
        return response

    def cb_start_video(self, request, response):
        success, msg = self.capture_mgr.start_video()
        response.success = success
        response.message = msg
        if success:
            self.publish_log("Enregistrement vidéo démarré", "success")
        else:
            self.publish_log(f"Erreur vidéo: {msg}", "error")
        return response

    def cb_stop_video(self, request, response):
        success, msg = self.capture_mgr.stop_video()
        response.success = success
        response.message = msg
        if success:
            self.get_logger().info(f"[VIDEO] ✓ Vidéo arrêtée et sauvegardée")
            self.publish_log(f"Vidéo sauvegardée: {msg}", "success")
            # Mettre à jour la galerie pour afficher la nouvelle vidéo
            self.gallery_mgr.publish_gallery(reason='local_video', log_update=True)
        else:
            self.get_logger().warning(f"[VIDEO] ✗ Erreur arrêt vidéo: {msg}")
            self.publish_log(f"Erreur arrêt vidéo: {msg}", "error")
        return response
    def cb_ptz_relay(self, msg):
        """Relaye les ordres de l'IHM vers le vrai topic /camera/ptz"""
        self.ptz_pub.publish(msg)
        
    def stop_server(self):
        if self.httpd:
            self.httpd.shutdown()
            self.httpd.server_close()

    def cb_auto_scan(self, request, response):
        """Déclenche la séquence de balayage définie dans CaptureManager"""
        # On lance dans un thread pour ne pas bloquer le Node pendant les 15s de scan
        threading.Thread(target=self.capture_mgr.run_auto_scan, daemon=True).start()
        response.success = True
        response.message = "Scan de mission démarré"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WebBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
