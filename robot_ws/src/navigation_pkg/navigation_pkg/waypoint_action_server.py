#!/usr/bin/env python3
"""
Serveur d'action ROS2 pour la navigation par waypoints.

Ce nœud expose l'action 'navigate_waypoints' (navigation_interfaces/NavigateWaypoints).
Il accepte une liste de coordonnées (x, y) et une liste de booléens take_photo en entrée,
puis délègue la navigation à Nav2 via NavigateThroughPoses.

Comportement clé :
- Si le client distant demande une annulation, le goal est ANNULÉ
  (cancel_callback renvoie CancelResponse.ACCEPT, l'annulation effective est gérée dans execute_callback).
- Pour les arrêts photo, c'est le serveur lui-même qui annule/relance Nav2 en interne.
"""

import asyncio
import threading
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from navigation_interfaces.action import NavigateWaypoints
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from camera.capture_manager import CaptureManager


class _NoOpGalleryManager:
    def publish_gallery(self):
        return


def find_config_file(filename='config_gps.json'):
    """Cherche le fichier de config en remontant les répertoires à partir du workspace."""
    # Commence depuis le répertoire du script
    current = Path(__file__).parent
    
    # Remonte jusqu'à trouver le fichier
    for parent in [current] + list(current.parents):
        config_path = parent / filename
        if config_path.exists():
            return str(config_path)
    
    # Fallback: cherche dans le package share
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('navigation_pkg')
        config_path = os.path.join(pkg_share, 'config', filename)
        if os.path.exists(config_path):
            return config_path
    except Exception as e:
        print(f"Warning: Could not access package share directory: {e}")
    
    raise FileNotFoundError(f"Cannot find {filename} in workspace or package directories")


config_path = find_config_file('config_gps.json')
print(f"Loading config from: {config_path}")

with open(config_path, "r") as f:
    config = json.load(f)
    print(config)
    origin_gps_coordinates_x = float(config["origin_gps_coordinates_x"])
    origin_gps_coordinates_y = float(config["origin_gps_coordinates_y"])

    home_meters_coordinates_x = float(config["home_meters_coordinates_x"])
    home_meters_coordinates_y = float(config["home_meters_coordinates_y"])
    


class WaypointActionServer(Node):

    def __init__(self):
        super().__init__('waypoint_action_server')


        self.declare_parameter('progress_tolerance_m', 1.0) # distance pour considérer un waypoint comme atteint dans le calcul de la progression
        self.declare_parameter('max_planner_no_path', 6) # nombre de feedback "no valid path found" consécutifs du planner avant de conclure à un trajet interdit et d'arrêter la mission
        self.declare_parameter('pause_resume_idle_cmd_sec', 60.0) # durée d'inactivité sur cmd_vel pendant la pause avant reprise auto de la mission
        self.declare_parameter('pause_resume_max_sec', 300.0) # durée de pause avant reprise auto uniquement si aucun cmd_vel n'a été détecté pendant toute la pause



        # region ros2 init
        # ReentrantCallbackGroup requis pour await dans execute_callback
        self._cb_group = ReentrantCallbackGroup()

        # ── Serveur d'action exposé vers l'extérieur ────────────────────────
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        # ── Client Nav2 ──────────────────────────────────────────────────────
        self._nav2_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            callback_group=self._cb_group,
        )

        self._nav2_goto_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )

        self._controller_set_params = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
            callback_group=self._cb_group,
        )
        # endregion

        # region Interface client <-> robot
        # ── Self-client : interface web → action server (même nœud) ─────────
        # Permet d'éviter un nœud bridge séparé : le nœud s'envoie ses propres
        # goals et transmet feedback/résultat sur les topics web.
        self._self_client = ActionClient(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            callback_group=self._cb_group,
        )
        self._web_goal_handle = None
        self._web_goal_pending = False
        self._current_ui_mission_id = ''
        
        # ── Publishers vers le site web ──────────────────────────────────────
        self._ui_feedback_pub = self.create_publisher(
            String, '/ui/mission_feedback', 10
        )
        self._ui_result_pub = self.create_publisher(
            String, '/ui/mission_result', 10
        )

        # ── Subscribers depuis le site web ───────────────────────────────────
        self.create_subscription(
            String,
            '/ui/start_mission',
            self._cb_ui_start,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            String,
            '/ui/cancel_mission',
            self._cb_ui_cancel,
            10,
            callback_group=self._cb_group,
        )

        # Activite teleop: utilise pour reprise auto watchdog pendant pause mission
        self.create_subscription(
            Twist,
            '/robot/cmd_vel',
            self._cb_cmd_vel_input,
            30,
            callback_group=self._cb_group,
        )

        # Écoute des logs ROS pour compter uniquement les échecs planner "no valid path found"
        self.create_subscription(
            Log,
            '/rosout',
            self._cb_rosout,
            50,
            callback_group=self._cb_group,
        )

        # endregion

        # region CaptureManager initalisation  ───────────────────────────────────────────────────
        gallery_path = os.path.expanduser('~/mission_gallery')
        os.makedirs(gallery_path, exist_ok=True)
        # Le serveur mission ne doit pas publier /ui/gallery_files en continu.
        # On garde un objet compatible pour CaptureManager si besoin.
        self.gallery_mgr = _NoOpGalleryManager()
        self._capture_mgr = CaptureManager(node=self, gallery_path=gallery_path)
        # endregion

        # region CONFIGURATION

        self._progress_tolerance_m: float = self.get_parameter('progress_tolerance_m').value
        self._max_planner_no_path: int = self.get_parameter('max_planner_no_path').value
        self._pause_resume_idle_cmd_sec: float = self.get_parameter('pause_resume_idle_cmd_sec').value
        self._pause_resume_max_sec: float = self.get_parameter('pause_resume_max_sec').value

        # endregion

        # region déclaration variables de mission



        self._coords: list[tuple[float, float]] = []
        self._take_photo: list[bool] = []
        self._photo_taken: list[bool] = []
        self._start_idx: int = 0
        self._is_taking_photo: bool = False
        self._photo_stop_idx: int | None = None
        self._current_nav2_gh = None          # goal handle Nav2 en cours
        self._outer_goal_handle = None         # goal handle de l'action client
        self._display_counter: int = 0
        self._last_displayed_wp: tuple[float, float] | None = None
        self._last_nav2_remaining: int | None = None
        self._last_nav2_current_idx: int = -1
        self._validated_progress_idx: int = -1
        self._last_resume_idx_attempted: int = -1
        self._same_resume_idx_count: int = 0
        self._consecutive_nav_failures: int = 0 # compteur de résultats Nav2 consécutifs en échec (abort ou cancel)
        self._planner_no_path_count: int = 0 # compteur de feedback Nav2 "no valid path found" consécutifs, pour bascule anti-boucle et/ou arrêt mission en cas de trajet interdit
        self._force_abort_due_to_no_path: bool = False 
        self._monitor_planner_no_path: bool = False
        self._pause_requested: bool = False
        self._is_paused: bool = False
        self._pause_started_monotonic: float | None = None
        self._last_cmd_input_monotonic: float | None = None
        self._pause_manual_activity_seen: bool = False 
        self._last_pause_feedback_monotonic: float | None = None
        self._last_robot_x: float = 0.0
        self._last_robot_y: float = 0.0
        self._ui_command_dedupe_sec: float = 0.8
        self._last_ui_start_raw: str = ''
        self._last_ui_start_monotonic: float = 0.0
        self._last_ui_cancel_cmd: str = ''
        self._last_ui_cancel_monotonic: float = 0.0
        # endregion
        
        self.get_logger().info('🚀 WaypointActionServer prêt, en attente d\'un goal...')

    # ── Callbacks du serveur d'action ────────────────────────────────────────

    def goal_callback(self, goal_request):
        """Accepte tout nouveau goal."""
        n = len(goal_request.waypoints_x)
        self.get_logger().info(f'Nouveau goal reçu : {n} waypoints')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accepte les demandes d'annulation (notamment depuis le bouton Stop du site web).
        L'annulation effective est gérée dans execute_callback.
        """
        self.get_logger().info('🛑 Demande d\'annulation reçue — acceptée.')
        if self._current_nav2_gh is not None:
            self._current_nav2_gh.cancel_goal_async()   # ← débloque get_result_async()
        return CancelResponse.ACCEPT

    # ── Exécution principale (async) ─────────────────────────────────────────

    async def execute_callback(self, goal_handle):
        """Point d'entrée de la mission.  S'exécute de façon asynchrone."""
        request = goal_handle.request

        # ── Initialisation depuis le goal ────────────────────────────────────
        self._coords = list(zip(request.waypoints_x, request.waypoints_y))
        self._take_photo = list(request.take_photo)

        # Rembourrage si take_photo est plus court que coords
        self._take_photo += [False] * (len(self._coords) - len(self._take_photo))

        self._photo_taken = [False] * len(self._coords)
        self._start_idx = 0
        self._photo_stop_idx = None
        self._current_nav2_gh = None
        self._display_counter = 0
        self._last_displayed_wp = None
        self._last_nav2_remaining = None
        self._last_nav2_current_idx = -1
        self._validated_progress_idx = -1
        self._last_resume_idx_attempted = -1
        self._same_resume_idx_count = 0
        self._consecutive_nav_failures = 0
        self._planner_no_path_count = 0
        self._force_abort_due_to_no_path = False
        self._monitor_planner_no_path = False
        self._pause_requested = False
        self._is_paused = False
        self._pause_started_monotonic = None
        self._last_cmd_input_monotonic = None
        self._pause_manual_activity_seen = False
        self._last_pause_feedback_monotonic = None
        self._outer_goal_handle = goal_handle

        # ── Attente Nav2 ─────────────────────────────────────────────────────
        self.get_logger().info('Attente de Nav2...')
        if not self._nav2_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 non disponible après 15 s')
            goal_handle.abort()
            return self._make_result(False, 'Nav2 indisponible')

        self.get_logger().info('Nav2 prêt ✓')

        # ── Boucle de navigation ─────────────────────────────────────────────
        while self._start_idx < len(self._coords):
            if self._is_paused:
                self.get_logger().warn('Mission en pause (attente resume ou cancel)...')
                while self._is_paused and not goal_handle.is_cancel_requested:
                    now = time.monotonic()

                    if (
                        self._last_pause_feedback_monotonic is None
                        or (now - self._last_pause_feedback_monotonic) >= 0.5
                    ):
                        self._publish_ui_pause_feedback(now)
                        self._last_pause_feedback_monotonic = now

                    if (
                        self._pause_started_monotonic is not None
                        and not self._pause_manual_activity_seen
                        and (now - self._pause_started_monotonic) >= self._pause_resume_max_sec
                    ):
                        self.get_logger().warn(
                            ' Reprise auto mission: pause max atteinte sans activité cmd_vel '
                            f'({self._pause_resume_max_sec:.0f}s).'
                        )
                        self._is_paused = False
                        self._pause_requested = False
                        self._pause_started_monotonic = None
                        self._last_cmd_input_monotonic = None
                        self._pause_manual_activity_seen = False
                        self._last_pause_feedback_monotonic = None
                        break

                    if (
                        self._pause_manual_activity_seen
                        and self._last_cmd_input_monotonic is not None
                        and (now - self._last_cmd_input_monotonic) >= self._pause_resume_idle_cmd_sec
                    ):
                        self.get_logger().warn(
                            ' Reprise auto mission: inactivité cmd_vel '
                            f'({self._pause_resume_idle_cmd_sec:.0f}s).'
                        )
                        self._is_paused = False
                        self._pause_requested = False
                        self._pause_started_monotonic = None
                        self._last_cmd_input_monotonic = None
                        self._pause_manual_activity_seen = False
                        self._last_pause_feedback_monotonic = None
                        break

                    await self._safe_sleep(0.2)

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('🛑 Mission annulée pendant la pause.')
                    goal_handle.canceled()
                    return self._make_result(False, "Mission annulée par l'opérateur")

                self.get_logger().info('▶ Reprise de mission après pause.')

            # Annulation demandée par le web (bouton Stop)
            if goal_handle.is_cancel_requested:
                self.get_logger().info('🛑 Annulation de la mission en cours...')
                if self._current_nav2_gh is not None:
                    await self._current_nav2_gh.cancel_goal_async()
                goal_handle.canceled()
                return self._make_result(False, "Mission annulée par l'opérateur")

            poses = self._create_waypoints(self._start_idx)
            if not poses:
                break

            self.get_logger().info(
                f'Envoi de {len(poses)} waypoints à Nav2 (depuis idx {self._start_idx})'
            )

            nav2_goal = NavigateThroughPoses.Goal()
            nav2_goal.poses = poses

            self._photo_stop_idx = None
            self._current_nav2_gh = None
            self._force_abort_due_to_no_path = False
            self._monitor_planner_no_path = True

            # Envoi du goal à Nav2
            send_future = self._nav2_client.send_goal_async(
                nav2_goal,
                feedback_callback=self._nav2_feedback_callback,
            )
            nav2_gh = await send_future

            if not nav2_gh.accepted:
                self.get_logger().error('Nav2 a refusé le goal')
                goal_handle.abort()
                return self._make_result(False, 'Nav2 a refusé le goal')

            self._current_nav2_gh = nav2_gh

            # Attente du résultat Nav2
            nav2_result = await nav2_gh.get_result_async()
            status = nav2_result.status

            if goal_handle.is_cancel_requested:
                self.get_logger().info('🛑 Nav2 annulé — fin de mission.')
                goal_handle.canceled()
                return self._make_result(False, "Mission annulée par l'opérateur")

            if status == GoalStatus.STATUS_SUCCEEDED:
                self._update_sequential_progress(self._last_robot_x, self._last_robot_y)

                # Garde-fou: Nav2 peut parfois valider trop tôt si la trajectoire
                # recroise la zone du dernier waypoint alors que des poses restent.
                final_x, final_y = self._coords[-1]
                dist_to_final = math.hypot(
                    self._last_robot_x - final_x,
                    self._last_robot_y - final_y,
                )
                all_waypoints_validated = (
                    self._validated_progress_idx >= (len(self._coords) - 1)
                )
                suspicious_success = not all_waypoints_validated

                if suspicious_success:
                    resume_idx = max(self._start_idx, self._validated_progress_idx + 1)
                    resume_idx = min(max(resume_idx, 0), len(self._coords))

                    if resume_idx == self._last_resume_idx_attempted:
                        self._same_resume_idx_count += 1
                    else:
                        self._last_resume_idx_attempted = resume_idx
                        self._same_resume_idx_count = 1

                    self.get_logger().warn(
                        '⚠ Succès Nav2 prématuré suspecté '
                        f'(validated_idx={self._validated_progress_idx}, '
                        f'remaining={self._last_nav2_remaining}, '
                        f'dist_final={dist_to_final:.2f} m, '
                        f'retry_same_idx={self._same_resume_idx_count}). '
                        f'Reprise mission depuis idx {resume_idx}.'
                    )

                    if self._same_resume_idx_count >= 3 and resume_idx < len(self._coords):
                        self.get_logger().warn(
                            f'🧭 Bascule anti-boucle: waypoint unique idx {resume_idx} '
                            '(NavigateToPose)'
                        )
                        advanced = await self._force_advance_one_waypoint(
                            goal_handle,
                            resume_idx,
                        )
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            return self._make_result(False, "Mission annulée par l'opérateur")
                        if advanced:
                            self._same_resume_idx_count = 0
                            self._last_resume_idx_attempted = -1
                            continue

                    self._start_idx = resume_idx
                    if self._start_idx >= len(self._coords):
                        break
                    await self._safe_sleep(0.2)
                    continue

                self._same_resume_idx_count = 0
                self._last_resume_idx_attempted = -1
                self._consecutive_nav_failures = 0
                self.get_logger().info('\033[92m✅ Tous les waypoints atteints !\033[0m')
                break

            elif status == GoalStatus.STATUS_CANCELED and self._photo_stop_idx is not None:
                idx = self._photo_stop_idx
                self._photo_taken[idx] = True
                self._is_taking_photo = True
                self._start_idx = idx + 1
                self.get_logger().info(f'\033[93m📸 Démarrage du scan au waypoint {idx}...\033[0m')

                # Lancer le scan dans un thread séparé
                import threading
                scan_done = threading.Event()

                def _run_scan():
                    self._capture_mgr.run_auto_scan()
                    scan_done.set()

                threading.Thread(target=_run_scan, daemon=True).start()

                # Boucle feedback + détection annulation pendant le scan (~20s max)
                SCAN_TIMEOUT = 20.0
                elapsed = 0.0
                while not scan_done.is_set() and elapsed < SCAN_TIMEOUT:
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info('🛑 Annulation pendant le scan photo...')
                        self._is_taking_photo = False
                        goal_handle.canceled()
                        return self._make_result(False, "Mission annulée par l'opérateur")

                    if self._outer_goal_handle is not None:
                        photo_fb = NavigateWaypoints.Feedback()
                        photo_fb.current_waypoint_index = max(0, idx)
                        photo_fb.waypoints_remaining = len(self._coords) - self._start_idx
                        photo_fb.distance_remaining = 0.0
                        photo_fb.estimated_time_remaining = 0.0
                        photo_fb.robot_x = self._last_robot_x
                        photo_fb.robot_y = self._last_robot_y
                        photo_fb.is_taking_photo = True
                        try:
                            self._outer_goal_handle.publish_feedback(photo_fb)
                        except Exception as e:
                            self.get_logger().warn(f'Feedback photo non envoyé : {e}')

                    time.sleep(0.5)
                    elapsed += 0.5

                self._is_taking_photo = False
                self.get_logger().info('\033[93m✅ Scan terminé, reprise de la navigation...\033[0m')

            elif status == GoalStatus.STATUS_CANCELED and self._pause_requested:
                self._pause_requested = False
                self._is_paused = True
                self._pause_started_monotonic = time.monotonic()
                self._last_cmd_input_monotonic = None
                self._pause_manual_activity_seen = False
                self._last_pause_feedback_monotonic = None
                self.get_logger().warn(
                    '⏸ Nav2 annulé pour pause opérateur. Mission en pause indéfinie.'
                )
                continue

            elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                self._monitor_planner_no_path = False
                if self._force_abort_due_to_no_path:
                    self.get_logger().warn(
                        '⚠ Mission stoppée après seuil planner no valid path found: '
                        'pas de chemin trouvé (trajet interdit).'
                    )
                    goal_handle.abort()
                    return self._make_result(
                        False,
                        'Mission non réussie: pas de chemin trouve (trajet interdit)'
                    )
                else:
                    self._consecutive_nav_failures += 1
                self.get_logger().warn(
                    f'⚠  Navigation interrompue (statut={status}) '
                    f'[{self._consecutive_nav_failures}/3]'
                )

                if self._consecutive_nav_failures >= 3:
                    self.get_logger().warn(
                        '⚠  3 échecs Nav2 consécutifs — abandon de mission et retour maison.'
                    )

                    MAX_RETRIES = 3
                    reached_home = False

                    for attempt in range(1, MAX_RETRIES + 1):
                        self.get_logger().warn(f'🏠 Tentative de repli {attempt}/{MAX_RETRIES}...')
                        reached_home = await self._go_to_home(goal_handle)

                        if goal_handle.is_cancel_requested:
                            return self._make_result(False, "Mission annulée durant le repli")

                        if reached_home:
                            self.get_logger().info(f'🏠 Repli réussi à la tentative {attempt}')
                            break

                        if attempt < MAX_RETRIES:
                            self.get_logger().warn(
                                f'🏠 Repli échoué (tentative {attempt}) — nouvelle tentative dans 3 s...'
                            )
                            time.sleep(3.0)

                    if not reached_home:
                        self.get_logger().fatal(
                            '🚨 APPEL AU SECOURS — Robot bloqué, impossible de rentrer à (0,0) '
                            'après 3 tentatives ! Intervention humaine requise.'
                        )

                    goal_handle.abort()
                    msg = (
                        'Mission arrêtée après 3 échecs de planification — robot de retour en (0,0)'
                        if reached_home else
                        'Mission arrêtée après 3 échecs de planification — repli impossible'
                    )
                    return self._make_result(False, msg)

                escaped = await self._reverse_unstuck_to_goal(goal_handle, 2.0)

                if goal_handle.is_cancel_requested:
                    return self._make_result(False, "Mission annulée par l'opérateur")

                if escaped:
                    self._consecutive_nav_failures = 0
                    prev_start_idx = self._start_idx
                    # Si le degagement a effectivement fait progresser le robot,
                    # reprendre directement au waypoint suivant valide.
                    if self._validated_progress_idx >= prev_start_idx:
                        self._start_idx = self._validated_progress_idx + 1
                        self.get_logger().info(
                            '🛟 Dégagement réussi avec progression: '
                            f'reprise depuis idx {self._start_idx} '
                            f'(validé={self._validated_progress_idx}).'
                        )
                    self.get_logger().info(
                        '🛟 Dégagement réussi, pause courte puis reprise sans marche arrière.'
                    )
                    await self._safe_sleep(2.0)
                    continue

                self.get_logger().warn(
                    '🛟 Dégagement non concluant — déclenchement du repli maison.'
                )

                MAX_RETRIES = 3
                reached_home = False

                for attempt in range(1, MAX_RETRIES + 1):
                    self.get_logger().warn(f'🏠 Tentative de repli {attempt}/{MAX_RETRIES}...')
                    reached_home = await self._go_to_home(goal_handle)

                    # Annulation opérateur pendant le repli — _go_to_home a déjà géré le cancel
                    if goal_handle.is_cancel_requested:
                        return self._make_result(False, "Mission annulée durant le repli")

                    if reached_home:
                        self.get_logger().info(f'🏠 Repli réussi à la tentative {attempt}')
                        break

                    if attempt < MAX_RETRIES:
                        self.get_logger().warn(
                            f'🏠 Repli échoué (tentative {attempt}) — nouvelle tentative dans 3 s...'
                        )
                        time.sleep(3.0)  # petite pause avant de réessayer

                if not reached_home:
                    self.get_logger().fatal(
                        '🚨 APPEL AU SECOURS — Robot bloqué, impossible de rentrer à (0,0) '
                        'après 3 tentatives ! Intervention humaine requise.'
                    )

                goal_handle.abort()
                msg = (
                    'Mission échouée — robot de retour en (0,0)'
                    if reached_home else
                    'Mission échouée — ROBOT BLOQUÉ, repli impossible après 3 tentatives'
                )
                return self._make_result(False, msg)

            else:
                self.get_logger().warn(f'Statut Nav2 inattendu : {status}')
                break

        # ── Mission terminée ─────────────────────────────────────────────────
        self.get_logger().info('\033[92m✅ Mission complète !\033[0m')
        goal_handle.succeed()
        return self._make_result(True, 'Tous les waypoints atteints avec succès')

    async def _go_to_home(self, goal_handle) -> bool:
        """
        Envoie le robot vers (0, 0) via NavigateToPose.
        Retourne True si arrivé, False si annulé/échoué.
        """
        self.get_logger().warn('🏠 Comportement de repli : retour à (0, 0)...')

        if not self._nav2_goto_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose non disponible pour le repli')
            return False

        home_goal = NavigateToPose.Goal()
        home_goal.pose = self._make_pose(home_meters_coordinates_x, home_meters_coordinates_y)

        send_future = self._nav2_goto_client.send_goal_async(home_goal)
        home_gh = await send_future

        if not home_gh.accepted:
            self.get_logger().error('Goal de repli refusé par Nav2')
            return False

        # Pendant le repli, on surveille quand même une annulation opérateur
        result_future = home_gh.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('🛑 Annulation pendant le repli')
                await home_gh.cancel_goal_async()
                goal_handle.canceled()
                return False
            time.sleep(0.2)  # libère l'event loop

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'🏠 Repli réussi : robot en ({home_meters_coordinates_x}, {home_meters_coordinates_y})')
            return True
        else:
            self.get_logger().warn(f'🏠 Repli échoué (statut Nav2 : {status})')
            return False

    async def _force_advance_one_waypoint(self, goal_handle, idx: int) -> bool:
        """
        Anti-boucle: envoie uniquement le prochain waypoint auquel aller via NavigateToPose.
        Si Nav2 le valide, on force l'avancement séquentiel à idx+1.
        --> le but est d'éviter que Nav2 pense que le robot a fini son trajet alors qu'il est juste revenu dans la zone du dernier waypoint, 
        ce qui peut arriver si les waypoints sont proches. Or aucun paramètre de nav2 waypoint_follower n'existe pour corriger ça
        """
        if idx < 0 or idx >= len(self._coords):
            return False

        if not self._nav2_goto_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose indisponible pour anti-boucle')
            return False

        x, y = self._coords[idx]
        single_goal = NavigateToPose.Goal()
        single_goal.pose = self._make_pose(x, y)

        send_future = self._nav2_goto_client.send_goal_async(single_goal)
        single_gh = await send_future
        if not single_gh.accepted:
            self.get_logger().error(f'Goal anti-boucle refusé (idx {idx})')
            return False

        self._current_nav2_gh = single_gh
        result_future = single_gh.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await single_gh.cancel_goal_async()
                return False
            await self._safe_sleep(0.2)

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Anti-boucle échoué pour idx {idx} (statut={status})'
            )
            return False

        self._validated_progress_idx = max(self._validated_progress_idx, idx)
        self._start_idx = idx + 1
        self.get_logger().info(
            f'🧭 Anti-boucle validé: progression forcée au waypoint {self._validated_progress_idx}'
        )
        return True

    def _update_sequential_progress(self, robot_x: float, robot_y: float) -> None:
        """
        Valide l'avancement uniquement dans l'ordre des waypoints.
        Un waypoint est validé si le robot entre dans la tolérance interne.
        """
        next_idx = self._validated_progress_idx + 1
        progressed = False

        while 0 <= next_idx < len(self._coords):
            wx, wy = self._coords[next_idx]
            if math.hypot(robot_x - wx, robot_y - wy) <= self._progress_tolerance_m:
                self._validated_progress_idx = next_idx
                progressed = True
                next_idx += 1
            else:
                break

        if progressed:
            self.get_logger().info(
                f'📍 Progression validée jusqu\'au waypoint {self._validated_progress_idx}'
            )

    async def _set_allow_reversing(self, enabled: bool) -> bool:
        """Active/désactive dynamiquement FollowPath.allow_reversing dans Nav2."""
        if not self._controller_set_params.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                'Service /controller_server/set_parameters indisponible'
            )
            return False

        req = SetParameters.Request()
        param = Parameter()
        param.name = 'FollowPath.allow_reversing'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_BOOL,
            bool_value=enabled,
        )
        req.parameters = [param]

        try:
            response = await self._controller_set_params.call_async(req)
        except Exception as exc:
            self.get_logger().error(
                f'Échec appel set_parameters allow_reversing={enabled} : {exc}'
            )
            return False

        if not response.results:
            self.get_logger().error('Réponse vide de set_parameters')
            return False

        result = response.results[0]
        if not result.successful:
            reason = result.reason if result.reason else 'raison inconnue'
            self.get_logger().error(
                f'Nav2 a refusé allow_reversing={enabled} : {reason}'
            )
            return False

        self.get_logger().info(f'allow_reversing basculé à {enabled}')
        return True

    async def _reverse_unstuck_to_goal(self, goal_handle, escape_distance: float) -> bool:
        """
        Active temporairement la marche arrière et retente le segment vers le goal.
        On coupe ce mode dès que le robot s'est écarté de escape_distance (m),
        puis on repassera en marche avant uniquement.
        """
        self.get_logger().warn('\033[91m🛟 Tentative de dégagement en marche arrière...\033[0m')
        if not await self._set_allow_reversing(True):
            return False

        try:
            if not self._nav2_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('NavigateThroughPoses non disponible pour le dégagement')
                return False

            start_x = self._last_robot_x
            start_y = self._last_robot_y
            travelled = 0.0

            poses = self._create_waypoints(self._start_idx)
            if not poses:
                self.get_logger().warn('Aucun waypoint restant pour le dégagement')
                return False

            def _feedback_cb(feedback_msg):
                nonlocal travelled
                self._nav2_feedback_callback(feedback_msg)
                fb = feedback_msg.feedback
                x = fb.current_pose.pose.position.x
                y = fb.current_pose.pose.position.y
                self._last_robot_x = x
                self._last_robot_y = y
                travelled = math.hypot(x - start_x, y - start_y)

            unstuck_goal = NavigateThroughPoses.Goal()
            unstuck_goal.poses = poses

            send_future = self._nav2_client.send_goal_async(
                unstuck_goal,
                feedback_callback=_feedback_cb,
            )
            escape_gh = await send_future

            if not escape_gh.accepted:
                self.get_logger().error('Goal de dégagement refusé par Nav2')
                return False

            self._current_nav2_gh = escape_gh

            result_future = escape_gh.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('🛑 Annulation pendant le dégagement')
                    await escape_gh.cancel_goal_async()
                    goal_handle.canceled()
                    return False

                if travelled >= escape_distance:
                    self.get_logger().warn(
                        f'🛟 Distance de dégagement atteinte ({travelled:.2f} m), '
                        'arrêt du mode marche arrière.'
                    )
                    await escape_gh.cancel_goal_async()
                    return True

                await self._safe_sleep(0.2)

            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('🛟 Dégagement terminé : objectif atteint avant 20 m.')
                return True

            if status == GoalStatus.STATUS_CANCELED and travelled >= escape_distance:
                return True

            self.get_logger().warn(f'🛟 Dégagement échoué (statut Nav2 : {status})')
            return False

        finally:
            await self._set_allow_reversing(False)

    # ── Feedback Nav2 --> retransmis au site web ─────────────────────────

    def _nav2_feedback_callback(self, feedback_msg):
        """
        Reçoit le feedback Nav2 :
          - Retransmet les informations à l'action client externe.
          - Détecte les waypoints nécessitant une photo et déclenche l'arrêt interne.
        """
        fb = feedback_msg.feedback
        # Mémoriser la position pour la réutiliser pendant la pause photo
        self._last_robot_x = fb.current_pose.pose.position.x
        self._last_robot_y = fb.current_pose.pose.position.y
        remaining_sec = (
            fb.estimated_time_remaining.sec
            + fb.estimated_time_remaining.nanosec * 1e-9
        )
        # Calcul de l'index global du waypoint courant
        current_idx = len(self._coords) - fb.number_of_poses_remaining - 1
        self._last_nav2_remaining = int(fb.number_of_poses_remaining)
        self._last_nav2_current_idx = int(current_idx)

        self._update_sequential_progress(self._last_robot_x, self._last_robot_y)

        # ── Affichage console (1 message sur 30, sauf changement de waypoint) ─
        self._display_counter += 1
        current_wp = None
        if 0 <= current_idx < len(self._coords):
            current_wp = self._coords[current_idx]

        should_log = (self._display_counter % 30 == 0)
        if current_wp is not None and current_wp != self._last_displayed_wp:
            should_log = True

        if should_log:
            self._last_displayed_wp = current_wp
            self.get_logger().info(
                f'\033[96mRobot : ({fb.current_pose.pose.position.x:.2f}, '
                f'{fb.current_pose.pose.position.y:.2f}) | '
                f'Restants : {fb.number_of_poses_remaining} | '
                f'Distance : {fb.distance_remaining:.2f} m | '
                f'ETA : {remaining_sec:.2f} '
                f'Actual: {current_idx} | s\033[0m'
            )
            self._display_counter = 0

        # ── Retransmission du feedback à l'action client externe ─────────────
        if self._outer_goal_handle is not None:
            feedback = NavigateWaypoints.Feedback()
            feedback.current_waypoint_index = current_idx
            feedback.waypoints_remaining = fb.number_of_poses_remaining
            feedback.distance_remaining = fb.distance_remaining
            feedback.estimated_time_remaining = remaining_sec
            feedback.robot_x = fb.current_pose.pose.position.x
            feedback.robot_y = fb.current_pose.pose.position.y
            feedback.is_taking_photo = self._is_taking_photo
            self._outer_goal_handle.publish_feedback(feedback)

        # ── Détection d'un waypoint photo ────────────────────────────────────
        if 0 <= current_idx < len(self._coords):
            if self._take_photo[current_idx] and not self._photo_taken[current_idx]:
                self.get_logger().info(
                    f'📸 Waypoint photo détecté (idx {current_idx}), '
                    'annulation du segment Nav2...'
                )
                self._photo_stop_idx = current_idx
                if self._current_nav2_gh is not None:
                    # Annulation interne : le execute_callback reprendra après
                    self._current_nav2_gh.cancel_goal_async()

    def _cb_rosout(self, msg: Log):
        """Compte strictement les warnings planner 'no valid path found' pendant la mission active."""
        if not self._monitor_planner_no_path:
            return

        if msg.name != 'planner_server':
            return

        text = msg.msg.lower()
        if 'no valid path found' not in text:
            return

        self._planner_no_path_count += 1
        self.get_logger().warn(
            f'🧭 Planner no valid path found: '
            f'{self._planner_no_path_count}/{self._max_planner_no_path}'
        )

        if (
            self._planner_no_path_count >= self._max_planner_no_path
            and not self._force_abort_due_to_no_path
            and self._photo_stop_idx is None
        ):
            self._force_abort_due_to_no_path = True
            self.get_logger().warn(
                '⚠  Seuil planner no valid path found atteint — annulation du goal courant.'
            )
            if self._current_nav2_gh is not None:
                self._current_nav2_gh.cancel_goal_async()

    # ── Interface web (topics /ui/*) ─────────────────────────────────────────

    def _cb_ui_start(self, msg: String):
        """Reçoit un JSON {waypoints_x, waypoints_y, take_photo} et lance la mission."""
        if self._web_goal_pending or self._web_goal_handle is not None:
            self.get_logger().warn(
                'Commande /ui/start_mission ignoree: mission deja en cours (anti-preemption)'
            )
            return

        raw_payload = (msg.data or '').strip()
        now = time.monotonic()
        if (
            raw_payload
            and raw_payload == self._last_ui_start_raw
            and (now - self._last_ui_start_monotonic) < self._ui_command_dedupe_sec
        ):
            self.get_logger().warn('Commande /ui/start_mission dupliquee ignoree (anti-echo)')
            return
        self._last_ui_start_raw = raw_payload
        self._last_ui_start_monotonic = now

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON invalide sur /ui/start_mission : {e}')
            return

        waypoints_x = [float(v) for v in data.get('waypoints_x', [])]
        waypoints_y = [float(v) for v in data.get('waypoints_y', [])]
        take_photo  = [bool(v)  for v in data.get('take_photo',  [])]
        incoming_mission_id = str(data.get('mission_id', '')).strip()

        if not waypoints_x:
            self.get_logger().warn('Aucun waypoint reçu — mission ignorée')
            return

        self.get_logger().info(f'🌐 Mission web reçue : {len(waypoints_x)} waypoints')
        if incoming_mission_id:
            self._current_ui_mission_id = incoming_mission_id
        else:
            self._current_ui_mission_id = f'm_{int(time.time() * 1000)}'

        if not self._self_client.server_is_ready():
            self.get_logger().error('Action server navigate_waypoints non disponible')
            err = String()
            err.data = json.dumps({'success': False, 'message': 'Serveur action indisponible'})
            self._ui_result_pub.publish(err)
            return

        goal = NavigateWaypoints.Goal()
        goal.waypoints_x = waypoints_x
        goal.waypoints_y = waypoints_y
        goal.take_photo  = take_photo

        self._web_goal_pending = True
        future = self._self_client.send_goal_async(
            goal,
            feedback_callback=self._web_feedback_callback,
        )
        future.add_done_callback(self._web_goal_response_callback)

    def _cb_ui_cancel(self, _msg: String):
        """Commande mission web: cancel, pause, ou resume."""
        command = (_msg.data or '').strip().lower()
        now = time.monotonic()
        if (
            command
            and command == self._last_ui_cancel_cmd
            and (now - self._last_ui_cancel_monotonic) < self._ui_command_dedupe_sec
        ):
            self.get_logger().warn(
                f"Commande /ui/cancel_mission dupliquee ignoree (anti-echo): '{command}'"
            )
            return
        self._last_ui_cancel_cmd = command
        self._last_ui_cancel_monotonic = now

        if command == 'pause':
            if self._web_goal_handle is None:
                self.get_logger().warn('Pause demandée mais aucune mission active')
                return

            if self._is_paused:
                self.get_logger().warn('Mission déjà en pause')
                return

            self._pause_requested = True
            self.get_logger().info('⏸ Pause web demandée')
            if self._current_nav2_gh is not None:
                self._current_nav2_gh.cancel_goal_async()
            else:
                # Pas de goal Nav2 actif: pause immédiate côté serveur mission.
                self._pause_requested = False
                self._is_paused = True
                self._pause_started_monotonic = time.monotonic()
                self._last_cmd_input_monotonic = None
                self._pause_manual_activity_seen = False
                self._last_pause_feedback_monotonic = None
                self.get_logger().warn('⏸ Pause mission activée (sans goal Nav2 actif).')
            return

        if command == 'resume':
            if not self._is_paused:
                self.get_logger().warn('Resume demandé mais mission non en pause')
                return
            self._is_paused = False
            self._pause_started_monotonic = None
            self._last_cmd_input_monotonic = None
            self._pause_manual_activity_seen = False
            self._last_pause_feedback_monotonic = None
            self.get_logger().info('▶ Resume web demandé')
            return

        if command != 'cancel':
            self.get_logger().warn(
                f"Commande /ui/cancel_mission inconnue: '{command}' (attendu: cancel|pause|resume)"
            )
            return

        if self._web_goal_handle is None:
            self.get_logger().warn('Annulation web demandée mais aucune mission active')
            return
        self.get_logger().info('🛑 Annulation web demandée')
        self._web_goal_handle.cancel_goal_async()

    def _cb_cmd_vel_input(self, _msg: Twist):
        """Marque l'activité cmd_vel pendant une pause pour le watchdog de reprise auto."""
        if not self._is_paused:
            return

        self._last_cmd_input_monotonic = time.monotonic()
        self._pause_manual_activity_seen = True

    def _publish_ui_pause_feedback(self, now_monotonic: float) -> None:
        """Publie un feedback UI spécifique à la pause, avec compte à rebours watchdog."""
        if self._pause_manual_activity_seen and self._last_cmd_input_monotonic is not None:
            remaining = max(
                0.0,
                self._pause_resume_idle_cmd_sec - (now_monotonic - self._last_cmd_input_monotonic),
            )
            source = 'idle_cmd'
        elif self._pause_started_monotonic is not None:
            remaining = max(
                0.0,
                self._pause_resume_max_sec - (now_monotonic - self._pause_started_monotonic),
            )
            source = 'no_cmd'
        else:
            remaining = float(self._pause_resume_max_sec)
            source = 'no_cmd'

        current_idx = self._last_nav2_current_idx if self._last_nav2_current_idx >= 0 else 0
        payload = {
            'current_waypoint_index': int(current_idx),
            'waypoints_remaining': max(0, len(self._coords) - self._start_idx),
            'distance_remaining': 0.0,
            'robot_x': float(self._last_robot_x),
            'robot_y': float(self._last_robot_y),
            'is_taking_photo': False,
            'is_paused': True,
            'pause_seconds_remaining': float(remaining),
            'pause_watchdog_source': source,
            'mission_id': self._current_ui_mission_id,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._ui_feedback_pub.publish(msg)

    def _web_goal_response_callback(self, future):
        self._web_goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Erreur envoi goal mission web: {exc}')
            err = String()
            err.data = json.dumps({'success': False, 'message': 'Erreur envoi goal mission'})
            self._ui_result_pub.publish(err)
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal refusé par navigate_waypoints')
            err = String()
            err.data = json.dumps({'success': False, 'message': 'Goal refusé par le serveur'})
            self._ui_result_pub.publish(err)
            return
        self._web_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._web_result_callback)

    def _web_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        payload = {
            'current_waypoint_index':   fb.current_waypoint_index,
            'waypoints_remaining':     fb.waypoints_remaining,
            'distance_remaining':       float(fb.distance_remaining),
            'robot_x':                  float(fb.robot_x),
            'robot_y':                  float(fb.robot_y),
            'is_taking_photo':          bool(fb.is_taking_photo),
            'is_paused':                bool(self._is_paused),
            'pause_seconds_remaining':  None,
            'pause_watchdog_source':    None,
            'mission_id':               self._current_ui_mission_id,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._ui_feedback_pub.publish(msg)

    def _web_result_callback(self, future):
        self._web_goal_pending = False
        self._web_goal_handle = None
        result = future.result().result
        msg = String()
        msg.data = json.dumps({
            'success': result.success,
            'message': result.message,
            'mission_id': self._current_ui_mission_id,
        })
        self._ui_result_pub.publish(msg)
        self._current_ui_mission_id = ''
        self.get_logger().info(
            f'🌐 Résultat publié sur /ui/mission_result : success={result.success}'
        )

    # ── Utilitaires ──────────────────────────────────────────────────────────

    async def _safe_sleep(self, seconds: float) -> None:
        """
        Pause compatible ROS2: utilise asyncio si une boucle tourne,
        sinon bascule sur un sleep bloquant court.
        """
        try:
            asyncio.get_running_loop()
            await asyncio.sleep(seconds)
        except RuntimeError:
            time.sleep(seconds)

    def _make_result(self, success: bool, message: str) -> NavigateWaypoints.Result:
        """Construit un NavigateWaypoints.Result en une ligne."""
        result = NavigateWaypoints.Result()
        result.success = success
        result.message = message
        return result

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        """Construit un PoseStamped dans le repère 'map'."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def _create_waypoints(self, from_idx: int = 0) -> list[PoseStamped]:
        return [self._make_pose(x, y) for x, y in self._coords[from_idx:]]


# ── Point d'entrée ────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = WaypointActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()