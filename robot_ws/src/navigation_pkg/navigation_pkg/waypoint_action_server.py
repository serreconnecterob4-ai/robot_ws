#!/usr/bin/env python3
"""
Serveur d'action ROS2 pour la navigation par waypoints.

Ce nœud expose l'action 'navigate_waypoints' (navigation_interfaces/NavigateWaypoints).
Il accepte une liste de coordonnées (x, y) et une liste de booléens take_photo en entrée,
puis délègue la navigation à Nav2 via NavigateThroughPoses.

Comportement clé :
- Si le client distant coupe la connexion ou demande une annulation, le goal est ANNULÉ
  (cancel_callback renvoie CancelResponse.ACCEPT, l'annulation effective est gérée dans execute_callback).
- Pour les arrêts photo, c'est le serveur lui-même qui annule/relance Nav2 en interne.
"""

import asyncio
import threading
import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from navigation_interfaces.action import NavigateWaypoints
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from camera.capture_manager import CaptureManager


class WaypointActionServer(Node):

    def __init__(self):
        super().__init__('waypoint_action_server')

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

        # ── CaptureManager ───────────────────────────────────────────────────
        gallery_path = os.path.expanduser('~/mission_gallery')
        os.makedirs(gallery_path, exist_ok=True)
        self._capture_mgr = CaptureManager(node=self, gallery_path=gallery_path)
        self._loop = asyncio.get_event_loop()

        # ── État de la mission courante (une seule mission à la fois) ────────
        self._coords: list[tuple[float, float]] = []
        self._take_photo: list[bool] = []
        self._photo_taken: list[bool] = []
        self._start_idx: int = 0
        self._is_taking_photo: bool = False
        self._photo_stop_idx: int | None = None
        self._current_nav2_gh = None          # goal handle Nav2 en cours
        self._outer_goal_handle = None         # goal handle de l'action client
        self._display_counter: int = 0
        # Dernière position robot connue (mètres, repère map) – envoyée pendant la pause photo
        self._last_robot_x: float = 0.0
        self._last_robot_y: float = 0.0

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

            elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                self.get_logger().warn(
                    f'⚠  Navigation interrompue (statut={status}) — tentative de dégagement'
                )

                escaped = await self._reverse_unstuck_to_goal(goal_handle, 20.0)

                if goal_handle.is_cancel_requested:
                    return self._make_result(False, "Mission annulée par l'opérateur")

                if escaped:
                    self.get_logger().info(
                        '🛟 Dégagement réussi, pause courte puis reprise sans marche arrière.'
                    )
                    await asyncio.sleep(2.0)
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
        home_goal.pose = self._make_pose(-5.576510, 22.827500)  # légèrement décalé pour éviter les collisions avec les murs

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
            self.get_logger().info('🏠 Repli réussi : robot en (0, 0)')
            return True
        else:
            self.get_logger().warn(f'🏠 Repli échoué (statut Nav2 : {status})')
            return False

    # ── Feedback Nav2 → retransmis au client externe ─────────────────────────

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

        # ── Affichage console (tous les 5 messages) ──────────────────────────
        self._display_counter += 1
        if self._display_counter % 5 == 0:
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

                await asyncio.sleep(0.2)

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

    # ── Interface web (topics /ui/*) ─────────────────────────────────────────

    def _cb_ui_start(self, msg: String):
        """Reçoit un JSON {waypoints_x, waypoints_y, take_photo} et lance la mission."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON invalide sur /ui/start_mission : {e}')
            return

        waypoints_x = [float(v) for v in data.get('waypoints_x', [])]
        waypoints_y = [float(v) for v in data.get('waypoints_y', [])]
        take_photo  = [bool(v)  for v in data.get('take_photo',  [])]

        if not waypoints_x:
            self.get_logger().warn('Aucun waypoint reçu — mission ignorée')
            return

        self.get_logger().info(f'🌐 Mission web reçue : {len(waypoints_x)} waypoints')

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

        future = self._self_client.send_goal_async(
            goal,
            feedback_callback=self._web_feedback_callback,
        )
        future.add_done_callback(self._web_goal_response_callback)

    def _cb_ui_cancel(self, _msg: String):
        """Annule la mission en cours depuis le bouton Stop du site web."""
        if self._web_goal_handle is None:
            self.get_logger().warn('Annulation web demandée mais aucune mission active')
            return
        self.get_logger().info('🛑 Annulation web demandée')
        self._web_goal_handle.cancel_goal_async()

    def _web_goal_response_callback(self, future):
        goal_handle = future.result()
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
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._ui_feedback_pub.publish(msg)

    def _web_result_callback(self, future):
        self._web_goal_handle = None
        result = future.result().result
        msg = String()
        msg.data = json.dumps({'success': result.success, 'message': result.message})
        self._ui_result_pub.publish(msg)
        self.get_logger().info(
            f'🌐 Résultat publié sur /ui/mission_result : success={result.success}'
        )

    # ── Utilitaires ──────────────────────────────────────────────────────────

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