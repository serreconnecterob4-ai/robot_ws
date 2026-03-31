#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
import threading
import time

class WaypointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')

        # Action client Nav2
        self.client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses'
        )

        # Timer pour lancer l'envoi
        self.timer = self.create_timer(2.0, self.send_goals)
        self.sent = False
        self.display_counter_fb = 0

        # Coordonnées des waypoints
        self.coords = [

            (4.680080, 26.815600),
            (9.680080, 26.815600),
            (4.680080, 26.815600)
        ]

        # Bool pour photo : True = prendre photo sur ce waypoint
        self.take_photo = [False, True, False]

        # Pour éviter de prendre plusieurs fois la photo
        self.photo_taken = [False] * len(self.coords)
        self.waiting = False
        self._goal_handle = None  # Handle du goal courant pour l'annulation
        self.start_idx = 0         # Index de départ pour le re-send après photo


    def send_goals(self):
        if self.sent:
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Nav2 pas prêt")
            return

        self.get_logger().info("Nav2 prêt")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.create_waypoints(from_idx=self.start_idx)

        if not goal_msg.poses:
            self.get_logger().info("\033[92m✅ Aucun waypoint restant, mission terminée\033[0m")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Envoi des waypoints depuis le waypoint {self.start_idx}")
        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.sent = True

    def create_waypoints(self, from_idx=0):
        waypoints = []
        for x, y in self.coords[from_idx:]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            waypoints.append(pose)
        return waypoints

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("\033[93mGoal refusé par Nav2\033[0m")
            rclpy.shutdown()
            return

        self._goal_handle = goal_handle
        self.get_logger().info("\033[94mGoal accepté ! Navigation démarrée...\033[0m")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        remaining_sec = fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec * 1e-9
        self.display_counter_fb += 1

        # Affichage tous les 5 messages
        if self.display_counter_fb % 5 == 0:
            print(
                f"\033[96mRobot at ({fb.current_pose.pose.position.x:.2f}, {fb.current_pose.pose.position.y:.2f}) | "
                f"Waypoints restants: {fb.number_of_poses_remaining} | "
                f"Distance restante: {fb.distance_remaining:.2f} m | "
                f"Recoveries: {fb.number_of_recoveries} | "
                f"estimated_time_remaining: {remaining_sec:.2f} s\033[0m"
            )
            self.display_counter_fb = 0

        # Détecter le waypoint courant
        current_idx = len(self.coords) - fb.number_of_poses_remaining - 1
        if 0 <= current_idx < len(self.coords):
            if self.take_photo[current_idx] and not self.photo_taken[current_idx]:
                # Si le point est marqué pour prendre une photo et qu'on ne l'a pas encore prise
                self.get_logger().info(f"📸 Prendre photo au waypoint {current_idx}...")
                self.pause_and_take_photo(current_idx)

    def pause_and_take_photo(self, idx):
        self.photo_taken[idx] = True
        self.start_idx = idx + 1  # Reprend au waypoint suivant après la photo
        self.waiting = True
        # Exécution dans un thread pour ne pas bloquer le spin ROS2
        threading.Thread(target=self._photo_thread, args=(idx,), daemon=True).start()

    def _photo_thread(self, idx):
        self.get_logger().info("\033[93mAnnulation du goal en cours...\033[0m")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            time.sleep(1.0)  # Laisse Nav2 traiter l'annulation

        # === Appel caméra ici ===
        self.get_logger().info(f"\033[93m📸 Photo prise au waypoint {idx} !\033[0m")

        self.get_logger().info("\033[93mAttente de 10 secondes avant de reprendre...\033[0m")
        time.sleep(10.0)

        self.get_logger().info("\033[93mRenvoi du goal après la pause...\033[0m")
        self.waiting = False
        self.sent = False  # Déclenche send_goals() au prochain tick du timer

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("\033[92m✅ Tous les waypoints atteints avec succès !\033[0m")

        elif status == GoalStatus.STATUS_ABORTED:
            print("\033[93m⚠ Waypoint impossible à atteindre "
                  "(dans un obstacle ou hors de la map)\033[0m")

        elif status == GoalStatus.STATUS_CANCELED:
            if self.waiting:
                # Annulation volontaire pour prise de photo — le thread relancera le goal
                return
            print("\033[93mNavigation annulée par l'utilisateur\033[0m")

        else:
            print(f"\033[91mNavigation échouée, status code {status}\033[0m")

        rclpy.shutdown()


def main():
    rclpy.init()
    node = WaypointClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()