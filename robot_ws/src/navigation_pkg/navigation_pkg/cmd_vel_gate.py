#!/usr/bin/env python3
"""Passerelle de commandes vitesse avec verrouillage selon l'etat mission.

Routage:
- Entree teleop: /robot/cmd_vel (geometry_msgs/Twist)
- Sortie robot:  /cmd_vel (geometry_msgs/TwistStamped)

Regle:
- Autorise si mission inactive OU mission en pause.
- Bloque si mission active non pausee.

Etat mission derive depuis les topics UI existants:
- /ui/start_mission (std_msgs/String)
- /ui/cancel_mission (std_msgs/String) avec data: cancel|pause|resume
- /ui/mission_result (std_msgs/String)
"""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

@dataclass
class MissionState:
    active: bool = False
    paused: bool = False

    @property
    def teleop_allowed(self) -> bool:
        return (not self.active) or self.paused


class CmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_gate')

        self.declare_parameter('input_topic', '/robot/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('start_topic', '/ui/start_mission')
        self.declare_parameter('cancel_topic', '/ui/cancel_mission')
        self.declare_parameter('result_topic', '/ui/mission_result')
        self.declare_parameter('stamped_frame_id', 'base_link')
        self.declare_parameter('publish_zero_on_block', True)

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._start_topic = self.get_parameter('start_topic').get_parameter_value().string_value
        self._cancel_topic = self.get_parameter('cancel_topic').get_parameter_value().string_value
        self._result_topic = self.get_parameter('result_topic').get_parameter_value().string_value
        self._stamped_frame_id = (
            self.get_parameter('stamped_frame_id').get_parameter_value().string_value
        )
        self._publish_zero_on_block = (
            self.get_parameter('publish_zero_on_block').get_parameter_value().bool_value
        )

        self._state = MissionState()
        self._was_blocked_last_cmd = False

        self._cmd_pub = self.create_publisher(
            TwistStamped,
            self._output_topic,
            10,
        )

        self.create_subscription(Twist, self._input_topic, self._on_input_cmd, 10)
        self.create_subscription(String, self._start_topic, self._on_start_mission, 10)
        self.create_subscription(String, self._cancel_topic, self._on_cancel_command, 10)
        self.create_subscription(String, self._result_topic, self._on_mission_result, 10)

        self.get_logger().info(
            f'CmdVelGate pret: {self._input_topic} -> {self._output_topic} '
            f'| start={self._start_topic} cancel={self._cancel_topic} result={self._result_topic}'
        )

    def _publish_stamped(self, twist: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._stamped_frame_id
        stamped.twist = twist
        self._cmd_pub.publish(stamped)

    def _on_input_cmd(self, msg: Twist) -> None:
        if self._state.teleop_allowed:
            self._publish_stamped(msg)
            self._was_blocked_last_cmd = False
            return

        if self._publish_zero_on_block and not self._was_blocked_last_cmd:
            zero = Twist()
            self._publish_stamped(zero)
            self._was_blocked_last_cmd = True

    def _on_start_mission(self, _msg: String) -> None:
        self._state.active = True
        self._state.paused = False
        self._log_state('start mission')

    def _on_cancel_command(self, msg: String) -> None:
        command = (msg.data or '').strip().lower()

        if command == 'pause':
            if self._state.active:
                self._state.paused = True
                self._log_state('pause mission')
            else:
                self.get_logger().warn('pause recu sans mission active')
            return

        if command == 'resume':
            if self._state.active:
                self._state.paused = False
                self._log_state('resume mission')
            else:
                self.get_logger().warn('resume recu sans mission active')
            return

        if command == 'cancel':
            self._state.active = False
            self._state.paused = False
            self._log_state('cancel mission')
            return

        self.get_logger().warn(
            f"commande /ui/cancel_mission inconnue: '{command}' (attendu: cancel|pause|resume)"
        )

    def _on_mission_result(self, _msg: String) -> None:
        self._state.active = False
        self._state.paused = False
        self._log_state('mission result')

    def _log_state(self, source: str) -> None:
        allowed = 'YES' if self._state.teleop_allowed else 'NO'
        self.get_logger().info(
            f'[state] source={source} active={self._state.active} '
            f'paused={self._state.paused} teleop_allowed={allowed}'
        )


def main() -> None:
    rclpy.init()
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
