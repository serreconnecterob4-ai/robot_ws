import json
import errno
import os
import socket
import ssl
import struct
import threading
import base64
import hashlib
from urllib.parse import urlparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String


class MinimalWebSocketClient:
    def __init__(self, url, timeout=2.0):
        self.url = url
        self.timeout = timeout
        self.sock = None
        self.scheme = ''
        self.host = ''
        self.port = 0
        self.path = '/'
        self._parse_url()

    def _parse_url(self):
        parsed = urlparse(self.url)
        if parsed.scheme not in ('ws', 'wss'):
            raise ValueError(f'Unsupported websocket scheme: {parsed.scheme}')

        if not parsed.hostname:
            raise ValueError(f'Invalid websocket URL: {self.url}')

        self.scheme = parsed.scheme
        self.host = parsed.hostname
        self.port = parsed.port or (443 if self.scheme == 'wss' else 80)
        self.path = parsed.path or '/'
        if parsed.query:
            self.path = f'{self.path}?{parsed.query}'

    def connect(self):
        raw_sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        raw_sock.settimeout(self.timeout)

        if self.scheme == 'wss':
            context = ssl.create_default_context()
            self.sock = context.wrap_socket(raw_sock, server_hostname=self.host)
        else:
            self.sock = raw_sock

        self._perform_handshake()

    def _perform_handshake(self):
        if self.sock is None:
            raise RuntimeError('Socket not connected')

        key = base64.b64encode(os.urandom(16)).decode('ascii')
        request = (
            f'GET {self.path} HTTP/1.1\r\n'
            f'Host: {self.host}:{self.port}\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\n'
            'Sec-WebSocket-Version: 13\r\n'
            '\r\n'
        ).encode('ascii')

        self.sock.sendall(request)
        response = self._read_http_response()

        status_ok = response.startswith('HTTP/1.1 101') or response.startswith('HTTP/1.0 101')
        if not status_ok:
            raise RuntimeError(f'WebSocket handshake failed: {response.splitlines()[0] if response else "empty response"}')

        headers = {}
        for line in response.split('\r\n')[1:]:
            if ': ' in line:
                k, v = line.split(': ', 1)
                headers[k.strip().lower()] = v.strip()

        expected = base64.b64encode(
            hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode('ascii')).digest()
        ).decode('ascii')
        if headers.get('sec-websocket-accept') != expected:
            raise RuntimeError('Invalid Sec-WebSocket-Accept from server')

    def _read_http_response(self):
        if self.sock is None:
            raise RuntimeError('Socket not connected')

        data = b''
        while b'\r\n\r\n' not in data:
            chunk = self.sock.recv(1024)
            if not chunk:
                break
            data += chunk
            if len(data) > 65536:
                raise RuntimeError('HTTP response too large during websocket handshake')
        return data.decode('latin1', errors='replace')

    def send_text(self, text):
        if self.sock is None:
            raise RuntimeError('Socket not connected')

        payload = text.encode('utf-8')
        frame = self._build_frame(0x1, payload)
        self.sock.sendall(frame)

    @staticmethod
    def _build_frame(opcode, payload):
        # Client-to-server frames MUST be masked (RFC6455).
        fin_opcode = 0x80 | (opcode & 0x0F)
        mask_bit = 0x80
        length = len(payload)
        header = bytearray([fin_opcode])

        if length <= 125:
            header.append(mask_bit | length)
        elif length <= 0xFFFF:
            header.append(mask_bit | 126)
            header.extend(struct.pack('!H', length))
        else:
            header.append(mask_bit | 127)
            header.extend(struct.pack('!Q', length))

        mask = os.urandom(4)
        header.extend(mask)
        masked_payload = bytes(payload[i] ^ mask[i % 4] for i in range(length))
        return bytes(header) + masked_payload

    def _read_exact(self, nbytes):
        if self.sock is None:
            raise RuntimeError('Socket not connected')

        data = b''
        while len(data) < nbytes:
            chunk = self.sock.recv(nbytes - len(data))
            if not chunk:
                raise RuntimeError('WebSocket closed by peer')
            data += chunk
        return data

    def recv_text(self, timeout=0.01):
        if self.sock is None:
            return None

        self.sock.settimeout(timeout)
        try:
            head = self._read_exact(2)
        except (socket.timeout, BlockingIOError):
            return None
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return None
            raise

        if not head:
            return None

        b1 = head[0]
        b2 = head[1]
        opcode = b1 & 0x0F
        masked = (b2 & 0x80) != 0
        length = b2 & 0x7F

        if length == 126:
            length = struct.unpack('!H', self._read_exact(2))[0]
        elif length == 127:
            length = struct.unpack('!Q', self._read_exact(8))[0]

        mask = self._read_exact(4) if masked else b''
        payload = self._read_exact(length) if length > 0 else b''

        if masked:
            payload = bytes(payload[i] ^ mask[i % 4] for i in range(length))

        if opcode == 0x8:  # close
            raise RuntimeError('WebSocket close frame received')

        if opcode == 0x9:  # ping
            pong = self._build_frame(0xA, payload)
            self.sock.sendall(pong)
            return None

        if opcode == 0xA:  # pong
            return None

        if opcode != 0x1:  # text only
            return None

        return payload.decode('utf-8', errors='replace')

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class OdomRosbridgeRelay(Node):
    def __init__(self):
        super().__init__('odom_rosbridge_relay')

        self.declare_parameter('bridge_host', '100.123.147.56')
        self.declare_parameter('bridge_port', 9090)
        self.declare_parameter('source_topic', '/odometry/filtered')
        self.declare_parameter('target_topic', '/odometry/filtered')
        self.declare_parameter('mission_result_source_topic', '/ui/mission_result')
        self.declare_parameter('mission_result_target_topic', '/ui/mission_result')
        self.declare_parameter('mission_feedback_source_topic', '/ui/mission_feedback')
        self.declare_parameter('mission_feedback_target_topic', '/ui/mission_feedback')
        self.declare_parameter('mission_start_local_topic', '/ui/start_mission')
        self.declare_parameter('mission_start_bridge_topic', '/ui/start_mission')
        self.declare_parameter('mission_cancel_local_topic', '/ui/cancel_mission')
        self.declare_parameter('mission_cancel_bridge_topic', '/ui/cancel_mission')
        self.declare_parameter('reconnect_period_sec', 2.0)

        self.bridge_host = self.get_parameter('bridge_host').get_parameter_value().string_value
        self.bridge_port = self.get_parameter('bridge_port').get_parameter_value().integer_value
        self.source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        self.target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.mission_result_source_topic = self.get_parameter(
            'mission_result_source_topic'
        ).get_parameter_value().string_value
        self.mission_result_target_topic = self.get_parameter(
            'mission_result_target_topic'
        ).get_parameter_value().string_value
        self.mission_feedback_source_topic = self.get_parameter(
            'mission_feedback_source_topic'
        ).get_parameter_value().string_value
        self.mission_feedback_target_topic = self.get_parameter(
            'mission_feedback_target_topic'
        ).get_parameter_value().string_value
        self.mission_start_local_topic = self.get_parameter(
            'mission_start_local_topic'
        ).get_parameter_value().string_value
        self.mission_start_bridge_topic = self.get_parameter(
            'mission_start_bridge_topic'
        ).get_parameter_value().string_value
        self.mission_cancel_local_topic = self.get_parameter(
            'mission_cancel_local_topic'
        ).get_parameter_value().string_value
        self.mission_cancel_bridge_topic = self.get_parameter(
            'mission_cancel_bridge_topic'
        ).get_parameter_value().string_value
        self.reconnect_period_sec = self.get_parameter('reconnect_period_sec').get_parameter_value().double_value

        self.bridge_url = f'ws://{self.bridge_host}:{self.bridge_port}'

        self._ws = None
        self._ws_lock = threading.Lock()
        self._advertised_topics = set()
        self._subscribed_topics = set()

        self._topic_specs = [
            {
                'source': self.source_topic,
                'target': self.target_topic,
                'type': 'nav_msgs/msg/Odometry',
            },
            {
                'source': self.mission_result_source_topic,
                'target': self.mission_result_target_topic,
                'type': 'std_msgs/msg/String',
            },
            {
                'source': self.mission_feedback_source_topic,
                'target': self.mission_feedback_target_topic,
                'type': 'std_msgs/msg/String',
            },
        ]

        self._inbound_specs = [
            {
                'bridge_topic': self.mission_start_bridge_topic,
                'type': 'std_msgs/msg/String',
                'local_publisher': self.create_publisher(String, self.mission_start_local_topic, 20),
            },
            {
                'bridge_topic': self.mission_cancel_bridge_topic,
                'type': 'std_msgs/msg/String',
                'local_publisher': self.create_publisher(String, self.mission_cancel_local_topic, 20),
            },
        ]
        self._inbound_by_topic = {spec['bridge_topic']: spec for spec in self._inbound_specs}

        self.create_subscription(Odometry, self.source_topic, self._on_odometry, 20)
        self.create_subscription(String, self.mission_result_source_topic, self._on_mission_result, 20)
        self.create_subscription(String, self.mission_feedback_source_topic, self._on_mission_feedback, 20)
        self.create_timer(self.reconnect_period_sec, self._ensure_connected)
        self.create_timer(0.05, self._poll_incoming)

        self.get_logger().info(
            'Relay rosbridge actif: '
            f'outbound={[s["target"] for s in self._topic_specs]} '
            f'inbound={[s["bridge_topic"] for s in self._inbound_specs]} '
            f'bridge={self.bridge_url}'
        )

    def _ensure_connected(self):
        if self._ws is not None:
            return

        with self._ws_lock:
            if self._ws is not None:
                return

            try:
                ws = MinimalWebSocketClient(self.bridge_url, timeout=2.0)
                ws.connect()
                self._ws = ws
                self._advertised_topics = set()
                self._subscribed_topics = set()
                self._advertise_all_topics()
                self._subscribe_all_inbound()
                self.get_logger().info(f'Connecte a rosbridge: {self.bridge_url}')
            except Exception as exc:
                self._ws = None
                self.get_logger().warn(f'Connexion rosbridge echouee ({self.bridge_url}): {exc}')

    def _advertise_topic(self, target_topic, ros_type):
        if self._ws is None or target_topic in self._advertised_topics:
            return

        advertise_msg = {
            'op': 'advertise',
            'topic': target_topic,
            'type': ros_type,
        }

        if self._send_json(advertise_msg):
            self._advertised_topics.add(target_topic)

    def _advertise_all_topics(self):
        for spec in self._topic_specs:
            self._advertise_topic(spec['target'], spec['type'])

    def _subscribe_topic(self, bridge_topic, ros_type):
        if self._ws is None or bridge_topic in self._subscribed_topics:
            return

        sub_msg = {
            'op': 'subscribe',
            'topic': bridge_topic,
            'type': ros_type,
            'queue_length': 10,
            'throttle_rate': 0,
        }

        if self._send_json(sub_msg):
            self._subscribed_topics.add(bridge_topic)

    def _subscribe_all_inbound(self):
        for spec in self._inbound_specs:
            self._subscribe_topic(spec['bridge_topic'], spec['type'])

    def _send_json(self, payload):
        if self._ws is None:
            return False

        try:
            self._ws.send_text(json.dumps(payload))
            return True
        except Exception as exc:
            self.get_logger().warn(f'Erreur envoi rosbridge: {exc}')
            self._close_ws()
            return False

    def _poll_incoming(self):
        if self._ws is None:
            return

        # Drain a few frames per tick to keep callbacks short.
        for _ in range(10):
            try:
                text = self._ws.recv_text(timeout=0.01)
            except (socket.timeout, BlockingIOError):
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self.get_logger().warn(f'Erreur reception rosbridge: {exc}')
                self._close_ws()
                return
            except RuntimeError as exc:
                # "close frame" or peer closed connection: reconnect.
                self.get_logger().warn(f'Connexion rosbridge fermee: {exc}')
                self._close_ws()
                return
            except Exception as exc:
                self.get_logger().warn(f'Erreur reception rosbridge: {exc}')
                self._close_ws()
                return

            if text is None:
                return

            self._handle_incoming_message(text)

    def _handle_incoming_message(self, text):
        try:
            payload = json.loads(text)
        except Exception:
            return

        if payload.get('op') != 'publish':
            return

        topic = payload.get('topic', '')
        spec = self._inbound_by_topic.get(topic)
        if spec is None:
            return

        msg_field = payload.get('msg', {})
        if not isinstance(msg_field, dict):
            return

        msg = String()
        msg.data = str(msg_field.get('data', ''))
        spec['local_publisher'].publish(msg)

    def _on_odometry(self, msg):
        self._publish_message(self.target_topic, 'nav_msgs/msg/Odometry', msg)

    def _on_mission_result(self, msg):
        self._publish_message(self.mission_result_target_topic, 'std_msgs/msg/String', msg)

    def _on_mission_feedback(self, msg):
        self._publish_message(self.mission_feedback_target_topic, 'std_msgs/msg/String', msg)

    def _publish_message(self, target_topic, ros_type, msg):
        if self._ws is None:
            return

        if target_topic not in self._advertised_topics:
            self._advertise_topic(target_topic, ros_type)

        publish_msg = {
            'op': 'publish',
            'topic': target_topic,
            'msg': message_to_ordereddict(msg),
        }
        self._send_json(publish_msg)

    def _close_ws(self):
        with self._ws_lock:
            if self._ws is not None:
                try:
                    self._ws.close()
                except Exception:
                    pass
            self._ws = None
            self._advertised_topics = set()
            self._subscribed_topics = set()

    def destroy_node(self):
        self._close_ws()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomRosbridgeRelay()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()