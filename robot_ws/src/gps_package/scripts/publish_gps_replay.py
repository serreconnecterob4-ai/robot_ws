#!/usr/bin/env python3
"""
Rejoue une séquence de points GPS statiques sous forme de NavSatFix sur /gps/fix.
Usage : python3 publish_gps_replay.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import time

GPS_DATA = [
    # (timestamp, latitude, longitude, altitude_mm, fix_type, num_sv, h_acc_m, v_acc_m)
    (1771589481.4917388, 48.8044432, 2.0759291, 130124.0, 3, 32, 0.014, 0.01),
    (1771589481.5917459, 48.8044432, 2.0759291, 130127.0, 3, 32, 0.014, 0.01),
    (1771589481.6899927, 48.8044431, 2.0759291, 130128.0, 3, 32, 0.014, 0.01),
    (1771589481.7877216, 48.8044431, 2.0759291, 130126.0, 3, 32, 0.014, 0.01),
    (1771589481.8921063, 48.8044431, 2.0759291, 130124.0, 3, 32, 0.014, 0.01),
    (1771589481.9930222, 48.8044431, 2.0759291, 130126.0, 3, 32, 0.014, 0.01),
    (1771589482.1022613, 48.8044431, 2.0759291, 130127.0, 3, 32, 0.014, 0.01),
    (1771589482.2917717, 48.8044431, 2.0759292, 130114.0, 3, 32, 0.014, 0.01),
    (1771589482.397036,  48.8044431, 2.0759292, 130111.0, 3, 32, 0.014, 0.01),
    (1771589482.4888813, 48.8044432, 2.0759292, 130108.0, 3, 32, 0.014, 0.01),
    (1771589482.5910974, 48.8044432, 2.0759292, 130112.0, 3, 32, 0.014, 0.01),
    (1771589482.6857321, 48.8044432, 2.0759291, 130114.0, 3, 32, 0.014, 0.01),
    (1771589482.7906673, 48.8044432, 2.0759292, 130110.0, 3, 32, 0.014, 0.01),
    (1771589482.8917086, 48.8044432, 2.0759292, 130108.0, 3, 32, 0.014, 0.01),
    (1771589483.0345595, 48.8044432, 2.0759292, 130108.0, 3, 32, 0.014, 0.01),
]


def make_navsatfix(ts: float, lat: float, lon: float, alt_mm: float,
                   fix_type: int, h_acc: float, v_acc: float) -> NavSatFix:
    msg = NavSatFix()

    # Header
    sec = int(ts)
    nanosec = int((ts - sec) * 1e9)
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = "gps"

    # Status
    msg.status.status = (NavSatStatus.STATUS_FIX
                         if fix_type >= 2
                         else NavSatStatus.STATUS_NO_FIX)
    msg.status.service = NavSatStatus.SERVICE_GPS

    # Position
    msg.latitude  = lat
    msg.longitude = lon
    msg.altitude  = alt_mm / 1000.0          # mm → m

    # Covariance diagonale (σ² = accuracy²)
    h2 = h_acc ** 2
    v2 = v_acc ** 2
    msg.position_covariance = [
        h2,  0.0, 0.0,
        0.0, h2,  0.0,
        0.0, 0.0, v2,
    ]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

    return msg


class GpsReplayNode(Node):
    def __init__(self):
        super().__init__("gps_replay")
        self.pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.index = 0
        self.t0_data = GPS_DATA[0][0]
        self.t0_wall = time.time()

        # Premier message immédiatement, les suivants calés sur les δt d'origine
        self.timer = self.create_timer(0.0, self._tick)

    def _tick(self):
        self.timer.cancel()

        if self.index >= len(GPS_DATA):
            self.get_logger().info("Replay terminé.")
            rclpy.shutdown()
            return

        row = GPS_DATA[self.index]
        ts, lat, lon, alt, fix, _, h_acc, v_acc = row

        msg = make_navsatfix(ts, lat, lon, alt, fix, h_acc, v_acc)
        self.pub.publish(msg)
        self.get_logger().info(
            f"[{self.index+1}/{len(GPS_DATA)}] "
            f"lat={lat:.7f}  lon={lon:.7f}  alt={alt/1000:.3f} m"
        )

        self.index += 1

        if self.index < len(GPS_DATA):
            next_ts = GPS_DATA[self.index][0]
            delay = next_ts - ts          # δt original entre deux mesures
            self.timer = self.create_timer(delay, self._tick)
        else:
            # Dernier message publié : on attend un court instant puis on quitte
            self.timer = self.create_timer(0.5, self._tick)


def main():
    rclpy.init()
    node = GpsReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
