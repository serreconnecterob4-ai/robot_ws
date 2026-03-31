#!/usr/bin/env python3
import math
import threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GpsOdomImuVisualizer(Node):
    def __init__(self):
        super().__init__('gps_odom_imu_visualizer')

        self.create_subscription(NavSatFix, '/gps/fix', self.on_gps, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_filtered_odom, 10)
        self.create_subscription(Odometry, '/odometry/local', self.on_local_odom, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 10)

        self._lock = threading.Lock()

        self._ref_lat: Optional[float] = None
        self._ref_lon: Optional[float] = None
        self._m_per_deg_lat: Optional[float] = None
        self._m_per_deg_lon: Optional[float] = None

        self._gps_xy: Optional[Tuple[float, float]] = None
        self._odom_xy: Optional[Tuple[float, float]] = None
        self._filtered_odom_xy: Optional[Tuple[float, float]] = None
        self._local_odom_xy: Optional[Tuple[float, float]] = None
        self._imu_yaw: Optional[float] = None

        self._gps_path: List[Tuple[float, float]] = []
        self._odom_path: List[Tuple[float, float]] = []
        self._filtered_odom_path: List[Tuple[float, float]] = []
        self._local_odom_path: List[Tuple[float, float]] = []
        self._max_path = 2000

    def on_gps(self, msg: NavSatFix) -> None:
        with self._lock:
            if self._ref_lat is None or self._ref_lon is None:
                self._ref_lat = float(msg.latitude)
                self._ref_lon = float(msg.longitude)
                self._m_per_deg_lat = 111320.0
                self._m_per_deg_lon = 40075000.0 * math.cos(math.radians(self._ref_lat)) / 360.0
                self.get_logger().info(
                    f"GPS ref set: lat={self._ref_lat:.6f}, lon={self._ref_lon:.6f}"
                )

            if self._m_per_deg_lat is None or self._m_per_deg_lon is None:
                return

            x = (float(msg.longitude) - self._ref_lon) * self._m_per_deg_lon
            y = (float(msg.latitude) - self._ref_lat) * self._m_per_deg_lat
            self._gps_xy = (x, y)
            self._gps_path.append((x, y))
            if len(self._gps_path) > self._max_path:
                self._gps_path = self._gps_path[-self._max_path :]

    def on_odom(self, msg: Odometry) -> None:
        with self._lock:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            self._odom_xy = (x, y)
            self._odom_path.append((x, y))
            if len(self._odom_path) > self._max_path:
                self._odom_path = self._odom_path[-self._max_path :]

    def on_filtered_odom(self, msg: Odometry) -> None:
        with self._lock:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            self._filtered_odom_xy = (x, y)
            self._filtered_odom_path.append((x, y))
            if len(self._filtered_odom_path) > self._max_path:
                self._filtered_odom_path = self._filtered_odom_path[-self._max_path :]

    def on_local_odom(self, msg: Odometry) -> None:
        with self._lock:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            self._local_odom_xy = (x, y)
            self._local_odom_path.append((x, y))
            if len(self._local_odom_path) > self._max_path:
                self._local_odom_path = self._local_odom_path[-self._max_path :]

    def on_imu(self, msg: Imu) -> None:
        with self._lock:
            q = msg.orientation
            self._imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def get_snapshot(self):
        with self._lock:
            gps_xy = self._gps_xy
            odom_xy = self._odom_xy
            filtered_odom_xy = self._filtered_odom_xy
            local_odom_xy = self._local_odom_xy
            imu_yaw = self._imu_yaw
            gps_path = list(self._gps_path)
            odom_path = list(self._odom_path)
            filtered_odom_path = list(self._filtered_odom_path)
            local_odom_path = list(self._local_odom_path)
        return gps_xy, odom_xy, filtered_odom_xy, local_odom_xy, imu_yaw, gps_path, odom_path, filtered_odom_path, local_odom_path


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomImuVisualizer()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title('GPS (x,y) / Odom / IMU')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

    gps_line, = ax.plot([], [], 'b-', linewidth=1.5, label='GPS path')
    odom_line, = ax.plot([], [], 'g-', linewidth=1.5, label='Odom path')
    filtered_odom_line, = ax.plot([], [], 'r-', linewidth=1.5, label='Filtered Odom')
    local_odom_line, = ax.plot([], [], 'm-', linewidth=1.5, label='Local Odom')
    gps_point, = ax.plot([], [], 'bo', label='GPS')
    odom_point, = ax.plot([], [], 'go', label='Odom')
    filtered_odom_point, = ax.plot([], [], 'ro', label='Filtered')
    local_odom_point, = ax.plot([], [], 'mo', label='Local')
    imu_arrow = ax.quiver([], [], [], [], angles='xy', scale_units='xy', scale=1.0, color='r')

    text_box = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top')

    ax.legend(loc='upper right')

    def update(_):
        gps_xy, odom_xy, filtered_odom_xy, local_odom_xy, imu_yaw, gps_path, odom_path, filtered_odom_path, local_odom_path = node.get_snapshot()

        if gps_path:
            xs, ys = zip(*gps_path)
            gps_line.set_data(xs, ys)
        if odom_path:
            xs, ys = zip(*odom_path)
            odom_line.set_data(xs, ys)
        if filtered_odom_path:
            xs, ys = zip(*filtered_odom_path)
            filtered_odom_line.set_data(xs, ys)
        if local_odom_path:
            xs, ys = zip(*local_odom_path)
            local_odom_line.set_data(xs, ys)

        if gps_xy is not None:
            gps_point.set_data([gps_xy[0]], [gps_xy[1]])
        if odom_xy is not None:
            odom_point.set_data([odom_xy[0]], [odom_xy[1]])
        if filtered_odom_xy is not None:
            filtered_odom_point.set_data([filtered_odom_xy[0]], [filtered_odom_xy[1]])
        if local_odom_xy is not None:
            local_odom_point.set_data([local_odom_xy[0]], [local_odom_xy[1]])

        if imu_yaw is not None and odom_xy is not None:
            dx = math.cos(imu_yaw)
            dy = math.sin(imu_yaw)
            imu_arrow.set_offsets([odom_xy])
            imu_arrow.set_UVC([dx], [dy])

        gps_str = f"GPS x,y: {gps_xy[0]:.2f}, {gps_xy[1]:.2f}" if gps_xy else "GPS x,y: --"
        odom_str = f"Odom x,y: {odom_xy[0]:.2f}, {odom_xy[1]:.2f}" if odom_xy else "Odom x,y: --"
        filtered_str = f"Filtered x,y: {filtered_odom_xy[0]:.2f}, {filtered_odom_xy[1]:.2f}" if filtered_odom_xy else "Filtered x,y: --"
        local_str = f"Local x,y: {local_odom_xy[0]:.2f}, {local_odom_xy[1]:.2f}" if local_odom_xy else "Local x,y: --"
        imu_str = f"IMU yaw: {math.degrees(imu_yaw):.1f}°" if imu_yaw is not None else "IMU yaw: --"
        text_box.set_text(gps_str + "\n" + odom_str + "\n" + filtered_str + "\n" + local_str + "\n" + imu_str)

        return gps_line, odom_line, filtered_odom_line, local_odom_line, gps_point, odom_point, filtered_odom_point, local_odom_point, imu_arrow, text_box

    ani = FuncAnimation(fig, update, interval=100)

    def on_close(_):
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    fig.canvas.mpl_connect('close_event', on_close)

    plt.show()


if __name__ == '__main__':
    main()
