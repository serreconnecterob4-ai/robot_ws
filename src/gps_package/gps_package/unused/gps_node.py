#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import Serial
from pyubx2 import UBXReader
from datetime import datetime, timezone
import threading
import math

PORT = "/dev/ttyACM0"
BAUD = 38400
TIMEOUT = 1


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        try:
            self.ser = Serial(PORT, BAUD, timeout=TIMEOUT)
        except Exception as e:
            self.get_logger().error(f"Erreur ouverture port {PORT}: {e}")
            raise

        self.ubr = UBXReader(self.ser, validate=True)

        self.thread = threading.Thread(target=self.read_gps_loop, daemon=True)
        self.thread.start()

    def read_gps_loop(self):
        while rclpy.ok():
            try:
                raw, parsed = self.ubr.read()
            except Exception as e:
                self.get_logger().error(f"Erreur lecture GPS: {e}")
                continue

            if parsed is None:
                continue

            if parsed.identity != "NAV-PVT":
                continue

            # -------------------------
            # Position
            # -------------------------
            lat = float(parsed.lat)
            lon = float(parsed.lon)
            alt = float(parsed.hMSL)  # déjà en mètres (selon ton hypothèse)

            # -------------------------
            # Qualité du fix
            # -------------------------
            fix_type = int(parsed.fixType)
            num_sv = int(parsed.numSV)

            # -------------------------
            # Précision GNSS (déjà en mètres)
            # -------------------------
            h_acc = float(parsed.hAcc)* 1e-3  # horizontal accuracy (mm)
            v_acc = float(parsed.vAcc)* 1e-3  # vertical accuracy (mm)


            # Sécurité : bornes raisonnables
            h_acc = max(h_acc, 0.01)
            v_acc = max(v_acc, 0.01)

            cov_xy = h_acc ** 2
            cov_z = v_acc ** 2

            msg = NavSatFix()
            msg.header.frame_id = 'gps_link'

            # -------------------------
            # Temps GPS
            # -------------------------
            try:
                dt = datetime(
                    int(parsed.year),
                    int(parsed.month),
                    int(parsed.day),
                    int(parsed.hour),
                    int(parsed.min),
                    int(parsed.sec),
                    tzinfo=timezone.utc
                )
                ts = dt.timestamp()
                sec = int(ts)
                nano = int(getattr(parsed, 'nano', 0))
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nano if nano >= 0 else 0
            except Exception:
                msg.header.stamp = self.get_clock().now().to_msg()

            # -------------------------
            # Remplissage NavSatFix
            # -------------------------
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt

            # Mapping fix u-blox → NavSatStatus
            if fix_type >= 3:
                msg.status.status = NavSatStatus.STATUS_FIX
            else:
                msg.status.status = NavSatStatus.STATUS_NO_FIX

            msg.status.service = NavSatStatus.SERVICE_GPS

            # Covariance dynamique correcte (ENU, m²)
            msg.position_covariance = [
                cov_xy, 0.0,    0.0,
                0.0,    cov_xy, 0.0,
                0.0,    0.0,    cov_z
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.publisher_.publish(msg)

            self.get_logger().info(
                f"GPS fix: lat={lat:.7f} lon={lon:.7f} "
                f"hAcc={h_acc:.2f}m vAcc={v_acc:.2f}m "
                f"sats={num_sv}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()