#!/usr/bin/env python3
"""
Générateur simple de données GPS/IMU/Odom avec bruit
Trajectoire circulaire basique pour tester robot_localization
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from transforms3d.euler import euler2quat
import random
import math
import time
from tf2_ros import TransformBroadcaster

class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        
        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer 10Hz
        self.timer = self.create_timer(0.1, self.tick)
        
        # Compteur pour GPS à 2Hz (publié tous les 5 ticks)
        self.gps_tick_count = 0

        # Trajectoire simple: cercle 5m rayon, 0.5 m/s
        self.radius = 5.0
        self.speed = 0.5
        self.omega = self.speed / self.radius
        self.t = 0.0

        # GPS proche origine pour éviter problèmes
        self.lat0 = 0.001  # ~111m nord équateur 
        self.lon0 = 0.001  # ~111m est méridien
        self.m_per_deg = 111320.0  # conversion simple
        
        # Bruits simples
        self.gps_noise = 0.2    # 20cm GPS
        self.imu_noise = 0.005   # IMU précis
        self.odom_noise = 0.21   # 1cm odom
        self.odom_scale = 1.005  # 0.5% erreur échelle

        self.last_time = time.time()
        self.get_logger().info(f"Fake Robot - Cercle R={self.radius}m, V={self.speed}m/s")
        
    def tick(self):
        now = self.get_clock().now().to_msg()
        dt = time.time() - self.last_time
        self.last_time = time.time()
        self.t += dt
        
        # Incrémenter compteur GPS
        self.gps_tick_count += 1
        
        # TF base_link <-> capteurs
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "imu_link"
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([tf])
        
        # Trajectoire circulaire centrée origine
        angle = self.omega * self.t
        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle) 
        yaw = angle + math.pi/2
        vx = -self.radius * self.omega * math.sin(angle)
        vy = self.radius * self.omega * math.cos(angle)
        
        # GPS à 2Hz (tous les 5 ticks)
        if self.gps_tick_count >= 15:
            self.gps_tick_count = 0
            gps = NavSatFix()
            gps.header.stamp = now
            gps.header.frame_id = "base_link"
            gps.status.status = NavSatStatus.STATUS_FIX
            gps_x = x + random.gauss(0, self.gps_noise)
            gps_y = y + random.gauss(0, self.gps_noise)
            gps.latitude = self.lat0 + gps_y / self.m_per_deg
            gps.longitude = self.lon0 + gps_x / self.m_per_deg
            gps.altitude = 0.0
            gps.position_covariance = [self.gps_noise**2] * 3 + [0] * 6
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.gps_pub.publish(gps)
        
        # IMU simple
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        yaw_noisy = yaw + random.gauss(0, self.imu_noise)
        q = euler2quat(0, 0, yaw_noisy)
        imu.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        imu.angular_velocity = Vector3(x=0.0, y=0.0, z=self.omega + random.gauss(0, self.imu_noise))
        # Accélération centripète + bruit
        ax = -self.radius * self.omega**2 * math.cos(angle) + random.gauss(0, 0.02)
        ay = -self.radius * self.omega**2 * math.sin(angle) + random.gauss(0, 0.02)
        imu.linear_acceleration = Vector3(x=ax, y=ay, z=9.81 + random.gauss(0, 0.05))
        # Covariances simples
        imu.orientation_covariance = [self.imu_noise**2] * 3 + [0] * 6
        imu.angular_velocity_covariance = [self.imu_noise**2] * 3 + [0] * 6  
        imu.linear_acceleration_covariance = [0.02**2] * 3 + [0] * 6
        self.imu_pub.publish(imu)
        
        # Odométrie avec erreur d'échelle
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # Position avec bruit et erreur échelle
        odom.pose.pose.position.x = (x + random.gauss(0, self.odom_noise)) * self.odom_scale
        odom.pose.pose.position.y = (y + random.gauss(0, self.odom_noise)) * self.odom_scale
        odom.pose.pose.position.z = 0.0
        q = euler2quat(0, 0, yaw + random.gauss(0, self.imu_noise/2))
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # Vitesses
        odom.twist.twist.linear = Vector3(x=vx*self.odom_scale, y=vy*self.odom_scale, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=self.omega)
        # Covariances simples
        pos_var = (self.odom_noise * self.odom_scale)**2
        odom.pose.covariance = [pos_var,0,0,0,0,0, 0,pos_var,0,0,0,0, 0,0,0.1,0,0,0, 0,0,0,0.1,0,0, 0,0,0,0,0.1,0, 0,0,0,0,0,0.01]
        vel_var = 0.01**2
        odom.twist.covariance = [vel_var,0,0,0,0,0, 0,vel_var,0,0,0,0, 0,0,0.1,0,0,0, 0,0,0,0.1,0,0, 0,0,0,0,0.1,0, 0,0,0,0,0,0.001]
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
