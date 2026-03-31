#!/usr/bin/env python3
"""
Script de test pour vérifier les distances et coordonnées dans le système robot_localization
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException
import math

class DistanceChecker(Node):
    def __init__(self):
        super().__init__('distance_checker')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.local_sub = self.create_subscription(Odometry, '/odometry/local', self.local_callback, 10)
        self.gps_odom_sub = self.create_subscription(Odometry, '/odometry/gps', self.gps_odom_callback, 10)
        self.global_sub = self.create_subscription(Odometry, '/odometry/filtered', self.global_callback, 10)
        
        # Timer pour affichage
        self.timer = self.create_timer(3.0, self.check_distances)
        
        # Données
        self.gps_ref = None
        self.local_pos = None
        self.gps_odom_pos = None
        self.global_pos = None
        
        self.get_logger().info("Distance Checker démarré - vérification des coordonnées...")
        
    def gps_callback(self, msg):
        if self.gps_ref is None:
            self.gps_ref = (msg.latitude, msg.longitude)
            self.get_logger().info(f"GPS référence: {msg.latitude:.6f}, {msg.longitude:.6f}")
            
    def local_callback(self, msg):
        self.local_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def gps_odom_callback(self, msg):
        self.gps_odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def global_callback(self, msg):
        self.global_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def check_distances(self):
        self.get_logger().info("=== VÉRIFICATION DES DISTANCES ===")
        
        # Position locale
        if self.local_pos:
            dist_local = math.sqrt(self.local_pos[0]**2 + self.local_pos[1]**2)
            self.get_logger().info(f"🟢 EKF Local: ({self.local_pos[0]:.3f}, {self.local_pos[1]:.3f}) - Distance origine: {dist_local:.3f}m")
        else:
            self.get_logger().warning("🔴 Pas de données EKF Local")
            
        # Position GPS transformée
        if self.gps_odom_pos:
            dist_gps = math.sqrt(self.gps_odom_pos[0]**2 + self.gps_odom_pos[1]**2)
            if dist_gps > 1000:
                self.get_logger().error(f"🔴 GPS Transform: ({self.gps_odom_pos[0]:.1f}, {self.gps_odom_pos[1]:.1f}) - DISTANCE EXCESSIVE: {dist_gps:.1f}m")
            else:
                self.get_logger().info(f"🟢 GPS Transform: ({self.gps_odom_pos[0]:.3f}, {self.gps_odom_pos[1]:.3f}) - Distance origine: {dist_gps:.3f}m")
        else:
            self.get_logger().warning("🔴 Pas de données GPS Transform")
            
        # Position globale
        if self.global_pos:
            dist_global = math.sqrt(self.global_pos[0]**2 + self.global_pos[1]**2)
            if dist_global > 1000:
                self.get_logger().error(f"🔴 EKF Global: ({self.global_pos[0]:.1f}, {self.global_pos[1]:.1f}) - DISTANCE EXCESSIVE: {dist_global:.1f}m")
            else:
                self.get_logger().info(f"🟢 EKF Global: ({self.global_pos[0]:.3f}, {self.global_pos[1]:.3f}) - Distance origine: {dist_global:.3f}m")
        else:
            self.get_logger().warning("🔴 Pas de données EKF Global")
            
        # Vérification TF map->odom
        try:
            tf = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            tx, ty = tf.transform.translation.x, tf.transform.translation.y
            dist_tf = math.sqrt(tx**2 + ty**2)
            if dist_tf > 1000:
                self.get_logger().error(f"🔴 TF map->odom: ({tx:.1f}, {ty:.1f}) - DISTANCE EXCESSIVE: {dist_tf:.1f}m")
            else:
                self.get_logger().info(f"🟢 TF map->odom: ({tx:.3f}, {ty:.3f}) - Distance: {dist_tf:.3f}m")
        except LookupException:
            self.get_logger().warning("🔴 Pas de TF map->odom")
            
        # Recommandations
        if self.gps_odom_pos and math.sqrt(self.gps_odom_pos[0]**2 + self.gps_odom_pos[1]**2) > 1000:
            self.get_logger().error("❗ PROBLÈME: Les coordonnées GPS sont trop éloignées de l'origine!")
            self.get_logger().info("💡 SOLUTION: Utiliser 'ros2 launch gps_package ekf_simple_launch.py' avec fake_robot_simple")
            
        self.get_logger().info("=====================================")

def main():
    rclpy.init()
    checker = DistanceChecker()
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()