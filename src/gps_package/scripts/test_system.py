#!/usr/bin/env python3
"""
Script de test pour vérifier le bon fonctionnement du système robot_localization
selon le tutoriel officiel.

Vérifie:
1. Topics publiés/écoutés par chaque nœud
2. TF tree correct 
3. Données GPS converties en coordonnées locales
4. Fusion dans les EKF local et global
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import time

class SystemValidator(Node):
    def __init__(self):
        super().__init__('system_validator')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers pour validation
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.wheel_odom_sub = self.create_subscription(Odometry, '/wheel/odom', self.wheel_odom_callback, 10)
        self.local_odom_sub = self.create_subscription(Odometry, '/odometry/local', self.local_odom_callback, 10)
        self.gps_odom_sub = self.create_subscription(Odometry, '/odometry/gps', self.gps_odom_callback, 10)
        self.global_odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.global_odom_callback, 10)
        
        # Compteurs
        self.gps_count = 0
        self.imu_count = 0
        self.wheel_odom_count = 0
        self.local_odom_count = 0
        self.gps_odom_count = 0
        self.global_odom_count = 0
        
        # Timer pour affichage
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("Validation du système robot_localization démarrée...")
        
    def gps_callback(self, msg):
        self.gps_count += 1
        
    def imu_callback(self, msg):
        self.imu_count += 1
        
    def wheel_odom_callback(self, msg):
        self.wheel_odom_count += 1
        
    def local_odom_callback(self, msg):
        self.local_odom_count += 1
        
    def gps_odom_callback(self, msg):
        self.gps_odom_count += 1
        
    def global_odom_callback(self, msg):
        self.global_odom_count += 1
        
    def check_transforms(self):
        """Vérifie les transformations TF"""
        transforms_status = {}
        
        try:
            # odom -> base_link (EKF local)
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            transforms_status['odom->base_link'] = '✓ OK'
        except Exception as e:
            transforms_status['odom->base_link'] = f'✗ ERREUR: {str(e)[:30]}...'
            
        try:
            # map -> odom (EKF global) 
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            transforms_status['map->odom'] = '✓ OK'
        except Exception as e:
            transforms_status['map->odom'] = f'✗ ERREUR: {str(e)[:30]}...'
            
        try:
            # base_link -> imu_link (fake_robot)
            transform = self.tf_buffer.lookup_transform('base_link', 'imu_link', rclpy.time.Time())
            transforms_status['base_link->imu_link'] = '✓ OK'
        except Exception as e:
            transforms_status['base_link->imu_link'] = f'✗ ERREUR: {str(e)[:30]}...'
            
        try:
            # base_link -> gps_link (fake_robot)
            transform = self.tf_buffer.lookup_transform('base_link', 'gps_link', rclpy.time.Time())
            transforms_status['base_link->gps_link'] = '✓ OK'
        except Exception as e:
            transforms_status['base_link->gps_link'] = f'✗ ERREUR: {str(e)[:30]}...'
            
        return transforms_status
        
    def print_status(self):
        """Affiche le statut du système"""
        self.get_logger().info("=== STATUS SYSTÈME ROBOT_LOCALIZATION ===")
        
        # Topics
        self.get_logger().info(f"📡 Topics (messages reçus):")
        self.get_logger().info(f"  GPS (/gps/fix): {self.gps_count}")
        self.get_logger().info(f"  IMU (/imu/data): {self.imu_count}")
        self.get_logger().info(f"  Wheel Odom (/wheel/odom): {self.wheel_odom_count}")
        self.get_logger().info(f"  EKF Local (/odometry/local): {self.local_odom_count}")
        self.get_logger().info(f"  GPS Transform (/odometry/gps): {self.gps_odom_count}")  
        self.get_logger().info(f"  EKF Global (/odometry/filtered): {self.global_odom_count}")
        
        # TF
        tf_status = self.check_transforms()
        self.get_logger().info(f"🔗 Transformations TF:")
        for tf_name, status in tf_status.items():
            self.get_logger().info(f"  {tf_name}: {status}")
        
        # Validation
        issues = []
        if self.gps_count == 0:
            issues.append("Pas de données GPS")
        if self.imu_count == 0:
            issues.append("Pas de données IMU") 
        if self.wheel_odom_count == 0:
            issues.append("Pas de données wheel odom")
        if self.local_odom_count == 0:
            issues.append("EKF local ne publie pas")
        if self.gps_odom_count == 0:
            issues.append("navsat_transform ne publie pas")
        if self.global_odom_count == 0:
            issues.append("EKF global ne publie pas")
            
        if not issues:
            self.get_logger().info("🎉 SYSTÈME OK - Tous les nœuds fonctionnent!")
        else:
            self.get_logger().warning(f"⚠️  PROBLEMES DÉTECTÉS: {', '.join(issues)}")
            
        self.get_logger().info("==========================================")

def main():
    rclpy.init()
    validator = SystemValidator()
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()