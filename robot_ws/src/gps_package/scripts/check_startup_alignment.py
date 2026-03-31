#!/usr/bin/env python3
"""
Script simple pour vérifier que GPS et odométrie démarrent au même point
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math

class StartupChecker(Node):
    def __init__(self):
        super().__init__('startup_checker')
        
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.wheel_sub = self.create_subscription(Odometry, '/wheel/odom', self.wheel_callback, 10)
        
        self.gps_first = None
        self.wheel_first = None
        self.gps_ref = None
        
        self.get_logger().info("Vérification du démarrage GPS vs Odom...")
        
    def gps_callback(self, msg):
        if self.gps_ref is None:
            self.gps_ref = (msg.latitude, msg.longitude)
            
        if self.gps_first is None:
            # Calculer position relative (0,0 = premier point GPS)
            dx = (msg.longitude - self.gps_ref[1]) * 111320.0
            dy = (msg.latitude - self.gps_ref[0]) * 111320.0
            self.gps_first = (dx, dy)
            self.get_logger().info(f"🗺️  Premier point GPS: ({dx:.3f}, {dy:.3f})m")
            self.check_alignment()
            
    def wheel_callback(self, msg):
        if self.wheel_first is None:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.wheel_first = (x, y)
            self.get_logger().info(f"🚗 Premier point Wheel Odom: ({x:.3f}, {y:.3f})m")
            self.check_alignment()
            
    def check_alignment(self):
        if self.gps_first is not None and self.wheel_first is not None:
            dx = self.gps_first[0] - self.wheel_first[0]
            dy = self.gps_first[1] - self.wheel_first[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"📊 ALIGNEMENT INITIAL:")
            self.get_logger().info(f"   GPS relatif:  ({self.gps_first[0]:.3f}, {self.gps_first[1]:.3f})m")
            self.get_logger().info(f"   Wheel Odom:   ({self.wheel_first[0]:.3f}, {self.wheel_first[1]:.3f})m")
            self.get_logger().info(f"   Décalage:     ({dx:.3f}, {dy:.3f})m")
            self.get_logger().info(f"   Distance:     {distance:.3f}m")
            
            if distance < 0.1:
                self.get_logger().info("✅ PARFAIT: GPS et Odom démarrent au même point!")
            elif distance < 1.0:
                self.get_logger().warn(f"⚠️  Petit décalage: {distance:.3f}m - acceptable")
            else:
                self.get_logger().error(f"❌ PROBLÈME: Gros décalage {distance:.3f}m - navsat_transform va avoir des difficultés")
            
            self.get_logger().info("=" * 50)
            
            # Arrêter après vérification
            rclpy.shutdown()

def main():
    rclpy.init()
    checker = StartupChecker()
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    checker.destroy_node()

if __name__ == '__main__':
    main()