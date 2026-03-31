#!/usr/bin/env python3
"""
Script de diagnostic pour comprendre le décalage entre GPS et odométrie
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math

class ReferentialDiagnostic(Node):
    def __init__(self):
        super().__init__('referential_diagnostic')
        
        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.wheel_odom_sub = self.create_subscription(Odometry, '/wheel/odom', self.wheel_odom_callback, 10)
        self.gps_odom_sub = self.create_subscription(Odometry, '/odometry/gps', self.gps_odom_callback, 10)
        
        # Data storage
        self.gps_ref = None
        self.gps_positions = []
        self.wheel_positions = []
        self.gps_odom_positions = []
        
        self.get_logger().info("Diagnostic du décalage GPS/Odom démarré...")
        
    def gps_callback(self, msg):
        if self.gps_ref is None:
            self.gps_ref = (msg.latitude, msg.longitude)
            self.get_logger().info(f"GPS référence: {msg.latitude:.6f}, {msg.longitude:.6f}")
            
        # Calculer position relative au premier point GPS
        if len(self.gps_positions) > 0:
            # Conversion basique lat/lon -> mètres
            m_per_deg_lat = 111320.0
            m_per_deg_lon = 111320.0 * math.cos(math.radians(self.gps_ref[0]))
            
            dx = (msg.longitude - self.gps_ref[1]) * m_per_deg_lon
            dy = (msg.latitude - self.gps_ref[0]) * m_per_deg_lat
            
            self.gps_positions.append((dx, dy))
            
            # Calculer centre du cercle GPS (moyenne des positions)
            if len(self.gps_positions) > 10:
                avg_x = sum(pos[0] for pos in self.gps_positions[-20:]) / min(20, len(self.gps_positions))
                avg_y = sum(pos[1] for pos in self.gps_positions[-20:]) / min(20, len(self.gps_positions))
                
                if len(self.gps_positions) % 20 == 0:  # Affichage périodique
                    self.get_logger().info(f"Centre GPS (20 derniers): {avg_x:.2f}, {avg_y:.2f}")
        else:
            self.gps_positions.append((0.0, 0.0))  # Premier point = origine
            
    def wheel_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.wheel_positions.append((x, y))
        
        # Calculer centre du cercle odométrie
        if len(self.wheel_positions) > 10:
            avg_x = sum(pos[0] for pos in self.wheel_positions[-20:]) / min(20, len(self.wheel_positions))
            avg_y = sum(pos[1] for pos in self.wheel_positions[-20:]) / min(20, len(self.wheel_positions))
            
            if len(self.wheel_positions) % 20 == 0:  # Affichage périodique
                self.get_logger().info(f"Centre Wheel Odom (20 derniers): {avg_x:.2f}, {avg_y:.2f}")
                
    def gps_odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gps_odom_positions.append((x, y))
        
        # Calculer centre du cercle GPS transformé
        if len(self.gps_odom_positions) > 10:
            avg_x = sum(pos[0] for pos in self.gps_odom_positions[-20:]) / min(20, len(self.gps_odom_positions))
            avg_y = sum(pos[1] for pos in self.gps_odom_positions[-20:]) / min(20, len(self.gps_odom_positions))
            
            if len(self.gps_odom_positions) % 20 == 0:  # Affichage périodique
                self.get_logger().info(f"Centre GPS Transform (20 derniers): {avg_x:.2f}, {avg_y:.2f}")
                
                # Calculer décalage si on a les deux
                if len(self.wheel_positions) >= 20:
                    wheel_avg_x = sum(pos[0] for pos in self.wheel_positions[-20:]) / 20
                    wheel_avg_y = sum(pos[1] for pos in self.wheel_positions[-20:]) / 20
                    
                    offset_x = avg_x - wheel_avg_x
                    offset_y = avg_y - wheel_avg_y
                    
                    self.get_logger().warn(f"🎯 DÉCALAGE: GPS_transform - Wheel_odom = ({offset_x:.2f}, {offset_y:.2f})m")

def main():
    rclpy.init()
    node = ReferentialDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()