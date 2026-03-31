#!/usr/bin/env python3
"""Script pour initialiser le datum du navsat_transform_node"""
import rclpy
from robot_localization.srv import SetDatum
import sys
import time

def main():
    rclpy.init()
    node = rclpy.create_node('set_datum_client')
    
    # Attendre que le service soit disponible
    client = node.create_client(SetDatum, '/navsat_transform/set_datum')
    
    # Paramètres du datum fixe (même que dans fake_robot)
    latitude = 48.8566
    longitude = 2.3522
    heading = 0.0  # Radians
    
    # Attendre le service
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Attente du service set_datum...')
    
    # Créer la requête - SetDatum.Request() accepte juste lat/lon/heading
    request = SetDatum.Request()
    request.lat = latitude
    request.lon = longitude
    request.heading = heading
    
    node.get_logger().info(f'Initialisation du datum: lat={latitude}, lon={longitude}, heading={heading}')
    
    # Appeler le service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Datum initialisé avec succès')
        return 0
    else:
        node.get_logger().error('Erreur lors de l\'initialisation du datum')
        return 1

if __name__ == '__main__':
    sys.exit(main())
