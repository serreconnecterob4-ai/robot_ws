import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

'''

CameraPublisher : Publie les images CLEAR 
de la caméra sur un topic ROS.
sortie : /camera/clear (Image)
'''
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.pub_clear = self.create_publisher(Image, '/camera/clear', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("rtsp://localhost:8554/mystream")  # Bon chemin MediaMTX

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.pub_clear.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main():
    rclpy.init()
    node = CameraPublisher()
    # Créer un timer à 15 FPS
    node.create_timer(1/15, node.timer_callback)
    rclpy.spin(node)