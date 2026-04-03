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
        
        # Déclarations des paramètres avec 'localhost' en dur par défaut
        self.declare_parameter('robot_ip', '127.0.0.1')
        self.declare_parameter('rtsp_port', 8554)
        self.declare_parameter('stream_name', 'mystream')

        # Récupération (si le fichier YAML est présent, il écrasera le 127.0.0.1)
        ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        port = self.get_parameter('rtsp_port').get_parameter_value().integer_value
        stream = self.get_parameter('stream_name').get_parameter_value().string_value

        self.pub_clear = self.create_publisher(Image, '/camera/clear', 10)
        self.bridge = CvBridge()
        #self.cap = cv2.VideoCapture("rtsp://localhost:8554/mystream")  # Bon chemin MediaMTX
        # URL construite dynamiquement
        rtsp_url = f"rtsp://{ip}:{port}/{stream}"
        self.get_logger().info(f"Tentative de connexion : {rtsp_url}")
        self.cap = cv2.VideoCapture(rtsp_url)

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