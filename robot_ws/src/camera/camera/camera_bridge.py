import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

'''

CameraBridge : Gère la capture et la publication des images de la caméra
en utilisant le flux RTSP direct de la caméra. (FLUENT & CLEAR)
Obsolete : les flux ne sont pas utilisés pour la 
capture/photo/vidéo.
'''

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        self.pub_fluent = self.create_publisher(Image, '/camera/fluent', 10)
        self.pub_clear = self.create_publisher(Image, '/camera/clear', 10)
        self.bridge = CvBridge()
        
        self.rtsp_url = "rtsp://admin:ros2_2025@10.42.0.188:554/h264Preview_01_main"
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
        
        # Le timer qui appelle la fonction ci-dessous
        self.timer = self.create_timer(0.04, self.timer_callback)

    def timer_callback(self):
        """Récupère et publie les images"""
        # On vide le buffer pour éviter la latence
        for _ in range(2):
            self.cap.grab()
            
        ret, frame = self.cap.retrieve()
        if ret:
            # Flux HD pour les captures (CaptureManager)
            self.pub_clear.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            
            # Flux léger pour l'IHM (Fluidité)
            fluent_frame = cv2.resize(frame, (640, 360))
            self.pub_fluent.publish(self.bridge.cv2_to_imgmsg(fluent_frame, "bgr8"))
        else:
            self.cap.open(self.rtsp_url)

def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()