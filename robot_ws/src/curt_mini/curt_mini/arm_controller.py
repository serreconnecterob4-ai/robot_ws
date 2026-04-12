import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial
import time


class ArduinoBridge(Node):
   def __init__(self):
       super().__init__('arduino_bridge')
       self.serial_port = None
       self.reconnect_interval_s = 5.0
       self.next_reconnect_time = 0.0
      
       self.connect_to_arduino()


       self.current_speed_pct = 50.0 
       self.target_pos_pct = None    
      
       # Mémoires pour le seuil
       self.last_sent_pos_pct = -100.0
       self.last_sent_speed_pct = -100.0


       qos_profile = QoSProfile(
           reliability=QoSReliabilityPolicy.BEST_EFFORT,
           history=QoSHistoryPolicy.KEEP_LAST,
           depth=1
       )
       self.create_subscription(Float32, '/robot/arm_speed', self.speed_callback, qos_profile)
       self.create_subscription(Float32, '/robot/arm_position', self.pos_callback, qos_profile)


       # ⚡ Fréquence divisée par 2 : On vérifie toutes les 0.2s (5 Hz max)
       self.timer = self.create_timer(0.2, self.timer_callback)


       self.get_logger().info("🤖 Pont ROS 2 Prêt ! (Protocole P/V et Seuil activés)")


   def connect_to_arduino(self):
       now = time.monotonic()
       if now < self.next_reconnect_time:
           return False
       self.next_reconnect_time = now + self.reconnect_interval_s

       ports_to_try = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
      
       for port in ports_to_try:
           try:
               self.serial_port = serial.Serial(port, 115200, timeout=0.1, write_timeout=0.1)
               self.get_logger().info(f"✅ Connecté à l'Arduino sur {port} !")
               time.sleep(2)
               self.next_reconnect_time = 0.0
              
               # Forcer l'envoi au prochain cycle
               self.last_sent_pos_pct = -100.0
               return True
           except:
               continue
              
       self.get_logger().error("❌ Arduino introuvable. En attente...")
       return False


   def speed_callback(self, msg):
       self.current_speed_pct = max(0.0, min(100.0, msg.data))


   def pos_callback(self, msg):
       self.target_pos_pct = max(0.0, min(100.0, msg.data))


   def timer_callback(self):
       if self.target_pos_pct is None:
           return


       if self.serial_port is None or not self.serial_port.is_open:
           self.connect_to_arduino()
           return


       # ⚡ LE FILTRE DE SEUIL : On n'envoie que si ça a bougé de + de 1.5% (Pos) ou 5.0% (Vit)
       pos_changed = abs(self.target_pos_pct - self.last_sent_pos_pct) > 1.5
       speed_changed = abs(self.current_speed_pct - self.last_sent_speed_pct) > 5.0


       if pos_changed or speed_changed:
           mapped_pos = int(40 + (self.target_pos_pct / 100.0) * (1000 - 40))
           mapped_speed = int(15 + (self.current_speed_pct / 100.0) * (127 - 15))


           # ⚡ LE NOUVEAU PROTOCOLE BLINDÉ
           new_command = f"P{mapped_pos},V{mapped_speed}\n"


           try:
               self.serial_port.write(new_command.encode('utf-8'))
               self.serial_port.flush()
              
               # Mise à jour des mémoires
               self.last_sent_pos_pct = self.target_pos_pct
               self.last_sent_speed_pct = self.current_speed_pct
              
               self.get_logger().info(f"-> Envoyé : {new_command.strip()}")
              
           except serial.SerialTimeoutException:
               self.get_logger().error("⚠️ Écriture bloquée. Reconnexion...")
               self.serial_port.close()
               self.serial_port = None
           except Exception as e:
               self.get_logger().error(f"⚠️ Crash USB détecté ({e}). Reconnexion...")
               self.serial_port.close()
               self.serial_port = None


def main(args=None):
   rclpy.init(args=args)
   node = ArduinoBridge()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()