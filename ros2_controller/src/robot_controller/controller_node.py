#!/usr/bin/env python3
"""
ROS2 Robot Controller - wysyła komendy ruchu do robota przez ros1_bridge
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('🎮 ROS2 Robot Controller uruchomiony!')
        
        # Publisher komend ruchu - bridge przekaże to do ROS1
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber odometrii - odbiera dane z ROS1 przez bridge
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Timer do wysyłania komend co 4 sekundy (czas na jeden bok/obrót)
        self.cmd_timer = self.create_timer(4.0, self.send_movement_command)
        
        # Stan controllera
        self.command_counter = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.get_logger().info('✅ Controller gotowy!')
        self.get_logger().info('📡 Publikuje komendy na /cmd_vel')
        self.get_logger().info('📡 Subskrybuje odometrię z /odom')
    
    def odom_callback(self, msg):
        """Callback - otrzymujemy odometrię z robota (przez bridge)"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Log pokazujący że otrzymaliśmy dane (DEMONSTRACJA!)
        self.get_logger().info(
            f'📍 Pozycja robota: x={self.robot_x:.2f}, y={self.robot_y:.2f}'
        )
    
    def send_movement_command(self):
        """Wysyła cykliczne komendy ruchu - kwadrat 2x2m"""
        cmd = Twist()
        
        # Kwadrat 2x2m: timer co 4s
        # Boki: 2m przy 0.5m/s = 4 sekundy
        # Obroty: 90° (π/2 rad) przy 0.4 rad/s ≈ 4 sekundy
        
        cycle = self.command_counter % 8
        
        if cycle in [0, 2, 4, 6]:
            # Jedź prosto 2m (4s przy 0.5m/s)
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            movement = f"⬆️ BÓK {cycle//2 + 1}"
        else:
            # Obrót 90° w prawo
            cmd.linear.x = 0.0
            cmd.angular.z = -0.4
            movement = f"🔄 OBRÓT {(cycle+1)//2}"
        
        # Publikuj komendę
        self.cmd_pub.publish(cmd)
        
        # Log demonstracyjny
        self.get_logger().info(
            f'🎮 Komenda #{self.command_counter}: {movement} '
            f'(lin={cmd.linear.x:.1f}, ang={cmd.angular.z:.1f})'
        )
        
        self.command_counter += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = RobotController()
        controller.get_logger().info('🔄 Controller działa... Wysyłam komendy do robota!')
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.get_logger().info('🛑 Controller zatrzymany')
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()