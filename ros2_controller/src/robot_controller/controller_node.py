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
        
        # Timer do wysyłania komend co 2 sekundy
        self.cmd_timer = self.create_timer(2.0, self.send_movement_command)
        
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
        """Wysyła cykliczne komendy ruchu - DEMO"""
        cmd = Twist()
        
        # Różne wzory ruchu dla demonstracji
        if self.command_counter % 4 == 0:
            # Jedź do przodu
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            movement = "⬆️ PRZÓD"
        elif self.command_counter % 4 == 1:
            # Skręć w prawo
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            movement = "➡️ PRAWO"
        elif self.command_counter % 4 == 2:
            # Jedź do tyłu
            cmd.linear.x = -0.3
            cmd.angular.z = 0.0
            movement = "⬇️ TYŁ"
        else:
            # Skręć w lewo
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            movement = "⬅️ LEWO"
        
        # Publikuj komendę
        self.cmd_pub.publish(cmd)
        
        # Log demonstracyjny
        self.get_logger().info(
            f'🎮 Wysłano komendę #{self.command_counter}: {movement} '
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