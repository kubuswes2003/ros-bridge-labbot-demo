#!/usr/bin/env python3
"""
ROS2 Robot Controller - wysyła komendy ruchu do robota przez ros1_bridge
Kwadrat 2x2m z zatrzymaniem między ruchami
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('🎮 ROS2 Robot Controller uruchomiony!')
        
        # Publisher komend ruchu
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber odometrii
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Timer co 0.5s
        self.cmd_timer = self.create_timer(0.5, self.send_movement_command)
        
        # Stan controllera
        self.command_counter = 0  # Który bok/obrót (0-7)
        self.step_counter = 0      # Który krok w aktualnej komendzie (0-7)
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.get_logger().info('✅ Controller gotowy!')
        self.get_logger().info('📡 Publikuje komendy na /cmd_vel')
        self.get_logger().info('📡 Subskrybuje odometrię z /odom')
    
    def odom_callback(self, msg):
        """Callback odometrii"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        self.get_logger().info(
            f'📍 Pozycja robota: x={self.robot_x:.2f}, y={self.robot_y:.2f}'
        )
    
    def send_movement_command(self):
        """
        Wysyła komendy w cyklu:
        - 7 kroków (3.5s) ruchu
        - 1 krok (0.5s) STOP
        = 4s total na akcję
        """
        cmd = Twist()
        cycle = self.command_counter % 8
        
        # Krok 7 = STOP (zatrzymanie między komendami)
        if self.step_counter == 7:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            movement = "⏸️ STOP"
            
            # Reset do następnej komendy
            self.step_counter = 0
            self.command_counter += 1
            
        else:
            # Normalna komenda ruchu
            if cycle in [0, 2, 4, 6]:
                # Bok kwadratu
                cmd.linear.x = 0.5
                cmd.angular.z = 0.0
                movement = f"⬆️ BÓK {cycle//2 + 1}"
            else:
                # Obrót 90 stopni
                cmd.linear.x = 0.0
                cmd.angular.z = -0.45
                movement = f"🔄 OBRÓT {(cycle+1)//2}"
            
            self.step_counter += 1
        
        # Publikuj komendę
        self.cmd_pub.publish(cmd)
        
        # Log co 4 kroki żeby nie spamować
        if self.step_counter % 4 == 0 or self.step_counter == 0:
            self.get_logger().info(
                f'🎮 Komenda #{self.command_counter}, krok {self.step_counter}: {movement} '
                f'(lin={cmd.linear.x:.1f}, ang={cmd.angular.z:.1f})'
            )

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