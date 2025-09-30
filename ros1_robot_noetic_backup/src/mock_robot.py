#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Mock Robot Node - symuluje robota labbot w ROS1
Odbiera komendy ruchu i publikuje fake dane odometrii
"""

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import time

class MockLabbot:
    def __init__(self):
        # Inicjalizacja node ROS1
        rospy.init_node('mock_labbot', anonymous=True)
        rospy.loginfo("🤖 Mock Labbot uruchomiony!")
        
        # Subskrypcja komendy ruchu (tu bridge będzie przekazywać z ROS2)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Publikacja fake odometrii (żeby pokazać że robot "żyje")
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Stan robota (pozycja, prędkość)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Timer do publikowania odometrii co 100ms
        self.odom_timer = rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        
        rospy.loginfo("✅ Mock robot gotowy do odbierania komend na /cmd_vel")
        rospy.loginfo("📡 Publikuje fake odometrię na /odom")

    def cmd_vel_callback(self, msg):
        """
        Callback wywoływany gdy otrzymamy komendę ruchu
        msg.linear.x - prędkość do przodu/tyłu
        msg.angular.z - prędkość obrotowa
        """
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        # Log pokazujący że otrzymaliśmy komendę (DEMONSTRACJA!)
        rospy.loginfo(f"🎮 Otrzymano komendę: lin={self.linear_vel:.2f}, ang={self.angular_vel:.2f}")
        
        # Symuluj ruch robota (aktualizuj pozycję)
        dt = 0.1  # 100ms
        
        # Prosta kinematyka różnicowa robota
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # Normalizacja kąta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def publish_odometry(self, event):
        """Publikuje fake odometrię - pokazuje że robot "żyje" """
        odom = Odometry()
        
        # Header
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Pozycja
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientacja (quaternion z kąta theta)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Prędkość
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        # Publikuj
        self.odom_pub.publish(odom)

    def run(self):
        """Główna pętla - czeka na komendy"""
        rospy.loginfo("🔄 Mock robot działa... Czekam na komendy z ROS2!")
        rospy.spin()  # Czeka na komendy

if __name__ == '__main__':
    try:
        robot = MockLabbot()
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Mock robot zatrzymany")
        pass