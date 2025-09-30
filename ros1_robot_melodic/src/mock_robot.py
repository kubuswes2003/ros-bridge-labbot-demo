#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MockLabbotMelodic:
    def __init__(self):
        rospy.init_node('mock_labbot_melodic', anonymous=True)
        rospy.loginfo("Mock Labbot Melodic uruchomiony!")
        
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Stan robota
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Timer aktualizujący pozycję co 0.1s
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_position)
        self.odom_timer = rospy.Timer(rospy.Duration(0.1), self.publish_odometry)
        
        rospy.loginfo("Mock robot Melodic gotowy na /cmd_vel")
    
    def cmd_vel_callback(self, msg):
        """Tylko zapisz prędkości, nie aktualizuj pozycji tutaj"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        
        rospy.loginfo("MELODIC otrzymal: lin={:.2f}, ang={:.2f}".format(
            self.linear_vel, self.angular_vel))
    
    def update_position(self, event):
        """Aktualizuj pozycję na podstawie aktualnych prędkości"""
        dt = 0.1
        
        # Oblicz nową pozycję
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # Normalizuj kąt do [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def publish_odometry(self, event):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        self.odom_pub.publish(odom)
    
    def run(self):
        rospy.loginfo("Mock robot Melodic dziala...")
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = MockLabbotMelodic()
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mock robot Melodic zatrzymany")