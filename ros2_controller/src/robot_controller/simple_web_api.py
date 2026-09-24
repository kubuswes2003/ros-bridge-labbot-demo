#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from flask import Flask, jsonify
from flask_cors import CORS
import threading

app = Flask(__name__)
CORS(app)

# Global state
robot_data = {
    'x': 0.0,
    'y': 0.0,
    'theta': 0.0,
    'theta_deg': 0.0,
    'linear_x': 0.0,
    'angular_z': 0.0,
    'success': True
}

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('simple_web_api')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.get_logger().info('🌐 Simple Web API uruchomione!')
        self.get_logger().info('🌐 API dostępne na http://localhost:5001/api/robot_state')
    
    def odom_callback(self, msg):
        global robot_data
        robot_data['x'] = msg.pose.pose.position.x
        robot_data['y'] = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        import math
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        robot_data['theta'] = theta
        robot_data['theta_deg'] = math.degrees(theta)
    
    def cmd_callback(self, msg):
        global robot_data
        robot_data['linear_x'] = msg.linear.x
        robot_data['angular_z'] = msg.angular.z

@app.route('/api/robot_state')
def get_robot_state():
    return jsonify(robot_data)

def run_ros():
    rclpy.init()
    node = RobotStateNode()
    rclpy.spin(node)

def run_flask():
    app.run(host='0.0.0.0', port=5001)

def main():
    # Start ROS2 in separate thread
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    # Start Flask
    run_flask()

if __name__ == '__main__':
    main()