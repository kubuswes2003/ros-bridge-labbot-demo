#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from flask import Flask, jsonify
from flask_cors import CORS
import threading
import math
import time

app = Flask(__name__)
CORS(app)

class SimpleWebAPI(Node):
    def __init__(self):
        super().__init__('simple_web_api')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.robot_data = {
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,
            'timestamp': time.time()
        }
        
        self.get_logger().info('🌐 Simple Web API uruchomione!')
        
    def odom_callback(self, msg):
        self.robot_data['x'] = msg.pose.pose.position.x
        self.robot_data['y'] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_data['theta'] = math.atan2(siny_cosp, cosy_cosp)
        self.robot_data['timestamp'] = time.time()

web_api = None

@app.route('/api/robot_state')
def get_robot_state():
    if web_api:
        return jsonify({
            'success': True,
            'x': web_api.robot_data['x'],
            'y': web_api.robot_data['y'],
            'theta': web_api.robot_data['theta'],
            'theta_deg': math.degrees(web_api.robot_data['theta'])
        })
    return jsonify({'success': False})

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

def main(args=None):
    global web_api
    rclpy.init(args=args)
    web_api = SimpleWebAPI()
    
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    web_api.get_logger().info('🌐 API dostępne na http://localhost:5000/api/robot_state')
    
    try:
        rclpy.spin(web_api)
    except KeyboardInterrupt:
        pass
    finally:
        web_api.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()