cat > ros2_controller/src/robot_controller/web_api.py << 'EOF'
#!/usr/bin/env python3
from http.server import HTTPServer, BaseHTTPRequestHandler
import json
import threading

class ROSWebAPI:
    def __init__(self):
        self.latest_cmd = {'linear': 0.0, 'angular': 0.0, 'timestamp': ''}
        self.latest_odom = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'timestamp': ''}
    
    def update_cmd(self, linear, angular, timestamp):
        self.latest_cmd = {'linear': linear, 'angular': angular, 'timestamp': timestamp}
    
    def update_odom(self, x, y, theta, timestamp):
        self.latest_odom = {'x': x, 'y': y, 'theta': theta, 'timestamp': timestamp}

class RequestHandler(BaseHTTPRequestHandler):
    api = None
    
    def do_GET(self):
        if self.path == '/ros_data':
            data = {
                'cmd_vel': self.api.latest_cmd,
                'odom': self.api.latest_odom
            }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())
        else:
            self.send_response(404)
            self.end_headers()

def start_web_api(api_instance):
    RequestHandler.api = api_instance
    server = HTTPServer(('0.0.0.0', 8081), RequestHandler)
    print("Web API running on port 8081")
    server.serve_forever()

ros_api = ROSWebAPI()

def start_api_server():
    thread = threading.Thread(target=start_web_api, args=(ros_api,), daemon=True)
    thread.start()
EOF