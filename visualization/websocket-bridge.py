cat > visualization/websocket-bridge.py << 'EOF'
#!/usr/bin/env python3
import asyncio
import websockets
import json
import subprocess
import threading
import time

class ROSWebSocketBridge:
    def __init__(self):
        self.latest_data = {
            'ros2_command': 'Inicjalizacja...',
            'ros1_response': 'Czekam na dane...',
            'robot_position': {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        }
        
    def monitor_ros_topics(self):
        """Monitor ROS topics and update data"""
        while True:
            try:
                # Symulacja odczytu danych ROS (w prawdziwym systemie użyj rospy/rclpy)
                import random
                self.latest_data['ros2_command'] = f"Komenda #{int(time.time()) % 100}"
                self.latest_data['robot_position']['x'] += random.uniform(-0.1, 0.1)
                time.sleep(2)
            except:
                time.sleep(1)
    
    async def handler(self, websocket, path):
        """WebSocket handler"""
        try:
            async for message in websocket:
                # Wyślij aktualne dane
                await websocket.send(json.dumps(self.latest_data))
        except:
            pass

if __name__ == "__main__":
    bridge = ROSWebSocketBridge()
    
    # Start ROS monitoring in background
    threading.Thread(target=bridge.monitor_ros_topics, daemon=True).start()
    
    # Start WebSocket server
    start_server = websockets.serve(bridge.handler, "0.0.0.0", 8081)
    print("WebSocket bridge running on port 8081")
    
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
EOF