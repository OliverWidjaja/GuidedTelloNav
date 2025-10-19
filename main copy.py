import socket
import json
import time
from tello import TelloController
from controllers import PDController

# Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 5059
HEIGHT_WAYPOINTS = [1.0, 0.75, 0.5]  # meters
WAYPOINT_TOLERANCE = 0.05  # meters
KP = 80.0
KD = 25.0

# Initialize components
height_pd = PDController(kp=KP, kd=KD, max_output=60)
tello = TelloController()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def get_tello_position():
    try:
        data, _ = sock.recvfrom(4096)
        msg = json.loads(data.decode())
        tello_data = msg.get("rigid_bodies", {}).get("Tello")
        if tello_data:
            position = tello_data.get("position", [0, 0, 0])
            return position  # [x, y, z]
    except:
        pass
    return [0, 0, 0]

def run_mission():
    # Connect and takeoff
    if not tello.connect():
        print("Failed to connect to Tello")
        return
    
    if not tello.takeoff():
        print("Takeoff failed")
        return
    
    print(f"Mission started: {HEIGHT_WAYPOINTS}")
    
    # Execute waypoints
    for i, target_height in enumerate(HEIGHT_WAYPOINTS):
        print(f"Waypoint {i+1}/{len(HEIGHT_WAYPOINTS)}: {target_height}m")
        
        waypoint_start = time.time()
        height_pd.reset()
        
        while time.time() - waypoint_start < 30:  # 30s timeout per waypoint
            position = get_tello_position()
            current_height = position[2]
            
            if current_height == 0:
                continue
            
            # PD control
            control_output = height_pd.compute(target_height, current_height)
            tello.send_rc_control(0, 0, int(control_output), 0)
            
            # Print current position
            print(f"Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]m | Target: {target_height}m | Error: {target_height - current_height:.3f}m")
            
            if abs(target_height - current_height) <= WAYPOINT_TOLERANCE:
                print(f"Reached waypoint {i+1}")
                time.sleep(1)  # Stabilize
                break
            
            time.sleep(0.05)
    
    # Land and cleanup
    tello.land()
    tello.disconnect()
    sock.close()
    print("Mission complete")

if __name__ == "__main__":
    time.sleep(2)  # Wait for OptiTrack
    run_mission()
    