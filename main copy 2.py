import time
import math
from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PDController

# Configuration
HEIGHT_WAYPOINTS = [1.0, 0.5, 1.0]  # meters
WAYPOINT_TOLERANCE = 0.05  # meters
KP = 80.0
KD = 25.0

# Global variables for Motive data
rigid_body_id_to_name = {}
current_rigid_bodies = {}
num_frames = 0
last_frame_time = 0

# Initialize components
height_pd = PDController(kp=KP, kd=KD, max_output=60)
tello = TelloController()

def quaternion_to_euler(x, y, z, w):
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(2 * (w * y - z * x))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll

def receive_new_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    # Populate the mapping
    for rigid_body in desc.rigid_bodies:
        rigid_body_id_to_name[rigid_body.id_num] = rigid_body.name

def receive_new_frame(data_frame: DataFrame):
    global num_frames, current_rigid_bodies, last_frame_time
    num_frames += 1
    last_frame_time = time.time()
    
    # Update current rigid bodies data
    for rigid_body in getattr(data_frame, "rigid_bodies", []):
        name = rigid_body_id_to_name.get(rigid_body.id_num, f"RigidBody_{rigid_body.id_num}")
        pos = rigid_body.pos
        rot = rigid_body.rot  # 4D Quaternion

        # Convert quaternion to Euler angles
        yaw, pitch, roll = quaternion_to_euler(rot[0], rot[1], rot[2], rot[3])
        
        # Update the rigid body data
        current_rigid_bodies[name] = {
            "position": [float(f"{p:.4f}") for p in pos],
            "orientation": {
                "quaternion": [float(f"{r:.4f}") for r in rot],
                "euler": [float(f"{yaw:.4f}"), float(f"{pitch:.4f}"), float(f"{roll:.4f}")]
            },
            "id": rigid_body.id_num
        }

def get_tello_position():
    """Get Tello position directly from Motive data"""
    tello_data = current_rigid_bodies.get("Tello")
    if tello_data:
        position = tello_data.get("position", [0, 0, 0])
        return position  # [x, y, z]
    return None  # Return None instead of [0,0,0] to distinguish no data

def is_motive_data_fresh():
    """Check if Motive data is recent (within last 0.1 seconds)"""
    return time.time() - last_frame_time < 0.1

def run_mission(streaming_client):
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
        frames_without_data = 0
        
        while time.time() - waypoint_start < 30:  # 30s timeout per waypoint
            # Update Motive data more aggressively
            streaming_client.update_sync()
            
            position = get_tello_position()
            
            # Check if we have fresh data
            if position is None or not is_motive_data_fresh():
                frames_without_data += 1
                if frames_without_data > 10:  # If no data for 10 cycles
                    print("⚠️  No fresh Motive data! Waiting...")
                    frames_without_data = 0
                time.sleep(0.01)
                continue
            
            frames_without_data = 0
            current_height = position[2]
            
            # PD control
            control_output = height_pd.compute(target_height, current_height)
            tello.send_rc_control(0, 0, int(control_output), 0)
            
            # Print current position
            print(f"Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]m | Target: {target_height}m | Error: {target_height - current_height:.3f}m | Frames: {num_frames}")
            
            if abs(target_height - current_height) <= WAYPOINT_TOLERANCE:
                print(f"Reached waypoint {i+1}")
                time.sleep(1)  # Stabilize
                break
            
            time.sleep(0.02)  # Faster loop for better responsiveness
    
    # Land and cleanup
    tello.land()
    tello.disconnect()
    print("Mission complete")

if __name__ == "__main__":
    # Initialize Motive client
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)
    
    with streaming_client:
        streaming_client.request_modeldef()
        
        # Wait for Motive to start streaming and get some initial data
        print("Waiting for Motive data...")
        initial_frames = num_frames
        timeout = time.time() + 5  # 5 second timeout
        
        while time.time() < timeout:
            streaming_client.update_sync()
            if num_frames > initial_frames + 5:  # Wait for at least 5 frames
                print(f"Motive data streaming! Received {num_frames} frames")
                break
            time.sleep(0.1)
        else:
            print("❌ Timeout waiting for Motive data! Check Motive is running and streaming.")
            exit(1)
        
        # Run the mission
        run_mission(streaming_client)