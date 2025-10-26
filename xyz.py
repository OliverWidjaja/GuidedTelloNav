import time
import math
from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PController

# Waypoints format: [x, y, z, yaw] in meters and radians
WAYPOINTS = [
    [0.0, 0.0, 0.75, 0.0],
    [0.0, 0.0, 0.5, 0],
    [0.25, 0.0, 0.5, 0]
]

WAYPOINT_TOLERANCE = 0.1  # meters for position
YAW_TOLERANCE = 0.12      # radians for yaw

# PID gains for each axis
KP_X = 80.0
KP_Y = 80.0  
KP_Z = 140.0
KP_YAW = 60.0

id_name = {}  # mapping from ID to name
rigid_bodies = {}
num_frames = 0

# Create controllers for each degree of freedom
x_control = PController(kp=KP_X)
y_control = PController(kp=KP_Y)
z_control = PController(kp=KP_Z)
yaw_control = PController(kp=KP_YAW)

tello = TelloController()

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (yaw, pitch, roll)
    Returns yaw in range [-pi, pi]
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return yaw, pitch, roll

def normalize_angle(angle):
    """Normalize angle to range [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def receive_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    for rigid_body in desc.rigid_bodies:
        id_name[rigid_body.id_num] = rigid_body.name

def receive_frame(data_frame: DataFrame):
    global num_frames, rigid_bodies
    num_frames += 1
    
    for rigid_body in getattr(data_frame, "rigid_bodies", []):
        # Use the mapping from receive_desc to get the name
        name = id_name.get(rigid_body.id_num, f"RigidBody_{rigid_body.id_num}")
        pos = rigid_body.pos
        rot = rigid_body.rot  # 4D Quaternion
        yaw, pitch, roll = quaternion_to_euler(rot[0], rot[1], rot[2], rot[3])

        rigid_bodies[name] = {
            "position": [float(f"{p:.4f}") for p in pos],
            "orientation": {
                "quaternion": [float(f"{r:.4f}") for r in rot],
                "euler": [float(f"{yaw:.4f}"), float(f"{pitch:.4f}"), float(f"{roll:.4f}")]
            },
            "id": rigid_body.id_num
        }

def get_tello_pose():
    """Get Tello's current position and yaw orientation"""
    tello_data = rigid_bodies.get("Tello")
    if tello_data and "position" in tello_data and "orientation" in tello_data:
        position = tello_data["position"]  # [x, y, z]
        position[1] = -position[1]  # Invert Y-axis
        yaw = tello_data["orientation"]["euler"][0]  # yaw angle in radians
        return position, yaw
    return None, None

def run_mission(streaming_client: NatNetClient):
    try:
        if not tello.connect():
            print("Failed to connect to Tello")
            return
        
        battery = tello.get_battery()
        print(f"Tello battery: {battery}%")
        if battery is not None and battery > 20:
            tello.takeoff()
            # time.sleep(3)  # Allow time for takeoff to complete
        
        print(f"Mission started with {len(WAYPOINTS)} waypoints")
        
        for i, waypoint in enumerate(WAYPOINTS):
            target_x, target_y, target_z, target_yaw = waypoint
            print(f"Waypoint {i+1}/{len(WAYPOINTS)}: "
                  f"X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m, Yaw={math.degrees(target_yaw):.1f}°")
            
            waypoint_start = time.time()
            frames_without_data = 0
            waypoint_reached = False

            while time.time() - waypoint_start < 30:  # 30s timeout per waypoint
                streaming_client.update_sync()
                
                position, current_yaw = get_tello_pose()
                
                # Error Fail-safe
                if position is None or current_yaw is None:
                    frames_without_data += 1
                    if frames_without_data > 0 and frames_without_data < 20:
                        print("⚠️  No Motive data! Waiting...")
                        frames_without_data = 0
                    elif frames_without_data >= 20:
                        print("❌ Lost Motive data. Landing.")
                        tello.land()
                        tello.disconnect()
                        return
                    time.sleep(0.01)
                    continue
                
                frames_without_data = 0
                
                # Calculate errors
                error_x = target_x - position[0]
                error_y = target_y - position[1]  
                error_z = target_z - position[2]
                
                # Normalize yaw error to shortest path
                error_yaw = normalize_angle(target_yaw - current_yaw)
                
                # Compute control outputs
                control_x = x_control.compute(target_x, position[0])
                control_y = y_control.compute(target_y, position[1])
                control_z = z_control.compute(target_z, position[2])
                control_yaw = yaw_control.compute(target_yaw, current_yaw)
                control_yaw = -control_yaw  # Invert yaw control for Tello
                
                # Send RC controls (left_right, forward_back, up_down, yaw)
                tello.send_rc_control(int(control_y), int(control_x), int(control_z), int(control_yaw))
                
                # Check if waypoint is reached (position and yaw within tolerance)
                position_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                if position_error <= WAYPOINT_TOLERANCE and abs(error_yaw) <= YAW_TOLERANCE:
                    print(f"✓ Reached waypoint {i+1}")
                    waypoint_reached = True
                    time.sleep(2)  # Stabilize at waypoint
                    break

                if num_frames % 100:
                    print(f"cur x,y,z,yaw: {position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}, {math.degrees(current_yaw):.1f}° | "
                            f"err x,y,z,yaw: {error_x:.2f}, {error_y:.2f}, {error_z:.2f}, {math.degrees(error_yaw):.1f}° | abs error: {position_error:.2f}")
                    print("RC Command:", control_x, control_y, control_z, control_yaw)
                
                time.sleep(0.01)  # Control loop rate (~100Hz)
            
            if not waypoint_reached:
                print(f"⚠️  Timeout on waypoint {i+1}, proceeding to next waypoint")
        
        # Mission complete
        print("Mission complete! Landing...")
        tello.land()
        tello.disconnect()

    except KeyboardInterrupt:
        print("\n⚠️  Mission interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during mission: {e}")
    finally:
        # Safety cleanup
        print("🛬 Ensuring drone lands safely...")
        try:
            tello.land()
            tello.disconnect()
        except:
            pass  # Ignore errors during cleanup

if __name__ == "__main__":
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_frame)
    
    with streaming_client:
        streaming_client.request_modeldef()
        
        # print("Waiting for Motive data...")
        # timeout = time.time() + 2  # 2s timeout
        
        # while time.time() < timeout:
        #     streaming_client.update_sync()

        #     # Check if we found Tello in data descriptions AND have frame data
        #     if "Tello" in rigid_bodies:
        #         print("✓ Tello tracking! Starting mission.")
        #         break
        #     time.sleep(0.1)
        # else:
        #     if not "Tello" in id_name.values():
        #         print("   - Tello rigid body not defined in Motive")
        #         print("   - Available rigid bodies:", list(id_name.values()))
        #     exit(1)
        
        run_mission(streaming_client)