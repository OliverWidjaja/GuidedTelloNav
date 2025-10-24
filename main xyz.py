import time
import math
from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PController, PDController

# Waypoints: [x, y, z, yaw] - yaw in radians
WAYPOINTS = [  
    [0.0, 0.0, 0.5, 0.0],
    [0.25, 0.0, 0.5, 0.0],
    [0.25, 0.0, 0.25, 0.0]
]

WAYPOINT_TOLERANCE = 0.05  # meters
YAW_TOLERANCE = 0.1       # radians

# Controller gains
KP_X = 448.0
KD_X = 250.0
KP_Y = 180.0
KD_Y = 150.0
KP_Z = 200.0
KP_YAW = 140.0

id_name = {}  # mapping ID to name
rigid_bodies = {}
num_frames = 0

x_control = PDController(kp=KP_X, kd=KD_X)
y_control = PDController(kp=KP_Y, kd=KD_Y)
z_control = PController(kp=KP_Z)
yaw_control = PController(kp=KP_YAW)

tello = TelloController()

# Velocity tracking variables
prev_position = None
prev_time = None
velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s


def quaternion_to_euler(x, y, z, w):
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(2 * (w * y - z * x))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll

def calculate_velocity(current_position, current_time):
    """Calculate velocity based on position change and time delta"""
    global prev_position, prev_time, velocity
    
    # First measurement
    if prev_position is None or prev_time is None:
        prev_position = current_position.copy()
        prev_time = current_time
        return [0.0, 0.0, 0.0]
    
    dt = current_time - prev_time
    if dt <= 0:
        return velocity
    
    vx = (current_position[0] - prev_position[0]) / dt
    vy = (current_position[1] - prev_position[1]) / dt
    vz = (current_position[2] - prev_position[2]) / dt
    
    prev_position = current_position.copy()
    prev_time = current_time
    
    velocity = [vx, vy, vz]
    return velocity

def receive_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    for rigid_body in desc.rigid_bodies:
        id_name[rigid_body.id_num] = rigid_body.name

def receive_frame(data_frame: DataFrame):
    global num_frames, rigid_bodies
    num_frames += 1
    
    for rigid_body in getattr(data_frame, "rigid_bodies", []):
        # Use  mapping from receive_desc to get name
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

def get_data():
    tello_data = rigid_bodies.get("Tello")
    if tello_data and "position" in tello_data and "orientation" in tello_data:
        position = tello_data["position"]  # [x, y, z]
        position[1] = -position[1]  # Invert Y-axis
        yaw = tello_data["orientation"]["euler"][0]  # yaw from euler angles
        
        current_time = time.time()
        current_velocity = calculate_velocity(position, current_time)
        
        return position, yaw, current_velocity
    return None, None, None

def run_mission(streaming_client: NatNetClient):
    try:
        if not tello.connect():
            print("Failed to connect to Tello")
            return
        
        battery = tello.get_battery()
        print(f"Tello battery: {battery}%")
        if battery is not None and battery > 20:
            tello.takeoff()
        
        print(f"Mission started: {len(WAYPOINTS)} waypoints")
        
        for i, waypoint in enumerate(WAYPOINTS):
            target_x, target_y, target_z, target_yaw = waypoint
            print(f"Waypoint {i+1}/{len(WAYPOINTS)}: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]m, yaw: {math.degrees(target_yaw):.1f}°")
            
            waypoint_start = time.time()

            while time.time() - waypoint_start < 20:  # 20s timeout per waypoint
                streaming_client.update_sync()
                
                position, current_yaw, current_velocity = get_data()
                if position is None or current_yaw is None:
                    print("Data skipped: No Tello position/orientation")
                    time.sleep(0.01)
                    continue
                
                current_x, current_y, current_z = position
                
                control_x = x_control.compute(target_x, current_x, current_velocity[0], target_velocity=0)
                control_y = y_control.compute(target_y, current_y, current_velocity[1], target_velocity=0)
                control_z = z_control.compute(target_z, current_z)
                control_yaw = yaw_control.compute(target_yaw, current_yaw)
                
                tello.send_rc_control(
                    int(control_y),      # left_right
                    int(control_x),      # forward_back
                    int(control_z),      # up_down
                    int(control_yaw)
                )
                
                position_error = math.sqrt(
                    (target_x - current_x)**2 + 
                    (target_y - current_y)**2 + 
                    (target_z - current_z)**2
                )
                yaw_error = abs(target_yaw - current_yaw)

                if num_frames % 250 == 0:
                    speed_magnitude = math.sqrt(current_velocity[0]**2 + current_velocity[1]**2 + current_velocity[2]**2)
                    print(f"Pos: [{current_x:.3f}, {current_y:.3f}, {current_z:.3f}]m | "
                          f"Vel: [{current_velocity[0]:.3f}, {current_velocity[1]:.3f}, {current_velocity[2]:.3f}] m/s | "
                          f"Speed: {speed_magnitude:.3f} m/s | "
                          f"Yaw: {math.degrees(current_yaw):.1f}° | "
                          f"Target: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]m, {math.degrees(target_yaw):.1f}° | "
                          f"Frames: {num_frames}")
                    print(f"Pos Error: {position_error:.4f} m, Yaw Error: {math.degrees(yaw_error):.2f}° | "
                          f"Vel Errors, X: {-current_velocity[0]:.3f}, Y: {-current_velocity[1]:.3f} m/s")
                    print(f"RC Control Outputs, X: {control_x:.2f}, Y: {control_y:.2f}, Z: {control_z:.2f}, Yaw: {control_yaw:.2f}")
                
                if position_error <= WAYPOINT_TOLERANCE and yaw_error <= YAW_TOLERANCE:
                    print(f"Reached waypoint {i+1}")
                    time.sleep(1)  # Stabilize
                    break
                
                time.sleep(0.01)  # Control loop rate
        
        tello.land()
        tello.disconnect()
        print("Mission complete")

    except KeyboardInterrupt:
        print("\nMission interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during mission: {e}")
    finally:
        print("Landing drone...") # Always runs, even on Ctrl+C or errors
        try:
            tello.land()
            tello.disconnect()
        except:
            pass


if __name__ == "__main__":
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_desc) # Handlers
    streaming_client.on_data_frame_received_event.handlers.append(receive_frame) # Handlers
    
    with streaming_client:
        streaming_client.request_modeldef()
        
        print("Waiting for Motive data...")
        timeout = time.time() + 2  # 2s timeout
        
        """
        Mission
        """
        while time.time() < timeout:
            streaming_client.update_sync()

            if "Tello" in rigid_bodies:
                print("✓ Tello tracking! Starting mission.")
                break
            time.sleep(0.1)
        else:
            if not "Tello" in id_name.values():
                print("   - Tello rigid body not defined in Motive")
                print("   - Available rigid bodies:", list(id_name.values()))
            exit(1)
        
        run_mission(streaming_client)

        """
        Debug
        """
        # target_x, target_y, target_z, target_yaw = [0.0, 0.0, 1.0, 0.0]

        # while(1):
        #     streaming_client.update_sync()
        #     position, current_yaw, current_velocity = get_data()
        #     if position is None or current_yaw is None:
        #         print("Data skipped: No Tello position/orientation")
        #         time.sleep(0.01)
        #         continue
            
        #     current_x, current_y, current_z = position
        #     speed_magnitude = math.sqrt(current_velocity[0]**2 + current_velocity[1]**2 + current_velocity[2]**2)
        #     print(f"Pos: [{current_x:.3f}, {current_y:.3f}, {current_z:.3f}]m | "
        #           f"Vel: [{current_velocity[0]:.3f}, {current_velocity[1]:.3f}, {current_velocity[2]:.3f}] m/s | "
        #           f"Speed: {speed_magnitude:.3f} m/s | "
        #           f"Yaw: {math.degrees(current_yaw):.1f}° | "
        #           f"Frames: {num_frames}")
            
        #     # Compute control outputs for each degree of freedom
        #     control_x = x_control.compute(target_x, current_x, current_velocity[0], target_velocity=0)
        #     control_y = y_control.compute(target_y, current_y, current_velocity[1], target_velocity=0)
        #     control_z = z_control.compute(target_z, current_z)
        #     control_yaw = yaw_control.compute(target_yaw, current_yaw)
        #     print(f"Control Outputs => X: {control_x:.2f}, Y: {control_y:.2f}, Z: {control_z:.2f}, Yaw: {control_yaw:.2f}")
        #     print(f"Velocity Errors => X: {-current_velocity[0]:.3f}, Y: {-current_velocity[1]:.3f} m/s")

        #     time.sleep(1)