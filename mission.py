from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PController
from scipy.interpolate import PchipInterpolator
import numpy as np
import math, time

# Waypoints format: [x, y, z, yaw] in meters and radians
WAYPOINTS = [
    [0.0, 0.0, 0.75, 0.0],
    [0, 0.0, 0.5, 0],
    [0.25, 0.0, 0.75, 0],
]

WAYPOINT_TOLERANCE = 0.1  # meters for position
YAW_TOLERANCE = 0.12      # radians for yaw
KP = [150, 150, 150, 60]  # [Kp_x, Kp_y, Kp_z, Kp_yaw]

id_name = {}  # mapping from ID to name
rigid_bodies = {}
num_frames = 0

# Velocity tracking variables
prev_position = None
prev_time = None
velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s

x_control = PController(kp=KP[0])
y_control = PController(kp=KP[1])
z_control = PController(kp=KP[2])
yaw_control = PController(kp=KP[3])

steps_traj = 50

tello = TelloController()

def interpolate_traj(waypoints, num_points=steps_traj):
    """Generate interpolated trajectory using PCHIP"""
    waypoints = np.array(waypoints)
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    z = waypoints[:, 2]
    yaw = waypoints[:, 3]

    pchip_x = PchipInterpolator(np.arange(len(x)), x)
    pchip_y = PchipInterpolator(np.arange(len(y)), y)
    pchip_z = PchipInterpolator(np.arange(len(z)), z)
    pchip_yaw = PchipInterpolator(np.arange(len(yaw)), yaw)

    t_new = np.linspace(0, len(x)-1, num_points)

    x_new = pchip_x(t_new)
    y_new = pchip_y(t_new)
    z_new = pchip_z(t_new)
    yaw_new = pchip_yaw(t_new)

    trajectory = np.array([x_new, y_new, z_new, yaw_new]).T
    return trajectory

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

def calculate_velocity(current_position, current_time):
    """Calculate velocity based on position change and time delta"""
    global prev_position, prev_time, velocity

    # first run
    if prev_position is None or prev_time is None:
        prev_position = current_position.copy()
        prev_time = current_time
        return [0.0, 0.0, 0.0]
    
    dt = current_time - prev_time
    if dt <= 1e-6:
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

        current_time = time.time()
        current_velocity = calculate_velocity(position, current_time)

        return position, yaw, current_velocity
    return None, None, None

def run_mission(streaming_client: NatNetClient):
    traj = interpolate_traj(waypoints=WAYPOINTS, num_points=steps_traj)
    
    try:
        if not tello.connect():
            print("Failed to connect to Tello")
            return
        
        battery = tello.get_battery()
        print(f"Tello battery: {battery}%")
        if battery is not None and battery > 30:
            tello.takeoff()
        else:
            print("low bat")
            exit(0)
        
        print(f"Mission started with {len(traj)} waypoints")
        
        for i, waypoint in enumerate(traj):
            target_x, target_y, target_z, target_yaw = waypoint
            print(f"Waypoint {i+1}/{len(traj)}: "
                  f"X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m, Yaw={math.degrees(target_yaw):.1f}°")
            
            waypoint_start = time.time()
            waypoint_reached = False

            while time.time() - waypoint_start < 1.5:  # 1.5s timeout per waypoint
                streaming_client.update_sync()

                position, current_yaw, current_velocity = get_tello_pose()
                if position is None or current_yaw is None or current_velocity is None:
                    time.sleep(0.01)
                    continue
                                
                error_x = target_x - position[0]
                error_y = target_y - position[1]  
                error_z = target_z - position[2]
                
                # Normalize yaw error to shortest path
                error_yaw = normalize_angle(target_yaw - current_yaw)
                
                control_x = x_control.compute(target_x, position[0], current_velocity[0])
                control_y = y_control.compute(target_y, position[1], current_velocity[1])
                control_z = z_control.compute(target_z, position[2], current_velocity[2])
                
                control_yaw = yaw_control.compute(target_yaw, current_yaw, 0)
                control_yaw = -control_yaw  # Invert yaw control for Tello
                
                tello.send_rc_control(int(control_y), int(control_x), int(control_z), int(control_yaw))
                
                position_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                if position_error <= WAYPOINT_TOLERANCE and abs(error_yaw) <= YAW_TOLERANCE:
                    waypoint_reached = True
                    break

                # if num_frames % 500:
                #     print(f"pose: {position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}, {math.degrees(current_yaw):.1f}° | "
                #             f"error: {error_x:.2f}, {error_y:.2f}, {error_z:.2f}, {math.degrees(error_yaw):.1f}° | len: {position_error:.2f}")
                #     print("RC Command:", control_x, control_y, control_z, control_yaw)
                
                time.sleep(0.01)  # Control loop rate (~100Hz)
            
            if not waypoint_reached:
                print(f"⚠️  Timeout on waypoint {i+1}, proceeding to next waypoint")
        
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
        
        """ Mission mode """
        # run_mission(streaming_client)

        """ Debug mode """
        while True:
            streaming_client.update_sync()
            position, yaw, velocity = get_tello_pose()
            if position and yaw is not None and velocity is not None:
                print(f"Pos: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]m | "
                  f"Vel: [{velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}] m/s | "
                  f"Yaw: {math.degrees(yaw):.1f}° | "
                  f"Frames: {num_frames}")
            time.sleep(0.01)

    