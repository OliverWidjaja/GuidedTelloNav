import time
import math
from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PController

HEIGHT_WAYPOINTS = [1.0, 0.5, 1.0]  # meters
WAYPOINT_TOLERANCE = 0.07  # meters
KP = 140.0

id_name = {}  # mapping from ID to name-
rigid_bodies = {}
num_frames = 0

altitude_control = PController(kp=KP)
tello = TelloController()


def quaternion_to_euler(x, y, z, w):
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(2 * (w * y - z * x))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return yaw, pitch, roll

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

def get_tello():
    tello_data = rigid_bodies.get("Tello")
    if tello_data and "position" in tello_data:
        tello_data["position"][1] = -tello_data["position"][1]  # Invert Y-axis
        return tello_data["position"]  # [x, y, z]
    return None

def run_mission(streaming_client: NatNetClient):
    try:
        if not tello.connect():
            print("Failed to connect to Tello")
            return
        
        battery = tello.get_battery()
        print(f"Tello battery: {battery}%")
        if battery is not None and battery > 20:
            tello.takeoff()
        
        print(f"Mission started: {HEIGHT_WAYPOINTS}")
        
        for i, target_height in enumerate(HEIGHT_WAYPOINTS):
            print(f"Waypoint {i+1}/{len(HEIGHT_WAYPOINTS)}: {target_height}m")
            
            waypoint_start = time.time()

            while time.time() - waypoint_start < 20:  # 20s timeout per waypoint
                streaming_client.update_sync()
                
                position = get_tello()
                if position is None:
                    print("Data skipped: No Tello position")
                    time.sleep(0.01)
                    continue
                
                present_height = position[2]
                
                control_output = altitude_control.compute(target_height, present_height)
                tello.send_rc_control(0, 0, int(control_output), 0)
                
                print(f"Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]m | Target: {target_height}m | Error: {target_height - present_height:.3f}m | Frames: {num_frames}")
                
                if abs(target_height - present_height) <= WAYPOINT_TOLERANCE:
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
            pass  # Ignoring errors during cleanup

if __name__ == "__main__":
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_desc) # Handlers
    streaming_client.on_data_frame_received_event.handlers.append(receive_frame) # Handlers
    
    with streaming_client:
        streaming_client.request_modeldef()
        
        print("Waiting for Motive data...")
        timeout = time.time() + 2  # 2s timeout
        
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
        