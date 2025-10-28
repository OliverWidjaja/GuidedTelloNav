from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PController
from scipy.interpolate import PchipInterpolator
import numpy as np
import math, time
import asyncio
from bleak import BleakClient
from pyvesc.protocol.interface import encode
from pyvesc.VESC.messages import SetCurrent, SetCurrentBrake, SetDutyCycle, SetPosition, SetRPM

# VESC Bluetooth configuration
VESC_ADDRESS = "CF:9D:22:EF:60:F9"
VESC_RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
VESC_TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

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

class BluetoothVESC:    
    def __init__(self, client, rx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
    
    async def set_pos(self, new_pos, can_id=None):
        if can_id is not None:
            buffer = encode(SetPosition(new_pos, can_id=can_id))
        else:
            buffer = encode(SetPosition(new_pos))
        
        print(f"Sending position command: {new_pos}° (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

    async def set_current(self, new_current, can_id=None):
        if can_id is not None:
            buffer = encode(SetCurrent(new_current, can_id=can_id))
        else:
            buffer = encode(SetCurrent(new_current))
        
        print(f"Sending current command: {new_current} mA (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

    async def set_current_brake(self, new_current, can_id=None):
        if can_id is not None:
            buffer = encode(SetCurrentBrake(new_current, can_id=can_id))
        else:
            buffer = encode(SetCurrentBrake(new_current))
        
        print(f"Sending current command: {new_current} mA (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

    async def set_duty_cycle(self, new_duty_cycle, can_id=None):
        if can_id is not None:
            buffer = encode(SetDutyCycle(new_duty_cycle, can_id=can_id))
        else:
            buffer = encode(SetDutyCycle(new_duty_cycle))
        
        print(f"Sending duty cycle command: {new_duty_cycle} (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

    async def set_rpm(self, new_rpm, can_id=None):
        if can_id is not None:
            buffer = encode(SetRPM(new_rpm, can_id=can_id))
        else:
            buffer = encode(SetRPM(new_rpm))
        
        print(f"Sending RPM command: {new_rpm} RPM (CAN ID: {can_id})")  
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

async def as_takeoff(tello_controller, vesc_motor):
    """
    Run Tello takeoff asynchronously while simultaneously controlling VESC
    Returns when takeoff is complete
    """    
    takeoff_complete = False
    
    async def vesc_when_takeoff(dt=0.05):
        """VESC control loop that runs during takeoff"""
        while not takeoff_complete:
            # manual timing correction
            time.sleep(0.25) # (blocking)
            await vesc_motor.set_current(0.70)
            await asyncio.sleep(dt)
    
    async def takeoff_sequence():
        """Run the blocking takeoff command in a thread"""
        nonlocal takeoff_complete
        try:
            # blocking command
            tello_controller.takeoff()

            # stabilize period
            time.sleep(5.0)
            takeoff_complete = True
            print("Tello takeoff command completed")
        except Exception as e:
            print(f"Error during takeoff: {e}")
            exit(1)
    
    # Start both tasks concurrently
    vesc_task = asyncio.create_task(vesc_when_takeoff())
    takeoff_task = asyncio.create_task(takeoff_sequence())
    
    await takeoff_task
    
    await asyncio.sleep(0.1)
    
    vesc_task.cancel()
    try:
        await vesc_task
    except asyncio.CancelledError:
        pass
    
    print("Takeoff sequence finished")
    return True

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

def get_pose(name="Tello"):
    """Get position and yaw orientation of object by name"""
    tello_data = rigid_bodies.get(name)
    if tello_data and "position" in tello_data and "orientation" in tello_data:
        position = tello_data["position"]  # [x, y, z]
        position[1] = -position[1]  # Invert Y-axis
        yaw = tello_data["orientation"]["euler"][0]  # yaw angle in radians

        current_time = time.time()
        current_velocity = calculate_velocity(position, current_time)

        return position, yaw, current_velocity
    return None, None, None

async def run_mission_async(streaming_client: NatNetClient, vesc_motor: BluetoothVESC):
    traj = interpolate_traj(waypoints=WAYPOINTS, num_points=steps_traj)
    
    try:
        if not tello.connect():
            print("Failed to connect to Tello")
            return
        
        battery = tello.get_battery()
        print(f"Tello battery: {battery}%")
        if battery is not None and battery > 30:
            # Run asynchronous takeoff with VESC control
            await as_takeoff(tello, vesc_motor)
        else:
            print("low bat")
            exit(0)
        
        print(f"Mission started with {len(traj)} waypoints")
        
        # # Main mission loop - VESC commands are stopped after takeoff
        # for i, waypoint in enumerate(traj):
        #     target_x, target_y, target_z, target_yaw = waypoint
        #     print(f"Waypoint {i+1}/{len(traj)}: "
        #           f"X={target_x:.2f}m, Y={target_y:.2f}m, Z={target_z:.2f}m, Yaw={math.degrees(target_yaw):.1f}°")
            
        #     waypoint_start = time.time()
        #     waypoint_reached = False

        #     while time.time() - waypoint_start < 1.5:  # 1.5s timeout per waypoint
        #         streaming_client.update_sync()

        #         position, current_yaw, current_velocity = get_pose(tello)
        #         if position is None or current_yaw is None or current_velocity is None:
        #             await asyncio.sleep(0.01)
        #             continue
                                
        #         error_x = target_x - position[0]
        #         error_y = target_y - position[1]  
        #         error_z = target_z - position[2]
                
        #         # Normalize yaw error to shortest path
        #         error_yaw = normalize_angle(target_yaw - current_yaw)
                
        #         control_x = x_control.compute(target_x, position[0], current_velocity[0])
        #         control_y = y_control.compute(target_y, position[1], current_velocity[1])
        #         control_z = z_control.compute(target_z, position[2], current_velocity[2])
                
        #         control_yaw = yaw_control.compute(target_yaw, current_yaw, 0)
        #         control_yaw = -control_yaw  # Invert yaw control for Tello
                
        #         # Send RC control to drone (VESC commands are stopped after takeoff)
        #         tello.send_rc_control(int(control_y), int(control_x), int(control_z), int(control_yaw))
                
        #         position_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        #         if position_error <= WAYPOINT_TOLERANCE and abs(error_yaw) <= YAW_TOLERANCE:
        #             waypoint_reached = True
        #             break

        #         await asyncio.sleep(0.01)  # Control loop rate (~100Hz)
            
        #     if not waypoint_reached:
        #         print(f"⚠️  Timeout on waypoint {i+1}, proceeding to next waypoint")
        
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
            # Ensure VESC is stopped (in case of interruption during takeoff)
            await vesc_motor.set_current(0)
            tello.land()
            tello.disconnect()
        except:
            pass  # Ignore errors during cleanup

async def main():
    # Initialize VESC connection
    try:
        vesc_client = BleakClient(VESC_ADDRESS)
        await vesc_client.connect()
        print(f"VESC Connected: {vesc_client.is_connected}")

        paired = await vesc_client.pair(protection_level=2)
        print(f"VESC Paired: {paired}")

        vesc_motor = BluetoothVESC(vesc_client, VESC_RX_CHARACTERISTIC)
        print("VESC Motor object created")

        # Initialize OptiTrack
        streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
        streaming_client.on_data_description_received_event.handlers.append(receive_desc)
        streaming_client.on_data_frame_received_event.handlers.append(receive_frame)
        
        with streaming_client:
            streaming_client.request_modeldef()
            
            # Run the mission with VESC control
            await run_mission_async(streaming_client, vesc_motor)

    except Exception as e:
        print(f"Failed to connect to VESC: {e}")
    finally:
        # Cleanup VESC connection
        if 'vesc_client' in locals() and vesc_client.is_connected:
            await vesc_motor.set_current(0)  # Stop motor
            await vesc_client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())