from natnet_client import DataDescriptions, DataFrame, NatNetClient
from tello import TelloController
from controllers import PController
from scipy.interpolate import PchipInterpolator
import numpy as np
import math, time
import asyncio
from bleak import BleakClient
from pyvesc.protocol.interface import encode
from pyvesc.VESC.messages import SetCurrent, SetPosition, SetDutyCycle
from typing import Union
import uuid
from bleak.backends.characteristic import BleakGATTCharacteristic
import logging
from enum import Enum

# VESC Bluetooth configuration
BLE_ADDRESS = "D5:38:71:28:C1:36"
VESC_RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
VESC_TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Waypoints: [x, y, z, yaw] in meters and radians
WAYPOINTS = [
    [0.0, 0.0, 0.75, 0.0],
    [0, 0.0, 0.5, 0],
    [0, 0.0, 0.75, 0],
]

# Control parameters
WAYPOINT_TOLERANCE = 0.05  # meters for position
YAW_TOLERANCE = 0.12      # radians for yaw
KP = [150, 150, 150, 60]  # [Kp_x, Kp_y, Kp_z, Kp_yaw]
TIMEOUT = 1.5
IDLE_TAKEOFF_TIME = 3
VESC_DT = 0.05

class State(Enum):
    IDLE = 0
    TAKEOFF = 1
    ALTITUDE_CONTROL = 2
    LANDING = 3
    IDLE_2 = 4

# Optitrack variables
id_name = {}  # mapping from ID to name
rigid_bodies = {}
num_frames = 0
tello_pose_lock = asyncio.Lock()  # Changed to asyncio Lock for async access
tello_pose = None  # Will store [position, yaw, velocity]

# Velocity variables
prev_position = None
prev_time = None
velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s

x_controller = PController(kp=KP[0])
y_controller = PController(kp=KP[1])
z_controller = PController(kp=KP[2])
yaw_controller = PController(kp=KP[3])

state = State.IDLE
last_event_time = time.time()
waypoint_reached = False

class BluetoothVESC:    
    def __init__(self, client: BleakClient, rx_characteristic: Union[BleakGATTCharacteristic, int, str, uuid.UUID], control_period: float = 0.001, debug: bool = False):
        self.client: BleakClient = client
        self.rx_characteristic: Union[BleakGATTCharacteristic, int, str, uuid.UUID] = rx_characteristic

        # Setup logging
        logging.basicConfig(level=logging.DEBUG if debug else logging.WARN)
        self.logger = logging.getLogger('BluetoothVESC')
        self.last_call_time = time.time()
    
    async def set_pos(self, new_pos, can_id=None):
        if can_id is not None:
            buffer = encode(SetPosition(new_pos, can_id=can_id))
        else:
            buffer = encode(SetPosition(new_pos))
        
        self.logger.info(f"Sending position command: {new_pos}° (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

    async def set_current(self, new_current, can_id=None):
        if can_id is not None:
            buffer = encode(SetCurrent(new_current, can_id=can_id))
        else:
            buffer = encode(SetCurrent(new_current))
        
        self.logger.info(f"Sending current command: {new_current} mA (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)
        time_past = time.time() - self.last_call_time
        if time_past > 0.1:
            print("Once")
        self.last_call_time = time.time()
    
    async def set_duty(self, new_duty, can_id=None):
        if can_id is not None:
            buffer = encode(SetDutyCycle(new_duty, can_id=can_id))
        else:
            buffer = encode(SetDutyCycle(new_duty))
        
        self.logger.info(f"Sending duty command: {new_duty} mA (CAN ID: {can_id})")
        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

async def VESC_thread(vesc_motor: BluetoothVESC):
    """VESC FSM Control Thread"""
    global state
    global last_event_time

    while True:
        try:
            if state == State.IDLE:
                await vesc_motor.set_current(0, can_id=0x77)
            elif state == State.TAKEOFF:                
                elapsed_time = time.time() - last_event_time
                if elapsed_time < 0.3:
                    await vesc_motor.set_duty(0.03, can_id=0x77)
                elif elapsed_time >= 0.3:
                    await vesc_motor.set_current(-0.05, can_id=0x77)
            elif state == State.ALTITUDE_CONTROL:
                await vesc_motor.set_current(0, can_id=0x77)
            elif state == State.LANDING:
                await vesc_motor.set_current(-0.35, can_id=0x77)
            await asyncio.sleep(VESC_DT)
        except Exception as e:
            print(f"VESC thread error: {e}")
            raise e

async def Tello_thread(tello: TelloController):
    """Tello FSM Control Thread"""
    global state
    global last_event_time
    mission_complete = False
    hover_start_time = None
    hover_duration = 5.0  # Hover for 5 seconds
    
    while not mission_complete:
        try:
            if state == State.IDLE:
                _ = tello.get_battery()
                if time.time() - last_event_time >= IDLE_TAKEOFF_TIME:
                    state = State.TAKEOFF
                    print("Tello: Transition to TAKEOFF state")
                    last_event_time = time.time()
                    await tello.takeoff()
            elif state == State.TAKEOFF:
                await asyncio.sleep(3)
                state = State.ALTITUDE_CONTROL
                print("Tello: Transition to ALTITUDE_CONTROL state")
                hover_start_time = time.time()
            elif state == State.ALTITUDE_CONTROL:
                current_pose = tello_pose
                
                if current_pose is not None:
                    position, yaw, velocity = current_pose
                    print(f"Tello Pose - Position: {position}, Yaw: {yaw:.3f} rad, Velocity: {velocity}")
                    
                    await tello.rc(0, 0, 0, 0)
                    
                    if hover_start_time is not None and (time.time() - hover_start_time) >= hover_duration:
                        print(f"Hovering complete ({hover_duration} seconds), transitioning to LANDING")
                        state = State.LANDING
                else:
                    print("Waiting for Tello pose data...")
                    await tello.rc(0, 0, 0, 0)
                
                await asyncio.sleep(0.1)  # Control loop frequency
                
            elif state == State.LANDING:
                print("Tello: Transition to LANDING state")
                # Stop any movement before landing
                await tello.rc(0, 0, 0, 0) 
                await asyncio.sleep(0.5)
                await tello.land()
                state = State.IDLE_2
                mission_complete = True
        
        except Exception as e:
            print(f"Tello thread error: {e}")
            raise e

async def NatNet_thread(streaming_client: NatNetClient, tello_name: str = "Tello"):
    """ NatNet Client Thread - Now async """
    global tello_pose
    
    try:
        streaming_client.run_async()
        while True:
            position, yaw, velocity = get_pose(tello_name)

            if position is not None and yaw is not None and velocity is not None:
                tello_pose = [position, yaw, velocity]

            await asyncio.sleep(0.01)  # 100 Hz update rate - changed to async sleep
    except Exception as e:
        print(f"NatNet thread error: {e}")
    finally:
        streaming_client.stop_async()
        print("NatNet thread stopped")

async def entry_2(streaming_client: NatNetClient, vesc_motor: BluetoothVESC, tello: TelloController):    
    """Main mission entry point"""
    global state
    global last_event_time
    last_event_time = time.time()
    state = State.IDLE

    # Start NatNet thread as async task
    natnet_task = asyncio.create_task(NatNet_thread(streaming_client, "Tello"))
    vesc_task = asyncio.create_task(VESC_thread(vesc_motor))
    tello_task = asyncio.create_task(Tello_thread(tello))
    
    try:
        battery = tello.get_battery()
        if battery is not None and battery < 20:
            print(f"Low battery warning {battery}% - Aborting mission")
            return
        else:
            print(f"Tello battery: {battery}%")
        
        print("Starting mission - transitioning to TAKEOFF state")
        await asyncio.gather(tello_task, natnet_task, return_exceptions=True)
        print("Mission complete!")
    except KeyboardInterrupt:
        print("Mission interrupted by user")
        state = State.IDLE_2  # Emergency landing
        await asyncio.sleep(7)  # Give time for landing
        
    except Exception as e:
        print(f"Mission error: {e}")
        state = State.IDLE_2  # Emergency landing
        await asyncio.sleep(7)  # Give time for landing
    finally:
        vesc_task.cancel()
        natnet_task.cancel()
        if not tello_task.done():
            tello_task.cancel()
        
        try:
            await asyncio.gather(vesc_task, natnet_task, tello_task, return_exceptions=True)
        except asyncio.CancelledError:
            pass

def pchip_traj(waypoints, num_points=50): # change to use VESC_DT
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

    # First run
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

def get_pose(name):
    """Get position and yaw orientation of object by name"""
    m_data = rigid_bodies.get(name)
    if m_data and "position" in m_data and "orientation" in m_data:
        position = m_data["position"]  # [x, y, z]
        position[1] = -position[1]  # Invert Y-axis
        yaw = m_data["orientation"]["euler"][0]  # yaw angle in radians

        current_time = time.time()
        current_velocity = calculate_velocity(position, current_time)

        return position, yaw, current_velocity
    return None, None, None

async def main():
    # Tello Connection
    tello = TelloController()
    tello.connect()

    # BLE VESC Connection
    async with BleakClient(BLE_ADDRESS) as BLE_client:
        print(f"VESC Connected: {BLE_client.is_connected}")

        paired = await BLE_client.pair(protection_level=2)
        print(f"VESC Paired: {paired}")

        vesc_motor = BluetoothVESC(BLE_client, VESC_RX_CHARACTERISTIC)
        print("VESC Motor object created")

        # Optitrack Connection
        streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
        streaming_client.on_data_description_received_event.handlers.append(receive_desc)
        streaming_client.on_data_frame_received_event.handlers.append(receive_frame)
        
        with streaming_client:
            streaming_client.request_modeldef()
            await entry_2(streaming_client, vesc_motor, tello)

    tello.disconnect()

if __name__ == "__main__":
    asyncio.run(main())