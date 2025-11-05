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
import os
from typing import Union
import uuid
from bleak.backends.characteristic import BleakGATTCharacteristic
import logging

# VESC Bluetooth configuration
BLE_ADDRESS = "CF:9D:22:EF:60:F9"
VESC_RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
VESC_TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Tello WiFi SSIDs
# TELLO-B6426A
# TELLO-56FD1B

# Waypoints: [x, y, z, yaw] in meters and radians
WAYPOINTS = [
    [0.0, 0.0, 0.75, 0.0],
    [0, 0.0, 0.5, 0],
    [0, 0.0, 0.75, 0],
]
waypoint_reached = False

# Control parameters
WAYPOINT_TOLERANCE = 0.05  # meters for position
YAW_TOLERANCE = 0.12      # radians for yaw
KP = [150, 150, 150, 60]  # [Kp_x, Kp_y, Kp_z, Kp_yaw]
timeout = 1.5

# Optitrack variables
id_name = {}  # mapping from ID to name
rigid_bodies = {}
num_frames = 0

# Velocity variables
prev_position = None
prev_time = None
velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s

VESC_DT = 0.005

x_controller = PController(kp=KP[0])
y_controller = PController(kp=KP[1])
z_controller = PController(kp=KP[2])
yaw_controller = PController(kp=KP[3])


class BluetoothVESC:    
    def __init__(self, client: BleakClient, rx_characteristic: Union[BleakGATTCharacteristic, int, str, uuid.UUID], control_period: float = 0.001, debug: bool = False):
        self.client: BleakClient = client
        self.rx_characteristic: Union[BleakGATTCharacteristic, int, str, uuid.UUID] = rx_characteristic
        self.control_period = control_period
        self.last_control_timestamp = time.time()

        # Setup logging
        logging.basicConfig(level=logging.DEBUG if debug else logging.WARN) # or INFO/ WARNING/ CRITICAL
        self.logger = logging.getLogger('BluetoothVESC')
    
    async def set_pos(self, new_pos, can_id=None):
        if can_id is not None:
            buffer = encode(SetPosition(new_pos, can_id=can_id))
        else:
            buffer = encode(SetPosition(new_pos))
        
        self.logger.info(f"Sending position command: {new_pos}° (CAN ID: {can_id})")
        if time.time() - self.last_control_timestamp > self.control_period:
            self.last_control_timestamp = time.time()
            await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

    async def set_current(self, new_current, can_id=None):
        if can_id is not None:
            buffer = encode(SetCurrent(new_current, can_id=can_id))
        else:
            buffer = encode(SetCurrent(new_current))
        
        self.logger.info(f"Sending current command: {new_current} mA (CAN ID: {can_id})")
        if time.time() - self.last_control_timestamp > self.control_period:
            self.last_control_timestamp = time.time()
            await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)
    
    async def set_duty(self, new_duty, can_id=None):
        if can_id is not None:
            buffer = encode(SetDutyCycle(new_duty, can_id=can_id))
        else:
            buffer = encode(SetDutyCycle(new_duty))
        
        self.logger.info(f"Sending duty command: {new_duty} mA (CAN ID: {can_id})")
        if time.time() - self.last_control_timestamp > self.control_period:
            self.last_control_timestamp = time.time()
            await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)

async def as_takeoff_land(tello_controller: TelloController, vesc_motor: BluetoothVESC):
    """
    Asynchronously run Tello (takeoff + landing) while simultaneously controlling VESC
    Returns when landing is complete
    """    
    takeoff_complete = asyncio.Event()
    landing_complete = asyncio.Event()

    async def amp_controller(dt=VESC_DT):
        profile_time = time.time()
        while not takeoff_complete.is_set() and not landing_complete.is_set():
            if time.time() - profile_time < 0.3:
                print("send da duty")
                await vesc_motor.set_duty(0.03, can_id=0x77)
            else:
                print("send da takeoff cur")
                await vesc_motor.set_current(-0.05, can_id=0x77)
            await asyncio.sleep(dt)
        
        while takeoff_complete.is_set() and not landing_complete.is_set():
            print("send da landing cur")
            await vesc_motor.set_current(-0.35, can_id=0x77)
            await asyncio.sleep(dt)
            
    async def takeoff_sequence():
        """Run the blocking takeoff command in a thread"""
        try:
            ret = await asyncio.to_thread(tello_controller.takeoff)
            if ret: 
                takeoff_complete.set()
            else:
                raise Exception("takeoff fail")
            await asyncio.sleep(3) # Mysterious settling time
        except Exception as e:
            print(f"Error during takeoff: {e}")
            return False
        return True
    
    async def landing_sequence():
        """Run the blocking landing command in a thread"""
        try:
            await asyncio.to_thread(tello_controller.land)
            await asyncio.sleep(5) # Double check this settling time
            landing_complete.set()
        except Exception as e:
            print(f"Error during landing: {e}")
            return False
        return True
    
    try:
        # Start forever-amp controller
        vesc_task = asyncio.create_task(amp_controller())

        # Take-off block; Is time should vesc or takeoff task be created first?
        print("Start takeoff")
        takeoff_task = asyncio.create_task(takeoff_sequence())
        start_time = time.time()
        takeoff_success = await takeoff_task
        if takeoff_success:
            print(f"Takeoff completed. Duration: {time.time() - start_time:.2f} seconds")
        else:
            vesc_task.cancel()
            raise Exception("Takeoff failed")

        # Landing block
        print("Start landing")
        landing_task = asyncio.create_task(landing_sequence())
        landing_success = await landing_task
        if landing_success:
            print(f"Landing completed. Duration: {time.time() - start_time:.2f} seconds")
        else:
            vesc_task.cancel()
            raise Exception("Landing failed")
        
        vesc_task.cancel()
        
        return True
    except Exception as e:
        print(f"Something went wrong: {e}")
        
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

async def as_mission(streaming_client: NatNetClient, vesc_motor: BluetoothVESC, tello: TelloController):    
    # traj = pchip_traj(waypoints=WAYPOINTS, num_points=int(round(1/VESC_DT)))

    try:
        battery = tello.get_battery()
        if battery is not None and battery < 20:
            print(f"Low battery warning {battery}%")
            raise Exception("Low battery")
        else:
            print(f"Tello battery: {battery}%")

        await as_takeoff_land(tello, vesc_motor)
        
        print("Mission complete! Landing...")
    except KeyboardInterrupt:
        """ Hard landing """
        print("Mission interrupted by user")
        tello.land()

async def main():
    # # Tello Network Connection
    # os.system(f'''cmd /c "netsh wlan connect name=TELLO-56FD1B"''')

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
            
            await as_mission(streaming_client, vesc_motor, tello)

    tello.disconnect()

if __name__ == "__main__":
    asyncio.run(main())