import numpy as np
import time
import asyncio
import math
from bleak import BleakClient
from pyvesc.protocol.interface import encode, encode_request, decode
from pyvesc.VESC.messages import SetPosition, GetValues
from natnet_client import DataDescriptions, DataFrame, NatNetClient

# Bluetooth configuration
ADDRESS = "CF:9D:22:EF:60:F9"
RX_CHARACTERISTIC = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHARACTERISTIC = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Motion capture variables
id_name = {}
rigid_bodies = {}
num_frames = 0

# Velocity tracking variables
prev_position = None
prev_time = None
velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s

# Control parameters
SPOOL_RATE = 100.0  # degrees per meter of distance difference
UPDATE_RATE = 0.01  # seconds

first_run = True

class BluetoothVESC:    
    def __init__(self, client, rx_characteristic, tx_characteristic):
        self.client = client
        self.rx_characteristic = rx_characteristic
        self.tx_characteristic = tx_characteristic
        self._buffer = bytearray()
        
        # Pre-encode messages for efficiency
        self._primary_request_msg = encode_request(GetValues())
        self._secondary_request_msg = encode_request(GetValues(can_id=119))
    
    async def set_pos(self, new_pos, can_id=None):
        if can_id is not None:
            buffer = encode(SetPosition(new_pos, can_id=can_id))
        else:
            buffer = encode(SetPosition(new_pos))

        await self.client.write_gatt_char(self.rx_characteristic, buffer, response=False)
    
    async def get_position(self, can_id=None):
        """Get current position from VESC (primary or secondary)"""
        try:
            # Clear buffer before new request
            self._buffer.clear()
            
            # Send request
            if can_id is not None:
                await self.client.write_gatt_char(self.rx_characteristic, self._secondary_request_msg)
            else:
                await self.client.write_gatt_char(self.rx_characteristic, self._primary_request_msg)
            
            # Wait for response with timeout
            response = await self._wait_for_response(timeout=1.0)
            
            if response is not None:
                # Extract position from response
                if hasattr(response, 'pid_pos_now'):
                    return response.pid_pos_now
                else:
                    print(f"No position data in response")
                    return None
            else:
                print("No response received")
                return None
                
        except Exception as e:
            print(f"Error getting position: {e}")
            return None
    
    async def _wait_for_response(self, timeout=1.0):
        """Wait for and decode response with timeout"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if len(self._buffer) > 0:
                try:
                    response, consumed = decode(bytes(self._buffer))
                    if consumed > 0:
                        self._buffer = self._buffer[consumed:]
                        return response
                except Exception as e:
                    print(f"Error decoding response: {e}")
                    self._buffer.clear()
                    return None
            
            await asyncio.sleep(0.01)
        
        print("Timeout waiting for response")
        return None
    
    async def notification_handler(self, sender, data):
        """Handle incoming notifications (responses from VESCs)"""
        # Add new data to buffer
        self._buffer.extend(data)

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

def get_pose(name):
    """Get current position and yaw orientation of a rigid body by name"""
    data = rigid_bodies.get(name)
    if data and "position" in data and "orientation" in data:
        position = data["position"]  # [x, y, z]
        position[1] = -position[1]  # Invert Y-axis
        yaw = data["orientation"]["euler"][0]  # yaw angle in radians

        current_time = time.time()
        current_velocity = calculate_velocity(position, current_time)

        return position, yaw, current_velocity
    return None, None, None

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance between two 3D positions"""
    if pos1 is None or pos2 is None:
        print("err calc dist")
        return None
    
    dist = math.sqrt(
        (pos1[0] - pos2[0])**2 +
        (pos1[1] - pos2[1])**2 +
        (pos1[2] - pos2[2])**2
    )
    return float(dist)

async def active_mode(motor: BluetoothVESC):
    """Control VESC motor based on distance between VESC and Tello"""
    
    # Get initial positions to establish baseline
    print("Getting current positions from VESCs...")
    
    angle_1_offset = await motor.get_position(can_id=None)  # Primary VESC
    if angle_1_offset is None:
        print("Warning: Could not get position from primary VESC, using 0 as offset")
        angle_1_offset = 0
    
    angle_2_offset = await motor.get_position(can_id=119)  # Secondary VESC
    if angle_2_offset is None:
        print("Warning: Could not get position from secondary VESC, using 0 as offset")
        angle_2_offset = 0

    print(f"Initial VESC positions - Primary: {angle_1_offset}, Secondary: {angle_2_offset}")
    
    # Initialize motion capture client
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_frame)
    
    with streaming_client:
        streaming_client.request_modeldef()
        
        print("Starting distance-based motor control...")
        
        try:
            while True:
                # Update motion capture data
                streaming_client.update_sync()
                
                # Get current positions
                vesc_position, _, _ = get_pose("VESC")
                tello_position_12, _, _ = get_pose("Tello12")
                tello_position_119, _, _ = get_pose("Tello119")

                global first_run
                if first_run:
                    offset12 = calculate_distance(vesc_position, tello_position_12)
                    offset119 = calculate_distance(vesc_position, tello_position_119)
                    if offset12 is None or offset119 is None:
                        print("Waiting for initial Tello positions...")
                        await asyncio.sleep(UPDATE_RATE)
                        continue
                    first_run = False
                    print(f"Initial offsets set - Tello12: {offset12}m, Tello119: {offset119}m")

                if vesc_position and tello_position_12 and tello_position_119:
                    # Calculate current distance
                    dist12 = calculate_distance(vesc_position, tello_position_12)
                    dist119 = calculate_distance(vesc_position, tello_position_119)
                    if dist12 is None or dist119 is None:
                        print("unable to get pos...")
                        await asyncio.sleep(UPDATE_RATE)
                        continue

                    # Calculate distance error
                    dist_error_12 = dist12 - offset12
                    dist_error_119 = dist119 - offset119

                    # Calculate motor adjustment
                    # Positive error: Tello is too far -> spool out (increase angle)
                    # Negative error: Tello is too close -> spool in (decrease angle)
                    angle_adjust_12 = dist_error_12 * SPOOL_RATE
                    angle_adjust_119 = dist_error_119 * SPOOL_RATE
                    
                    # Apply limits
                    angle_adjust_12 = max(min(angle_adjust_12, 360), -360)
                    angle_adjust_119 = max(min(angle_adjust_119, 360), -360)

                    # Calculate new target angles
                    target_angle_12 = angle_1_offset + angle_adjust_12
                    target_angle_119 = angle_2_offset + angle_adjust_119

                    # Send commands to both motors
                    await motor.set_pos(target_angle_12)
                    await motor.set_pos(target_angle_119, can_id=119)
                    
                    # Print status
                    print(f"Dist12: {dist12}m (Err: {dist_error_12}m) | "
                          f"Dist119: {dist119}m (Err: {dist_error_119}m) | ")

                else:
                    print("Waiting for both VESC and Tello positions...")
                
                # Wait before next update
                await asyncio.sleep(UPDATE_RATE)
                
        except KeyboardInterrupt:
            print("Motor control stopped by user")

async def main():
    async with BleakClient(ADDRESS) as client:
        print(f"Connected: {client.is_connected}")

        try:
            paired = await client.pair(protection_level=2)
            print(f"Paired: {paired}")
        except Exception as e:
            print(f"Pairing failed or not required: {e}")

        motor = BluetoothVESC(client, RX_CHARACTERISTIC, TX_CHARACTERISTIC)
        print("Motor object created")

        # Set up notification handler for receiving responses
        await client.start_notify(TX_CHARACTERISTIC, motor.notification_handler)
        print("Notification handler set up")   

        # Run distance-based motor control
        await active_mode(motor)

        # Clean up
        await client.stop_notify(TX_CHARACTERISTIC)

if __name__ == "__main__":
    asyncio.run(main())