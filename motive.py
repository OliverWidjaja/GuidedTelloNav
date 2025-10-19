from natnet_client import DataDescriptions, DataFrame, NatNetClient
import socket
import json
import time
import math


UDP_IP = "127.0.0.1"
UDP_PORT = 5059
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Create a global mapping from rigid body ID to name
rigid_body_id_to_name = {}

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
    global num_frames
    num_frames += 1
    
    # Create a nested dictionary for all rigid bodies
    rigid_bodies_data = {}
    
    for rigid_body in getattr(data_frame, "rigid_bodies", []):
        name = rigid_body_id_to_name.get(rigid_body.id_num, f"RigidBody_{rigid_body.id_num}")
        pos = rigid_body.pos
        rot = rigid_body.rot  # 4D Quaternion

        # Convert quaternion to Euler angles
        yaw, pitch, roll = quaternion_to_euler(rot[0], rot[1], rot[2], rot[3])
        
        # Add this rigid body to the dictionary
        rigid_bodies_data[name] = {
            "position": [float(f"{p:.4f}") for p in pos],
            "orientation": {
                "quaternion": [float(f"{r:.4f}") for r in rot],
                "euler": [float(f"{yaw:.4f}"), float(f"{pitch:.4f}"), float(f"{roll:.4f}")]
            },
            "id": rigid_body.id_num
        }
    
    # Create the main data dictionary
    data = {
        "timestamp": time.time(),
        "frame_count": num_frames,
        "rigid_bodies": rigid_bodies_data
    }
    
    # Send the complete nested dictionary
    sock.sendto(json.dumps(data).encode(), (UDP_IP, UDP_PORT))

num_frames = 0

if __name__ == "__main__":
    streaming_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

    with streaming_client:
        streaming_client.request_modeldef()
        while True:
            time.sleep(0.1)
            streaming_client.update_sync()
            print(f"Received {num_frames} frames.")