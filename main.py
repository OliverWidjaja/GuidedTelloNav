import socket, json, time


UDP_IP = "127.0.0.1"
UDP_PORT = 5059

# Store data for all rigid bodies
rigid_bodies = {}
frame_count = 0
timestamp = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # Non-blocking

def update_rigid_bodies():
    global rigid_bodies, frame_count, timestamp
    try:
        data, _ = sock.recvfrom(4096)  # Increased buffer size for larger JSON
        msg = json.loads(data.decode())
        
        # Extract the nested rigid bodies data
        rigid_bodies = msg.get("rigid_bodies", {})
        frame_count = msg.get("frame_count", 0)
        timestamp = msg.get("timestamp", 0)
        
    except BlockingIOError:
        pass  # No new data 
    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e}")
    except Exception as e:
        print(f"Error receiving data: {e}")

while True:
    update_rigid_bodies()
    
    # Print all rigid bodies
    print(f"Frame: {frame_count}, Timestamp: {timestamp:.3f}")
    for name, data in rigid_bodies.items():
        pos = data.get("position", [0, 0, 0])
        euler = data.get("orientation", {}).get("euler", [0, 0, 0])
        print(f"  {name}: Pos{pos}, Euler{euler}")
    
    print("-" * 50)
    time.sleep(0.1)