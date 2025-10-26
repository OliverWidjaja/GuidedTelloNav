from djitellopy import Tello
import time
import logging

class TelloController:
    def __init__(self, debug=False):
        """
        Initialize Tello drone controller
        
        Args:
            debug (bool): Enable debug logging
        """
        self.tello = Tello()
        self.is_connected = False
        self.debug = debug
        
        log_level: int = logging.WARN # del to debug
        Tello.LOGGER.setLevel(log_level) # del to debug
        
        # Setup logging
        logging.basicConfig(level=logging.DEBUG if debug else logging.CRITICAL) # or INFO/ WARNING/ CRITICAL
        self.logger = logging.getLogger(__name__)
        
        # State variables
        self.battery_level = 0
        self.flight_time = 0
        
    def connect(self):
        """Connect to Tello drone"""
        try:
            self.tello.connect()
            self.is_connected = True
            self.battery_level = self.tello.get_battery()
            self.logger.info(f"Connected to Tello! Battery: {self.battery_level}%")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from Tello drone"""
        if self.is_connected:
            self.tello.end()
            self.is_connected = False
            self.logger.info("Disconnected from Tello")
    
    def takeoff(self):
        """Takeoff drone"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            self.tello.takeoff()
            self.logger.info("Takeoff command sent")
            return True
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            return False
    
    def land(self):
        """Land drone"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            self.tello.land()
            self.logger.info("Land command sent")
            return True
        except Exception as e:
            self.logger.error(f"Land failed: {e}")
            return False
    
    def emergency_stop(self):
        """Emergency stop - immediately stops motors"""
        if not self.is_connected:
            return False
        
        try:
            self.tello.emergency()
            self.logger.warning("EMERGENCY STOP activated")
            return True
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")
            return False
    
    # Movement commands
    def move_forward(self, distance):
        """Move forward by specified distance (cm)"""
        return self._safe_move(self.tello.move_forward, distance)
    
    def move_back(self, distance):
        """Move backward by specified distance (cm)"""
        return self._safe_move(self.tello.move_back, distance)
    
    def move_left(self, distance):
        """Move left by specified distance (cm)"""
        return self._safe_move(self.tello.move_left, distance)
    
    def move_right(self, distance):
        """Move right by specified distance (cm)"""
        return self._safe_move(self.tello.move_right, distance)
    
    def move_up(self, distance):
        """Move up by specified distance (cm)"""
        return self._safe_move(self.tello.move_up, distance)
    
    def move_down(self, distance):
        """Move down by specified distance (cm)"""
        return self._safe_move(self.tello.move_down, distance)
    
    def _safe_move(self, move_func, distance):
        """Safe wrapper for movement functions"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            result = move_func(distance)
            self.logger.debug(f"Move command: {move_func.__name__}({distance}cm) - Result: {result}")
            return result
        except Exception as e:
            self.logger.error(f"Move command failed: {e}")
            return False
    
    def rotate_clockwise(self, degrees):
        """Rotate clockwise by specified degrees"""
        return self._safe_rotate(self.tello.rotate_clockwise, degrees)
    
    def rotate_counter_clockwise(self, degrees):
        """Rotate counter-clockwise by specified degrees"""
        return self._safe_rotate(self.tello.rotate_counter_clockwise, degrees)
    
    def _safe_rotate(self, rotate_func, degrees):
        """Safe wrapper for rotation functions"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            result = rotate_func(degrees)
            self.logger.debug(f"Rotate command: {rotate_func.__name__}({degrees}°) - Result: {result}")
            return result
        except Exception as e:
            self.logger.error(f"Rotate command failed: {e}")
            return False
    
    def go_xyz_speed(self, x, y, z, speed):
        """
        Move to relative position (x, y, z) at specified speed
        
        Args:
            x (int): X distance (cm)
            y (int): Y distance (cm)
            z (int): Z distance (cm)
            speed (int): Speed (cm/s)
        """
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            result = self.tello.go_xyz_speed(x, y, z, speed)
            self.logger.info(f"Go to position: X:{x}cm, Y:{y}cm, Z:{z}cm, Speed:{speed}cm/s")
            return result
        except Exception as e:
            self.logger.error(f"Go XYZ command failed: {e}")
            return False
    
    def send_rc_control(self, left_right, forward_back, up_down, yaw):
        """
        Send RC control commands for manual control
        
        Args:
            left_right: Left/Right velocity (-100 to 100)
            forward_back: Forward/Backward velocity (-100 to 100)
            up_down: Up/Down velocity (-100 to 100)
            yaw: Yaw velocity (-100 to 100)
        """
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            self.tello.send_rc_control(left_right, forward_back, up_down, yaw)
            return True
        except Exception as e:
            self.logger.error(f"RC control failed: {e}")
            return False
    
    # Sensor data methods
    def get_battery(self):
        """Get battery level"""
        if not self.is_connected:
            return 0
        
        try:
            self.battery_level = self.tello.get_battery()
            return self.battery_level
        except Exception as e:
            self.logger.error(f"Failed to get battery: {e}")
            return 0
    
    def get_distance_tof(self):
        """Get time-of-flight distance (cm)"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_distance_tof()
        except Exception as e:
            self.logger.error(f"Failed to get TOF distance: {e}")
            return 0
    
    def get_height(self):
        """Get current height (cm)"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_height()
        except Exception as e:
            self.logger.error(f"Failed to get height: {e}")
            return 0
    
    # IMU data methods
    def get_acceleration_x(self):
        """Get X-axis acceleration"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_acceleration_x()
        except Exception as e:
            self.logger.error(f"Failed to get X acceleration: {e}")
            return 0
    
    def get_acceleration_y(self):
        """Get Y-axis acceleration"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_acceleration_y()
        except Exception as e:
            self.logger.error(f"Failed to get Y acceleration: {e}")
            return 0
    
    def get_acceleration_z(self):
        """Get Z-axis acceleration"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_acceleration_z()
        except Exception as e:
            self.logger.error(f"Failed to get Z acceleration: {e}")
            return 0
    
    def get_yaw(self):
        """Get yaw angle"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_yaw()
        except Exception as e:
            self.logger.error(f"Failed to get yaw: {e}")
            return 0
    
    def get_pitch(self):
        """Get pitch angle"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_pitch()
        except Exception as e:
            self.logger.error(f"Failed to get pitch: {e}")
            return 0
    
    def get_roll(self):
        """Get roll angle"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_roll()
        except Exception as e:
            self.logger.error(f"Failed to get roll: {e}")
            return 0
    
    def get_speed_x(self):
        """Get X-axis speed"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_speed_x()
        except Exception as e:
            self.logger.error(f"Failed to get X speed: {e}")
            return 0
    
    def get_speed_y(self):
        """Get Y-axis speed"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_speed_y()
        except Exception as e:
            self.logger.error(f"Failed to get Y speed: {e}")
            return 0
    
    def get_speed_z(self):
        """Get Z-axis speed"""
        if not self.is_connected:
            return 0
        
        try:
            return self.tello.get_speed_z()
        except Exception as e:
            self.logger.error(f"Failed to get Z speed: {e}")
            return 0
    
    # System commands
    def query_serial_number(self):
        """Get drone serial number"""
        if not self.is_connected:
            return None
        
        try:
            return self.tello.query_serial_number()
        except Exception as e:
            self.logger.error(f"Failed to get serial number: {e}")
            return None
    
    def reboot(self):
        """Reboot the drone"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            self.tello.reboot()
            self.logger.info("Reboot command sent")
            return True
        except Exception as e:
            self.logger.error(f"Reboot failed: {e}")
            return False
    
    def send_keepalive(self):
        """Send keepalive signal"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            self.tello.send_keepalive()
            return True
        except Exception as e:
            self.logger.error(f"Keepalive failed: {e}")
            return False
    
    def get_all_sensor_data(self):
        """Get comprehensive sensor data"""
        return {
            'battery': self.get_battery(),
            'height': self.get_height(),
            'tof_distance': self.get_distance_tof(),
            'acceleration': {
                'x': self.get_acceleration_x(),
                'y': self.get_acceleration_y(),
                'z': self.get_acceleration_z()
            },
            'orientation': {
                'yaw': self.get_yaw(),
                'pitch': self.get_pitch(),
                'roll': self.get_roll()
            },
            'speed': {
                'x': self.get_speed_x(),
                'y': self.get_speed_y(),
                'z': self.get_speed_z()
            }
        }
    
    def print_status(self):
        """Print current drone status"""
        if not self.is_connected:
            print("Drone not connected")
            return
        
        sensor_data = self.get_all_sensor_data()
        print("\n=== Tello Status ===")
        print(f"Battery: {sensor_data['battery']}%")
        print(f"Height: {sensor_data['height']}cm")
        print(f"TOF Distance: {sensor_data['tof_distance']}cm")
        print(f"Orientation - Yaw: {sensor_data['orientation']['yaw']}°, "
              f"Pitch: {sensor_data['orientation']['pitch']}°, "
              f"Roll: {sensor_data['orientation']['roll']}°")
        print("===================\n")

# Example usage
if __name__ == "__main__":
    drone = TelloController(debug=True)
    
    if drone.connect():
        # Print initial status
        drone.print_status()
        
        # Example flight sequence
        if drone.takeoff():
            time.sleep(2)
            
            # Move around
            drone.move_up(50)
            time.sleep(2)
            
            drone.move_forward(50)
            time.sleep(2)
            
            drone.rotate_clockwise(90)
            time.sleep(2)
            
            # Print status during flight
            drone.print_status()
            
            # Land
            drone.land()
        
        drone.disconnect()