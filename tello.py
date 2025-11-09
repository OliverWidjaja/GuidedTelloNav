from djitellopy import Tello
import time
import logging
import asyncio

class TelloController:
    def __init__(self, debug=True):
        """
        Initialize Tello drone controller
        
        Args:
            debug (bool): Enable debug logging
        """
        self.tello = Tello()
        self.is_connected = False
        self.debug = debug
        
        logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)
        self.logger = logging.getLogger('TelloWrapper')
        
        # State variables
        self.battery_level = 0
        self.flight_time = 0
        
    def connect(self):
        """Connect to Tello drone"""
        self.tello.connect()
        self.is_connected = True
        self.battery_level = self.tello.get_battery()
        self.logger.info(f"Connected to Tello! Battery: {self.battery_level}%")

    
    def disconnect(self):
        """Disconnect from Tello drone"""
        self.tello.end()
        if self.is_connected:
            self.is_connected = False
            self.logger.info("Disconnected from Tello")
    
    async def takeoff(self):
        """Takeoff drone"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            # Something it takes a looooot of time to take off and return a succesful takeoff.
            # So we better wait. Otherwise, it would give us an error on the following calls.
            response = "max retries exceeded"
            command = 'takeoff'
            for i in range(0, self.tello.retry_count):
                diff = time.time() - self.tello.last_received_command_timestamp
                if diff < Tello.TIME_BTW_COMMANDS:
                    self.tello.LOGGER.debug('Waiting {} seconds to execute command: {}...'.format(diff, command))
                    await asyncio.sleep(diff)

                self.tello.LOGGER.info(f"Send command: {command}")
                timestamp = time.time()

                Tello.client_socket.sendto(command.encode('utf-8'), self.tello.address)

                responses = self.tello.get_own_udp_object()['responses']

                while not responses:
                    if time.time() - timestamp > Tello.TAKEOFF_TIMEOUT:
                        message = "Aborting command '{}'. Did not receive a response after {} seconds".format(command, Tello.TIME_BTW_COMMANDS)
                        self.tello.LOGGER.warning(message)
                        print("5")
                        raise Exception(message)
                    await asyncio.sleep(0.1)  # Sleep during send command

                self.tello.last_received_command_timestamp = time.time()

                first_response = responses.pop(0)  # first datum from socket
                try:
                    response = first_response.decode("utf-8")
                except UnicodeDecodeError as e:
                    self.tello.LOGGER.error(e)
                    # return "response decode error"
                    raise Exception("response decode error")
                response = response.rstrip("\r\n")

                self.tello.LOGGER.info("Response {}: '{}'".format(command, response))
            
                if 'ok' in response.lower():
                    self.is_flying = True
                    self.logger.info("Takeoff command sent")
                    return True

                self.tello.LOGGER.debug("Command attempt #{} failed for command: '{}'".format(i, command))

            self.tello.raise_result_error(command, response)
        except Exception as e:
            self.logger.error(f"Takeoff failed: {e}")
            return False
    
    async def land(self):
        """Land drone"""
        if not self.is_connected:
            self.logger.error("Not connected to Tello")
            return False
        
        try:
            response = "max retries exceeded"
            command = 'land'
            for i in range(0, self.tello.retry_count):
                diff = time.time() - self.tello.last_received_command_timestamp
                if diff < Tello.TIME_BTW_COMMANDS:
                    self.tello.LOGGER.debug('Waiting {} seconds to execute command: {}...'.format(diff, command))
                    await asyncio.sleep(diff)

                self.tello.LOGGER.info(f"Send command: {command}")
                timestamp = time.time()

                Tello.client_socket.sendto(command.encode('utf-8'), self.tello.address)

                responses = self.tello.get_own_udp_object()['responses']

                while not responses:
                    if time.time() - timestamp > Tello.TAKEOFF_TIMEOUT:
                        message = "Aborting command '{}'. Did not receive a response after {} seconds".format(command, Tello.TIME_BTW_COMMANDS)
                        self.tello.LOGGER.warning(message)
                        print("5")
                        raise Exception(message)
                    await asyncio.sleep(0.1)  # Sleep during send command

                self.tello.last_received_command_timestamp = time.time()

                first_response = responses.pop(0)  # first datum from socket
                try:
                    response = first_response.decode("utf-8")
                except UnicodeDecodeError as e:
                    self.tello.LOGGER.error(e)
                    # return "response decode error"
                    raise Exception("response decode error")
                response = response.rstrip("\r\n")

                self.tello.LOGGER.info("Response {}: '{}'".format(command, response))
            
                if 'ok' in response.lower():
                    self.is_flying = True
                    self.logger.info("Takeoff command sent")
                    return True

                self.tello.LOGGER.debug("Command attempt #{} failed for command: '{}'".format(i, command))

            self.tello.raise_result_error(command, response)
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