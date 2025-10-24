import time

class PController:
    def __init__(self, kp, max_output=100, min_output=-100):
        self.kp = kp
        self.max_output = max_output
        self.min_output = min_output
    
    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        
        output = self.kp * error
        
        output = max(self.min_output, min(self.max_output, output))
        
        return output

class PDController:
    def __init__(self, kp, kd, max_output=100, min_output=-100):
        self.kp = kp
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0
        self.prev_time = None
    
    def compute(self, setpoint, current_value, current_velocity=0, target_velocity=0):
        error = setpoint - current_value
        current_time = time.time()
        
        p_term = self.kp * error
        
        velocity_error = target_velocity - current_velocity # target_velocity is 0, this becomes -current_velocity
        d_term = self.kd * velocity_error
        
        # Alternative implementation using error derivative (commented out)
        # if self.prev_time is not None and current_time > self.prev_time:
        #     dt = current_time - self.prev_time
        #     if dt > 0:
        #         error_derivative = (error - self.prev_error) / dt
        #         d_term = self.kd * error_derivative
        #     else:
        #         d_term = 0
        # else:
        #     d_term = 0
        
        self.prev_error = error
        self.prev_time = current_time
        
        output = p_term + d_term

        output = max(self.min_output, min(self.max_output, output))
        
        return output
    
if __name__ == "__main__":
    controller = PController(kp=100.0)
    setpoint = 1
    current_value = 0.5 
    output = controller.compute(setpoint, current_value)
    print(f"Control output: {output}")