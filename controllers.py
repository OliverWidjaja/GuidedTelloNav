import time

class PDController:
    def __init__(self, kp, kd, max_output=100, min_output=-100):
        self.kp = kp
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.previous_error = 0
        self.last_time = time.time()
    
    def compute(self, setpoint, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
            
        error = setpoint - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        derivative_term = self.kd * derivative
        
        # Compute output
        output = proportional + derivative_term
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.previous_error = 0
        self.last_time = time.time()