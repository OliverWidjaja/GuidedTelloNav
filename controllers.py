import time

class PController:
    def __init__(self, kp, max_output=100, min_output=-100):
        self.kp = kp
        self.max_output = max_output
        self.min_output = min_output
    
    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        
        # Proportional term only
        output = self.kp * error
        
        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        return output
    