class PController:
    def __init__(self, kp, max_output=100, min_output=-100):
        self.kp = kp
        self.max_output = max_output
        self.min_output = min_output
    
    def compute(self, setpoint, current_value, current_velocity):
        error = setpoint - current_value

        velocity_err = 0 - current_velocity
        d_term = 20.0 * velocity_err
        
        # Proportional term only
        output = self.kp * error + d_term

        # Clamp output
        output = max(self.min_output, min(self.max_output, output))
        
        return output
    
if __name__ == "__main__":
    print("1")
    