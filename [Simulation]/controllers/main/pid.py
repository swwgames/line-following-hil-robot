class PID:
    def __init__(self):
        self.kp = 0.0010
        self.ki = 0.0001
        self.kd = 0.0050

        self.last_error = 0.0
        self.integral_error = 0.0
    
    def compute_error(self, robot):
        """
        Compute lateral error from the front‐array:
        leftmost minus rightmost sensor.
        """
        front = robot.read_ground_sensors('front')
        left, right = front[1], front[3]
        return left - right

    def compute_control(self, robot, error):
        p = self.kp * error
        self.integral_error += error
        max_int = 5.0 / max(self.ki, 1e-6)
        self.integral_error = max(-max_int, min(max_int, self.integral_error))
        i = self.ki * self.integral_error
        d = self.kd * ((error - self.last_error) / robot.time_step)
        self.last_error = error
        return p + i + d