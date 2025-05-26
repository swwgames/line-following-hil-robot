class LineTracer:
    def __init__(self, pid, robot):
        """Initialize the class with a valid PID controller and a robot instance.

        Args:
            pid (obj): existing PID instance
            robot (obj): existing robot instance
        """

        self.robot = robot
        self.pid = pid
        self.base_speed = 4.0
        self.max_speed = 5.0

    def step(self): 
        """Step the robot. Computes PID control and sets wheel speeds.

        Returns:
            bool: wether or not the robot is able to step.
        """
        if not self.robot.step():
            return False
        
        error = self.pid.compute_error(self.robot)
        corr = self.pid.compute_control(self.robot, error)
        ls = max(-self.max_speed, min(self.max_speed, self.base_speed + corr))
        rs = max(-self.max_speed, min(self.max_speed, self.base_speed - corr))
        self.robot.set_wheel_speeds(ls, rs)

        return True

    def pivot_into_direction(self, direction='CCW', turn_speed=2.0):
        """Spin in place toward the given direction (clockwise or counterclockwise), with the given turn speed.

        Steps:
            1) clear the old line under the front array (outer sensors on the opposite side)
            2) wait until both new‐branch sensors see black at least once (outer sensors on turn side)
            3) wait until the adjacent center-array sensor reads white, which means the center sensor is on the line

        Args:
            direction (str): 'CCW' or 'CW'.
            turn_speed (float): speed of the turn, default is 2.0.

        Raises:
            ValueError: if direction is not 'CCW' or 'CW'.
        """

        # stop any forward/PID motion
        self.robot.stop()

        # determine spin direction and which sensors to monitor
        if direction == 'CCW':
            ls, rs = -turn_speed, turn_speed
            old_idxs = [3, 4]   # right sensors must clear first
            new_idxs = [0, 1]   # then left sensors must see the branch
        elif direction == 'CW':
            ls, rs = turn_speed, -turn_speed
            old_idxs = [0, 1]   # left sensors must clear first
            new_idxs = [3, 4]   # then right sensors must see the branch
        else:
            raise ValueError("direction must be 'CCW' or 'CW'")

        # start spinning
        self.robot.set_wheel_speeds(ls, rs)

        # 1) clear the old line under the front array
        clear_count = 0
        DEBOUNCE = 3
        while True:
            if not self.robot.step(): 
                return
            
            vals = self.robot.read_ground_sensors('front')
            if all(vals[i] >= self.robot.line_threshold for i in old_idxs):
                clear_count += 1
                if clear_count >= DEBOUNCE:
                    break
            else:
                clear_count = 0

        # 2) wait until both new‐branch sensors see black at least once
        seen = set()
        while True:
            if not self.robot.step():
                return
            
            vals = self.robot.read_ground_sensors('front')
            for i in new_idxs:
                if vals[i] < self.robot.line_threshold:
                    seen.add(i)
            if set(new_idxs).issubset(seen):
                break

        # 3) wait until the adjacent sensor reads white, which means the center sensor is on the line
        check_idx = 1 if direction == 'CCW' else 3
        while True:
            if not self.robot.step():
                return
            
            vals = self.robot.read_ground_sensors('front')
            if vals[check_idx] >= self.robot.line_threshold:
                self.robot.stop()
                break

    def follow_until_junction(self):
        """Follow PID on front array until a side‐array junction is detected

        Steps:
            1) stabilize on front line first to prevent a junction detection from a bad starting angle
            2) detect and debounce side-array junctions with side array sensors
            3) if the junction is a dead end, stop the robot
            4) return the detected directions and hand control over to the caller

        Returns:
            list: possible directions at the junction, e.g. ['L', 'F', 'R'].
        """

        debounce_steps=3
        stabilizing_steps=10

        # 1) stabilize on front line
        for _ in range(stabilizing_steps):
            if not self.step():
                return

        # 2) detect and debounce side-array junctions with side array sensors
        count = 0
        while True:
            if not self.step():
                return
            
            left_mid  = self.robot.read_line_sensors('left')[2]
            right_mid = self.robot.read_line_sensors('right')[2]
            if left_mid or right_mid:
                count += 1
                if count >= debounce_steps:
                    # once debounced, sample all three centers
                    front_mid = self.robot.read_line_sensors('front')[2]
                    dirs = []
                    if left_mid:  dirs.append('L')
                    if front_mid: dirs.append('F')
                    if right_mid: dirs.append('R')
                    print(f"Junction detected: possible directions: {', '.join(dirs)}")
                    break
            else:
                count = 0

        # 3) if the junction is a dead end, stop the robot
        if 'F' not in dirs:
            self.robot.stop()

        # 4) return the detected directions and hand control over to the caller
        return dirs    
        
    def drive_forward_until_bump(self):
        """Follow the line with PID control until the bump sensor trips. Then return to caller"""

        while True:
            if not self.step():
                return

            if self.robot.bumped():
                break

        self.robot.stop()
        print("Bump detected, assumed pick/drop")

    def drive_backward_until_junction(self, threshold: int = 2, debounce_steps: int = 3):
        """Reverse the robot until the right side array detects a junction

        Args:
            threshold (int): number of sensors that must detect the line to consider it a junction.
            debounce_steps (int): number of consecutive steps with the line detected to confirm the junction.
        """

        count = 0
        while True:
            if not self.robot.step():
                return
            self.robot.set_wheel_speeds(-2.5, -2.5)

            flags = self.robot.read_line_sensors('right')
            if sum(flags) >= threshold:
                count += 1
                if count >= debounce_steps:
                    break
            else:
                count = 0

        self.robot.stop()
        print("Back at junction")
