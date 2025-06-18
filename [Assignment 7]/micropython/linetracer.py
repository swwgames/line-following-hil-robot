class LineTracer:
    def __init__(self, pid, robot):
        """Initialize the class with a valid PID controller and a robot instance.

        Args:
            pid (obj): existing PID instance
            robot (obj): existing robot instance
        """
        self.robot = robot
        self.pid = pid
        self.base_speed = 2
        self.max_speed = 3

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

    def _execute_turn_sequence(self, old_idxs, new_idxs, check_idx, debounce=3):
        """
        Executes the physical turn-and-align sequence.
        This helper function should be called after wheel speeds are set.

        Args:
            old_idxs (list): the list of old line sensors.
            new_idxs (list): the list of new line sensors.
            check_idx (int): value corresponding to the sensor that needs to read white when the robot is centered on the line.
            debounce (int): Amount of readings to be sure of detecting a line.
        """
        # 1) Wait until old‐line sensors clear the line
        clear_count = 0
        while True:
            vals = self.robot.read_ground_sensors('front')
            if all(vals[i] >= self.robot.line_threshold for i in old_idxs):
                clear_count += 1
                if clear_count >= debounce:
                    break
            else:
                clear_count = 0
            time.sleep(self.delay)

        # 2) Wait until both new‐branch sensors see black at least once
        seen = set()
        while True:
            vals = self.robot.read_ground_sensors('front')
            for i in new_idxs:
                if vals[i] < self.robot.line_threshold:
                    seen.add(i)
            if set(new_idxs).issubset(seen):
                break
            time.sleep(self.delay)

        # 3) Wait until the adjacent sensor reads white, which means the center is on the line
        while True:
            vals = self.robot.read_ground_sensors('front')
            if vals[check_idx] >= self.robot.line_threshold:
                self.robot.stop()
                break
            time.sleep(self.delay)

    def pivot_into_direction(self, direction='CCW', turn_speed=2.0):
        """
        Spins in place toward the given direction with robust alignment.

        This function first determines the parameters for the turn, then calls
        a helper to execute the physical sensor-based alignment.

        Args:
            direction (str): The direction to turn to.
            turn_speed (float): The turn speed.
        """
        # Stop any previous motion
        self.robot.stop()

        # --- 1. Determine Turn Parameters ---
        if direction == 'CCW':
            ls, rs = -turn_speed, turn_speed
            old_idxs = [3, 4]
            new_idxs = [0, 1]
            check_idx = 1

        elif direction == 'CW':
            ls, rs = turn_speed, -turn_speed
            old_idxs = [0, 1]
            new_idxs = [3, 4]
            check_idx = 3

        elif direction == '180':
            # Default to a CCW turn
            ls, rs = -turn_speed, turn_speed
            old_idxs = [3, 4]
            new_idxs = [0, 1]
            check_idx = 1

            # Check for a T-junction with only a left branch, which requires a CW turn
            vals_l = self.robot.read_ground_sensors('left')
            vals_r = self.robot.read_ground_sensors('right')
            has_left = any(v < self.robot.line_threshold for v in vals_l)
            has_right = any(v < self.robot.line_threshold for v in vals_r)

            if has_left and not has_right:
                # Override parameters for a CW turn
                ls, rs = turn_speed, -turn_speed
                old_idxs = [0, 1]
                new_idxs = [3, 4]
                check_idx = 3
        else:
            raise ValueError("direction must be 'CCW', 'CW', or '180'")

        # --- 2. Execute the Turn ---
        # Start the robot spinning with the configured speeds
        self.robot.set_wheel_speeds(ls, rs)

        # Call the helper to perform the reliable alignment sequence
        self._execute_turn_sequence(old_idxs, new_idxs, check_idx)

    def follow_until_junction(self) -> list | None:
        """Follow PID on front array until a side‐array junction is detected

        Steps:
            1) stabilize on front line first to prevent a junction detection from a bad starting angle
            2) detect and debounce side-array junctions with side array sensors
            3) if the junction is a dead end, stop the robot
            4) return the detected directions and hand control over to the caller

        Returns:
            list: possible directions at the junction, e.g. ['L', 'F', 'R'], or False if a bump is detected before reaching a junction.
        """

        debounce_steps = 3
        stabilizing_steps = 5

        # 1) stabilize
        for _ in range(stabilizing_steps):
            if not self.step():
                return None

            if self.robot.bumped():
                self.robot.stop()
                return False

        # 2) debounce branch‐center detection on either side
        count = 0
        while True:
            if not self.step():
                return None

            if self.robot.bumped():
                self.robot.stop()
                return False

            left_mid = self.robot.read_line_sensors('left')[2]
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

        # 3) stop the robot if it has to turn
        if 'F' not in dirs:
            self.robot.stop()

        print(f'return dirs: {dirs}')
        # 4) return the detected directions and hand control over to the caller
        return dirs

    def drive_forward_until_bump(self) -> None:
        """Follow the line with PID control until the bump sensor trips. Then return to caller"""

        while True:
            if not self.step():
                return

            if self.robot.bumped():
                break

        self.robot.stop()
        print("Bump detected, assumed pick/drop")

    def drive_backward_until_junction(self, threshold: int = 2, debounce_steps: int = 3) -> None:
        """Reverse the robot until the right side array detects a junction

        Args:
            threshold (int): number of sensors that must detect the line to consider it a junction.
            debounce_steps (int): number of consecutive steps with the line detected to confirm the junction.
        """

        count = 0
        while True:
            if not self.robot.step():
                return
            self.robot.set_wheel_speeds(-0.75, -0.75)

            flags = self.robot.read_line_sensors('right')
            if sum(flags) >= threshold:
                count += 1
                if count >= debounce_steps:
                    break
            else:
                count = 0

        self.robot.stop()
        print("Back at junction")
