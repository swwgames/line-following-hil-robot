import time

class LineTracer:
    def __init__(self, pid, robot, delay=0.01):
        """Initialize the class with a valid PID controller and a robot instance.

        Args:
            pid (obj): existing PID instance
            robot (obj): existing robot instance
        """

        self.delay = delay
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
        """Spin in place toward the given direction, with the given turn speed.

        For 'CCW' and 'CW', it executes a 90-degree turn at an intersection.
        For '180', it performs a 180-degree turn, correctly handling both
        straight lines and various junction types using side sensors.

        Args:
            direction (str): 'CCW', 'CW', or '180'.
            turn_speed (float): Speed of the turn, default is 2.0.

        Raises:
            ValueError: if direction is not 'CCW', 'CW', or '180'.
        """

        # Stop any forward/PID motion
        self.robot.stop()

        # Default turn direction is CCW. This will be used for 'CCW' and the default '180' turn.
        ls, rs = -turn_speed, turn_speed
        if direction == 'CW':
            ls, rs = turn_speed, -turn_speed
        elif direction not in ['CCW', '180']:
            raise ValueError("direction must be 'CCW', 'CW', or '180'")

        # --- Special logic for 180-degree turns ---
        if direction == '180':
            # Use dedicated side sensors for robust junction detection, as you suggested.
            initial_vals_l = self.robot.read_ground_sensors('left')
            initial_vals_r = self.robot.read_ground_sensors('right')

            # Correctly check if any sensor in the array sees a line
            has_left_branch = any(val < self.robot.line_threshold for val in initial_vals_l)
            has_right_branch = any(val < self.robot.line_threshold for val in initial_vals_r)
            is_junction = has_left_branch or has_right_branch

            if is_junction:
                # --- JUNCTION 180 LOGIC ---
                # If there's only a left branch, turn CW (away from it). Otherwise, turn CCW.
                if has_left_branch and not has_right_branch:
                    ls, rs = turn_speed, -turn_speed  # Turn CW
                    side_branch_sensors = [3, 4]  # Monitor right front sensors
                else:
                    ls, rs = -turn_speed, turn_speed  # Turn CCW
                    side_branch_sensors = [0, 1]  # Monitor left front sensors

                self.robot.set_wheel_speeds(ls, rs)
                DEBOUNCE = 3

                # 1) Wait until we see the side branch with the appropriate sensors
                seen_side_branch = False
                while not seen_side_branch:
                    if not self.robot.step(): return
                    vals = self.robot.read_ground_sensors('front')
                    if any(vals[i] < self.robot.line_threshold for i in side_branch_sensors):
                        seen_side_branch = True

                # 2) Now, wait until we have completely cleared that side branch
                clear_count = 0
                while True:
                    if not self.robot.step(): return
                    vals = self.robot.read_ground_sensors('front')
                    if all(vals[i] >= self.robot.line_threshold for i in side_branch_sensors):
                        clear_count += 1
                        if clear_count >= DEBOUNCE:
                            break  # Side branch has been passed
                    else:
                        clear_count = 0

                # 3) Finally, find the original path with the center sensor
                while True:
                    if not self.robot.step(): return
                    vals = self.robot.read_ground_sensors('front')
                    if vals[2] < self.robot.line_threshold:
                        self.robot.stop()
                        break
            else:
                # --- STRAIGHT LINE 180 LOGIC ---
                self.robot.set_wheel_speeds(ls, rs)  # Use default CCW turn
                center_idx = 2
                DEBOUNCE = 3

                # 1) Wait until the center sensor has cleared the line
                clear_count = 0
                while True:
                    if not self.robot.step(): return
                    vals = self.robot.read_ground_sensors('front')
                    if vals[center_idx] >= self.robot.line_threshold:
                        clear_count += 1
                        if clear_count >= DEBOUNCE:
                            break
                    else:
                        clear_count = 0

                # 2) Now, wait until the center sensor finds the line again
                while True:
                    if not self.robot.step(): return
                    vals = self.robot.read_ground_sensors('front')
                    if vals[center_idx] < self.robot.line_threshold:
                        self.robot.stop()
                        break
            return  # End the function for all 180 turns

        # --- Logic for 90-degree turns (CW/CCW) ---
        self.robot.set_wheel_speeds(ls, rs)
        DEBOUNCE = 3

        old_idxs = [3, 4] if direction == 'CCW' else [0, 1]
        new_idxs = [0, 1] if direction == 'CCW' else [3, 4]

        # 1) Clear the old line
        clear_count = 0
        while True:
            if not self.robot.step(): return
            vals = self.robot.read_ground_sensors('front')
            if all(vals[i] >= self.robot.line_threshold for i in old_idxs):
                clear_count += 1
                if clear_count >= DEBOUNCE: break
            else:
                clear_count = 0

        # 2) Find the new branch
        seen = set()
        while True:
            if not self.robot.step(): return
            vals = self.robot.read_ground_sensors('front')
            for i in new_idxs:
                if vals[i] < self.robot.line_threshold:
                    seen.add(i)
            if set(new_idxs).issubset(seen): break

        # 3) Center on the new line
        check_idx = 1 if direction == 'CCW' else 3
        while True:
            if not self.robot.step(): return
            vals = self.robot.read_ground_sensors('front')
            if vals[check_idx] >= self.robot.line_threshold:
                self.robot.stop()
                break

    def follow_until_junction(self) -> list | bool:
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
            self.step()

            if self.robot.bumped():
                self.robot.stop()
                return False

            time.sleep(self.delay)

        # 2) debounce branch‐center detection on either side
        count = 0
        while True:
            self.step()

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

            time.sleep(self.delay)

        # 3) stop the robot if it has to turn
        if 'F' not in dirs:
            self.robot.stop()

        print(f'return dirs: {dirs}')
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
            self.robot.set_wheel_speeds(-1.5, -1.5)

            flags_r = self.robot.read_line_sensors('right')
            flags_l = self.robot.read_line_sensors('left')
            if sum(flags_r) >= threshold or sum(flags_l) >= threshold:
                count += 1
                if count >= debounce_steps:
                    break
            else:
                count = 0
        
        self.robot.stop()
        print("Back at junction")
