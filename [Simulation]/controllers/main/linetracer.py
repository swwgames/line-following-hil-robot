from robot import EPUCKRobot

class LineTracer:
    """
    PID-based line tracer using EPUCKRobot with 5‐sensor front array
    and 5‐sensor side arrays for junction detection.
    """
    def __init__(self, robot: EPUCKRobot, base_speed=4.0, max_speed=5.0, kp=0.0010, ki=0.0001, kd=0.0050):
        self.robot = robot
        self.base_speed = base_speed
        self.max_speed = max_speed
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral_error = 0.0

    def compute_error(self):
        """
        Compute lateral error from the front‐array:
        leftmost minus rightmost sensor.
        """
        front = self.robot.read_ground_sensors('front')  # 5 values
        left, center, right = front[1], front[2], front[3]
        return left - right

    def compute_control(self, error):
        p = self.kp * error
        self.integral_error += error
        max_int = self.max_speed / max(self.ki, 1e-6)
        self.integral_error = max(-max_int, min(max_int, self.integral_error))
        i = self.ki * self.integral_error
        d = self.kd * ((error - self.last_error) / self.robot.time_step)
        self.last_error = error
        return p + i + d

    def step(self):
        if not self.robot.step():
            return False
        error = self.compute_error()
        corr = self.compute_control(error)
        ls = max(-self.max_speed, min(self.max_speed, self.base_speed + corr))
        rs = max(-self.max_speed, min(self.max_speed, self.base_speed - corr))
        self.robot.set_wheel_speeds(ls, rs)
        return True

    def pivot_into_direction(self, direction='CCW', turn_speed=2.0):
        """
        Spin in place toward the given branch (left or right):
          1) clear the old line under the front array (outer sensors on the opposite side),
          2) detect the new branch line under the front array (outer sensors on turn side),
          3) finally lock the center sensor on the new line.
        """
        # stop any forward/PID motion
        self.robot.stop()

        # determine spin direction and which sensors to monitor
        if direction == 'CCW':
            ls, rs = -turn_speed, turn_speed
            old_idxs = [3, 4]   # right‐of‐center sensors must clear first
            new_idxs = [0, 1]   # then left‐of‐center sensors must see the branch
        elif direction == 'CW':
            ls, rs = turn_speed, -turn_speed
            old_idxs = [0, 1]   # left‐of‐center sensors must clear first
            new_idxs = [3, 4]   # then right‐of‐center sensors must see the branch
        else:
            raise ValueError("direction must be 'CCW' or 'CW'")

        # start spinning
        self.robot.set_wheel_speeds(ls, rs)

        # 1) wait until old‐line sensors clear (debounced)
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

        # 3) wait until the adjacent sensor reads white → center is on the line
        check_idx = 1 if direction == 'CCW' else 3
        while True:
            if not self.robot.step():
                return
            vals = self.robot.read_ground_sensors('front')
            if vals[check_idx] >= self.robot.line_threshold:
                self.robot.stop()
                break

    def follow_until_junction(self, debounce_steps=3, stabilizing_steps=10, turn_direction=None, turn_speed=2.0):
        """
        Follow PID on front array until a side‐array junction is detected:
          1) stabilize on front line,
          2) debounce side‐array detection,
          3) drive straight until side‐center clears,
          4) pivot cleanly into branch,
          5) stabilize on new front line.
        """
        # 1) stabilize
        for _ in range(stabilizing_steps):
            if not self.step():
                return

        # 2) debounce branch‐center detection on either side
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
                    print(f"Junction detected → possible: {', '.join(dirs)}")
                    break
            else:
                count = 0

        # 3) branch‐specific clearance
        if turn_direction in ('CCW', 'CW'):
            self.pivot_into_direction(direction=turn_direction, turn_speed=turn_speed)
            for _ in range(stabilizing_steps):
                if not self.step():
                    return
        else:
            # no turn requested: go straight if possible, else dead end
            if self.robot.read_line_sensors('front')[2]:
                print("No turn → continuing straight")
            else:
                print("Dead end → stopping")
                self.robot.stop()
                return
            
    def drive_forward_until_bump(self):
        """
        Follow the line with PID control until the bump sensor trips.
        Assumes self.robot.bumped() returns True on collision.
        """
        # 1) keep running your PID step()…
        while True:
            # if simulation ended, give up
            if not self.step():
                return
            # check bump sensor after each step
            if self.robot.bumped():
                break
        # 2) stop cleanly
        self.robot.stop()
        print("💥 Bump detected – assumed pick/drop")

    def drive_backward_until_junction(
        self,
        threshold: int = 2,
        debounce_steps: int = 3
    ):
        """
        Reverse line-following with PID until the front-array detects a junction
        (≥threshold sensors black), debounced for debounce_steps.
        """
        count = 0
        # 1) keep backing with PID-reversal
        while True:
            # step simulation
            if not self.robot.step():
                return
            self.robot.set_wheel_speeds(-3, -3)

            # 2) check for junction under the front array
            flags = self.robot.read_line_sensors('right')  # bool[5]
            if sum(flags) >= threshold:
                count += 1
                if count >= debounce_steps:
                    break
            else:
                count = 0

        # 3) stop once we’ve backed onto the junction
        self.robot.stop()
        print("◀️  Back at junction (debounced)")
