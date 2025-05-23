from controller import Robot

class EPUCKRobot:
    """
    Low-level interface to the e-puck in Webots.
    Provides stepping, ground-sensor reading, and motor control.
    """
    def __init__(
        self,
        time_step=64,
        sensor_names=('gs0','gs1','gs2'),
        left_motor_name='left wheel motor',
        right_motor_name='right wheel motor',
        line_threshold=400
    ):
        self.time_step = time_step
        self.line_threshold = line_threshold

        # initialize Webots Robot
        self.robot = Robot()

        # setup motors
        self.left_motor = self.robot.getDevice(left_motor_name)
        self.right_motor = self.robot.getDevice(right_motor_name)
        for m in (self.left_motor, self.right_motor):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        # setup ground sensors
        self.sensors = []
        for i in range(0, 14):
            name = f"gs{i}"
            sensor = self.robot.getDevice(name)
            sensor.enable(self.time_step)
            self.sensors.append(sensor)

    def step(self):
        """
        Advance simulation by one time step.
        Returns False if simulation ended.
        """
        return self.robot.step(self.time_step) != -1

    def read_ground_sensors(self, array: str = 'front'):
        """
        Return raw values from one of the three 5-sensor arrays.
        :param array: 'front', 'left' or 'right'
        """
        groups = {
            'left': self.sensors[0:5],
            'front':  self.sensors[5:10],
            'right': self.sensors[10:15],
        }
        try:
            devs = groups[array]
        except KeyError:
            raise ValueError("array must be 'front', 'left' or 'right'")
        return [s.getValue() for s in devs]

    def read_line_sensors(self, array: str = 'front'):
        """
        Return booleans for one array indicating line detection.
        (value < line_threshold).
        :param array: 'front', 'left' or 'right'
        """
        return [v < self.line_threshold for v in self.read_ground_sensors(array)]

    def set_wheel_speeds(self, left_speed, right_speed):
        """
        Set velocities for left and right wheel motors.
        """
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def stop(self):
        """
        Immediately stop both motors.
        """
        self.set_wheel_speeds(0.0, 0.0)