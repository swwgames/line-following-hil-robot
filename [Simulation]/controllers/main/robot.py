from controller import Robot

class EPUCKRobot:
    def __init__(self):
        self.time_step = 64
        self.line_threshold = 400

        # initialize Webots Robot
        self.robot = Robot()

        # setup motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
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
        
        # setup proximity sensor
        self.prox = self.robot.getDevice('ps7')
        self.prox.enable(self.time_step)

    def step(self):
        """
        Advance simulation by one time step.
        Returns False if simulation ended.
        """
        return self.robot.step(self.time_step) != -1

    def read_ground_sensors(self, array: str = 'front'):
        """
        Return raw values from one of the three 5-sensor arrays.
        'front', 'left' or 'right'
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

    def bumped(self) -> bool:
        """
        Return True if the front‐facing PS sensor reading exceeds
        the configured threshold (i.e. an object is very close).
        """
        val = self.prox.getValue()
        return val > 80.0