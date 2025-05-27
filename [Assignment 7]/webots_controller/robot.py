from controller import Robot

class EPUCKRobot:
    def __init__(self):
        """Initialize the class with a Webots robot instance, motors, sensors, and proximity sensor."""

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
        """Perform a single simulation step.

        Returns:
            bool: True if the step was successful, False if the simulation ended.
        """

        return self.robot.step(self.time_step) != -1

    def read_ground_sensors(self, array: str = 'front') -> list:
        """Return sensor values for one array of ground sensors.

        Args:
            array (str): 'front', 'left' or 'right'

        Raises:
            ValueError: if the array is not 'front', 'left' or 'right'.

        Returns:
            list: sensor values for the specified array.
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

    def read_line_sensors(self, array: str = 'front') -> list:
        """Read line sensors and return a list of boolean values indicating whether each sensor sees black.

        Args:
            array (str): 'front', 'left' or 'right'

        Returns:
            list: boolean values for each sensor in the specified array, True if the sensor sees black.
        """

        return [v < self.line_threshold for v in self.read_ground_sensors(array)]

    def set_wheel_speeds(self, left_speed: float, right_speed: float):
        """Set the speed of the left and right motors.

        Args:
            left_speed (float): speed for the left motor.
            right_speed (float): speed for the right motor.
        """

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def stop(self):
        """Immediately stops both motors."""

        self.set_wheel_speeds(0.0, 0.0)

    def bumped(self) -> bool:
        """Check if the robot's proximity sensor detects an obstacle.

        Returns:
            bool: True if the proximity sensor value is greater than 80.0, indicating an obstacle is close.
        """

        val = self.prox.getValue()
        return val > 80.0