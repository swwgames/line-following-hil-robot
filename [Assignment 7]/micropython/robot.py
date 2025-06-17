import struct
from communicator import Communicator
from time import sleep

class EPUCKRobot:
    def __init__(self, com: Communicator):
        """Initialize the class, motors, sensors, and proximity sensor."""

        self.time_step = 64
        self.line_threshold = 400

        self.com = com

        # Initialize last sensor readings
        self.last_ground_sensor_values = {
            'left': [0.0] * 5,
            'front': [0.0] * 5,
            'right': [0.0] * 5,
        }

        self.bumped_v = False


    def step(self) -> bool:
        """Perform a single simulation step.

        Returns:
            bool: True if the step was successful, False if the simulation ended.
        """

        updated = False
        for _ in range(3):
            result = self.com.read_packet_from_socket(timeout_ms=50)
            if not result:
                packet = struct.pack('!fff', 0.0, 0.0, 0.0)
                self.com.send_packet_to_socket(b's', packet)
                continue
            packet_type, data = result
            if packet_type == b'g':
                # Unpack 15 ground sensor values as 2-byte integers
                unpacked = struct.unpack('!14h', data)

                # Convert to float by dividing back
                float_values = [v / 10 for v in unpacked[:15]]

                self.last_ground_sensor_values['left'] = float_values[0:5]
                self.last_ground_sensor_values['front'] = float_values[5:10]
                self.last_ground_sensor_values['right'] = float_values[10:15]
                updated = True
            elif packet_type == b'b':
                if data == b'1':
                    self.bumped_v = True
                else:
                    self.bumped_v = False
                updated = True

        return updated

    def read_ground_sensors(self, array: str = 'front') -> list:
        """Return sensor values for one array of ground sensors.

        Args:
            array (str): 'front', 'left' or 'right'

        Raises:
            ValueError: if the array is not 'front', 'left' or 'right'.

        Returns:
            list: sensor values for the specified array.
        """

        if array not in self.last_ground_sensor_values:
            raise ValueError("array must be 'front', 'left', or 'right'")
        return self.last_ground_sensor_values[array]

    def read_line_sensors(self, array: str = 'front'):
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

        payload = struct.pack('!hh', int(right_speed * 100), int(left_speed * 100))
        self.com.send_packet_to_socket(b'm', payload)

    def stop(self):
        """Immediately stops both motors."""

        self.set_wheel_speeds(0.0, 0.0)

    def stop_and_close(self):
        self.set_wheel_speeds(0.0, 0.0)
        sleep(0.5)
        self.com.close()

    def bumped(self) -> bool:
        """Check if the robot's proximity sensor detects an obstacle.

        Returns:
            bool: True if the proximity sensor value is greater than 80.0, indicating an obstacle is close.
        """

        return self.bumped_v
