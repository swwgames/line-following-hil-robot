import struct
from communicator import Communicator

class EPUCKRobot:
    def __init__(self, com: Communicator):
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
        self.last_encoder_values = [0.0, 0.0]
        self.last_odometry = (0.0, 0.0, 0.0)

    def step(self):
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

                # Convert to float by dividing back (e.g., if scaled by 100)
                float_values = [v / 10 for v in unpacked[:15]]

                self.last_ground_sensor_values['left'] = float_values[0:5]
                self.last_ground_sensor_values['front'] = float_values[5:10]
                self.last_ground_sensor_values['right'] = float_values[10:15]
                updated = True

            elif packet_type == b'e':
                # Unpack: time as unsigned int (ms), encoders as 2-byte signed integers
                scaled_time, enc_left, enc_right = struct.unpack('!Ihh', data)

                sim_time = scaled_time / 1000  # convert ms to seconds
                self.last_encoder_values = [enc_left / 100, enc_right / 100]
                updated = True
            elif packet_type == b'b':
                if data == b'1':
                    self.bumped_v = True
                else:
                    self.bumped_v = False
                updated = True
            elif packet_type == b't':
                self.last_odometry = struct.unpack('!fff', data)
                updated = True
        return updated

    def read_ground_sensors(self, array: str = 'front'):
        if array not in self.last_ground_sensor_values:
            raise ValueError("array must be 'front', 'left', or 'right'")
        return self.last_ground_sensor_values[array]

    def read_line_sensors(self, array: str = 'front'):
        return [v < self.line_threshold for v in self.read_ground_sensors(array)]

    def read_encoders(self) -> list:
        return self.last_encoder_values

    def set_wheel_speeds(self, left_speed, right_speed):
        payload = struct.pack('!hh', int(right_speed * 100), int(left_speed * 100))
        self.com.send_packet_to_socket(b'm', payload)

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def bumped(self) -> bool:
        return self.bumped_v
