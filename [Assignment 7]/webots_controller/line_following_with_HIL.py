"""line_following_with_HIL controller."""

from controller import Robot
import struct
import serial
import time

# Constants for serial communication
HEADER = b'S'
PACKET_TYPE = b'p'

class LineFollower(Robot):
    """Encapsulates the Webots robot setup for a line-following simulation using HiL."""
    def __init__(self):
        self.max_speed = 6.28

        self.speed = 0.6 * self.max_speed

        self.robot = Robot()

        self.timestep = int(self.robot.getBasicTimeStep())


        # distance sensors
        self.ps = []
        self.psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
        for i in range(8):
            self.ps.append(self.robot.getDevice(self.psNames[i]))
            self.ps[i].enable(self.timestep)

        # ground sensors
        self.gs = []
        self.gsNames = ['gs0', 'gs1', 'gs2']
        for i in range(3):
            self.gs.append(self.robot.getDevice(self.gsNames[i]))
            self.gs[i].enable(self.timestep)

        self.encoder = []
        self.encoderNames = ['left wheel sensor', 'right wheel sensor']
        for i in range(2):
            self.encoder.append(self.robot.getDevice(self.encoderNames[i]))
            self.encoder[i].enable(self.timestep)

        # motors
        self.leftMotor = self.robot.getDevice('left wheel motor')
        self.rightMotor = self.robot.getDevice('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        self.leftSpeed, self.rightSpeed = 0.0, 0.0

    def read_ground_sensors(self) -> list:
        """Read and return ground sensor values as a list [right, center, left]."""
        return [self.gs[i].getValue() for i in range(3)]

    def read_encoders(self) -> list:
        """Read and return encoder values as a list [right, left]."""
        return [self.encoder[i].getValue() for i in range(2)]

    def set_motor_speeds(self, left: float, right: float) -> None:
        """Set motor speed values for left and right motors."""
        self.leftMotor.setVelocity(left)
        self.rightMotor.setVelocity(right)


def read_packet(ser: serial.Serial) -> tuple[float, float] | tuple[None, None]:
    """Read a packet from serial starting with 'Sp' and followed by 8 bytes."""
    while ser.in_waiting:
        start = ser.read(1)
        if start != HEADER:
            continue

        kind = ser.read(1)
        if kind != PACKET_TYPE:
            continue

        data = ser.read(8)
        if len(data) < 8:
            print("Incomplete data:", data)
            continue

        try:
            right_speed, left_speed = struct.unpack('!ff', data)
            return right_speed, left_speed
        except struct.error as e:
            print("Unpacking error:", e)
            continue

    return None, None

def send_packet(ser, packet_type: bytes, payload: bytes):
    payload_size = len(payload)
    packet = HEADER + packet_type + bytes([payload_size]) + payload
    ser.write(packet)

def receive_packet(ser) -> tuple[bytes, bytes] | None:
    while ser.in_waiting >= 3:
        if ser.read(1) != HEADER:
            print("Incomplete data:", ser.read())
            continue
        packet_type = ser.read(1)
        size = ord(ser.read(1))
        data = ser.read(size)
        if len(data) < size:
            print("Incomplete packet received")
            continue
        return packet_type, data
    return None


def main():
    """Main simulation loop: read sensors, send data to ESP32, receive motor speeds."""

    try:
        ser = serial.Serial(port='COM13', baudrate=115200, timeout=5)
    except:
        print("Communication failed.")
        raise

    lf = LineFollower()

    while lf.robot.step(lf.timestep) != -1:
        gs_values = lf.read_ground_sensors()
        gs_payload = struct.pack('!fff', *gs_values)
        send_packet(ser, b'g', gs_payload)

        encoder_values = lf.read_encoders()
        encoder_payload = struct.pack('!ff', *encoder_values)
        send_packet(ser, b'e', encoder_payload)


        response = receive_packet(ser)
        if response:
            packet_type, data = response
            if packet_type == b'm':
                right_speed, left_speed = struct.unpack('!ff', data)
                lf.set_motor_speeds(left_speed, right_speed)

            if packet_type == b't':
                x, y, theta = struct.unpack('!fff', data)
                print(f'x: {x}, y: {y}, theta: {theta}')
        time.sleep(0.02)
    ser.close()

if __name__ == '__main__':
    main()
