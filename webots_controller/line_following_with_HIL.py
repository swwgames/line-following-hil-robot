"""line_following_with_HIL controller."""

from controller import Robot
import struct
import serial

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

def main():
    """Main simulation loop: read sensors, send data to ESP32, receive motor speeds."""

    try:
        ser = serial.Serial(port='COM8', baudrate=115200, timeout=5)
    except:
        print("Communication failed.")
        raise

    lf = LineFollower()

    while lf.robot.step(lf.timestep) != -1:
        gs_values = lf.read_ground_sensors()
        packet = struct.pack('!fff', gs_values[0], gs_values[1], gs_values[2])
        ser.write(HEADER + packet)

        right_speed, left_speed = read_packet(ser)
        if right_speed is not None and left_speed is not None:
            print("Received Right Speed:", right_speed)
            print("Received Left Speed:", left_speed)
            lf.set_motor_speeds(left_speed, right_speed)

    ser.close()

if __name__ == '__main__':
    main()
