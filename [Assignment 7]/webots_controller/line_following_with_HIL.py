"""line_following_with_HIL controller."""

from controller import Robot
import struct
import socket
from communicator import Communicator


class LineFollower(Robot):
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
        return [self.gs[i].getValue() for i in range(3)]

    def read_encoders(self) -> list:
        return [self.encoder[i].getValue() for i in range(2)]

    def set_motor_speeds(self, left: float, right: float) -> None:
        self.leftMotor.setVelocity(left)
        self.rightMotor.setVelocity(right)


def main():
    com = Communicator()
    lf = LineFollower()

    while lf.robot.step(lf.timestep) != -1:
        try:
            sim_current_time = lf.robot.getTime()
            gs_values = lf.read_ground_sensors()
            gs_payload = struct.pack('!fff', *gs_values)
            com.send_packet_to_socket(b'g', gs_payload)

            encoder_values = lf.read_encoders()
            encoder_payload = struct.pack('!dff', sim_current_time, *encoder_values)
            com.send_packet_to_socket(b'e', encoder_payload)

            response = com.receive_packet_from_socket(timeout_sec=0.5)
            if response:
                packet_type, data = response
                if packet_type == b'm':
                    right_speed, left_speed = struct.unpack('!ff', data)
                    lf.set_motor_speeds(left_speed, right_speed)
                elif packet_type == b't':
                    x, y, theta = struct.unpack('!fff', data)
                    print(f'ESP32 Odom -> x: {x:.3f}, y: {y:.3f}, theta: {theta:.3f}')


        except socket.error as e:
            print(f"Socket error during loop: {e}")
            break
        except Exception as e:
            print(f"An error occurred in Webots loop: {e}")
            break

    if com.client_socket:
        com.client_socket.close()
    print("Webots controller stopped.")

if __name__ == '__main__':
    main()