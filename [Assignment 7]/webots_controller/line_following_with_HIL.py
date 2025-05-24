"""line_following_with_HIL controller."""

import struct
import socket
from communicator import Communicator
from robot import EPUCKRobot

def main():
    epuck_robot = EPUCKRobot()
    com = Communicator()
    print("Webots controller started. Attempting to connect to ESP32...")

    while epuck_robot.step():
        try:
            sim_current_time = epuck_robot.robot.getTime()

            num_ground_sensors = len(epuck_robot.sensors)

            gs_values = [sensor.getValue() for sensor in epuck_robot.sensors]

            gs_payload_format = '!' + 'f' * num_ground_sensors

            gs_payload = struct.pack(gs_payload_format, *gs_values)

            com.send_packet_to_socket(b'g', gs_payload)

            encoder_values = [epuck_robot.encoder[enc].getValue() for enc in range(2)]
            encoder_payload = struct.pack('!dff', sim_current_time, *encoder_values)
            com.send_packet_to_socket(b'e', encoder_payload)

            # Receive motor commands or other data
            response = com.receive_packet_from_socket(timeout_sec=1.0) # Using a small timeout for non-blocking behavior
            if response:
                packet_type, data = response
                if packet_type == b'm':
                    right_speed, left_speed = struct.unpack('!ff', data)
                    epuck_robot.set_wheel_speeds(left_speed, right_speed)
                elif packet_type == b't':
                    x, y, theta = struct.unpack('!fff', data)
                    print(f'ESP32 Odom -> x: {x:.3f}, y: {y:.3f}, theta: {theta:.3f}')

        except socket.timeout:
            pass
        except socket.error as e:
            print(f"Socket error during loop: {e}")
            epuck_robot.stop()
            break
        except Exception as e:
            print(f"An error occurred in Webots loop: {e}")
            epuck_robot.stop()
            break

    if com.client_socket:
        com.client_socket.close()

    epuck_robot.stop()
    print("Webots controller stopped.")

if __name__ == '__main__':
    main()