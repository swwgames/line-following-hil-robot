"""line_following_with_HIL controller."""

import struct
import socket
from communicator import Communicator
from robot import EPUCKRobot


def main():
    step = 0
    epuck_robot = EPUCKRobot()
    com = Communicator()
    print("Webots controller started. Attempting to connect to ESP32...")

    while epuck_robot.step():
        try:
            step += 1
            sim_current_time = epuck_robot.robot.getTime()

            num_ground_sensors = len(epuck_robot.sensors)

            # Quantize ground sensor values (e.g., multiply by 100 to keep two decimal places)
            gs_values = [int(sensor.getValue() * 10) for sensor in epuck_robot.sensors]
            gs_values.append(step)  # Keep step as-is, assumed to be integer

            # Use 2-byte signed integers ('h') instead of 4-byte floats
            gs_payload_format = '!' + 'h' * num_ground_sensors + 'I'  # 'I' = 4-byte unsigned int for step

            gs_payload = struct.pack(gs_payload_format, *gs_values)
            com.send_packet_to_socket(b'g', gs_payload)

            # Quantize encoder values and time
            scaled_time = int(sim_current_time * 1000)  # ms precision
            encoder_values = [int(epuck_robot.encoder[enc].getValue() * 100) for enc in range(2)]

            # Pack time as 4-byte unsigned int, encoders as 2-byte signed ints
            encoder_payload = struct.pack('!Ihh', scaled_time, *encoder_values)
            com.send_packet_to_socket(b'e', encoder_payload)

            # Receive motor commands or other data
            response = com.receive_packet_from_socket(timeout_sec=0.4)
            if response:
                packet_type, data = response
                if packet_type == b'm':
                    right_speed, left_speed = struct.unpack('!hh', data)
                    epuck_robot.set_wheel_speeds(left_speed / 100, right_speed / 100)
                elif packet_type == b't':
                    x, y, theta = struct.unpack('!fff', data)
                    print(f'ESP32 Odom -> x: {x:.3f}, y: {y:.3f}, theta: {theta:.3f}')
                elif packet_type == b's':
                    #print('Skip step, gathering sensor data')
                    continue

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