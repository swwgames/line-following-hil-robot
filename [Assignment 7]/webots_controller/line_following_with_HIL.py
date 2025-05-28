"""line_following_with_HIL WeBots controller.
This controller connects to the ESP32 device, sends sensor data, and receives motor commands.
The ESP32 device also sends a current node and planned path to the controller. Which can be used to update the Streamlit dashboard.
"""

import struct
import socket
from communicator import Communicator
from robot import EPUCKRobot
import threading
import socket
import json

def send_to_streamlit(current_node: str, planned_path: list[str]):
    """Sends data to the Streamlit dashboard.
    Args:
        current_node (str): The current node of the robot.
        planned_path (list[str]): Planned path of the robot.
    """
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect(('localhost', 7000))
        payload = json.dumps({"node": current_node, "path": planned_path})
        client.sendall(payload.encode())
    except ConnectionRefusedError:
        print("Connection refused. Is the server running?")
    except Exception as e:
        print(f"Error sending data: {e}")
    finally:
        client.close()

def main():
    """Main function to run the WeBots controller."""
    epuck_robot = EPUCKRobot()
    com = Communicator()
    bumped_prev = False
    send_to_streamlit('', []) # Signal a reset
    print("Webots controller started. Attempting to connect to ESP32...")

    while epuck_robot.step():
        try:
            num_ground_sensors = len(epuck_robot.sensors)

            gs_values = [int(sensor.getValue() * 10) for sensor in epuck_robot.sensors]

            # Use 2-byte signed integers ('h') instead of 4-byte floats
            gs_payload_format = '!' + 'h' * num_ground_sensors
            gs_payload = struct.pack(gs_payload_format, *gs_values)
            com.send_packet_to_socket(b'g', gs_payload)

            bumped = epuck_robot.bumped()
            if bumped != bumped_prev:
                bumped_prev = bumped
                com.send_packet_to_socket(b'b', b'1' if bumped else b'0')

            # Receive motor commands or other data
            response = com.receive_packet_from_socket(timeout_sec=0.4)
            if response:
                packet_type, data, payload_size = response
                if packet_type == b'm':
                    right_speed, left_speed = struct.unpack('!hh', data)
                    epuck_robot.set_wheel_speeds(left_speed / 100, right_speed / 100)
                elif packet_type == b'r':
                    path = [data.decode('utf-8')[i:i + 2] for i in range(0, len(data.decode('utf-8')), 2)] # Create a list of nodes from a string of nodes, every two characters are one node.
                    send_to_streamlit(path[0], path)

                elif packet_type == b'n':
                    node = data.decode('utf-8')
                    send_to_streamlit(node, [])

                elif packet_type == b's':
                    # Skip step to gather more sensor data, and keep WeBots and ESP in sync
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