"""
Line Follower Controller (MicroPython - ESP32) - WIRELESS TCP SERVER

Receives sensor data via TCP from Webots,
computes correction, and sends motor speeds back.
Also sends location data such as current node and planned path back to Webots.
"""

from communicator import Communicator
from robot import EPUCKRobot
from linetracer import LineTracer
from tomtom import TomTom
from pid import PID

def main() -> None:
    com       = Communicator()
    robot     = EPUCKRobot(com)
    pid       = PID()
    tracer    = LineTracer(pid, robot)
    navigator = TomTom(tracer)

    result = navigator.locate_self(known_heading='N')
    if result:
        node, new_heading = result
    else:
        print("Fatal: Robot could not localize itself.")
        robot.stop()
        com.client_socket.close()
        return

    try:
        navigator.perform_box_run('P1','P5', node, new_heading)
        navigator.perform_box_run('P2','P6', navigator.current_node, navigator.heading)
        navigator.perform_box_run('P3','P7', navigator.current_node, navigator.heading)
        navigator.perform_box_run('P4','P8', navigator.current_node, navigator.heading)
    finally:
        robot.stop()

if __name__ == '__main__':
    main()