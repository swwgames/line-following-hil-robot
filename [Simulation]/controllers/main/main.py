from robot import EPUCKRobot
from linetracer import LineTracer
from tomtom import TomTom
from pid import PID

def main():
    robot     = EPUCKRobot()
    pid       = PID()
    tracer    = LineTracer(pid, robot)
    navigator = TomTom(tracer)

    node, new_heading = navigator.locate_self(known_heading='N')
    try:
        navigator.perform_box_run('P1','P5', node, new_heading)
        navigator.perform_box_run('P2','P6', navigator.current_node, navigator.heading)
        navigator.perform_box_run('P3','P7', navigator.current_node, navigator.heading)
        navigator.perform_box_run('P4','P8', navigator.current_node, navigator.heading)
    finally:
        robot.stop()

if __name__ == "__main__":
    main()
