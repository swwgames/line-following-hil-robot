# main.py

from robot import EPUCKRobot
from linetracer import LineTracer
from tomtom import TomTom

def main():
    robot     = EPUCKRobot()
    tracer    = LineTracer(robot)
    navigator = TomTom(tracer)

    node, new_heading = navigator.locate_self(known_heading='N')
    robot.stop()

if __name__ == "__main__":
    main()
