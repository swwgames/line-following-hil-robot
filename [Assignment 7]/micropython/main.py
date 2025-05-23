"""
Line Follower Controller (MicroPython - ESP32) - WIRELESS TCP SERVER

Receives ground sensor data via TCP from Webots,
computes correction, and sends motor speeds back.
"""

import struct
from machine import Pin
import time
from communicator import Communicator
from odometer import Odometer


MAX_SPEED = 6.28
BASE_SPEED = 0.4 * MAX_SPEED

Kp, Ki, Kd = 2.0, 0.0, 0.05

class LineFollowerController:
    def __init__(self, communicator: Communicator):
        self.com = communicator

        self.previous_error = 0.0
        self.integral = 0.0

        self.last_odom_update_time = 0
        self.last_encoder_sim_time = 0
        # Initialize LEDs
        self.led_yellow = Pin(4, Pin.OUT)
        self.led_blue = Pin(23, Pin.OUT)
        self.led_green = Pin(22, Pin.OUT)
        self.led_red = Pin(21, Pin.OUT)

        self.led_yellow.off()
        self.led_blue.off()
        self.led_green.off()
        self.led_red.off()

        self.button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)

    def update_leds(self, gs_left: float, gs_center: float, gs_right: float) -> None:
        """Set LED indicators based on sensor values."""
        self.led_red.value(gs_left > 600)
        self.led_green.value(gs_center > 600)
        self.led_blue.value(gs_right > 600)

    def compute_position(self, gs_left: float, gs_center: float, gs_right: float) -> float:
        """Compute a normalized position value from sensor readings."""
        left_w = 1000 - gs_left
        center_w = 1000 - gs_center
        right_w = 1000 - gs_right
        total = left_w + center_w + right_w
        if total == 0:
            return 0.0
        return (-1 * left_w + 0 * center_w + 1 * right_w) / total

    def compute_pid(self, position: float) -> float:
        """Compute the PID correction based on the position error."""
        error = position
        self.integral += error
        derivative = error - self.previous_error
        correction = Kp * error + Ki * self.integral + Kd * derivative
        self.previous_error = error
        return correction

    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min(value, max_val), min_val)

    def run(self) -> None:
        """Main control loop."""
        if not self.com.client_socket:
            print("No client connected. Aborting run.")
            return

        odom = Odometer(wheel_radius=0.0205, wheel_base=0.052, ticks_per_rev=72)

        self.last_odom_update_time = time.ticks_ms()

        while True:
            try:
                result = self.com.read_packet_from_socket(timeout_ms=100)
                if result:
                    packet_type, data = result
                    if packet_type == b'g':
                        gs_right, gs_center, gs_left = struct.unpack('!fff', data)

                        self.update_leds(gs_left, gs_center, gs_right)
                        position = self.compute_position(gs_left, gs_center, gs_right)
                        correction = self.compute_pid(position)

                        left_speed = self.clamp(BASE_SPEED - correction, -MAX_SPEED, MAX_SPEED)
                        right_speed = self.clamp(BASE_SPEED + correction, -MAX_SPEED, MAX_SPEED)

                        motor_payload = struct.pack('!ff', right_speed, left_speed)

                        self.com.send_packet_to_socket( b'm', motor_payload)
                    elif packet_type == b'e':

                        sim_time_encoders, ang_l, ang_r = struct.unpack('!dff', data)

                        if 0 <= self.last_encoder_sim_time < sim_time_encoders:
                            actual_delta_t_sec = sim_time_encoders - self.last_encoder_sim_time
                        else:
                            actual_delta_t_sec = 0.02

                        self.last_encoder_sim_time = sim_time_encoders

                        x, y, theta = odom.update(ang_l, ang_r, delta_t=actual_delta_t_sec)

                        messages_payload = struct.pack('!fff', x, y, theta)
                        self.com.send_packet_to_socket(b't', messages_payload)

                    time.sleep_ms(10)

            except OSError as e:
                print(f"Socket error: {e}. Client likely disconnected.")
                self.com.client_socket.close()
                self.com.client_socket = None

                print("Restart ESP32 and Webots controller to reconnect.")
                break
            except Exception as e:
                print(f"An error occurred: {e}")
                time.sleep(1)

def main() -> None:
    com = Communicator()
    controller = LineFollowerController(com)
    if com.client_socket:
        controller.run()
    else:
        print("Failed to establish client connection.")


if __name__ == '__main__':
    main()