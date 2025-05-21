"""
Line Follower Controller (MicroPython - ESP32)

Receives ground sensor data via UART from Webots,
computes correction, and sends motor speeds back.
"""

import struct
from machine import Pin, UART
import time
import math


# === Constants ===
HEADER = b'S'
PACKET_TYPE = b'p'

MAX_SPEED = 6.28
BASE_SPEED = 0.4 * MAX_SPEED

Kp, Ki, Kd = 6.0, 0.0, 2.0  # PID coefficients


class Odometry:
    def __init__(self, wheel_radius, wheel_base, ticks_per_rev):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_rev = ticks_per_rev
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0

    def update(self, ticks_left, ticks_right):
        if self.ticks_per_rev == 0 or self.wheel_base == 0:
            return

        ΔL_ticks = ticks_left - self.prev_ticks_left
        ΔR_ticks = ticks_right - self.prev_ticks_right

        self.prev_ticks_left = ticks_left
        self.prev_ticks_right = ticks_right

        ΔL = 2 * 3.1415 * self.wheel_radius * (ΔL_ticks / self.ticks_per_rev)
        ΔR = 2 * 3.1415 * self.wheel_radius * (ΔR_ticks / self.ticks_per_rev)

        ΔC = (ΔL + ΔR) / 2
        Δθ = (ΔR - ΔL) / self.wheel_base

        self.theta += Δθ
        self.x += ΔC * math.cos(self.theta - Δθ / 2)
        self.y += ΔC * math.sin(self.theta - Δθ / 2)



class LineFollowerController:
    def __init__(self):
        # PID state
        self.previous_error = 0.0
        self.integral = 0.0

        # Initialize LEDs
        self.led_board = Pin(2, Pin.OUT)
        self.led_yellow = Pin(4, Pin.OUT)
        self.led_blue = Pin(23, Pin.OUT)
        self.led_green = Pin(22, Pin.OUT)
        self.led_red = Pin(21, Pin.OUT)

        # Buttons
        self.button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)
        self.button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)

        # Wait for user to start
        self.wait_for_start()

    def wait_for_start(self) -> None:
        print("Click the button on the ESP32 to continue. Then, close Thonny and run the Webots simulation.")
        while not self.button_left():
            time.sleep(0.25)
            self.led_board.value(not self.led_board())
        self.uart = UART(1, 115200, tx=1, rx=3)

    def read_uart_packet(self) -> tuple | None:
        """Read and return unpacked ground sensor values from UART."""
        if self.uart.any() >= 13:
            if self.uart.read(1) == HEADER:
                data = self.uart.read(12)
                if len(data) == 12:
                    return struct.unpack('!fff', data)
        return None

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

    def send_motor_speeds(self, left_speed: float, right_speed: float) -> None:
        """Send motor speeds to Webots via UART."""
        packet = PACKET_TYPE + struct.pack('!ff', right_speed, left_speed)
        self.uart.write(HEADER + packet)

    def clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min(value, max_val), min_val)

    def run(self) -> None:
        """Main control loop."""
        odom = Odometry(wheel_radius=0.0205, wheel_base=0.052, ticks_per_rev=72)
        while True:
            result = read_packet(self.uart)
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
                    send_packet(self.uart, b'm', motor_payload)
                elif packet_type == b'e':
                    ticks_left, ticks_right = struct.unpack('!ff', data)
                    odom.update(ticks_left, ticks_right)

            time.sleep(0.02)

def read_packet(uart, timeout_ms=100):
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if uart.any():
            byte = uart.read(1)
            if byte == HEADER:
                while uart.any() < 2 and time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                    pass
                if uart.any() < 2:
                    continue

                packet_type = uart.read(1)
                size_byte = uart.read(1)
                size = size_byte[0]

                while uart.any() < size and time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                    pass
                if uart.any() < size:
                    continue

                data = uart.read(size)
                if len(data) == size:
                    return packet_type, data

    return None

def send_packet(uart, packet_type: bytes, payload: bytes):
    size = len(payload)
    packet = HEADER + packet_type + bytes([size]) + payload
    uart.write(packet)

def main() -> None:
    controller = LineFollowerController()
    controller.run()


if __name__ == '__main__':
    main()
