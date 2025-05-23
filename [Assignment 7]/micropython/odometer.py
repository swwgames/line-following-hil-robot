import math

class Odometer:
    def __init__(self, wheel_radius, wheel_base, ticks_per_rev):
        self.x = 0.5
        self.y = -0.101
        self.theta = 1.5708
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.ticks_per_rev = ticks_per_rev
        self.prev_ang_diff_l = 0
        self.prev_ang_diff_r = 0

    def update(self, ang_l: float, ang_r: float, delta_t: float) -> tuple[float, float, float]:
        """Calculate new position based on wheel rotation."""
        if self.ticks_per_rev == 0 or self.wheel_base == 0:
            return self.x, self.y, self.theta

        ang_diff_l = ang_l - self.prev_ang_diff_l
        ang_diff_r = ang_r - self.prev_ang_diff_r

        self.prev_ang_diff_l = ang_l
        self.prev_ang_diff_r = ang_r

        wl = ang_diff_l / delta_t
        wr = ang_diff_r / delta_t

        u = self.wheel_radius / 2.0 * (wr + wl)
        w = self.wheel_radius / self.wheel_base * (wr - wl)

        delta_theta = w * delta_t
        self.theta += delta_theta

        if self.theta >= math.pi:
            self.theta -= 2 * math.pi
        elif self.theta < -math.pi:
            self.theta += 2 * math.pi

        delta_x = u * math.cos(self.theta) * delta_t
        delta_y = u * math.sin(self.theta) * delta_t

        self.x += delta_x
        self.y += delta_y

        return self.x, self.y, self.theta