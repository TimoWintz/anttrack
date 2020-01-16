import numpy as np

class Velodrome():
    def __init__(self, p1, p2, radius, window_size, track_length=500):
        """
               .___________.
              /             \
             |  p1     	 p2  |  -
              \             /   | radius
               ______>______    -
        """
        assert(p1.shape == (2,))
        assert(p2.shape == (2,))
        assert(np.linalg.norm(p2 - p1) > 0)
        self.p1 = p1
        self.p2 = p2

        self.straight_direction = self.p2 - self.p1
        self.straight_direction /= np.linalg.norm(self.straight_direction)

        self.normal_direction = np.zeros(2)
        self.normal_direction[0] = self.straight_direction[1]
        self.normal_direction[1] = -self.straight_direction[0]

        self.finish_line_pos = 0.5 * (self.p1 + self.p2) + radius * self.normal_direction
        assert(radius > 0)
        self.radius = radius

        self.straight_length = self._straight_length()
        self.arc_length = self._arc_length()
        self.track_length = track_length
        self.window_unit_per_m = 2*(self.straight_length + self.arc_length) / self.track_length

        self.straight_direction = self.straight_direction * self.window_unit_per_m
        self.normal_direction = self.normal_direction * self.window_unit_per_m

        self.radius = self.radius / self.window_unit_per_m

        self.straight_length, self.arc_length = (0.5 * self.straight_length / (self.arc_length + self.straight_length) * track_length,
                    0.5 * self.arc_length / (self.arc_length + self.straight_length) * track_length)

        self.window_size = np.array(window_size)

    def _straight_length(self):
        return np.linalg.norm(self.p1 - self.p2)

    def _arc_length(self):
        return np.pi*self.radius

    def pos_from_distance(self, d):
        assert(d >= 0)
        assert(d < self.track_length)
        if d < 0.5 * self.straight_length:
            x = self.finish_line_pos + d * self.straight_direction
        elif d < 0.5 * self.straight_length + self.arc_length:
            angle = (d - 0.5 * self.straight_length) / self.arc_length * np.pi
            x = self.p2 + self.radius * (np.cos(angle) * self.normal_direction + np.sin(angle) * self.straight_direction)
        elif d < 1.5 * self.straight_length+ self.arc_length:
            x = self.p2 - self.normal_direction * self.radius - self.straight_direction * (d - 0.5 * self.straight_length - self.arc_length)
        elif d < 1.5*self.straight_length+ 2*self.arc_length:
            angle = (d - 1.5*self.straight_length- self.arc_length) / self.arc_length * np.pi
            x = self.p1 - self.radius * (np.cos(angle) * self.normal_direction + np.sin(angle) * self.straight_direction)
        else:
            x = self.finish_line_pos + (d - 2*self.straight_length - 2*self.arc_length) * self.straight_direction

        return x * self.window_size
