import time
import pyglet

class Cyclist():
    def __init__(self, sprite, velodrome):
        self.sprite = sprite
        self.distance = 0
        self.speed = 0
        self.power = 400
        self.CdA = 0.3
        self.Cxx = 0.005
        self.rho = 1.3
        self.mass = 70
        self.max_acceleration = 20
        self.velodrome = velodrome
        self.last_pass = time.time()
        self.lap_time = 0.0

    def update(self, dt):
        self.update_speed(dt)
        self.distance += dt * self.speed

        if (self.distance >= self.velodrome.track_length):
            dx = self.distance - self.velodrome.track_length
            delta = dx / self.speed
            t = time.time()
            self.lap_time = t - self.last_pass - delta
            self.last_pass = t - delta
            self.distance -= self.velodrome.track_length
            print("lap time = %.2f"%self.lap_time)

    def update_speed(self, dt):
        if self.speed == 0:
            self.speed += self.max_acceleration * dt
            return
        v = self.speed
        P = self.power
        a = 1 / self.mass * (self.power / self.speed -  0.5 * self.rho * self.CdA * v * v) - self.Cxx * 9.81
        if a > self.max_acceleration:
            a = self.max_acceleration
        self.speed = self.speed + dt * a
        if self.speed < 0:
            self.speed = 0

    def draw(self):
        pos = self.velodrome.pos_from_distance(self.distance)
        self.sprite.update(x = pos[0], y = pos[1])
        self.sprite.draw()

        label_speed = pyglet.text.Label('Speed: %.1fkm/h'%(self.speed*3.6), font_name='Times New Roman', font_size=36, x=10, y=10)
        label_speed.draw()

        label_lap = pyglet.text.Label('Lap time: %.2fs'%(self.lap_time), font_name='Times New Roman', font_size=36, x=10, y=50)
        label_lap.draw()

class PowerMeterCyclist(Cyclist):
    def __init__(self, sprite, velodrome, monitor):
        Cyclist.__init__(self, sprite, velodrome)
        self.monitor = monitor

    def update_power(self):
        self.power = self.monitor.power

class ConstantPCyclist(Cyclist):
    def __init__(self, sprite, velodrome, power):
        Cyclist.__init__(self, sprite, velodrome)
        self.power = power

    def update_power(self):
        pass
