import pyglet
import math
import time
import numpy as np
import speed_sensor

from velodrome import Velodrome
from cyclist import Cyclist, ConstantPCyclist, PowerMeterCyclist

p1 = np.array([0.21, 0.49])
p2 = np.array([0.77, 0.49])
radius = 0.15

window_w = 800
window_h = 800

window = pyglet.window.Window(800, 800)

background_image = pyglet.resource.image('anquetil.png')
cyclist_image = pyglet.resource.image('cycle.png')
cyclist_image_2 = pyglet.resource.image('cycle2.png')

background_sprite = pyglet.sprite.Sprite(background_image, x=0, y=0)
background_sprite.update(scale_x=window_w/background_sprite.width, scale_y=window_h/background_sprite.height)
cyclist_sprite = pyglet.sprite.Sprite(cyclist_image, x=0, y=0)
cyclist_sprite_2 = pyglet.sprite.Sprite(cyclist_image_2, x=0, y=0)
# cyclist_sprite.update(scale=0.1)

velodrome = Velodrome(p1, p2, radius, [window_w, window_h])
cyclist = ConstantPCyclist(cyclist_sprite, velodrome, 350)
cyclist_2 = ConstantPCyclist(cyclist_sprite_2, velodrome, 360)

# speed_sensor_monitor = speed_sensor.Monitor(speed_sensor.TYPE_SPEED)
# speed_sensor_monitor.start()

@window.event
def on_draw():
    background_sprite.draw()
    cyclist.draw()
    cyclist_2.draw()

while True:
    dt = pyglet.clock.tick()

    cyclist.update_power()
    cyclist.update(dt)

    cyclist_2.update_power()
    cyclist_2.update(dt)

    for window in pyglet.app.windows:
        window.switch_to()
        window.dispatch_events()
        window.dispatch_event('on_draw')
        window.flip()
