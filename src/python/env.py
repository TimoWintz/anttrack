import gym
from gym import spaces
import numpy as np
import os
import subprocess
import atexit
import torch
import mmap
import json
import time


class TrackCyclingInstance():
    def __init__(self):
        
        self.pipein = os.open("status", os.O_RDONLY)
        self.rider_ids = [0,1]
        self.desc_size = 24
        # write a simple example file
        with open("commands", "w+b") as f:
            f.write(b"\x00" * 1024)
            for id in self.rider_ids:
                print(id)
                f.seek(0)
                f.write(b'%8x%8f%8f' % (id, 0, 0))
            self.commands_mmap = mmap.mmap(f.fileno(), 1024)
        self.p = subprocess.Popen("src/CycloTron")
        atexit.register(self.cleanup)


    def send_command(self, rider_id, power, steering):
        self.commands_mmap[self.desc_size * len(self.rider_ids)] = 1
        time.sleep(0.01) # avoir race condition ? lol
        idx = self.rider_ids.index(rider_id)
        offset = self.desc_size * idx
        self.commands_mmap.seek(offset)
        byte_array =b'%8i%8i%8i' % (rider_id, power, steering)
        self.commands_mmap.write(byte_array)
        self.commands_mmap[self.desc_size * len(self.rider_ids)] = 0

    def read_status(self, rider_id):
        """
        self.send_command("status", rider_id)
        y = ""
        while(True):
            x = os.read(self.pipein, 1).decode()
            if x == "\n":
                break
            y = y + x
        y = y.split()
        res = {}
        for i in range(len(y) // 2):
            res[y[2*i]] = float(y[2*i+1])
        return res
        """

    def cleanup(self):
        self.p.kill()


class TrackCycling(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self, min_power, max_power, max_velocity, max_distance, track_width, max_steering,
                    rider_id, instance):
        super(TrackCycling, self).__init__()

        self.min_power = min_power
        self.max_power = max_power
        self.max_steering = max_steering
        self.max_velocity = max_velocity
        self.max_distance = max_distance
        self.track_width = track_width
        self.rider_id = rider_id

        self.instance = instance

        self.observations = ["velocity", "distance_to_finish", f"relative_height_{1-self.rider_id}",
                        f"distance_to_rider_{1-self.rider_id}"]
        self.min_obs = np.array([0.0, 0.0, -self.track_width, -self.max_distance], dtype=np.float32)
        self.max_obs = np.array([self.max_velocity, self.max_distance, self.track_width, self.max_distance], dtype=np.float32)

        self.actions = ["power", "steering"]
        self.min_action = np.array([self.min_power, -self.max_steering], dtype=np.float32)
        self.max_action = np.array([self.max_power, self.max_steering], dtype=np.float32)

        # Define action and observation space
        self.observation_space = spaces.Box(low=self.min_obs, high=self.max_obs, dtype=np.float32)
        self.action_space = spaces.Box(low=self.min_action, high=self.max_action, dtype=np.float32)

    def step(self, action):
        self.instance.send_command(self.rider_id, int(action[0]), int(action[1]))
        # status = self.instance.read_status(self.rider_id)
        # observation = np.zeros((len(self.observations), ), dtype=np.float32)
        # for i, o in enumerate(self.observations):
        #     observation[i] = status[o]
        # done = status["finished"] > 0
        # if done:
        #     reward = 1 if status["win"] > 0 else -1
        # else:
        #     reward = -0.1
        # return observation, reward, done, status

    def render(self, mode='human', close=False):
        pass

if __name__ == "__main__":
    instance = TrackCyclingInstance()
    tc_0 = TrackCycling(10, 400, 100, 100, 8, 45, 0, instance)
    tc_1 = TrackCycling(10, 400, 100, 100, 8, 45, 1, instance)
    # agent = DQNAgent(tc_0, use_conv=False)
    while True:
        tc_0.step(tc_0.action_space.sample())
        tc_1.step(tc_1.action_space.sample())
        time.sleep(0.1)
        #print(observation[1])
        #if done:
        #    break

    instance.p.wait()
