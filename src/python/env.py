import gym
from gym import spaces
import numpy as np
import os
import subprocess
import atexit
import torch
from vanillaDQN.dqn import DQNAgent
from common.utils import mini_batch_train

class ReplayMemory(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


class TrackCyclingInstance():
    def __init__(self):
        self.p = subprocess.Popen("src/CycloTron")
        self.pipein = os.open("status", os.O_RDONLY)
        self.pipeout = os.open("commands", os.O_WRONLY)
        atexit.register(self.cleanup)

    def send_command(self, command, rider_id, value=None):
        if value is not None:
            os.write(self.pipeout, f"{command} {rider_id} {value}\n".encode())
        else:
            os.write(self.pipeout, f"{command} {rider_id}\n".encode())

    def read_status(self, rider_id):
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
        self.instance.send_command("power", self.rider_id, action[0])
        self.instance.send_command("steering", self.rider_id, action[1])
        status = self.instance.read_status(self.rider_id)
        observation = np.zeros((len(self.observations), ), dtype=np.float32)
        for i, o in enumerate(self.observations):
            observation[i] = status[o]
        done = status["finished"] > 0
        if done:
            reward = 1 if status["win"] > 0 else -1
        else:
            reward = -0.1
        return observation, reward, done, status

    def reset(self):
        os.write(self.pipeout, f"reset".encode())

    def render(self, mode='human', close=False):
        pass

if __name__ == "__main__":
    instance = TrackCyclingInstance()
    tc_0 = TrackCycling(10, 400, 100, 100, 8, 45, 0, instance)
    tc_1 = TrackCycling(10, 400, 100, 100, 8, 45, 1, instance)
    # agent = DQNAgent(tc_0, use_conv=False)
    while True:
        observation, reward, done, status = tc_0.step(tc_0.action_space.sample())
        observation, reward, done, status = tc_1.step(tc_1.action_space.sample())
        print(observation[1])
        if done:
            break

    instance.p.wait()
