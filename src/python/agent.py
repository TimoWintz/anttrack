import gym
import math
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from collections import namedtuple
from itertools import count
from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

Transition = namedtuple('Transition',
    ('state', 'action', 'next_state', 'reward'))


env = gym.make('CartPole-v0').unwrapped

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
