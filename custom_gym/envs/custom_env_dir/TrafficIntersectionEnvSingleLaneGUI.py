from __future__ import print_function
from __future__ import absolute_import

import os
import sys
import optparse
from time import sleep

# checking for sumo_home variable and exiting if it is not found
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumo
import sumo.tools.sumolib as sumolib
from sumo.tools import traci
import gym

class TrafficIntersectionEnvSingleLaneGUI(gym.Env):

    def __init__(self) -> None:
        super().__init__()

    def step(self, action):
        pass

    def reset(self):
        pass
