from __future__ import print_function
from __future__ import absolute_import
from genericpath import exists

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
import numpy

from generateRouteFile import generate_routefile

TRAFFIC_INTERSECTION_TYPE = "triple"
file_separator = os.path.sep
POSSIBLE_TRAFFIC_LIGHT_CONFIGURATION = 4
TOTAL_NUMBER_OF_LANES = 24
NUMBER_OF_LANES_TO_OBSERVE = TOTAL_NUMBER_OF_LANES/2


class TrafficIntersectionEnvTripleLaneGUI(gym.Env):

    CONNECTION_LABEL = 0

    def __init__(self, sumocfg_file, network_file, route_file, use_gui=True, total_timesteps=5000, delta_time=30, min_green=15, max_green=120, yellow_time=7) -> None:
        super().__init__()

        self.sumocfg_file = sumocfg_file
        self.network_file = network_file
        self.route_file = route_file

        self.action_space = gym.spaces.Discrete(
            POSSIBLE_TRAFFIC_LIGHT_CONFIGURATION)
        self.observation_space = gym.spaces.Box(low=0, high=1000, shape=(
            int(NUMBER_OF_LANES_TO_OBSERVE), ), dtype=numpy.float64)
        self.state = numpy.zeros(int(NUMBER_OF_LANES_TO_OBSERVE))

        self.total_timesteps = total_timesteps
        self.delta_time = delta_time
        self.min_green = min_green
        self.max_green = max_green
        self.yellow_time = yellow_time
        self.label = TrafficIntersectionEnvTripleLaneGUI.CONNECTION_LABEL
        TrafficIntersectionEnvTripleLaneGUI.CONNECTION_LABEL += 1

        if use_gui:
            sumoBinary = sumolib.checkBinary('sumo-gui')
        else:
            sumoBinary = sumolib.checkBinary('sumo')

        self.sumoBinary = sumoBinary
        self.conn = None

    def step(self, action):

        done = False
        info = {}

        junction_with_lights = "J1"

        current_state = self.conn.trafficlight.getPhase(junction_with_lights)

        turn_yellow = True

        # if it's the same phase, then don't turn on yellow light
        if action == current_state / 2:  # dividing by 2 as green phases is 0, 2, 4, 6
            turn_yellow = False

        # Trying to turn on yellow light
        if turn_yellow:
            self.conn.trafficlight.setPhase(
                junction_with_lights, current_state * 2 + 1)

            delta_yellow_time = 0
            while delta_yellow_time < self.yellow_time:
                self.conn.simulationStep()
                delta_yellow_time += 1

        # Setting the required phase
        self.conn.trafficlight.setPhase(
            junction_with_lights, action * 2)

        delta_green_time = 0
        while delta_green_time < self.min_green:
            self.conn.simulationStep()
            delta_green_time += 1

        current_simulation_time = self.getSimulationTime()
        if current_simulation_time > self.total_timesteps:
            done = True
            self.reset()

        vehicle_count_lane_0 = self.conn.lane.getLastStepVehicleNumber("E0_0")
        vehicle_count_lane_1 = self.conn.lane.getLastStepVehicleNumber("E0_1")
        vehicle_count_lane_2 = self.conn.lane.getLastStepVehicleNumber("E0_2")

        vehicle_count_lane_3 = self.conn.lane.getLastStepVehicleNumber("-E1_0")
        vehicle_count_lane_4 = self.conn.lane.getLastStepVehicleNumber("-E1_1")
        vehicle_count_lane_5 = self.conn.lane.getLastStepVehicleNumber("-E1_2")

        vehicle_count_lane_6 = self.conn.lane.getLastStepVehicleNumber("-E2_0")
        vehicle_count_lane_7 = self.conn.lane.getLastStepVehicleNumber("-E2_1")
        vehicle_count_lane_8 = self.conn.lane.getLastStepVehicleNumber("-E2_2")

        vehicle_count_lane_9  = self.conn.lane.getLastStepVehicleNumber("-E3_0")
        vehicle_count_lane_10 = self.conn.lane.getLastStepVehicleNumber("-E3_1")
        vehicle_count_lane_11 = self.conn.lane.getLastStepVehicleNumber("-E3_2")

        lanes_observation = [vehicle_count_lane_0, vehicle_count_lane_1, vehicle_count_lane_2, vehicle_count_lane_3,
                             vehicle_count_lane_4, vehicle_count_lane_5, vehicle_count_lane_6, vehicle_count_lane_7,
                             vehicle_count_lane_8, vehicle_count_lane_9, vehicle_count_lane_10, vehicle_count_lane_11]
        self.state = numpy.array(lanes_observation)

        reward = self.calculate_reward()

        return self.state, reward, done, info

    def getSimulationTime(self):
        return self.conn.simulation.getTime()

    def reset(self):

        if self.conn is not None:

            traci.switch(self.label)
            traci.close()
            self.conn = None

        sumo_cmd = [self.sumoBinary,
                    '-n', self.network_file,
                    '-r', self.route_file,
                    '--waiting-time-memory', '10000',
                    '--start', '--quit-on-end',
                    "--time-to-teleport", "-1"  # This makes it so that the vehicles won't teleport
                    ]

        traci.start(sumo_cmd, label=self.label)
        self.conn = traci.getConnection(self.label)

        vehicle_count_lane_0 = self.conn.lane.getLastStepVehicleNumber("E0_0")
        vehicle_count_lane_1 = self.conn.lane.getLastStepVehicleNumber("E0_1")
        vehicle_count_lane_2 = self.conn.lane.getLastStepVehicleNumber("E0_2")

        vehicle_count_lane_3 = self.conn.lane.getLastStepVehicleNumber("-E1_0")
        vehicle_count_lane_4 = self.conn.lane.getLastStepVehicleNumber("-E1_1")
        vehicle_count_lane_5 = self.conn.lane.getLastStepVehicleNumber("-E1_2")

        vehicle_count_lane_6 = self.conn.lane.getLastStepVehicleNumber("-E2_0")
        vehicle_count_lane_7 = self.conn.lane.getLastStepVehicleNumber("-E2_1")
        vehicle_count_lane_8 = self.conn.lane.getLastStepVehicleNumber("-E2_2")

        vehicle_count_lane_9  = self.conn.lane.getLastStepVehicleNumber("-E3_0")
        vehicle_count_lane_10 = self.conn.lane.getLastStepVehicleNumber("-E3_1")
        vehicle_count_lane_11 = self.conn.lane.getLastStepVehicleNumber("-E3_2")

        lanes_observation = [vehicle_count_lane_0, vehicle_count_lane_1, vehicle_count_lane_2, vehicle_count_lane_3,
                             vehicle_count_lane_4, vehicle_count_lane_5, vehicle_count_lane_6, vehicle_count_lane_7,
                             vehicle_count_lane_8, vehicle_count_lane_9, vehicle_count_lane_10, vehicle_count_lane_11]
        
        self.state = numpy.array(lanes_observation)

        return self.state

    def calculate_reward(self):
        waiting_time_lane_0  = self.conn.lane.getWaitingTime("E0_0")
        waiting_time_lane_1  = self.conn.lane.getWaitingTime("E0_1")
        waiting_time_lane_2  = self.conn.lane.getWaitingTime("E0_2")
        waiting_time_lane_3  = self.conn.lane.getWaitingTime("-E1_0")
        waiting_time_lane_4  = self.conn.lane.getWaitingTime("-E1_1")
        waiting_time_lane_5  = self.conn.lane.getWaitingTime("-E1_2")
        waiting_time_lane_6  = self.conn.lane.getWaitingTime("-E2_0")
        waiting_time_lane_7  = self.conn.lane.getWaitingTime("-E2_1")
        waiting_time_lane_8  = self.conn.lane.getWaitingTime("-E2_2")
        waiting_time_lane_9  = self.conn.lane.getWaitingTime("-E3_0")
        waiting_time_lane_10 = self.conn.lane.getWaitingTime("-E3_1")
        waiting_time_lane_11 = self.conn.lane.getWaitingTime("-E3_2")

        total_waiting_time = \
            waiting_time_lane_0 + waiting_time_lane_1 + waiting_time_lane_2 + \
            waiting_time_lane_3 + waiting_time_lane_4 + waiting_time_lane_5 + \
            waiting_time_lane_6 + waiting_time_lane_7 + waiting_time_lane_8 + \
            waiting_time_lane_9 + waiting_time_lane_10 + waiting_time_lane_11

        return 1000 - total_waiting_time
