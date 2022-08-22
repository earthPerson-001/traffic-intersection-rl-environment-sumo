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

TRAFFIC_INTERSECTION_TYPE = "double"
file_separator = os.path.sep
POSSIBLE_TRAFFIC_LIGHT_CONFIGURATION = 4
TOTAL_NUMBER_OF_LANES = 16
NUMBER_OF_LANES_TO_OBSERVE = TOTAL_NUMBER_OF_LANES/2


class TrafficIntersectionEnvDoubleLaneGUI(gym.Env):

    CONNECTION_LABEL =0

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
        self.label = TrafficIntersectionEnvDoubleLaneGUI.CONNECTION_LABEL
        TrafficIntersectionEnvDoubleLaneGUI.CONNECTION_LABEL += 1

        if use_gui:
            sumoBinary = sumolib.checkBinary('sumo-gui')
        else:
            sumoBinary = sumolib.checkBinary('sumo')

        self.sumoBinary = sumoBinary
        self.conn = None


    def step(self, action):

        done = False
        info = {}

        junction_with_lights = "J11"

        current_state = self.conn.trafficlight.getPhase(junction_with_lights)

        turn_yellow = True

        # if it's the same phase, then don't turn on yellow light
        if action == current_state / 2:  # dividing by 2 as green phases is 0, 2, 4, 6
            turn_yellow = False

        # Trying to turn on yellow light
        if turn_yellow:
            self.conn.trafficlight.setPhase(
                junction_with_lights, current_state + 1)

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

        vehicle_count_lane_0 = self.conn.lane.getLastStepVehicleNumber("E9_0")
        vehicle_count_lane_1 = self.conn.lane.getLastStepVehicleNumber("E9_1")
        vehicle_count_lane_2 = self.conn.lane.getLastStepVehicleNumber("E8_0")
        vehicle_count_lane_3 = self.conn.lane.getLastStepVehicleNumber("E8_1")
        vehicle_count_lane_4 = self.conn.lane.getLastStepVehicleNumber("-E10_0")
        vehicle_count_lane_5 = self.conn.lane.getLastStepVehicleNumber("-E10_1")
        vehicle_count_lane_6 = self.conn.lane.getLastStepVehicleNumber("-E11_0")
        vehicle_count_lane_7 = self.conn.lane.getLastStepVehicleNumber("-E11_1")

        lanes_observation = [vehicle_count_lane_0, vehicle_count_lane_1, vehicle_count_lane_2, vehicle_count_lane_3,
                             vehicle_count_lane_4, vehicle_count_lane_5, vehicle_count_lane_6, vehicle_count_lane_7]
        self.state = numpy.array(lanes_observation)

        reward = self.calculate_reward()

        return self.state, reward, done, info

    def getSimulationTime(self):
        return self.conn.simulation.getTime()

    def reset(self):
        
        if self.conn is not None:
        
            # generating new route file each time
            # and not touching the previous route file but saving in that same location.

            route_file_dir = self.route_file.rsplit(file_separator, 1)[0]
            new_route_file = route_file_dir + file_separator + "{}-route.rou.xml".format(self.getSimulationTime())

            generate_routefile(new_route_file)

            self.route_file = new_route_file

            traci.switch(self.label)
            traci.close()
            self.conn = None


        sumo_cmd = [self.sumoBinary,
                     '-n', self.network_file,
                     '-r', self.route_file,
                     '--waiting-time-memory', '10000',
                     '--start', '--quit-on-end',
                     "--time-to-teleport", "-1"  # This makes it so the the vehicles won't teleport
                     ]

        traci.start(sumo_cmd, label=self.label)
        self.conn = traci.getConnection(self.label)

        vehicle_count_lane_0 = self.conn.lane.getLastStepVehicleNumber("E9_0")
        vehicle_count_lane_1 = self.conn.lane.getLastStepVehicleNumber("E9_1")
        vehicle_count_lane_2 = self.conn.lane.getLastStepVehicleNumber("E8_0")
        vehicle_count_lane_3 = self.conn.lane.getLastStepVehicleNumber("E8_1")
        vehicle_count_lane_4 = self.conn.lane.getLastStepVehicleNumber(
            "-E10_0")
        vehicle_count_lane_5 = self.conn.lane.getLastStepVehicleNumber(
            "-E10_1")
        vehicle_count_lane_6 = self.conn.lane.getLastStepVehicleNumber(
            "-E11_0")
        vehicle_count_lane_7 = self.conn.lane.getLastStepVehicleNumber(
            "-E11_1")

        lanes_observation = [vehicle_count_lane_0, vehicle_count_lane_1, vehicle_count_lane_2, vehicle_count_lane_3,
                             vehicle_count_lane_4, vehicle_count_lane_5, vehicle_count_lane_6, vehicle_count_lane_7]
        self.state = numpy.array(lanes_observation)

        return self.state

    def calculate_reward(self):
        waiting_time_lane_0 = self.conn.lane.getWaitingTime("E9_0")
        waiting_time_lane_1 = self.conn.lane.getWaitingTime("E9_1")
        waiting_time_lane_2 = self.conn.lane.getWaitingTime("E8_0")
        waiting_time_lane_3 = self.conn.lane.getWaitingTime("E8_1")
        waiting_time_lane_4 = self.conn.lane.getWaitingTime("-E10_0")
        waiting_time_lane_5 = self.conn.lane.getWaitingTime("-E10_1")
        waiting_time_lane_6 = self.conn.lane.getWaitingTime("-E11_0")
        waiting_time_lane_7 = self.conn.lane.getWaitingTime("-E11_1")

        total_waiting_time = waiting_time_lane_0 + waiting_time_lane_1 + waiting_time_lane_2 + \
            waiting_time_lane_3 + waiting_time_lane_4 + \
            waiting_time_lane_5 + waiting_time_lane_6 + waiting_time_lane_7

        return 1000 - total_waiting_time
