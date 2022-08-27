from __future__ import print_function
from __future__ import absolute_import
from genericpath import exists

import os
import sys
import optparse
from time import sleep

# checking for sumo_home variable and exiting if it is not found
if 'SUMO_HOME' in os.environ:
    tools: str = os.path.join(os.environ['SUMO_HOME'], 'tools')
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
POSSIBLE_TRAFFIC_LIGHT_CONFIGURATION = int(4)
TOTAL_NUMBER_OF_LANES = int(24)
NUMBER_OF_LANES_TO_OBSERVE = int(TOTAL_NUMBER_OF_LANES / 2)


class TrafficIntersectionEnvTripleLaneGUI(gym.Env):

    CONNECTION_LABEL = 0

    def __init__(self, sumocfg_file: str, network_file: str, route_file: str, use_gui: bool = True, total_timesteps: int = 5000, delta_time: int=30, min_green: int=15, max_green: int=120, yellow_time: int=7) -> None:

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

        self.lanes_to_observe = ["E0_0", "E0_1", "E0_2",
                                 "-E1_0", "-E1_1", "-E1_2",
                                 "-E2_0", "-E2_1", "-E2_2",
                                 "-E3_0", "-E3_1", "-E3_2"]

        self.label = TrafficIntersectionEnvTripleLaneGUI.CONNECTION_LABEL
        TrafficIntersectionEnvTripleLaneGUI.CONNECTION_LABEL += 1

        # for reward calculation
        self.last_waiting_time: float = 0

        if use_gui:
            sumoBinary = sumolib.checkBinary('sumo-gui')
        else:
            sumoBinary = sumolib.checkBinary('sumo')

        self.sumoBinary = sumoBinary
        self.conn: traci = None

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
                junction_with_lights, current_state + 1)

            delta_yellow_time = 0
            while delta_yellow_time < self.yellow_time:

                # Remove if unnecessary
                # Testing if phase duration defined in *.net.xml takes precedence over the traci.setPhase()
                self.conn.trafficlight.setPhase(junction_with_lights, current_state + 1)

                self.conn.simulationStep()

                delta_yellow_time += 1

        # Setting the required phase
        self.conn.trafficlight.setPhase(
            junction_with_lights, action * 2)

        delta_green_time = 0
        while delta_green_time < self.min_green:
            # Remove if unnecessary
            # Testing if phase duration defined in *.net.xml takes precedence over the traci.setPhase()
            self.conn.trafficlight.setPhase(junction_with_lights, action * 2)

            self.conn.simulationStep()
            delta_green_time += 1

        current_simulation_time = self.getSimulationTime()
        if current_simulation_time > self.total_timesteps:
            done = True
            self.reset()

        lanes_observation = numpy.zeros(int(NUMBER_OF_LANES_TO_OBSERVE))
        for i, lane in enumerate(self.lanes_to_observe):
            lanes_observation[i] = self.conn.lane.getLastStepVehicleNumber(lane)

        self.state = lanes_observation

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

        # for reward calculation
        self.last_waiting_time = 0

        # setting the vehicle count on observing lane as observation
        lanes_observation = numpy.zeros(int(NUMBER_OF_LANES_TO_OBSERVE))
        for i, lane in enumerate(self.lanes_to_observe):
            lanes_observation[i] = self.conn.lane.getLastStepVehicleNumber(lane)

        self.state = lanes_observation

        return self.state

    def calculate_waiting_time(self) -> float:
        total_waiting_time = 0
        for lane in self.lanes_to_observe:
            total_waiting_time += self.calculate_waiting_time_of_a_lane(lane)
        return total_waiting_time       

    def calculate_waiting_time_of_a_lane(self, lane: str) -> float:
        last_step_vehicles_ids = traci.lane.getLastStepVehicleIDs(lane)

        waiting_time = 0
        for vehicle in last_step_vehicles_ids:
            waiting_time += traci.vehicle.getAccumulatedWaitingTime(vehicle)

        return waiting_time

    def calculate_reward(self) -> float:
        
        total_waiting_time = self.calculate_waiting_time()

        decrease_in_waiting_time = self.last_waiting_time - total_waiting_time
        self.last_waiting_time = total_waiting_time

        return decrease_in_waiting_time
