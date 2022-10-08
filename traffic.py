#! /bin/python

'''
This script is to test the working of trained model.
A lot of unnecessary code is here, remove the code for different intersection when not required or 
make it work for general intersection.

For now all the codes is bundled unordered, separated by if else statements.

'''



from __future__ import print_function
from __future__ import absolute_import

import os
import sys
import optparse
import numpy

from generateRouteFile import generate_routefile

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
from stable_baselines3 import PPO

import sys
from pathlib import Path
import os

from custom_gym.envs.custom_env_dir import TrafficIntersectionEnvTripleLaneGUI

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # traffic-intersection-rl-environment-sumo root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# constants
TRAFFIC_INTERSECTION_TYPE="triple"
TOTAL_TIMESTEPS=50000  # This is the sumo timestep which is somewhat independent of steps in simulation
                       # rather this decides the number of steps in the simulation
GENERATE_CUSTOM_ROUTE=False

YELLOW_TIME = 10
MIN_GREEN_TIME = 30

# environment stuffs
time_to_teleport = "-1" # setting time to teleport to -1 makes it so that vehicles wont teleport
use_gui = True # gui

# sumo stuffs
SUMO_DIRECTORY= Path(str(ROOT) + "/sumo-files").resolve()
net_file=Path(str(SUMO_DIRECTORY) + "/small-map-{}-lane.net.xml".format(TRAFFIC_INTERSECTION_TYPE)).resolve()
route_file=Path(str(SUMO_DIRECTORY) + "/small-map-{}-lane.rou.xml".format(TRAFFIC_INTERSECTION_TYPE)).resolve()
sumocfg_file=Path(str(SUMO_DIRECTORY) + "/small-map-{}-lane.sumocfg".format(TRAFFIC_INTERSECTION_TYPE)).resolve()

env = gym.make('TrafficIntersectionEnv{}LaneGUI-v1'.format(TRAFFIC_INTERSECTION_TYPE.capitalize()), sumocfg_file=sumocfg_file, network_file=net_file, route_file=route_file, use_gui=use_gui)

# path to save the model or load the model from
models_path = Path(str(ROOT) + "/models").resolve()

# Here, formatting is done as to create error if wrong model is selected
# as, there won't be same model trained at exact same time and upto same timesteps
model_path = Path(str(models_path) + "/2022-10-08 15:34:31.621671-TrafficIntersection-{}LaneGUI-ppo-2500".format(TRAFFIC_INTERSECTION_TYPE.capitalize())).resolve()
model = PPO.load(str(model_path))

def run():
    step = 0

    while step < TOTAL_TIMESTEPS:

        if step == 0: # setting the initial configuration
            if TRAFFIC_INTERSECTION_TYPE == "single":
                traci.trafficlight.setPhase("J9", 4)
            elif TRAFFIC_INTERSECTION_TYPE == "double":
                traci.trafficlight.setPhase("J11", 4)
            elif TRAFFIC_INTERSECTION_TYPE == "triple":
                traci.trafficlight.setPhase("J1", 4)

            traci.simulationStep()
            step = traci.simulation.getTime()
            continue
        current_state = None
        next_configuration = current_state  # Doing this so that next configuration is defined if none of the conditions is met
        junction_with_lights = None

        if TRAFFIC_INTERSECTION_TYPE == "single":

            junction_with_lights = "J9"

            current_state = traci.trafficlight.getPhase(junction_with_lights)


            # starting from left-upper lane, skipping one lane
            vehicle_count_lane_0 = traci.lane.getLastStepVehicleNumber("-E8_0")
            vehicle_count_lane_1 = traci.lane.getLastStepVehicleNumber("-E9_0")
            vehicle_count_lane_2 = traci.lane.getLastStepVehicleNumber("-E10_0")
            vehicle_count_lane_3 = traci.lane.getLastStepVehicleNumber("E7_0")

            lanes_observation = [vehicle_count_lane_0, vehicle_count_lane_1, vehicle_count_lane_2, vehicle_count_lane_3]

            next_configuration, _state = model.predict(lanes_observation, deterministic=True)

            if next_configuration == current_state / 2: # dividing by 2 as green phases is 0, 2, 4, 6
                traci.trafficlight.setPhase(junction_with_lights, current_state)
            else:
                # Trying to turn on yellow light
                delta_yellow_time = 0
                while delta_yellow_time < YELLOW_TIME:
                    traci.trafficlight.setPhase(junction_with_lights, current_state + 1)
                    traci.simulationStep()
                    delta_yellow_time += 1

        elif TRAFFIC_INTERSECTION_TYPE == "double":

            junction_with_lights = "J11"

            current_state = traci.trafficlight.getPhase(junction_with_lights)


            vehicle_count_lane_0 = traci.lane.getLastStepVehicleNumber("E9_0")
            vehicle_count_lane_1 = traci.lane.getLastStepVehicleNumber("E9_1")
            vehicle_count_lane_2 = traci.lane.getLastStepVehicleNumber("E8_0")
            vehicle_count_lane_3 = traci.lane.getLastStepVehicleNumber("E8_1")
            vehicle_count_lane_4 = traci.lane.getLastStepVehicleNumber("-E10_0")
            vehicle_count_lane_5 = traci.lane.getLastStepVehicleNumber("-E10_1")
            vehicle_count_lane_6 = traci.lane.getLastStepVehicleNumber("-E11_0")
            vehicle_count_lane_7 = traci.lane.getLastStepVehicleNumber("-E11_1")

            lanes_observation = [vehicle_count_lane_0, vehicle_count_lane_1, vehicle_count_lane_2, vehicle_count_lane_3, vehicle_count_lane_4, vehicle_count_lane_5, vehicle_count_lane_6, vehicle_count_lane_7]
            next_configuration, _state = model.predict(lanes_observation, deterministic=True)

            if next_configuration == current_state / 2: # dividing by 2 as green phases is 0, 2, 4, 6
                traci.trafficlight.setPhase(junction_with_lights, current_state)
            else:
                # Trying to turn on yellow light
                delta_yellow_time = 0
                while delta_yellow_time < YELLOW_TIME:
                    traci.trafficlight.setPhase(junction_with_lights, current_state + 1)
                    traci.simulationStep()
                    delta_yellow_time += 1

        elif TRAFFIC_INTERSECTION_TYPE == "triple":

            junction_with_lights = "J1"

            current_state = traci.trafficlight.getPhase(junction_with_lights)

            lanes_to_observe = ["E0_0", "E0_1", "E0_2",
                             "-E1_0", "-E1_1", "-E1_2",
                             "-E2_0", "-E2_1", "-E2_2",
                             "-E3_0", "-E3_1", "-E3_2"]
                
            lanes_observation = numpy.zeros(lanes_to_observe.__len__())
            for i, lane in enumerate(lanes_to_observe):
                lanes_observation[i] = traci.lane.getLastStepVehicleNumber(lane)
                
            next_configuration, _state = model.predict(lanes_observation, deterministic=True)

            if next_configuration == current_state / 2: # dividing by 2 as green phases is 0, 2, 4, 6
                traci.trafficlight.setPhase(junction_with_lights, current_state)
            else:
                # Trying to turn on yellow light
                delta_yellow_time = 0
                while delta_yellow_time < YELLOW_TIME:
                    traci.trafficlight.setPhase(junction_with_lights, current_state + 1)
                    traci.simulationStep()
                    delta_yellow_time += 1

        # Turning green light for the predefined period of time
        delta_green_time = 0
        while delta_green_time < MIN_GREEN_TIME:
            traci.trafficlight.setPhase(junction_with_lights, next_configuration * 2)
            traci.simulationStep()
            delta_green_time += 1

        step = traci.simulation.getTime()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = sumolib.checkBinary('sumo')
    else:
        sumoBinary = sumolib.checkBinary('sumo-gui')

    # Generating custom route file
    if GENERATE_CUSTOM_ROUTE and TRAFFIC_INTERSECTION_TYPE == "double":
        route_file = generate_routefile()
   
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary,
    "-n", net_file,
    "-r", route_file,
    "--time-to-teleport", time_to_teleport])

    run()

    traci.close()