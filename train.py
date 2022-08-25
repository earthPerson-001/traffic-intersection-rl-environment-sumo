from datetime import date, datetime
from enum import auto
from stable_baselines3 import PPO
from stable_baselines3 import A2C
import stable_baselines3.common.env_checker
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv
from custom_gym.envs.custom_env_dir import TrafficIntersectionEnvDoubleLaneGUI
import gym

TOTAL_TIMESTEPS_FOR_SUMO=500000
TOTAL_TIMESTEPS_FOR_MODEL=25000 # Switch the traffic light * times

startTime = datetime.now()

TRAFFIC_INTERSECTION_TYPE="triple"

# sumo stuffs
SUMO_DIRECTORY="/home/bishal/Programming/reinforcement-learning-env-gui/sumo-files"
net_file=SUMO_DIRECTORY + "/small-map-{}-lane.net.xml".format(TRAFFIC_INTERSECTION_TYPE)
route_file=SUMO_DIRECTORY + "/small-map-{}-lane.rou.xml".format(TRAFFIC_INTERSECTION_TYPE)
sumocfg_file=SUMO_DIRECTORY + "/small-map-{}-lane.sumocfg".format(TRAFFIC_INTERSECTION_TYPE)

env = gym.make("TrafficIntersectionEnv{}LaneGUI-v1".format(TRAFFIC_INTERSECTION_TYPE.capitalize()), sumocfg_file=sumocfg_file, network_file=net_file, route_file=route_file, use_gui=True, total_timesteps=TOTAL_TIMESTEPS_FOR_SUMO, min_green=30, yellow_time=10)
# env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run
env.reset()

# stable_baselines3.common.env_checker.check_env(env, warn=True, skip_render_check=True)


modelType = "ppo"
model = PPO("MlpPolicy", env, device="auto", verbose=1, batch_size=256, n_steps=256, n_epochs=5, tensorboard_log="./logs/{}-trafficintersection-{}-lane-GUI/".format(modelType, TRAFFIC_INTERSECTION_TYPE))

# model = PPO.load("/home/bishal/Programming/reinforcement-learning-env-gui/models/2022-08-25 21:07:44.963046-TrafficIntersection-{}LaneGUI-ppo-75000".format(TRAFFIC_INTERSECTION_TYPE.capitalize(), modelType), env=env)

count = 1
while count < 30:
    model.learn(total_timesteps=int(TOTAL_TIMESTEPS_FOR_MODEL), reset_num_timesteps=False, tb_log_name=f"GUI-{modelType}-{startTime}")
    model.save("./models/{}-TrafficIntersection-{}LaneGUI-{}-{}".format(startTime, TRAFFIC_INTERSECTION_TYPE.capitalize(), modelType, count * TOTAL_TIMESTEPS_FOR_MODEL))
    count += 1
