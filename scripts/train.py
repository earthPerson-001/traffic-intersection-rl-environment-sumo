from stable_baselines3 import PPO
from stable_baselines3 import A2C
import stable_baselines3.common.env_checker
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv
import gym

from datetime import date, datetime

import sys
from pathlib import Path
import os


# Training constants
TRAFFIC_INTERSECTION_TYPE="triple"
TOTAL_TIMESTEPS_FOR_SUMO=500000
TOTAL_TIMESTEPS_FOR_MODEL=25000 # call env.step() this many times

#times
MIN_GREEN_TIME=30
YELLOW_TIME=10


def train():
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[1]  # traffic-intersection-rl-environment-sumo root directory
    if str(ROOT) not in sys.path:
        sys.path.append(str(ROOT))  # add ROOT to PATH
    ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

    from custom_gym.envs.custom_env_dir import TrafficIntersectionEnvDoubleLaneGUI

    # sumo stuffs
    SUMO_DIRECTORY= Path(str(ROOT) + "/sumo-files").resolve()
    net_file=Path(str(SUMO_DIRECTORY) + "/small-map-{}-lane.net.xml".format(TRAFFIC_INTERSECTION_TYPE)).resolve()
    route_file=Path(str(SUMO_DIRECTORY) + "/small-map-{}-lane.rou.xml".format(TRAFFIC_INTERSECTION_TYPE)).resolve()
    sumocfg_file=Path(str(SUMO_DIRECTORY) + "/small-map-{}-lane.sumocfg".format(TRAFFIC_INTERSECTION_TYPE)).resolve()

    startTime = datetime.now()

    # model stuffs
    modelType = "ppo"
    use_gui = False

    # path to save the model or load the model from
    models_path = Path(str(ROOT) + "/models").resolve()
    log_path = Path(str(ROOT) + "/logs/{}-trafficintersection-{}-lane-GUI/".format(modelType, TRAFFIC_INTERSECTION_TYPE)).resolve()

    env = gym.make("TrafficIntersectionEnv{}LaneGUI-v1".format(TRAFFIC_INTERSECTION_TYPE.capitalize()), sumocfg_file=sumocfg_file, network_file=net_file, route_file=route_file, use_gui=use_gui, total_timesteps=TOTAL_TIMESTEPS_FOR_SUMO, min_green=30, yellow_time=10)
    # env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run
    env.reset()

    # stable_baselines3.common.env_checker.check_env(env, warn=True, skip_render_check=True)

    model = PPO("MlpPolicy", env, device="auto", verbose=1, batch_size=256, n_steps=256, n_epochs=5, tensorboard_log=log_path)

    #model = PPO.load(Path(str(models_path) + "/2022-08-26 20:31:22.136701-TrafficIntersection-{}LaneGUI-ppo-300000".format(TRAFFIC_INTERSECTION_TYPE.capitalize(), modelType), env=env).resolve())

    count = 1
    while count < 30:
        model.learn(total_timesteps=int(TOTAL_TIMESTEPS_FOR_MODEL), reset_num_timesteps=False, tb_log_name=f"{'GUI' if use_gui else 'CLI'}-{modelType}-{startTime}")
        save_path = Path(str(models_path) + "/{}-TrafficIntersection-{}LaneGUI-{}-{}".format(startTime, TRAFFIC_INTERSECTION_TYPE.capitalize(), modelType, count * TOTAL_TIMESTEPS_FOR_MODEL)).resolve()
        model.save(str(save_path))
        count += 1

if __name__=="__main__":
    train()