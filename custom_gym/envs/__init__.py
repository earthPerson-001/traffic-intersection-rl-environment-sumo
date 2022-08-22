
from gym.envs.registration import register

register(id='TrafficIntersectionEnvDoubleLaneGUI-v1',
    entry_point='envs.custom_env_dir:TrafficIntersectionEnvDoubleLaneGUI'
)

register(id='TrafficIntersectionEnvSingleLaneGUI-v1',
    entry_point='envs.custom_env_dir:TrafficIntersectionEnvSingleLaneGUI'
)