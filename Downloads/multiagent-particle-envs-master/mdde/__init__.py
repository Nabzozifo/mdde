from gym.envs.registration import register

register(
    'mdde-v0',
    entry_point="mdee.environment:MultiAgentDrone",

)
