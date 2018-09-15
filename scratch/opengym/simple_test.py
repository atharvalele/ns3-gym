import gym
import argparse
from PyOpenGymNs3 import ns3env

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2018, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"


env = gym.make('ns3-v0')
env.reset()

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space)
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

stepIdx = 0

try:
    while True:
        stepIdx += 1
        print("Step: ", stepIdx)

        action = env.action_space.sample()
        print("---action: ", action)
        obs, reward, done, info = env.step(action)
        print("---obs, reward, done, info: ", obs, reward, done, info)

        if done:
            break

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    env.close()
    print("Done")