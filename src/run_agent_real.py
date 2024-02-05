#!/usr/bin/env python

import rospy
import os
import numpy as np
import random
import time
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float32
import torch
import torch.nn.functional as F
import gc
import torch.nn as nn
from collections import deque
from ppo.storage import Memory
from ppo.ppo_models import PPO
# from env.training_environment import Env as train_env
# from env.testing_environment import Env as test_env
# from src.env.testing_respawn_real import Respawn
from src.env.training_environment_real import Env as train_env
from src.env.testing_environment_real import Env as test_env
# from ppo_alg import PPO_agent
from ppo_alg_real import PPO_agent
from ddpg_alg import DDPG_agent

import pickle
import json

from dqn.replay_buffer import ReplayBuffer
from dqn.dqn_models import DQN
from dqn_alg import DQN_agent

# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
# dirPath = os.path.dirname(os.path.realpath(__file__))
# env_module_id = 2

# env_module_id = 0

# print(sys.argv[1])
# print(sys.argv[2])
# print(sys.argv[3])
# print(sys.argv[4])

# env_module_id = int(sys.argv[3])

if sys.argv[3] == "real":
    env_module_id = sys.argv[3]

else:
    env_module_id = int(sys.argv[3])


MAX_STEPS = 500
# MAX_EPISODES = 10001

# MAX_EPISODES = 1001
# MAX_EPISODES = 2001
# MAX_EPISODES = int(sys.argv[4]) + 1
# MAX_EPISODES = int(sys.argv[4]) + 1
# print(MAX_EPISODES)


from geometry_msgs.msg import Twist

from numpy import array

from sensor_msgs.msg import LaserScan


def hook():
    # print("stopping and returning to origin")
    # env.sendResetGoal(array([0, 0]))
    # rospy.sleep(0.5)
    # env.rate.sleep()
    print("stopping and returning to origin")
    vel_cmd = Twist()
    vel_cmd.linear.x = 0
    vel_cmd.angular.z = 0
    env.pub_cmd_vel.publish(vel_cmd)

    data = None
    # while data is None:
    #     try:
    #         data = rospy.wait_for_message('scan', LaserScan, timeout=5)
    #     except:
    #         pass

    try:
        data = rospy.wait_for_message('scan', LaserScan, timeout=5)
    except:
        pass

    env.sendResetGoal(array([0, 0]), data)
    rospy.sleep(0.5)
    env.rate.sleep()


if __name__ == '__main__':

    rospy.init_node('run_agent')

    global env

    Env = test_env
    load_ep = 0
    env = None
    agent = None

    # Choose correct environment
    if (sys.argv[1] == "train"):
        Env = train_env

    # # arg 4 set load ep if specified
    # if len(sys.argv) <= 3 + 3:
    #     load_ep = 0
    # else:
    #     load_ep = int(sys.argv[4])
    #
    #     try:
    #         with open(agent.savePath + "goals.txt", "rb") as f:
    #             env.goals, env.GoalRates, env.steps_to_goals = pickle.load(f)
    #
    #         with open(agent.savePath + "steps_to_goals.txt", "rb") as f:
    #             env.average_steps_to_goal_per_episode_list = pickle.load(f)
    #     except:
    #         env.goals, env.GoalRates, env.steps_to_goals = [], [], []
    #         env.average_steps_to_goal_per_episode_list = []

    # arg 2, agent PPO, DDPG, initialize Env(PPO) etc
    if (sys.argv[2] == "ppo"):

        if (sys.argv[1] == "train"):
            dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/saved_models/ppo/env-{}/'.format(env_module_id)
        else:
            dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/saved_models/ppo/'


        try:
            os.mkdir(dirPath)
        except OSError as e:
            #print("CSV Directory exists")
            pass

        env = Env("PPO", env_module_id)

        # arg 4 set load ep if specified
        if len(sys.argv) < 6 + 2:
            MAX_EPISODES = int(sys.argv[4])
            load_ep = 0
        else:
            MAX_EPISODES = int(sys.argv[4]) + 1
            load_ep = int(sys.argv[5])
            # print(load_ep)

            # with open(dirPath + "goals.txt", "rb") as f:
            #     # env.goals, env.GoalRates, env.steps_to_goals = pickle.load(f)
            #     env.goals, env.GoalRates = pickle.load(f)
            #
            # # with open(dirPath + "steps_to_goals.txt", "rb") as f:
            # #     env.average_steps_to_goal_per_episode_list = pickle.load(f)
            #
            # with open(dirPath + 'tot_num_goals.json') as outfile:
            #     param = json.load(outfile)
            #     # start_time = start_time - param.get('t_elapsed')
            #     env.number_of_goals = param.get('tot_num_goals')

            try:
                with open(dirPath + "goals.txt", "rb") as f:
                    # env.goals, env.GoalRates, env.steps_to_goals = pickle.load(f)
                    env.goals, env.GoalRates = pickle.load(f)

                # with open(dirPath + "steps_to_goals.txt", "rb") as f:
                #     env.average_steps_to_goal_per_episode_list = pickle.load(f)

                with open(dirPath + 'tot_num_goals.json') as outfile:
                    param = json.load(outfile)
                    # start_time = start_time - param.get('t_elapsed')
                    env.number_of_goals = param.get('tot_num_goals')

            except:
                # env.goals, env.GoalRates, env.steps_to_goals = [], [], []
                # env.average_steps_to_goal_per_episode_list = []

                env.goals, env.GoalRates = [], []

        agent = PPO_agent(load_ep, env, MAX_STEPS, dirPath)
    elif (sys.argv[2] == "ddpg"):

        if (sys.argv[1] == "train"):
            dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/saved_models/ddpg/env-{}/'.format(env_module_id)
        else:
            dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/saved_models/ddpg/'

        try:
            os.mkdir(dirPath)
        except OSError as e:
            #print("CSV Directory exists")
            pass

        env = Env("DDPG", env_module_id)

        # arg 4 set load ep if specified
        if len(sys.argv) <= 3 + 3:
            load_ep = 0
        else:
            load_ep = int(sys.argv[4])

            try:
                with open(dirPath + "goals.txt", "rb") as f:
                    # env.goals, env.GoalRates, env.steps_to_goals = pickle.load(f)
                    env.goals, env.GoalRates = pickle.load(f)

                # with open(dirPath + "steps_to_goals.txt", "rb") as f:
                #     env.average_steps_to_goal_per_episode_list = pickle.load(f)

                with open(dirPath + 'tot_num_goals.json') as outfile:
                    param = json.load(outfile)
                    # start_time = start_time - param.get('t_elapsed')
                    env.number_of_goals = param.get('tot_num_goals')
            except:
                # env.goals, env.GoalRates, env.steps_to_goals = [], [], []
                # env.average_steps_to_goal_per_episode_list = []

                env.goals, env.GoalRates = [], []

                env.number_of_goals = 0
                # start_time = time.time()


        agent = DDPG_agent(load_ep, env, MAX_STEPS, dirPath)

    elif (sys.argv[2] == "dqn"):

        if (sys.argv[1] == "train"):
            dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/saved_models/dqn/env-{}/'.format(env_module_id)
        else:
            dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/saved_models/dqn/'

        # print(dirPath)
        #
        # os.mkdir(dirPath)

        try:
            os.makedirs(dirPath)
            print("directory created")
        except OSError as e:
            #print("CSV Directory exists")
            pass

        env = Env("DQN", env_module_id)

        # arg 4 set load ep if specified
        if len(sys.argv) <= 3 + 3:
            load_ep = 0
        else:
            load_ep = int(sys.argv[4])

            try:
                with open(dirPath + "goals.txt", "rb") as f:
                    # env.goals, env.GoalRates, env.steps_to_goals = pickle.load(f)
                    env.goals, env.GoalRates = pickle.load(f)

                # with open(dirPath + "steps_to_goals.txt", "rb") as f:
                #     env.average_steps_to_goal_per_episode_list = pickle.load(f)

                with open(dirPath + 'tot_num_goals.json') as outfile:
                    param = json.load(outfile)
                    # start_time = start_time - param.get('t_elapsed')
                    env.number_of_goals = param.get('tot_num_goals')
            except:
                # env.goals, env.GoalRates, env.steps_to_goals = [], [], []
                # env.average_steps_to_goal_per_episode_list = []

                env.goals, env.GoalRates = [], []

                env.number_of_goals = 0
                # start_time = time.time()


        agent = DQN_agent(load_ep, env, MAX_STEPS, dirPath)


    # if len(sys.argv) <= 3 + 3:
    #     state = env.resetReal()
    # else:
    #     state = env.resetRealResume()

    if len(sys.argv) <= 3 + 3:
        state = env.resetRealResume()
    else:
        state = env.resetRealResume()


    for ep in range(load_ep, MAX_EPISODES, 1):
        collision = 0
        goal = 0
        running_reward = 0
        ep_steps = 0
        # state = env.reset()
        # # if agent.load_model:
        # #     state = env.resetRealResume()
        # # else:
        # #     state = env.resetReal()

        # print(sys.argv[5])
        #
        # if (sys.argv[5] == "real"):
        #     print("real reset")
        #     state = env.resetReal()
        # else:
        #     state = env.reset()

        # # state = env.resetRealDirect()
        # state = env.resetReal()
        # # state = env.resetRealDiffGoal()
        # # state = env.resetRealDiffStartingPoint()

        # if len(sys.argv) <= 3 + 3:
        #     state = env.resetReal()
        # else:
        #     state = env.resetRealResume()


        for step in range(MAX_STEPS):
            ep_steps += 1
            # state, reward, collision, goal = agent.step(state,ep)
            state, reward, done = agent.step(state,ep)
            running_reward += reward
            # if (collision or goal or step == MAX_STEPS - 1):
            #     break

            if ( done or step == MAX_STEPS - 1):

                env.DispEpisodeCSV_real(running_reward, ep_steps, env.GoalRates, env.number_of_goals)

                print("Episode " + str(ep))

                if (sys.argv[1] == "train"):
                    agent.save(ep)

                if (sys.argv[2] == "dqn"):
                    agent.decay_epsilon()

                    param_keys = ['tot_num_goals', 'epsilon']
                    param_values = [env.number_of_goals, agent.eps]
                    # param_values = [agent.epsilon]

                    param_dictionary = dict(zip(param_keys, param_values))

                    # with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
                    #     json.dump(param_dictionary, outfile)

                    with open(dirPath + 'tot_num_goals_epsilon.json', 'w') as outfile:
                        json.dump(param_dictionary, outfile)
                else:
                    param_keys = ['tot_num_goals']
                    param_values = [env.number_of_goals]
                    # param_values = [agent.epsilon]

                    param_dictionary = dict(zip(param_keys, param_values))

                    # with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
                    #     json.dump(param_dictionary, outfile)

                    with open(dirPath + 'tot_num_goals.json', 'w') as outfile:
                        json.dump(param_dictionary, outfile)

                with open(dirPath + "goals.txt", "wb") as f:
                    # pickle.dump([env.goals, env.GoalRates, env.steps_to_goals], f)
                    pickle.dump([env.goals, env.GoalRates], f)

                if step == MAX_STEPS - 1:
                    state = env.resetRealResume()
                else:
                    state = env.resetReal()

                break

        # env.logEpisode(running_reward, collision, goal, ep_steps)
        #
        # env.DispEpisodeCSV(running_reward, collision, goal, ep_steps, env.GoalRates, env.number_of_goals)


        # env.createLog_real(running_reward, done, ep_steps)

        # env.DispEpisodeCSV_real(running_reward, ep_steps, env.GoalRates, env.number_of_goals)

        # print("Episode " + str(ep))
        #
        # if (sys.argv[1] == "train"):
        #     agent.save(ep)
        #
        # if (sys.argv[2] == "dqn"):
        #     agent.decay_epsilon()
        #
        #     param_keys = ['tot_num_goals', 'epsilon']
        #     param_values = [env.number_of_goals, agent.eps]
        #     # param_values = [agent.epsilon]
        #
        #     param_dictionary = dict(zip(param_keys, param_values))
        #
        #     # with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
        #     #     json.dump(param_dictionary, outfile)
        #
        #     with open(dirPath + 'tot_num_goals_epsilon.json', 'w') as outfile:
        #         json.dump(param_dictionary, outfile)
        # else:
        #     param_keys = ['tot_num_goals']
        #     param_values = [env.number_of_goals]
        #     # param_values = [agent.epsilon]
        #
        #     param_dictionary = dict(zip(param_keys, param_values))
        #
        #     # with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
        #     #     json.dump(param_dictionary, outfile)
        #
        #     with open(dirPath + 'tot_num_goals.json', 'w') as outfile:
        #         json.dump(param_dictionary, outfile)
        #
        # with open(dirPath + "goals.txt", "wb") as f:
        #     # pickle.dump([env.goals, env.GoalRates, env.steps_to_goals], f)
        #     pickle.dump([env.goals, env.GoalRates], f)

        # # param_keys = ['epsilon', 't_elapsed', 'tot_num_goals']
        # # param_values = [agent.epsilon, t_elapsed, env.number_of_goals]
        # # # param_values = [agent.epsilon]
        #
        # param_keys = ['tot_num_goals']
        # param_values = [env.number_of_goals]
        # # param_values = [agent.epsilon]
        #
        # param_dictionary = dict(zip(param_keys, param_values))
        #
        # # with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
        # #     json.dump(param_dictionary, outfile)
        #
        # with open(dirPath + 'tot_num_goals.json', 'w') as outfile:
        #     json.dump(param_dictionary, outfile)

    print("Max episodes reached")

rospy.on_shutdown(hook)
