#!/usr/bin/env python


import rospy
import os
import numpy as np
import gc
import time
import sys
from dqn.replay_buffer import ReplayBuffer
from std_msgs.msg import Float32
from env.training_environment import Env
from dqn.dqn_models import DQN

import torch

# # hyperparams
# update_timestep = 2000      # update policy every n timesteps
# K_epochs = 50              # update policy for K epochs
# eps = 1              # clip parameter for PPO
# gamma = 0.99                # discount factor
#
# lr = 0.00025                # parameters for Adam optimizer
# betas = (0.9, 0.999)
#
# random_seed = None
#
# # state params
# state_dim = 364
# action_dim = 5
# ACTION_V_MIN = 0  # m/s
# ACTION_V_MAX = 0.4  # m/s
#
# replay_buffer_size = 1000000
# history_len = 4
#
# # hyper-parameters
# BATCH_SIZE = 64
# LR = 0.00025
# EPSILO_DECAY = 0.99
# EPSILO_MIN = 0.05
# MEMORY_CAPACITY = 1000000
# ENV_A_SHAPE = 0 if isinstance(env.action_space.sample(), int) else env.action_space.sample.shape

class DQN_agent:

    def __init__(self, load_ep, env, max_timesteps, dirPath):
        self.env = env
        self.time_step = 0
        self.past_action = np.array([0., 0.])
        self.max_timesteps = max_timesteps

        # hyperparams
        self.update_timestep = 2000      # update policy every n timesteps
        self.K_epochs = 50              # update policy for K epochs
        self.eps = 1              # clip parameter for PPO
        self.gamma = 0.99                # discount factor

        self.lr = 0.00025                # parameters for Adam optimizer
        self.betas = (0.9, 0.999)

        self.random_seed = None

        # state params
        # self.state_dim = 364
        self.state_dim = 28
        # self.action_dim = 5
        self.action_dim = 4
        self.ACTION_V_MIN = 0  # m/s
        self.ACTION_V_MAX = 0.4  # m/s

        self.actions = [[0,0], [0,self.ACTION_V_MAX], [self.ACTION_V_MAX, 0], [self.ACTION_V_MAX, self.ACTION_V_MAX]]
        self.past_action = np.array([0., 0.])

        self.replay_buffer_size = 1000000
        self.history_len = 4

        # hyper-parameters
        self.BATCH_SIZE = 64
        self.EPSILO_DECAY = 0.99
        self.EPSILO_MIN = 0.05
        self.MEMORY_CAPACITY = 1000000
        # self.ENV_A_SHAPE = 0 if isinstance(env.action_space.sample(), int) else env.action_space.sample.shape

        self.memory = ReplayBuffer(self.MEMORY_CAPACITY, self.state_dim)
        self.dqn = DQN(
                self.state_dim, self.action_dim, self.lr, self.betas, self.gamma, self.eps, dirPath,
                self.MEMORY_CAPACITY, self.BATCH_SIZE
                )

        if (load_ep > 0):
            # self.ppo.load_models(load_ep)

            self.dqn.load_models_latest(load_ep)


    # called every step
    def step(self, state, ep):

        self.time_step += 1
        action = self.dqn.select_action(state)


        print(action)


        # next_state, reward, collision, goal = self.env.stepDQN(action)

        next_state, reward, collision, goal = self.env.step(self.actions[action], self.past_action)
        self.past_action = self.actions[action]

        self.memory.store_transition(state, action, reward, next_state, collision, goal)

        if self.memory.memory_counter >= self.BATCH_SIZE:
            if (self.time_step % self.update_timestep == 0):
                self.dqn.update_target()
                # self.memory.clear_memory()
                # self.time_step = 0
            self.dqn.update(self.memory.memory)

        return state, reward, collision, goal

    # called every step
    def decay_epsilon(self):

        if self.eps > self.EPSILO_MIN:
            self.eps *= self.EPSILO_DECAY

    def save(self, ep):
        # self.ppo.save_models(ep)

        self.dqn.save_models_latest(ep)
