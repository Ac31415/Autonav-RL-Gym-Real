#!/usr/bin/env python


import rospy
import os
import numpy as np
import gc
import time
import sys
from ppo.storage import Memory
from std_msgs.msg import Float32
from env.testing_environment_real import Env
from ppo.ppo_models import PPO

import torch

# hyperparams
update_timestep = 500      # update policy every n timesteps
hidden_dim = 256            # constant std for action distribution (Multivariate Normal)
K_epochs = 50              # update policy for K epochs
eps_clip = 0.2              # clip parameter for PPO
gamma = 0.99                # discount factor

lr = 2e-4                # parameters for Adam optimizer
# lr = (2e-4) * 0.1                # parameters for Adam optimizer
betas = (0.9, 0.999)

random_seed = None

# state params
# state_dim = 28
state_dim = 366
action_dim = 4
ACTION_V_MIN = 0  # m/s
ACTION_V_MAX = 0.4  # m/s

class PPO_agent:

    def __init__(self, load_ep, env, max_timesteps, dirPath):
        self.env = env
        self.time_step = 0
        self.past_action = np.array([0., 0.])
        self.max_timesteps = max_timesteps

        self.memory = Memory()
        self.ppo = PPO(
                state_dim, action_dim, hidden_dim, lr, betas, gamma, K_epochs,
                ACTION_V_MIN, ACTION_V_MAX, eps_clip, dirPath
                )

        if (load_ep > 0):
            # self.ppo.load_models(load_ep)

            print(load_ep)

            # self.ppo.load_models_latest(load_ep)
            self.ppo.load_models_latest_combine(load_ep)


    # called every step
    def step(self, state, ep):

        self.time_step += 1
        action = self.ppo.select_action(state, self.memory)
        # print("action: ", action)
        # print("past action: ", self.past_action)
        # state, reward, collision, goal = self.env.step(action, self.past_action)
        next_state, reward, done = self.env.step(action, self.past_action)
        # print("state: ", next_state)
        # next_state, reward, done = self.env.step_real_guide_until_further_from_obstacle(action)
        # next_state, reward, done = self.env.step(action)

        self.past_action = action
        self.memory.rewards.append(reward)
        # self.memory.masks.append(float(collision or self.time_step == self.max_timesteps - 1))
        self.memory.masks.append(float(self.time_step == self.max_timesteps - 1))

        if (self.time_step % update_timestep == 0):
            self.ppo.update(self.memory)
            self.memory.clear_memory()
            self.time_step = 0

        # return state, reward, collision, goal
        # return state, reward, done
        return next_state, reward, done

    def save(self, ep):
        # self.ppo.save_models(ep)

        # self.ppo.save_models_latest(ep)
        self.ppo.save_models_latest_combine(ep)
