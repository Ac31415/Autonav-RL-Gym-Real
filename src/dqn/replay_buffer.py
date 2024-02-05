'''
https://github.com/berkeleydeeprlcourse/homework/blob/master/hw3/dqn_utils.py
modified from (batch, h, w, ch) to (batch, ch, h, w)
'''

import numpy as np
import random

class ReplayBuffer(object):
    def __init__(self, size, state_size):
        self.memory_counter = 0
        self.memory = np.zeros((size, state_size * 2 + 4))
        self.memory_capacity = size

    def store_transition(self, state, action, reward, next_state, goal, collision):
        transition = np.hstack((state, [action, reward], next_state, goal, collision))
        index = self.memory_counter % self.memory_capacity
        self.memory[index, :] = transition
        self.memory_counter += 1
