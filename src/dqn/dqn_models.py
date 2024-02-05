import torch
import torch.nn as nn
from torch.distributions import MultivariateNormal
import numpy as np
from torch.distributions import Categorical
import sys
import os

import torch.optim as optim


device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

dropout = 0.2

class Net(nn.Module):
    def __init__(self, state_size, action_size, learning_rate, dropout):
        super(Net, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.learning_rate = learning_rate
        self.dropout = dropout

        self.fc1 = nn.Linear(self.state_size, 64)
        self.fc2 = nn.Linear(64, 64)
        self.dropout_layer = nn.Dropout(self.dropout)
        self.fc3 = nn.Linear(64, self.action_size)

    def forward(self, x):
        x = nn.functional.relu(self.fc1(x))
        x = nn.functional.relu(self.fc2(x))
        x = self.dropout_layer(x)
        x = self.fc3(x)
        return x




class DQN:
    def __init__(self, state_dim, action_dim, lr, betas, gamma, eps, savePath, memory_capacity, batch_size):
        super(DQN, self).__init__()
        self.eval_net, self.target_net = Net(state_dim, action_dim, lr, dropout), Net(state_dim, action_dim, lr, dropout)

        print(self.eval_net)
        print(self.target_net)

        self.lr = lr
        self.betas = betas
        self.gamma = gamma
        self.eps = eps
        self.savePath = savePath


        self.action_dim = action_dim
        self.state_dim = state_dim

        self.loss_func = nn.MSELoss()
        self.optimizer = optim.RMSprop(self.eval_net.parameters(), lr=self.lr, alpha=0.9, eps=1e-06)

        self.memory_capacity = memory_capacity
        self.batch_size = batch_size

    def select_action(self, state):
        # if np.random.rand() <= self.eps:
        #     self.q_value = np.zeros(self.action_dim)
        #     return random.randrange(self.action_dim)
        # else:
        #     q_value = self.model.predict(state.reshape(1, len(state)))
        #     self.q_value = q_value
        #     return np.argmax(q_value[0])


        state = torch.unsqueeze(torch.FloatTensor(state), 0) # get a 1D array
        if np.random.randn() <= self.eps:# greedy policy
            action_value = self.eval_net.forward(state)
            action = torch.max(action_value, 1)[1].data.numpy()
            action = action[0]
        else: # random policy
            action = np.random.randint(0,self.action_dim)
            action = action
        return action

    def save_models(self, episode_count):
        # if not os.path.exists(os.path.join(self.savePath, '/ppo/')):
        #     os.mkdir(os.path.join(self.savePath, '/ppo/'))
        torch.save(self.eval_net.state_dict(), os.path.join(self.savePath, str(episode_count) + '_DQN_Agent.pth'))

    def load_models(self, episode):
        self.eval_net.load_state_dict(torch.load(os.path.join(self.savePath, str(episode_count) + '_DQN_Agent.pth')))



    def save_models_latest(self, episode_count):
        # if not os.path.exists(os.path.join(self.savePath, '/ppo/')):
        #     os.mkdir(os.path.join(self.savePath, '/ppo/'))
        torch.save(self.eval_net.state_dict(), os.path.join(self.savePath, 'DQN_Agent.pth'))

    def load_models_latest(self, episode):
        self.eval_net.load_state_dict(torch.load(os.path.join(self.savePath, 'DQN_Agent.pth')))


    def update_target(self):
        self.target_net.load_state_dict(self.eval_net.state_dict())


    def update(self, memory):
        #sample batch from memory
        sample_index = np.random.choice(self.memory_capacity, self.batch_size)

        # print(memory)

        batch_memory = memory[sample_index, :]
        batch_state = torch.FloatTensor(batch_memory[:, :self.state_dim])
        batch_action = torch.LongTensor(batch_memory[:, self.state_dim:self.state_dim+1].astype(int))
        batch_reward = torch.FloatTensor(batch_memory[:, self.state_dim+1:self.state_dim+2])
        batch_next_state = torch.FloatTensor(batch_memory[:,-self.state_dim:])

        #q_eval
        q_eval = self.eval_net(batch_state).gather(1, batch_action)
        q_next = self.target_net(batch_next_state).detach()
        q_target = batch_reward + self.gamma * q_next.max(1)[0].view(self.batch_size, 1)
        loss = self.loss_func(q_eval, q_target)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
