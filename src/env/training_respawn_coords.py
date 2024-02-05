#!/usr/bin/env python

import random
import math

import rospy

from std_srvs.srv import Empty


unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)

class module_empty():
    def __init__(self):
        self.name = "empty"
        self.model_x = -2
        self.model_y = 5
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        self.bot_x = 0
        self.bot_y = 0

        # print("goal pos = " + "x: " + str(self.goal_x) + " y: " + str(self.goal_y))
        # print("robot pos = " + "x: " + str(self.bot_x) + " y: " + str(self.bot_y))

        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        too_close = True

        # pause_proxy()

        while too_close:
            self.goal_x = random.uniform(-0.8, 0.8)
            self.goal_y = random.uniform(-0.8, 0.8)
            if (math.sqrt((self.goal_x - self.bot_x)**2 + (self.goal_y - self.bot_y)**2) >= 0.3):
                too_close = False

        # unpause_proxy()

        # print("goal pos = " + "x: " + str(self.goal_x) + " y: " + str(self.goal_y))

        return self.goal_x + self.model_x, self.goal_y + self.model_y

class module_move_away():
    def __init__(self):
        self.name = "move_away"
        self.model_x = 2
        self.model_y = 5
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        self.bot_x = random.uniform(-0.35, 0.35)
        self.bot_y = random.uniform(-0.5, -0.7)
        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        self.goal_x = random.uniform(-1.5, 1.5)
        self.goal_y = random.uniform(0.2, 1.5)
        return self.goal_x + self.model_x, self.goal_y + self.model_y


class module_left_right():
    def __init__(self):
        self.name = "left_right"
        self.model_x = -2
        self.model_y = 1
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        self.bot_x = 0
        self.bot_y = random.uniform(-1.5, 0)
        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        coefx = 1
        coefy = 1
        if random.random() < 0.5:
            coefx = -1
        if random.random() < 0.5:
            coefy = -1
        self.goal_x = random.uniform(1, 1.5) * coefx
        self.goal_y = 1.45 * coefy + 0.05 * coefy
        return self.goal_x + self.model_x, self.goal_y + self.model_y

class module_round_obstacle():
    def __init__(self):
        self.name = "round_obstacle"
        self.model_x = 2
        self.model_y = 1
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        self.bot_x = random.uniform(-1.4, 1.4)
        self.bot_y = -1.4
        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        self.goal_x = random.uniform(-1.4, 1.4)
        self.goal_y = 1.4
        return self.goal_x + self.model_x, self.goal_y + self.model_y

class module_static_obstacles():
    def __init__(self):
        self.name = "static_obstacles"
        self.model_x = -2
        self.model_y = -3
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        if random.random() > 0.5:
            self.bot_x = random.uniform(-1.5,1.5)
            self.bot_y = -1.55
        else:
            self.bot_x = random.uniform(-0.3,0.7)
            self.bot_y = -0.2

        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        self.goal_x = random.uniform(-1.5, 1.5)
        self.goal_y = 1.5
        return self.goal_x + self.model_x, self.goal_y + self.model_y

class module_moving_obstacles():
    def __init__(self):
        self.name = "moving_obstacles"
        self.model_x = 2
        self.model_y = -3
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        if random.random() > 0.5:
            self.bot_x = 0
            self.bot_y = 0
        else:
            self.bot_x = random.uniform(-1.5,1.5)
            self.bot_y = -1.5

        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        self.goal_x = random.uniform(-1.5, 1.5)
        self.goal_y = 1.5
        return self.goal_x + self.model_x, self.goal_y + self.model_y

class module_gate():
    def __init__(self):
        self.name = "gate"
        self.model_x = -2
        self.model_y = -7
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

    def genBotPos(self):
        self.bot_x = random.uniform(-1.5,1.5)
        self.bot_y = random.uniform(-1.4,-0.5)
        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        self.goal_x = random.uniform(-1.5,1.5)
        self.goal_y = random.uniform(0.5,1.4)
        return self.goal_x + self.model_x, self.goal_y + self.model_y




class module_sim_to_real():
    def __init__(self):
        self.name = "sim_to_real"
        self.model_x = 0
        self.model_y = 0
        self.goal_x = 0
        self.goal_y = 0
        self.bot_x = 0
        self.bot_y = 0

        self.obs_0_x = -2.73
        self.obs_0_y = 1.30
        self.obs_1_x = -2.69
        self.obs_1_y = 1.08
        self.obs_2_x = 2.69
        self.obs_2_y = 0.82
        self.obs_3_x = -1.59
        self.obs_3_y = 1.18
        self.obs_4_x = 1.50
        self.obs_4_y = 0.90
        self.obs_5_x = -1.33
        self.obs_5_y = 1.20
        self.obs_6_x = -0.83
        self.obs_6_y = -0.34
        self.obs_7_x = -0.79
        self.obs_7_y = -0.73
        self.obs_8_x = 0.02
        self.obs_8_y = 0.55
        self.obs_9_x = -0.04
        self.obs_9_y = 0.97
        self.obs_10_x = 1.87
        self.obs_10_y = 1.30
        self.obs_11_x = 2.09
        self.obs_11_y = 0.61
        self.obs_12_x = 2.07
        self.obs_12_y = 0.01
        self.obs_13_x = 1.69
        self.obs_13_y = 0.01
        self.obs_14_x = 2.01
        self.obs_14_y = -1.05
        self.obs_15_x = 1.78
        self.obs_15_y = -1.04

    def genBotPos(self):
        self.bot_x = 0
        self.bot_y = 0

        # print("goal pos = " + "x: " + str(self.goal_x) + " y: " + str(self.goal_y))
        # print("robot pos = " + "x: " + str(self.bot_x) + " y: " + str(self.bot_y))

        return self.bot_x + self.model_x, self.bot_y + self.model_y

    def genGoalPos(self):
        too_close = True
        in_obstacle = True

        # pause_proxy()

        while too_close or in_obstacle:
            self.goal_x = random.uniform(-2.5, 2.1)
            self.goal_y = random.uniform(-1.1, 1.28)
            if (math.sqrt((self.goal_x - self.bot_x)**2 + (self.goal_y - self.bot_y)**2) >= 0.8):
                too_close = False
            if (math.sqrt((self.goal_x - self.obs_0_x)**2 + (self.goal_y - self.obs_0_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_1_x)**2 + (self.goal_y - self.obs_1_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_2_x)**2 + (self.goal_y - self.obs_2_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_3_x)**2 + (self.goal_y - self.obs_3_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_4_x)**2 + (self.goal_y - self.obs_4_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_5_x)**2 + (self.goal_y - self.obs_5_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_6_x)**2 + (self.goal_y - self.obs_6_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_7_x)**2 + (self.goal_y - self.obs_7_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_8_x)**2 + (self.goal_y - self.obs_8_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_9_x)**2 + (self.goal_y - self.obs_9_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_10_x)**2 + (self.goal_y - self.obs_10_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_11_x)**2 + (self.goal_y - self.obs_11_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_12_x)**2 + (self.goal_y - self.obs_12_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_13_x)**2 + (self.goal_y - self.obs_13_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_14_x)**2 + (self.goal_y - self.obs_14_y)**2) >= 0.5) and \
            (math.sqrt((self.goal_x - self.obs_15_x)**2 + (self.goal_y - self.obs_15_y)**2) >= 0.5):
                in_obstacle = False

        # unpause_proxy()

        # print("goal pos = " + "x: " + str(self.goal_x) + " y: " + str(self.goal_y))

        return self.goal_x + self.model_x, self.goal_y + self.model_y
