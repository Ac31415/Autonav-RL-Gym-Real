#!/usr/bin/env python

import rospy
import random
import time
import os
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
# from testing_respawn_coords import *

import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from src.env.testing_respawn_coords import *

from tf.transformations import quaternion_from_euler


class Respawn():
    def __init__(self):

        # Tracking current module being trained in
        self.modules = [
            module_empty(),
            module_left_right(),
            module_move_away(),
            module_round_obstacle(),
            module_static_obstacles(),
            module_moving_obstacles(),
            module_gate()
            ]
        self.module_index = -1
        self.pub_module = rospy.Publisher('current_module', String, queue_size = 1)

        self.stage = rospy.get_param('/stage_number')

        # Get goal box model
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('Autonav-RL-Gym-Real/src/env',
                                                'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        # self.modelPath = '/home/act65/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf'
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()

        # Create initial goal pose
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'

        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        # self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0

    def getPosition(self, position_check=False, delete=False):
        if delete:
            pass
            # self.deleteModel()

        if self.stage != 4:
            while position_check:
                # goal_x = random.randrange(-12, 13) / 10.0
                # goal_y = random.randrange(-12, 13) / 10.0

                goal_x = random.randrange(-10, 10) / 10.0
                goal_y = random.randrange(-10, 10) / 10.0

                # if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
                #     position_check = True
                # elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                #     position_check = True
                # elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                #     position_check = True
                # elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                #     position_check = True
                # elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                #     position_check = True
                #
                # elif abs(goal_x - 1.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                #     position_check = True
                #
                # else:
                #     position_check = False




                if abs(goal_x - 1.0) <= 0.5 and abs(goal_y - 0.0) <= 0.5:
                    position_check = True

                elif abs(goal_x - 0.5) <= 0.5 and abs(goal_y - (0.5)) <= 0.5:
                    position_check = True

                elif abs(goal_x - (-0.5)) <= 0.5 and abs(goal_y - (-0.5)) <= 0.5:
                    position_check = True

                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

        else:
            while position_check:
                goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
                goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]

                self.index = random.randrange(0, 13)
                print(self.index, self.last_index)
                if self.last_index == self.index:
                    position_check = True
                else:
                    self.last_index = self.index
                    position_check = False

                self.goal_position.position.x = goal_x_list[self.index]
                self.goal_position.position.y = goal_y_list[self.index]

        time.sleep(0.5)
        # self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
