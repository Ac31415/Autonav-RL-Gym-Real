#!/usr/bin/env python


from __future__ import division


import rospy
import numpy as np
import math
import os
import time
import json
from math import pi
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from src.env.testing_respawn_real import Respawn

# from testing_respawn import Respawn
from math import e


from std_msgs.msg import Float32MultiArray

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import actionlib
from numpy import array
from nav_msgs.srv import GetPlan
import tf
from visualization_msgs.msg import Marker



class Env():
    def __init__(self, agent_type, env_module_id=None):
        # create tf listener
        self.listener = tf.TransformListener()

        self.rateHz = 100
        self.rate = rospy.Rate(self.rateHz)

        self.name = ''

        self.plan_service = '/move_base/NavfnROS/make_plan'
        self.global_frame = 'map'
        # self.global_frame = '/map'
        self.robot_frame = 'base_link'
        # self.robot_frame = '/base_link'
        # self.robot_frame = 'base_footprint'
        # self.robot_frame = '/base_footprint'

        # self.listener = tf.TransformListener()
        # self.listener.waitForTransform(
        #     self.global_frame, self.name+'/'+self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        # cond = 0
        # while cond == 0:
        #     try:
        #         rospy.loginfo('Waiting for the robot transform')
        #         (trans, rot) = self.listener.lookupTransform(
        #             self.global_frame, '/'+self.robot_frame, rospy.Time(0))
        #         cond = 1
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         cond == 0
        # self.position = array([trans[0], trans[1]])

        self.Realgoal = MoveBaseGoal()
        self.start = PoseStamped()
        self.end = PoseStamped()

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.marker_pub = rospy.Publisher('/goals', Marker,queue_size=1) # Publish Robot Position to RVIZ

        self.Realgoal.target_pose.header.frame_id = "map"
        self.Realgoal.target_pose.header.stamp = rospy.Time.now()

        rospy.wait_for_service(self.plan_service)

        self.make_plan = rospy.ServiceProxy(
            self.name+self.plan_service, GetPlan)
        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame

        self.agent_type = agent_type

        self.env_module_id = env_module_id  # if is none. then will randomly change between modules.
        self.run_type = sys.argv[1]

        self.envs_list = {}
        self.record_goals = 0
        self.sequential_goals = 0
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        # self.pub_cmd_vel_l = rospy.Publisher('cmd_vel_l', Twist, queue_size=5)
        # self.pub_cmd_vel_r = rospy.Publisher('cmd_vel_r', Twist, queue_size=5)

        # self.action_size = action_size

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.past_obstacle_distance = 0.
        self.ep_number = 0
        self.log_file = ""
        self.goal_hit = 0
        self.step_no = 1





        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        self.result = Float32MultiArray()

        self.goals = []
        self.GoalRates = []
        self.sum_of_goals = sum(self.goals)

        self.number_of_goals = 0

        self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.createLog()

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):

        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position[0], self.goal_y - self.position[1]), 2)
        # print(goal_distance)
        self.past_distance = goal_distance

        return goal_distance

    # def getOdometry(self, odom):
    #     self.position = odom.pose.pose.position
    #     orientation = odom.pose.pose.orientation
    #     orientation_list = [orientation.x,
    #         orientation.y, orientation.z, orientation.w]
    #     _, _, yaw = euler_from_quaternion(orientation_list)
    #
    #     goal_angle = math.atan2(
    #         self.goal_y - self.position.y, self.goal_x - self.position.x)
    #
    #     # print 'yaw', yaw
    #     # print 'gA', goal_angle
    #
    #     heading = goal_angle - yaw
    #     # print 'heading', heading
    #     if heading > pi:
    #         heading -= 2 * pi
    #
    #     elif heading < -pi:
    #         heading += 2 * pi
    #
    #     self.heading = round(heading, 3)

    def getOdometry(self, odom):

        # listen to transform
        # (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        # # print the transform
        # rospy.loginfo('---------')
        # rospy.loginfo('Translation: ' + str(trans))
        # rospy.loginfo('x: ' + str(trans[0]))
        # rospy.loginfo('y: ' + str(trans[1]))
        # rospy.loginfo('z: ' + str(trans[2]))
        # rospy.loginfo('Rotation: ' + str(rot))

        # try:
        #     # listen to transform
        #     (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        #     # print the transform
        #     rospy.loginfo('---------')
        #     rospy.loginfo('Translation: ' + str(trans))
        #     rospy.loginfo('Rotation: ' + str(rot))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
        #
        # # sleep to control the node frequency
        # self.rate.sleep()





        # self.position = odom.pose.pose.position
        # orientation = odom.pose.pose.orientation
        # orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

        self.position = trans
        orientation_list = rot

        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position[1], self.goal_x - self.position[0])

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        # self.heading = round(heading, 2)
        self.heading = round(heading, 3)

    def getState(self, scan, past_action):
        scan_range = []
        heading = self.heading
        # min_range = 0.16
        min_range = 0.13
        done = 0

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])
	    #print(scan_range[i])


        # print("min range: ", min_range)
        # print("min of scan range: ", min(scan_range))


        scan_range_filtered = []

        for item in scan_range:
            if item != 0:
                scan_range_filtered.append(item)

        # print(scan_range_filtered)

        obstacle_min_range = round(min(scan_range_filtered), 2)
        obstacle_angle = np.argmin(scan_range_filtered)

        if min_range > min(scan_range_filtered) > 0:
            done = 1


        # if min_range > min(scan_range) > 0:
        #     done = 1

        for pa in past_action:
            # print(pa)
            scan_range.append(pa)

        # current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        current_distance = round(math.hypot(self.goal_x - self.position[0], self.goal_y - self.position[1]), 2)
        if current_distance < 0.2:
            self.get_goalbox = True
        #print("Heading = " + str(heading))
        # return scan_range + [heading, current_distance], done
        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done

    # # check scan_range size and see if adding x y position of agent into the state would change things
    # def getStateDQN(self, scan):
    #     scan_range = []
    #     heading = self.heading
    #     min_range = 0.13
    #     done = False
    #
    #     for i in range(len(scan.ranges)):
    #         if scan.ranges[i] == float('Inf'):
    #             scan_range.append(3.5)
    #         elif np.isnan(scan.ranges[i]):
    #             scan_range.append(0)
    #         else:
    #             scan_range.append(scan.ranges[i])
    #
    #     obstacle_min_range = round(min(scan_range), 2)
    #     obstacle_angle = np.argmin(scan_range)
    #     if min_range > min(scan_range) > 0:
    #         done = True
    #
    #     current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
    #     if current_distance < 0.2:
    #         self.get_goalbox = True
    #
    #     return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done

    # check scan_range size and see if adding x y position of agent into the state would change things
    def move_away_from_goal(self):
        self.Realgoal.target_pose.pose.position.x = self.goal_x
        self.Realgoal.target_pose.pose.position.y = self.goal_y
        self.Realgoal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.Realgoal)

    def setReward(self, state, done):
        current_distance = state[-1]
        heading = state[-2]
    	#print("dist = " + str(current_distance))
    	#print("heading = " + str(heading))
    	#print("dist = " + str(current_distance))

    	#print("new past  d = " + str(self.past_distance))
    	#print("new curr  d = " + str(current_distance))
        distance_rate = (abs(self.past_distance) - abs(current_distance))

        if(distance_rate > 0.5):
            distance_rate = -1
    	#print(distance_rate)
        if distance_rate > 0:
            reward = 200.*distance_rate
        if distance_rate <= 0:
            reward = -5.

           # reward = 100/(1 + current_distance)
        self.past_distance = current_distance
        if done:
            rospy.loginfo("Collision!!")
            rospy.loginfo("record = " + str(self.record_goals))
            if self.record_goals < self.sequential_goals:
                self.record_goals = self.sequential_goals
            self.sequential_goals = 0
            reward = -1000.
            self.pub_cmd_vel_l.publish(Twist())
            self.pub_cmd_vel_r.publish(Twist())

            self.goals.append(0)
            self.sum_of_goals = sum(self.goals)
            goal_rate = self.sum_of_goals / len(self.goals)
            self.GoalRates.append(goal_rate)

        if self.get_goalbox:
            self.sequential_goals += 1
            rospy.loginfo("Goal!!")
            if self.record_goals < self.sequential_goals:
                self.record_goals = self.sequential_goals
            rospy.loginfo("current = " + str(self.sequential_goals))
            rospy.loginfo("record = " + str(self.record_goals))
            reward = 1000.
            self.pub_cmd_vel_l.publish(Twist())
            self.pub_cmd_vel_r.publish(Twist())

            self.goals.append(1)
            self.sum_of_goals = sum(self.goals)
            goal_rate = self.sum_of_goals / len(self.goals)
            self.GoalRates.append(goal_rate)

            self.number_of_goals += 1
            #self.goal_x, self.goal_y = self.respawn_goal.moduleRespawns()
            #self.goal_distance = self.getGoalDistace()
            #self.get_goalbox = False

    	#print("Reward = " + str(reward))

        return reward

    # def step(self, action, past_action):
    #     self.step_no += 1
    #
    #     # print(self.agent_type)
    #
    #
    #     if self.agent_type == 'dqn' or self.agent_type == 'DQN':
    #         action_size = 5
    #         max_angular_vel = 1.5
    #         ang_vel = ((action_size - 1)/2 - action) * max_angular_vel * 0.5
    #
    #         vel_cmd = Twist()
    #         vel_cmd.linear.x = 0.15
    #         vel_cmd.angular.z = ang_vel
    #         self.pub_cmd_vel.publish(vel_cmd)
    #
    #     else:
    #         wheel_vel_l = action[0]
    #         wheel_vel_r = action[1]
    #
    #         # print("action: " + str(action))
    #
    #         vel_cmd_l = Twist()
    #         vel_cmd_l.linear.x = wheel_vel_l
    #
    #         vel_cmd_r = Twist()
    #         vel_cmd_r.linear.x = wheel_vel_r
    #
    #         self.pub_cmd_vel_l.publish(vel_cmd_l)
    #         self.pub_cmd_vel_r.publish(vel_cmd_r)
    #
    #     data = None
    #     while data is None:
    #         try:
    #             data = rospy.wait_for_message('scan', LaserScan, timeout=5)
    #         except:
    #             pass
    #
    #
    #
    #
    #     if self.agent_type == 'dqn' or self.agent_type == 'DQN':
    #         state, done = self.getStateDQN(data)
    #         reward = self.setReward(state, done)
    #
    #     else:
    #         state, done = self.getState(data, past_action)
    #         reward = self.setReward(state, done)
    #
    #
    #     goal = False
    #     if self.get_goalbox:
    #         done = True
    #         self.get_goalbox = False
    #         goal = True
    #
    #     # print("state: " + str(state))
    #     # print("state as array: " + str(np.asarray(state)))
    #
    #     return np.asarray(state), reward, done, goal

    def setRewardCountGoals(self, state, done):
        # yaw_reward = []
        # current_distance = state[-3]
        # heading = state[-4]
        #
        # for i in range(5):
        #     angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
        #     tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
        #     yaw_reward.append(tr)
        #
        # distance_rate = 2 ** (current_distance / self.goal_distance)
        # reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)

        # current_distance = state[-1]
        # heading = state[-2]

        current_distance = state[-3]
        heading = state[-4]
        obstacle_distance = state[-2]
        obstacle_heading = state[-1]


        distance_rate = (abs(self.past_distance) - abs(current_distance))

        # if(distance_rate > 0.5):
        #     distance_rate = -1
    	# #print(distance_rate)
        # if distance_rate > 0:
        #     reward = 200.*distance_rate
        # if distance_rate <= 0:
        #     reward = -5.

        if (distance_rate > 0.5):
            distance_rate = -1
    	# print(distance_rate)
        if distance_rate > 0:
            reward = 200.*distance_rate*(1 - 4 * math.fabs(0.5 - math.modf(0.5 * (heading + math.pi) % (2 * math.pi) / math.pi)[0]))
            # reward = 400.*distance_rate*(1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * heading % (2 * math.pi) / math.pi)[0]))
            # reward = 200.*distance_rate*np.sin(heading)
        if distance_rate <= 0:
            reward = -10.


        obstacle_distance_rate = (abs(self.past_obstacle_distance) - abs(obstacle_distance))

        if (obstacle_distance_rate > 0.5):
            obstacle_distance_rate = 1
    	# print(distance_rate)
        if obstacle_distance_rate > 0:
            reward = reward - 400.*obstacle_distance_rate*(1 - 4 * math.fabs(0.5 - math.modf(0.5 * (obstacle_heading + math.pi) % (2 * math.pi) / math.pi)[0]))
            # reward = reward - 10.*np.sin(obstacle_heading)
        if obstacle_distance_rate <= 0:
            reward = reward + 5.

        # self.past_distance = current_distance

        print("reward = " + str(reward))
        # print("distance_rate = " + str(distance_rate))
        # print("obstacle_distance_rate = " + str(obstacle_distance_rate))

        # reward = 100/(1 + current_distance)
        self.past_distance = current_distance
        self.past_obstacle_distance = obstacle_distance

        if done and (not self.get_goalbox):
            rospy.loginfo("Collision!!")
            # reward = -150
            # self.pub_cmd_vel.publish(Twist())

            rospy.loginfo("record = " + str(self.record_goals))
            if self.record_goals < self.sequential_goals:
                self.record_goals = self.sequential_goals
            self.sequential_goals = 0
            reward = -1000.
            self.pub_cmd_vel.publish(Twist())

            self.points_delete_on_RVIZ()
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])
            self.goal_distance = self.getGoalDistace()

            self.goals.append(0)
            self.sum_of_goals = sum(self.goals)
            goal_rate = self.sum_of_goals / len(self.goals)
            self.GoalRates.append(goal_rate)

            # self.steps_to_goals.append(0)

        if self.get_goalbox:
            # rospy.loginfo("Goal!!")
            # reward = 200
            # self.pub_cmd_vel.publish(Twist())
            # self.points_delete_on_RVIZ()
            # self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            # self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])
            # self.goal_distance = self.getGoalDistace()

            self.sequential_goals += 1
            rospy.loginfo("Goal!!")
            if self.record_goals < self.sequential_goals:
                self.record_goals = self.sequential_goals
            rospy.loginfo("current = " + str(self.sequential_goals))
            rospy.loginfo("record = " + str(self.record_goals))
            reward = 1000.

            self.pub_cmd_vel.publish(Twist())
            self.points_delete_on_RVIZ()
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])
            self.goal_distance = self.getGoalDistace()

            self.goals.append(1)
            self.sum_of_goals = sum(self.goals)
            goal_rate = self.sum_of_goals / len(self.goals)
            self.GoalRates.append(goal_rate)

            # final_steps = t
            # steps = final_steps - self.init_steps
            # self.steps_to_goal.append(steps)
            # # print(self.steps_to_goal)
            # self.init_steps = t
            # self.number_of_goals += 1
            # # print(self.number_of_goals)

            # self.steps_to_goal = t
            # print(self.steps_to_goals)
            # self.steps_to_goals.append(self.steps_to_goal)
            self.number_of_goals += 1

            # if len(self.steps_to_goals) == 0:
            #     self.average_steps_to_goal = 0
            # else:
            #     sum_of_steps_to_goals = sum(self.steps_to_goals)
            #     self.average_steps_to_goal = math.ceil(sum_of_steps_to_goals / len(self.steps_to_goals))


            # if len(self.GoalRates) == 0:
            #     # if env.get_goalbox:
            #     #     pass
            #     # else:
            #     #     pass
            #
            #     self.steps_goal_result.data = [self.steps_to_goal, self.average_steps_to_goal]
            # else:
            #     self.steps_goal_result.data = [self.steps_to_goal, self.average_steps_to_goal]

            # self.pub_steps_goal_result.publish(self.steps_goal_result)

            self.get_goalbox = False
            done = True

        return reward, done


    def step(self, action, past_action):
        # max_angular_vel = 1.5
        # ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5
        wheel_radius = 33 * 10**(-3)
        bot_width = 160 * 10**(-3)
        self.step_no += 1
        wheel_vel_l = action[0]
        wheel_vel_r = action[1]

        # print(action)

        # omega_l = wheel_vel_l / wheel_radius
        # omega_r = wheel_vel_r / wheel_radius



        # vel_cmd = Twist()
        # vel_cmd.linear.x = 0.15
        # vel_cmd.angular.z = ang_vel
        # self.pub_cmd_vel.publish(vel_cmd)

        vel_cmd = Twist()
        # vel_cmd.linear.x = ((omega_l + omega_r) * wheel_radius) / 2
        vel_cmd.linear.x = (wheel_vel_l + wheel_vel_r) / 2
        vel_cmd.angular.z = (wheel_vel_r - wheel_vel_l) / bot_width
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
                # print("data received!")
                # print("data: ", data)
            except:
                pass

        state, done = self.getState(data, past_action)
        # reward = self.setReward(state, done, action)
        reward, done = self.setRewardCountGoals(state, done)

        return np.asarray(state), reward, done


    # def stepDQN(self, action):
    #     self.step_no += 1
    #
    #
    #     action_size = 5
    #     max_angular_vel = 1.5
    #     ang_vel = ((action_size - 1)/2 - action) * max_angular_vel * 0.5
    #
    #     vel_cmd = Twist()
    #     vel_cmd.linear.x = 0.15
    #     vel_cmd.angular.z = ang_vel
    #     self.pub_cmd_vel.publish(vel_cmd)
    #
    #     data = None
    #     while data is None:
    #         try:
    #             data = rospy.wait_for_message('scan', LaserScan, timeout=5)
    #         except:
    #             pass
    #
    #
    #
    #
    #     state, done = self.getStateDQN(data)
    #     reward = self.setReward(state, done)
    #
    #
    #     goal = False
    #     if self.get_goalbox:
    #         done = True
    #         self.get_goalbox = False
    #         goal = True
    #
    #     # print("state: " + str(state))
    #     # print("state as array: " + str(np.asarray(state)))
    #
    #     return np.asarray(state), reward, done, goal

    def check_scan(self, scan):
        scan_range = []
        heading = self.heading
        # min_range = 0.20
        min_range = 0.17
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)

        scan_range_filtered = []

        for item in scan_range:
            if item != 0:
                scan_range_filtered.append(item)

        return min_range, min(scan_range_filtered)

    def step_real_guide_until_further_from_obstacle(self, action):


        wheel_radius = 33 * 10**(-3)
        bot_width = 160 * 10**(-3)
        self.step_no += 1
        wheel_vel_l = action[0]
        wheel_vel_r = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = (wheel_vel_l + wheel_vel_r) / 2
        vel_cmd.angular.z = (wheel_vel_r - wheel_vel_l) / bot_width
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        min_range, dist_to_obstacle = self.check_scan(data)

        start_time = time.time()
        elapsed_time = time.time() - start_time

        if min_range > dist_to_obstacle > 0:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            self.pub_cmd_vel.publish(vel_cmd)

            self.move_away_from_goal()

            while min_range > dist_to_obstacle > 0 and elapsed_time <= 2:
            # while min_range > dist_to_obstacle > 0:

                elapsed_time = time.time() - start_time
                print("elapsed time: ", elapsed_time)

                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message('scan', LaserScan, timeout=5)
                    except:
                        pass

                min_range, dist_to_obstacle = self.check_scan(data)

                if dist_to_obstacle >= min_range:
                    self.client.cancel_goal()
                    # self.client.cancel_all_goal()
                    self.client.wait_for_result(timeout = rospy.Duration(100))

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0
                    vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    break
                else:
                    # pass
                    self.move_away_from_goal()

                    # if plan_fail_flag:
                    #     break
                    # else:
                    #     pass

                if elapsed_time >= 2:
                    self.client.cancel_goal()
                    # self.client.cancel_all_goal()
                    self.client.wait_for_result(timeout = rospy.Duration(100))

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0
                    vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    wheel_radius = 33 * 10**(-3)
                    bot_width = 160 * 10**(-3)
                    self.step_no += 1
                    wheel_vel_l = action[0]
                    wheel_vel_r = action[1]

                    vel_cmd = Twist()
                    vel_cmd.linear.x = (wheel_vel_l + wheel_vel_r) / 2
                    vel_cmd.angular.z = (wheel_vel_r - wheel_vel_l) / bot_width
                    self.pub_cmd_vel.publish(vel_cmd)

        else:
            pass

        # if min_range > dist_to_obstacle > 0:
        #     vel_cmd = Twist()
        #     vel_cmd.linear.x = 0
        #     vel_cmd.angular.z = 0
        #     self.pub_cmd_vel.publish(vel_cmd)
        #
        #     self.move_away_from_goal()
        #
        #     while min_range > dist_to_obstacle > 0:
        #
        #         data = None
        #         while data is None:
        #             try:
        #                 data = rospy.wait_for_message('scan', LaserScan, timeout=5)
        #             except:
        #                 pass
        #
        #         min_range, dist_to_obstacle = self.check_scan(data)
        #
        #         if dist_to_obstacle >= min_range:
        #             self.client.cancel_goal()
        #             # self.client.cancel_all_goal()
        #             self.client.wait_for_result(timeout = rospy.Duration(100))
        #
        #             vel_cmd = Twist()
        #             vel_cmd.linear.x = 0
        #             vel_cmd.angular.z = 0
        #             self.pub_cmd_vel.publish(vel_cmd)
        #
        #             break
        #         else:
        #             pass
        #
        # else:
        #     pass

        state, done = self.getState(data)
        # reward = self.setReward(state, done, action)

        reward, done = self.setRewardCountGoals(state, done)

        return np.asarray(state), reward, done

    def step_real_step_guidance_from_obstacle(self, action):
        wheel_radius = 33 * 10**(-3)
        bot_width = 160 * 10**(-3)
        self.step_no += 1
        wheel_vel_l = action[0]
        wheel_vel_r = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = (wheel_vel_l + wheel_vel_r) / 2
        vel_cmd.angular.z = (wheel_vel_r - wheel_vel_l) / bot_width
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        min_range, dist_to_obstacle = self.check_scan(data)

        if min_range > dist_to_obstacle > 0:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0
            vel_cmd.angular.z = 0
            self.pub_cmd_vel.publish(vel_cmd)

            self.move_away_from_goal()

            while min_range > dist_to_obstacle > 0:

                data = None
                while data is None:
                    try:
                        data = rospy.wait_for_message('scan', LaserScan, timeout=5)
                    except:
                        pass

                min_range, dist_to_obstacle = self.check_scan(data)

                if dist_to_obstacle >= min_range:
                    self.client.cancel_goal()
                    # self.client.cancel_all_goal()
                    self.client.wait_for_result(timeout = rospy.Duration(100))
                    break
                else:
                    pass

        else:
            pass

        state, done = self.getState(data)
        reward = self.setReward(state, done)

        return np.asarray(state), reward, done



    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            pass
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")
        data = None

        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                print("scan failed")
                pass


        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.moduleRespawns(True)
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.moduleRespawns(self.step_no >= 500 or self.goal_hit)

        if(self.step_no >= 500):
            self.step_no = 1
        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data, [0.,0.])
        self.past_distance = state[-1]
    	#print("resetted")
    	#print("past d = " + str(self.past_distance))

        return np.asarray(state)

    def resetReal(self):

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

            self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.goal_distance = self.getGoalDistace()
        # state, done = self.getState(data)
        state, done = self.getState(data, [0.,0.])

        # self.steps_to_goal = []
        # self.init_steps = 0
        # self.number_of_goals = 0


        self.sendResetGoal(array([0, 0]), data)
        rospy.sleep(0.5)
        self.rate.sleep()

        return np.asarray(state)

    def resetRealDiffGoal(self):

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        self.goal_x, self.goal_y = self.respawn_goal.getPosition()

        self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.goal_distance = self.getGoalDistace()
        # state, done = self.getState(data)
        state, done = self.getState(data, [0.,0.])

        # self.steps_to_goal = []
        # self.init_steps = 0
        # self.number_of_goals = 0


        self.sendResetGoal(array([0, 0]), data)
        rospy.sleep(0.5)
        self.rate.sleep()

        return np.asarray(state)

    def resetRealDiffStartingPoint(self):

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

            self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.goal_distance = self.getGoalDistace()
        # state, done = self.getState(data)
        state, done = self.getState(data, [0.,0.])

        # self.steps_to_goal = []
        # self.init_steps = 0
        # self.number_of_goals = 0


        # self.sendResetGoal(array([0, 0]), data)
        # rospy.sleep(0.5)
        # self.rate.sleep()

        return np.asarray(state)

    def resetRealResume(self):

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        # if self.initGoal:
        #     # self.goal_x, self.goal_y = self.respawn_goal.getPosition()
        #     self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=False)
        #     self.initGoal = False
        #
        #     self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)

        self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data, [0.,0.])

        # self.steps_to_goal = []
        # self.init_steps = 0
        # self.number_of_goals = 0


        self.sendResetGoal(array([0, 0]), data)
        rospy.sleep(0.5)
        self.rate.sleep()

        return np.asarray(state)

    def resetRealDirect(self):

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

            self.points_respawn_on_RVIZ([self.goal_x, self.goal_y])

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data, [0.,0.])

        self.sendResetGoalDirect(array([0, 0]))
        rospy.sleep(0.5)
        self.rate.sleep()

        return np.asarray(state)


    def sendResetGoal(self, point, scan):

        self.action_size = 5

        if scan == None:
            pass
        else:

            scan_range = []
            heading = self.heading
            # min_range = 0.20
            min_range = 0.17
            # done = False

            for i in range(len(scan.ranges)):
                if scan.ranges[i] == float('Inf'):
                    scan_range.append(3.5)
                elif np.isnan(scan.ranges[i]):
                    scan_range.append(0)
                else:
                    scan_range.append(scan.ranges[i])

            if scan_range[0] <= min_range: # the front of robot
                max_angular_vel = 1.5
                ang_vel = ((self.action_size - 1)/2 - 2) * max_angular_vel * 0.5

                start_time = time.time()
                elapsed_time = time.time() - start_time

                while elapsed_time <= 2:

                    vel_cmd = Twist()
                    vel_cmd.linear.x = -0.15
                    vel_cmd.angular.z = ang_vel
                    # vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    elapsed_time = time.time() - start_time

            elif scan_range[89] <= min_range:
                max_angular_vel = 1.5
                ang_vel = ((self.action_size - 1)/2 - 1) * max_angular_vel * 0.5

                start_time = time.time()
                elapsed_time = time.time() - start_time

                while elapsed_time <= 1:

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0.15
                    vel_cmd.angular.z = ang_vel
                    # vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    elapsed_time = time.time() - start_time

                while elapsed_time <= 1:

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0.15
                    vel_cmd.angular.z = 0
                    # vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    elapsed_time = time.time() - start_time

            elif scan_range[179] <= min_range:
                max_angular_vel = 1.5
                ang_vel = ((self.action_size - 1)/2 - 2) * max_angular_vel * 0.5

                start_time = time.time()
                elapsed_time = time.time() - start_time

                while elapsed_time <= 2:

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0.15
                    vel_cmd.angular.z = ang_vel
                    # vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    elapsed_time = time.time() - start_time

            elif scan_range[269] <= min_range:
                max_angular_vel = 1.5
                ang_vel = ((self.action_size - 1)/2 - 3) * max_angular_vel * 0.5

                start_time = time.time()
                elapsed_time = time.time() - start_time

                while elapsed_time <= 1:

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0.15
                    vel_cmd.angular.z = ang_vel
                    # vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    elapsed_time = time.time() - start_time

                while elapsed_time <= 1:

                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0.15
                    vel_cmd.angular.z = 0
                    # vel_cmd.angular.z = 0
                    self.pub_cmd_vel.publish(vel_cmd)

                    elapsed_time = time.time() - start_time

            # elif scan_range[359] <= 0.20:
            #     scan_range.append(0)
            else:
                pass



        # for i in range(5):
        #     max_angular_vel = 1.5
        #     ang_vel = ((self.action_size - 1)/2 - i) * max_angular_vel * 0.5
        #
        #     start_time = time.time()
        #     elapsed_time = time.time() - start_time
        #
        #     while elapsed_time <= 2:
        #
        #         vel_cmd = Twist()
        #         vel_cmd.linear.x = 0.15
        #         vel_cmd.angular.z = ang_vel
        #         # vel_cmd.angular.z = 0
        #         self.pub_cmd_vel.publish(vel_cmd)
        #
        #         elapsed_time = time.time() - start_time
        #
        #     start_time = time.time()
        #     elapsed_time = time.time() - start_time
        #
        #     while elapsed_time <= 2:
        #
        #         vel_cmd = Twist()
        #         vel_cmd.linear.x = -0.15
        #         vel_cmd.angular.z = ang_vel
        #         # vel_cmd.angular.z = 0
        #         self.pub_cmd_vel.publish(vel_cmd)
        #
        #         elapsed_time = time.time() - start_time

        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0
        self.pub_cmd_vel.publish(vel_cmd)

        self.Realgoal.target_pose.pose.position.x = point[0]
        self.Realgoal.target_pose.pose.position.y = point[1]
        self.Realgoal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.Realgoal)
        self.client.wait_for_result(timeout = rospy.Duration(1000))


    def sendResetGoalDirect(self, point):

        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0
        self.pub_cmd_vel.publish(vel_cmd)

        self.Realgoal.target_pose.pose.position.x = point[0]
        self.Realgoal.target_pose.pose.position.y = point[1]
        self.Realgoal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(self.Realgoal)
        self.client.wait_for_result(timeout = rospy.Duration(1000))


    def points_respawn_on_RVIZ(self, point):
        marker_data = Marker()
        marker_data.type = marker_data.POINTS
        marker_data.action = marker_data.ADD
        marker_data.header.frame_id = 'map'

        # marker_data.scale.x = 0.1 # width
        # marker_data.scale.y = 0.1 # Height

        marker_data.scale.x = 0.3 # width
        marker_data.scale.y = 0.3 # Height

        # marker_data.color.a = 1
        # marker_data.color.r = 1
        # marker_data.color.g = 0
        # marker_data.color.b = 0

        marker_data.color.r = 255.0/255.0
        marker_data.color.g = 0.0/255.0
        marker_data.color.b = 0.0/255.0
        marker_data.color.a=0.3

        marker_data.points.append(Point(point[0],point[1],0))
        self.marker_pub.publish(marker_data)


    def points_delete_on_RVIZ(self):
        marker_data = Marker()
        marker_data.action = marker_data.DELETEALL
        marker_data.header.frame_id = 'map'
        self.marker_pub.publish(marker_data)





    def logEpisode(self, reward, collision_count, goal_count, step_count):
        self.ep_number = self.ep_number + 1
        log = {
            "ep_number": self.ep_number,
            "environment": self.respawn_goal.currentModuleName(),
            "reward_for_ep": reward,
            "steps": step_count,
            "collision_count": collision_count,
            "goal_count": goal_count
        }
        logfile = open(self.log_file, "a")
        logfile.write(json.dumps(log) + "\n")
        logfile.close

        # Display data live and save them to csv files
    def DispEpisodeCSV(self, reward, collision_count, goal_count, step_count, GoalRates, num_goals):
        self.ep_number = self.ep_number + 1
        log = {
            "ep_number": self.ep_number,
            "environment": self.respawn_goal.currentModuleName(),
            "reward_for_ep": reward,
            "steps": step_count,
            "collision_count": collision_count,
            "goal_count": goal_count
        }

        # if len(GoalRates) == 0:
        #     self.result.data = [reward, self.respawn_goal.currentModuleIndex(), self.current_time, float("%.4f" % 0), num_goals]
        # else:
        #     self.result.data = [reward, self.respawn_goal.currentModuleIndex(), self.current_time, float("%.4f" % GoalRates[-1]), num_goals]

        if len(GoalRates) == 0:
            self.result.data = [reward, self.current_time, float("%.4f" % 0), num_goals]
        else:
            self.result.data = [reward, self.current_time, float("%.4f" % GoalRates[-1]), num_goals]


        self.pub_result.publish(self.result)




    def createLog(self):

        # dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/env/{}ing_logs/{}/env-{}'.format(sys.argv[1], self.agent_type, self.env_module_id)

        dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/env/{}ing_logs/{}/'.format(sys.argv[1], self.agent_type)

        logpath = os.path.dirname(os.path.realpath(__file__)) + dirPath

        self.current_time = int(time.time())

        self.log_file = logpath + "/" + self.agent_type + "-" + str(self.current_time) + ".txt"


        try:
            os.makedirs(logpath)
        except:
            pass

        logfile = open(self.log_file, "a")
        logfile.close


    def DispEpisodeCSV_real(self, reward, step_count, GoalRates, num_goals):
        self.ep_number = self.ep_number + 1
        log = {
            "ep_number": self.ep_number,
            "reward_for_ep": reward,
            "steps": step_count
        }

        # if len(GoalRates) == 0:
        #     self.result.data = [reward, self.respawn_goal.currentModuleIndex(), self.current_time, float("%.4f" % 0), num_goals]
        # else:
        #     self.result.data = [reward, self.respawn_goal.currentModuleIndex(), self.current_time, float("%.4f" % GoalRates[-1]), num_goals]

        if len(GoalRates) == 0:
            self.result.data = [reward, self.current_time, float("%.4f" % 0), num_goals]
        else:
            self.result.data = [reward, self.current_time, float("%.4f" % GoalRates[-1]), num_goals]


        self.pub_result.publish(self.result)


    def createLog_real(self):

        # dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/env/{}ing_logs/{}/env-{}'.format(sys.argv[1], self.agent_type, self.env_module_id)

        dirPath = '/home/wen-chung/catkin_noetic_ws/src/Autonav-RL-Gym/src/env/{}ing_logs/{}/'.format(sys.argv[1], self.agent_type)

        logpath = os.path.dirname(os.path.realpath(__file__)) + dirPath

        self.current_time = int(time.time())

        self.log_file = logpath + "/" + self.agent_type + "-" + str(self.current_time) + ".txt"


        try:
            os.makedirs(logpath)
        except:
            pass

        logfile = open(self.log_file, "a")
        logfile.close
