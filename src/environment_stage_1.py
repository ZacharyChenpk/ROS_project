#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
import random
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState, ModelStates
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn

WORLD_NAME = 'boxes_1'
#######################################
X_RANGE = {'boxes':5,
            'boxes_1':3,
            'boxes_2':15}
Y_RANGE = {'boxes':5,
            'boxes_1':5,
            'boxes_2':15}
X_CENTER = {'boxes':0.0,
            'boxes_1':-1.5,
            'boxes_2':0.0}
Y_CENTER = {'boxes':0.0,
            'boxes_1':-4,
            'boxes_2':0.0}
INIT_X = {'boxes':-2.0,
            'boxes_1':0.0,
            'boxes_2':0.0}
INIT_Y = {'boxes':-0.5,
            'boxes_1':0.0,
            'boxes_2':0.0}
COLLI = {'boxes':-10,
            'boxes_1':-2,
            'boxes_2':-10}
#######################################
bot_xrange = X_RANGE[WORLD_NAME]
bot_yrange = Y_RANGE[WORLD_NAME]
bot_xcenter = X_CENTER[WORLD_NAME]
bot_ycenter = Y_CENTER[WORLD_NAME]
BOT_INIT_X = INIT_X[WORLD_NAME]
BOT_INIT_Y = INIT_Y[WORLD_NAME]
colli_reward = COLLI[WORLD_NAME]

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.set_pos_proxy = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        self.respawn_goal = Respawn()
        self.colliding = False
        self.spawn_pos = ModelState()
        self.spawn_pos.pose.position.x = BOT_INIT_X
        self.spawn_pos.pose.position.y = BOT_INIT_Y
        self.spawn_pos.model_name = "turtlebot3_burger"

    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        for i in range(10):
            angle = -pi / 4 + heading + (pi / 8 * (i%5)) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            if i > 4:
                tr = tr * -1
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)-0.5
        #reward = 3 - distance_rate

        if done:
            if self.colliding == False:
                self.colliding = True
                rospy.loginfo("Collision!!")
            reward = colli_reward
            #self.pub_cmd_vel.publish(Twist())
        else:
            self.colliding = False

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 750
            self.pub_cmd_vel.publish(Twist())

            #self.get_goalbox = False

        return reward

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size/2 - 1)/2 - (action%5)) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        if action > 4:
            vel_cmd.linear.x = -0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=10)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, action)
        goalbox = False
        if self.get_goalbox:
            goalbox = True
            self.get_goalbox = False

        return np.asarray(state), reward, done, goalbox

    def reset(self, win=False):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")
        if self.initGoal:
            self.respawn_goal.respawnModel()
        else:
            rospy.wait_for_service('gazebo/set_model_state')
            if win:
                if WORLD_NAME != 'boxes_1':
                    new_x = random.random()*bot_xrange - bot_xrange/2 + bot_xcenter
                    new_y = random.random()*bot_yrange - bot_yrange/2 + bot_ycenter
                    flag = self.respawn_goal.judge_goal_collision(new_x, new_y) 
                    while flag:
                        #print('bot gene colli')
                        new_x = random.random()*bot_xrange - bot_xrange/2 + bot_xcenter
                        new_y = random.random()*bot_yrange - bot_yrange/2 + bot_ycenter
                        flag = self.respawn_goal.judge_goal_collision(new_x, new_y)
                    self.spawn_pos.pose.position.x = new_x
                    self.spawn_pos.pose.position.y = new_y
                else:
                    goal_x = -1.3
                    goal_y = -4.3
                    gene_dis = self.respawn_goal.get_gene_dis()
                    flag = True
                    while flag:
                        theta = random.random() * 2 * math.pi
                        new_x = goal_x + gene_dis * math.cos(theta)
                        new_y = goal_y + gene_dis * math.sin(theta)
                        flag = self.respawn_goal.judge_goal_collision(new_x, new_y)
                    self.spawn_pos.pose.position.x = new_x
                    self.spawn_pos.pose.position.y = new_y
            
            #STATE = ModelStates()
            #STATE.pose = [self.spawn_pos.pose]
            #STATE.twist = [self.spawn_pos.twist]
            #STATE.name = ["turtlebot3_burger"]
            try:
                self.set_pos_proxy(self.spawn_pos)
                print('set_model_state service')
            except (rospy.ServiceException) as e:
                print("gazebo/set_model_state service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=10)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(self.spawn_pos)
            self.initGoal = False
        else:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(self.spawn_pos, win, delete=win)

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)