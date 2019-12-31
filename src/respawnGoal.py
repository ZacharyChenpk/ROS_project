#!/usr/bin/env python

# We have read the source code in Turtlebot3_Machine_Learning for reference #
# Authors: Chen Zhibin, Gilbert #

import rospy
import random
import time
import os
import math
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

WORLD_NAME = 'boxes_3'  #boxes, boxes_1, boxes_2
#######################################
INIT_X = {'boxes':-2.0,
            'boxes_1':0.0,
            'boxes_2':0.0,
            'boxes_3':0.0}
INIT_Y = {'boxes':-0.5,
            'boxes_1':0.0,
            'boxes_2':0.0,
            'boxes_3':0.0}
START_DIS = {'boxes':0.25,
            'boxes_1':2.4,
            'boxes_2':0.26,
            'boxes_3':1}
LI_EPOCH = {'boxes':40,
            'boxes_1':40,
            'boxes_2':60,
            'boxes_3':90}
LI_RATE = {'boxes':0.1,
            'boxes_1':0.09,
            'boxes_2':0.129,
            'boxes_3':0.1}
#######################################
BOT_INIT_X = INIT_X[WORLD_NAME]
BOT_INIT_Y = INIT_Y[WORLD_NAME]
BOT_START_DIS = START_DIS[WORLD_NAME]
BOT_LI_EPOCH = LI_EPOCH[WORLD_NAME]
BOT_LI_RATE = LI_RATE[WORLD_NAME]

class Respawn():
    def __init__(self, prev_succ=0):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.obsPath = self.modelPath.replace('proj_api/src',
                                            'proj_api/worlds/'+WORLD_NAME+'.obs')
        self.modelPath = self.modelPath.replace('proj_api/src',
                                                'proj_api/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.goal_position = Pose()
        self.init_goal_x = BOT_INIT_X + BOT_START_DIS
        if WORLD_NAME == 'boxes_1' or WORLD_NAME == 'boxes_3':
            self.init_goal_x = BOT_INIT_X + 0.23
        self.init_goal_y = BOT_INIT_Y
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.obsdata = np.loadtxt(self.obsPath)
        self.obs_n = self.obsdata.shape[0]/2
        self.obs_size = self.obsdata[0:self.obs_n]
        self.obs_center = self.obsdata[self.obs_n:]
        self.success_time = prev_succ
        print(self.obs_size)
        print(self.obs_center)

    def get_gene_dis(self):
        return BOT_START_DIS + min(BOT_LI_EPOCH, self.success_time) * BOT_LI_RATE

    def judge_goal_collision(self, goal_x, goal_y):
        for i in range(self.obs_n):
            if abs(goal_x - self.obs_center[i][0]) <= self.obs_size[i][0]/2 + 0.2 and abs(goal_y - self.obs_center[i][1]) <= self.obs_size[i][1]/2 + 0.2:
                print('colli '+str(i))
                return True

        return False

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

    def getPosition(self, spawn_pos, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        gene_dis = self.get_gene_dis()
        if position_check:
            self.success_time += 1
            print('success +1')

        if WORLD_NAME != 'boxes_1':
            while position_check:
                theta = random.random() * 2 * math.pi
                goal_x = spawn_pos.pose.position.x + gene_dis * math.cos(theta)
                goal_y = spawn_pos.pose.position.y + gene_dis * math.sin(theta)

                if WORLD_NAME == 'boxes_3':
                    if spawn_pos.pose.position.x**2 + spawn_pos.pose.position.y**2 <= gene_dis**2 and random.random() > 0.5:
                        goal_x = random.random() - 0.5
                        goal_y = random.random() - 0.5
                    else:
                        theta = random.random() * 2 * math.pi
                        goal_x2 = spawn_pos.pose.position.x + gene_dis * math.cos(theta)
                        goal_y2 = spawn_pos.pose.position.y + gene_dis * math.sin(theta)
                        if goal_x**2+goal_y**2 > goal_x2**2 + goal_y2**2:
                            goal_x = goal_x2
                            goal_y = goal_y2

                        theta = random.random() * 2 * math.pi
                        goal_x2 = spawn_pos.pose.position.x + gene_dis * math.cos(theta)
                        goal_y2 = spawn_pos.pose.position.y + gene_dis * math.sin(theta)
                        if goal_x**2+goal_y**2 > goal_x2**2 + goal_y2**2:
                            goal_x = goal_x2
                            goal_y = goal_y2

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

                position_check = self.judge_goal_collision(goal_x, goal_y)
                if WORLD_NAME == 'boxes_3':
                    position_check = position_check or (abs(goal_x)>8 or abs(goal_y)>8)

                print('position checking')
        elif position_check:
            self.goal_position.position.x = -1.3
            self.goal_position.position.y = -4.3
        
        time.sleep(0.5)
        if delete:
            self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
