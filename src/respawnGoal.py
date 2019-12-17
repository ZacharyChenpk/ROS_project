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
import random
import time
import os
import math
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

WORLD_NAME = 'boxes'
BOT_INIT_X = -2.0
BOT_INIT_Y = -0.5

class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.obsPath = self.modelPath.replace('proj_api/src',
                                            'proj_api/worlds/'+WORLD_NAME+'.obs')
        self.modelPath = self.modelPath.replace('proj_api/src',
                                                'proj_api/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/stage_number')
        self.goal_position = Pose()
        self.init_goal_x = BOT_INIT_X + 0.25
        self.init_goal_y = BOT_INIT_Y
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
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0
        self.obsdata = np.loadtxt(self.obsPath)
        self.obs_n = self.obsdata.shape[0]/2
        self.obs_size = self.obsdata[0:self.obs_n]
        self.obs_center = self.obsdata[self.obs_n:]
        self.success_time = 0
        print(self.obs_size)
        print(self.obs_center)

    def get_gene_dis(self):
        return 0.3 + min(33, self.success_time) * 0.1

    def judge_goal_collision(self, goal_x, goal_y):
        for i in range(self.obs_n):
            if abs(goal_x - self.obs_center[i][0]) <= self.obs_size[i][0] + 0.2 and abs(goal_y - self.obs_center[i][1]) <= self.obs_size[i][1] + 0.2:
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

        while position_check:
            theta = random.random() * 2 * math.pi
            goal_x = spawn_pos.pose.position.x + gene_dis * math.cos(theta)
            goal_y = spawn_pos.pose.position.y + gene_dis * math.sin(theta)

            self.goal_position.position.x = goal_x
            self.goal_position.position.y = goal_y
            position_check = self.judge_goal_collision(goal_x, goal_y)

            print('position checking')

        # if self.stage != 4:
        #     while position_check:
        #         goal_x = random.randrange(-12, 13) / 10.0
        #         goal_y = random.randrange(-12, 13) / 10.0
        #         if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:
        #             position_check = True
        #         elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
        #             position_check = True
        #         elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
        #             position_check = True
        #         elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
        #             position_check = True
        #         elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
        #             position_check = True
        #         else:
        #             position_check = False

        #         if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
        #             position_check = True

        #         self.goal_position.position.x = goal_x
        #         self.goal_position.position.y = goal_y

        # else:
        #     while position_check:
        #         goal_x_list = [0.6, 1.9, 0.5, 0.2, -0.8, -1, -1.9, 0.5, 2, 0.5, 0, -0.1, -2]
        #         goal_y_list = [0, -0.5, -1.9, 1.5, -0.9, 1, 1.1, -1.5, 1.5, 1.8, -1, 1.6, -0.8]

        #         self.index = random.randrange(0, 13)
        #         print(self.index, self.last_index)
        #         if self.last_index == self.index:
        #             position_check = True
        #         else:
        #             self.last_index = self.index
        #             position_check = False

        #         self.goal_position.position.x = goal_x_list[self.index]
        #         self.goal_position.position.y = goal_y_list[self.index]

        
        time.sleep(0.5)
        if delete:
            self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
