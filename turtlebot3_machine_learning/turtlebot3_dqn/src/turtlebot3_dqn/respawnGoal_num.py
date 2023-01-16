#!/usr/bin/env python
#-*- coding:UTF-8 -*-
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

from syslog import LOG_EMERG
import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import numpy as np

class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
                                                'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/stage_number')
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        #self.obstacle_1 = -2, 0.9
        #self.obstacle_2 = 1, 0.9
        #self.obstacle_3 = -0.8, -1
        #self.obstacle_4 = 1, -0.7
        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False
        self.index = 0
        self.num=0

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

    def getPosition(self, position_check=True, delete=False):        #position_check，检查新的目标点是否与障碍物碰撞，False，取初始化目标点，True，检查 ############e
        goal_num=0
        import os
        os.chdir('/home/wst/catkin_t2/src/turtlebot3_machine_learning/turtlebot3_dqn/nodes/rewards') 
        if delete:
            self.deleteModel()

        if self.stage != 4:
            while position_check:
                d3qn_her=np.load('goal0803.npy')
                print(len(d3qn_her))
                print(self.num)
                #self.num=384
                # goal_num=random.randrange(1,5)
                if self.num<len(d3qn_her)-1:
                    #print('norandom')
                    goal_num = 10
                    goal_x = d3qn_her[self.num][0]
                    goal_y = d3qn_her[self.num][1]
                    print(goal_x,goal_y)
                    self.num+=1                                                                                                                                                        
                else:
                   goal_num=random.randrange(1,5)
                   print('random')
                   self.num+=1
                #goal_x = 0
                #goal_y = -3
                #goal_x=2.9
                #goal_y=1.8
                #print(goal_num)
                if goal_num == 1:
                   goal_x = random.randrange(-9, 12) / 10.0
                   goal_y = random.randrange(-15, 19) / 10.0
                if goal_num == 2:
                   goal_x = random.randrange(-28, -14) / 10.0
                   goal_y = random.randrange(-15, -4) / 10.0
                if goal_num == 3:
                   goal_x = random.randrange(-5, 13) / 10.0
                   goal_y = random.randrange(-30, -20) / 10.0
                if goal_num == 4:
                   goal_x = random.randrange(18, 30) / 10.0
                   goal_y = random.randrange(-30, -19) / 10.0
                if goal_num == 5:
                   goal_x = random.randrange(-58, -16) / 10.0
                   goal_y = random.randrange(2, 19) / 10.0
                if goal_num == 6:
                   goal_x = random.randrange(-58, -35) / 10.0
                   goal_y = random.randrange(-31, -4) / 10.0
                if goal_num == 7:
                   goal_x = random.randrange(-28, -11) / 10.0
                   goal_y = random.randrange(-31, -21) / 10.0
                if goal_num == 8:
                   goal_x = random.randrange(19, 31) / 10.0
                   goal_y = random.randrange(-12, 19) / 10.0
                position_check = False
                

                #goal_x = random.randrange(-58, -16) / 10.0  
                #goal_y = random.randrange(2, 19) / 10.0
                #goal_x = random.randrange(-58, -35) / 10.0
                #goal_y = random.randrange(-31, -4) / 10.0
                #goal_x = random.randrange(-28, -11) / 10.0
                #goal_y = random.randrange(-31, -21) / 10.0
                #goal_x = random.randrange(19, 31) / 10.0
                #goal_y = random.randrange(-12, 19) / 10.0



                # goal_x = random.randrange(-11, 11) / 10.0
                # goal_y = random.randrange(-11, 11) / 10.0

                # if abs(goal_x - self.obstacle_1[0]) <= 0.4 and abs(goal_y - self.obstacle_1[1]) <= 0.4:  #0.4
                #    position_check = True
                # elif abs(goal_x - self.obstacle_2[0]) <= 0.4 and abs(goal_y - self.obstacle_2[1]) <= 0.4:
                #    position_check = True
                # elif abs(goal_x - self.obstacle_3[0]) <= 0.4 and abs(goal_y - self.obstacle_3[1]) <= 0.4:
                #    position_check = True    
                # elif abs(goal_x - self.obstacle_4[0]) <= 0.4 and abs(goal_y - self.obstacle_4[1]) <= 0.4:
                #    position_check = True
                # elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                #    position_check = True
                # else:
                #    position_check = False
                if goal_x==0 and goal_y==0:
                    position_check = True
                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_position.position.x = goal_x
                self.goal_position.position.y = goal_y

        else:                                                                                     #stage_4有固定的目标点集合
            while  position_check:
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
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x                          #记录上一个目标点的位置
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
