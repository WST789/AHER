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

import rospy
import numpy as np
import math
from math import pi                                                                #pi 派 3.141596
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal_tw2 import Respawn

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
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)   #创建服务对象
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()

    def getGoalDistace(self):                                               #返回目标距离
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):                                            #里程计信息，给self.heading赋值，读取里程信息计算航向角
        self.position = odom.pose.pose.position                             #由里程计信息获得自身位置
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]   #X轴、Z轴、Y轴、旋转角度
        _, _, yaw = euler_from_quaternion(orientation_list)                               #四元组到欧拉角

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x) #距离目标的角度
        heading = goal_angle - yaw                                         #航向角heading（yaw） 
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)                             #按照指定的小数位数进行四舍五入运算

    def getState(self, scan):                                        #获取状态，由激光数据获取状态
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False
        #print(len(scan.ranges))
        for i in range(len(scan.ranges)):                     #scan.ranges一个周期的扫描数据 24维
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):                    #np.isnan()是判断是否是空值
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:                  #min(scan_range)最小的scan_range
            done = True                                      #done为true,撞击，发生碰撞

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)#math.hypot返回所有参数的平方和的平方根
        if current_distance < 0.2:
            self.get_goalbox = True                        #到达目标

        return scan_range + [heading, current_distance], done         #heading, current_distance加到scan_range[]后面

    def setReward(self, state, done, action):                   #设置奖励
        #print(state)
        #print(action)                                           #action 0 1 2 3 4
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]
        done_d3qn = False

        for i in range(5):                                                                               #5?
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate)                                    #奖励

        if done:                              #done为true,撞击，发生碰撞，-200
            rospy.loginfo("Collision!!")
            reward = -200
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:                 #到达目标点 200
            rospy.loginfo("Goal!!")
            reward = 200
            done_d3qn= True
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)   #给定新的目标点,新的目标距离
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward,done_d3qn

    def step(self, action):                                         
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)                                                 #发布一个线速度和角速度

        data = None
        while data is None:                                                               
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)               #等待激光雷达信息
            except:
                pass

        state, done = self.getState(data)                                              #获取状态
        reward,done_d3qn = self.setReward(state, done, action)                                   #设置奖励
       # if done | done_d3qn:
           # done = True
        #else:
            #done = False

        return np.asarray(state), reward, done,done_d3qn                                       #返回下一个状态，奖励，done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')                              #等待gazebo重置返回gazebo/reset_simulation
        try:
            self.reset_proxy()                                                         #创建服务对象
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)            #等待接受激光雷达信息
            except:
                pass

        if self.initGoal:                                                              #初始化目标点
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()                  
            self.initGoal = False

        self.goal_distance = self.getGoalDistace()                                     #目标距离
        state, done = self.getState(data)                                              #获取状态，返回状态

        return np.asarray(state)
