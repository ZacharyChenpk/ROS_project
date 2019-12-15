#!/usr/bin/env python
# 1700012764 Chen Zhibin
#from __future__ import print_function
import rospy
import math
import random
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
from gazebo_msgs.msg import ModelStates

pub = False
cur_goal = True
cur_status = {'pose': False, 'twist': False}
robot_name = "turtlebot3_burger"
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1
timestamp = 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0

# Input: a float tuple, the data from laser sensor
# Output: a boolean, whether the bot is collided
def collision_check(laser_ranges):
    if min(laser_ranges) < 0.125:
        return True
    return False

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    return input

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

# a toy decision maker
# always P90 rush B
def toyDecision(laser_ranges, cur_goal=cur_goal, cur_pose=cur_status["pose"], cur_twist=cur_status["twist"]):
    action_list = ["W","A","X","D","S","NO"]
    return action_list[int(random.random()*6)]
    #return "W"

# The function to publish commands
# Frequency: 5Hz
def callback(data):
    if cur_goal != False:
        ###################
        #  action_list: ["W","A","X","D","S","NO"]
        #  data_struct:
        #       laser_range: a tuple with 360 float elements
        #
        #       cur_goal:
        #           cur_goal.position:
        #               cur_goal.position.x: a float64
        #               cur_goal.position.y: a float64
        #               cur_goal.position.z: a float64
        #           cur_goal.orientation:
        #               cur_goal.orientation.x: a float64
        #               cur_goal.orientation.y: a float64
        #               cur_goal.orientation.z: a float64
        #               cur_goal.orientation.w: a float64
        #
        #       cur_pose:
        #           geometry_msgs/Point position
        #               float64 x
        #               float64 y
        #               float64 z
        #           geometry_msgs/Quaternion orientation
        #               float64 x
        #               float64 y
        #               float64 z
        #               float64 w
        #
        #       cur_twist:
        #           geometry_msgs/Vector3 linear
        #               float64 x
        #               float64 y
        #               float64 z
        #           geometry_msgs/Vector3 angular
        #               float64 x
        #               float64 y
        #               float64 z
        #
        # insert your decision maker HERE
        action = toyDecision(data.ranges)
        ###################
        print(action)
        
        if action == "NO":
            return

        global timestamp
        global target_linear_vel
        global target_angular_vel
        global control_linear_vel
        global control_angular_vel
        timestamp = timestamp + 1
        if action == "S":
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0
        elif action == "W":
            target_linear_vel = constrain(target_linear_vel + LIN_VEL_STEP_SIZE, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        elif action == "A":
            target_linear_vel = constrain(target_linear_vel - LIN_VEL_STEP_SIZE, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
        elif action == "D":
            target_angular_vel = constrain(target_angular_vel + ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        elif action == "X":
            target_angular_vel = constrain(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

        twist = Twist()

        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)
        #print(min(data.ranges))
        #print(cur_goal)
        #print("-------")
        #print(cur_status.twist)
    
def info_collect1(data):
    cur_goal = data.goal.target_pose.pose
    rospy.loginfo("goal changed!")
    #print(data)

def info_collect2(data):
    idx = data.name.index(robot_name)
    val = {'pose': data.pose[idx], 'twist': data.twist[idx]}
    #print(val)
    cur_status = val

def communicator():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'communicator' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('communicator', anonymous=True)

    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.Subscriber("move_base/goal", MoveBaseActionGoal, info_collect1)
    rospy.Subscriber("gazebo/model_states", ModelStates, info_collect2)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    communicator()