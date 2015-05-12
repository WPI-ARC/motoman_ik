#!/usr/bin/env python

from trajlib.srv import *

import os
import sys
import copy
import rospy
import StringIO
from std_msgs.msg import String, Header, Int64
from StringIO import StringIO

import moveit_msgs.msg
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest
from trajectory_msgs.msg import JointTrajectoryPoint


def runTrajectory(req):

    global Traj_server;
    
    print "---------------------------------"
    print req.task
    print " "
    print req.bin_num
    print " "
    print req.using_torso
    print "---------------------------------"
    
    if req.using_torso == "y":
        file_root = os.path.join(os.path.dirname(__file__), "../trajectories/Torso/bin"+req.bin_num);
    else:
        file_root = os.path.join(os.path.dirname(__file__), "../trajectories/bin"+req.bin_num);

    if req.task == "Forward":
        file_name = file_root+"/forward";
    elif req.task == "Drop":
        file_name = file_root+"/drop";
    else :
        return taskResponse(False);

    f = open(file_name,"r");
    plan_file = RobotTrajectory();
    buf = f.read();
    plan_file.deserialize(buf);
    
    plan = copy.deepcopy(plan_file);
    f.close(); 
    
    return GetTrajectoryResponse(plan, True);

def pos_init():		
	
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right");	
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left");

	arm_left_group.set_start_state_to_current_state();
	arm_left_group.go(left_arm_init_joint_value);
	
	arm_right_group.set_start_state_to_current_state();
	arm_right_group.go(right_arm_init_joint_value);

def Start_server():

    global Traj_server;    
    rospy.init_node('trajectory_service');
    s = rospy.Service('trajlib', GetTrajectory, runTrajectory);    
    print " >>>>>>>>>>>>>> Waiting For trajectory request... >>>>>>>>>>>>>>>>>>>";
    print " Request format:";
    print " Task: <Forward/Drop> + BinCode: < A ... L > + UsingTorso < y/n >";
    print " Default: Forward + A + n";
    rospy.spin()


if __name__ == '__main__':
    Start_server()
