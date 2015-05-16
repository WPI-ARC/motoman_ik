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
    elif req.task == "Pick":
        file_name = file_root+"/Pick";
    elif req.task == "Scan":
        file_name = file_root+"/scan";
    elif req.task == "Dump":
        file_name = file_root+"/Dump";
    elif req.task == "Lift":
        file_name = file_root+"/Lift";
    elif req.task == "Home":
        file_name = file_root+"/Home";
		
    else :
        return taskResponse(False);

    f = open(file_name,"r");
    plan_file = RobotTrajectory();
    buf = f.read();
    plan_file.deserialize(buf);
    
    plan = copy.deepcopy(plan_file);
    f.close(); 
    
    return GetTrajectoryResponse(plan, True);

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
