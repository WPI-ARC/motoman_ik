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

from motoman_configuration import arm_left_init, arm_right_init;
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;

def runTrajectory(req):

    print "---------------------------------"
    print req.task
    print " "
    print req.bin_num
    print " "
    print req.using_torso
    print "---------------------------------"
    
    # Get the trajectory
    if req.using_torso == "y":
        file_root = os.path.join(os.path.dirname(__file__), "../trajectories/Torso/bin"+req.bin_num);
    else:
        file_root = os.path.join(os.path.dirname(__file__), "../trajectories/bin"+req.bin_num);

    traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
    if req.task == "Forward":
        file_name = file_root+"/forward";
        traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");
    elif req.task == "Drop":
        file_name = file_root+"/drop";
        traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");
    elif req.task == "Pick":
        file_name = file_root+"/Pick";
        traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
    elif req.task == "Dump":
        file_name = file_root+"/Dump";
        traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
        
    else :
        return taskResponse(False);

    f = open(file_name,"r");
    plan_file = RobotTrajectory();
    buf = f.read();
    plan_file.deserialize(buf);    
    plan = copy.deepcopy(plan_file); 

    # Display Current Trajectory
    robot = moveit_commander.RobotCommander();
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory();
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);
   
    print "============ Waiting while", file_name, " is visualized (again)..." 
    traj_execute_group.execute(plan);
    print "Trajectory ", file_name, " finished!"
    f.close();
        
    return GetTrajectoryResponse(plan,True);

def pos_init():		
	
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso");	
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");

	arm_left_group.set_start_state_to_current_state();
	arm_left_group.go(arm_left_init);
	
	arm_right_group.set_start_state_to_current_state();
	arm_right_group.go(arm_right_init);

def Start_server():

    rospy.init_node('trajectory_service');
    s = rospy.Service('trajectory_display', GetTrajectory, runTrajectory);	
    Load_Bin_model(X_pos, Y_pos, Z_pos); 
    # Might be a problem
    pos_init();    
    print " >>>>>>>>>>>>>> Waiting For trajectory request... >>>>>>>>>>>>>>>>>>>";
    print " Request format:";
    print " Task: <Forward/Drop> + BinCode: < A ... L > + UsingTorso < y/n >";
    print " Default: Forward + A + n";
    rospy.spin()

if __name__ == '__main__':
    Start_server()
