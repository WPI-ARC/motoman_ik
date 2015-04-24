#!/usr/bin/env python

from trajectory_srv.srv import *

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

from goal_pos_generate import left_arm_init_joint_value,right_arm_init_joint_value;
from generate_trajlib import Add_bin_model, default_Bin_X, default_Bin_Y, default_Bin_Z;

def runTrajectory(req):

    global Traj_server;
    
    print "---------------------------------"
    print req.task
    print " "
    print req.bin_num
    print "---------------------------------"
    
    file_root = os.path.join(os.path.dirname(__file__), "../trajectories/bin"+req.bin_num)

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
  
        
    arm_right_group = moveit_commander.MoveGroupCommander("arm_right"); 
    arm_left_group = moveit_commander.MoveGroupCommander("arm_left");	
    arm_left_group.set_start_state_to_current_state();    
        
    StartPnt = JointTrajectoryPoint();
    StartPnt.positions = arm_left_group.get_current_joint_values();
    StartPnt.velocities = [0]*len(StartPnt.positions);
    StartPnt. accelerations = [0]*len(StartPnt.positions);
    
    #print StartPnt;
    plan.joint_trajectory.points[0] = StartPnt;
    
    #print plan;
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
    display_trajectory = moveit_msgs.msg.DisplayTrajectory();    
    robot = moveit_commander.RobotCommander();
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);  
   
   
    print "============ Waiting while", file_name, " is visualized (again)..." 
    arm_left_group.execute(plan);
    print "Trajectory ", file_name, " finished!"   
    
    f.close();
    
    return taskResponse(True);

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
    s = rospy.Service('trajectory_execute', task, runTrajectory);	
    Add_bin_model(default_Bin_X, default_Bin_Y, default_Bin_Z); 
    pos_init();
	
    rospy.spin()


if __name__ == '__main__':
    Start_server()
