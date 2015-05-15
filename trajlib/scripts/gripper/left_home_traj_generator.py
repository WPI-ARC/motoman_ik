#!/usr/bin/env python

import os
import sys
import copy
import rospy
import StringIO
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int64
from StringIO import StringIO

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.msg import RobotTrajectory

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState

# Variable
from gripper_goal_pos_generate import left_arm_torso_init_joint_value, right_arm_torso_init_joint_value;

planning_time = 120;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;
default_Bin_X = X_pos;
default_Bin_Y = Y_pos;
default_Bin_Z = Z_pos;
from motoman_configuration import arm_left_drop, arm_left_home;


def Generate_joint_state_msg(group_handle,jnt_value):
	return JointState(name = group_handle.get_joints()[:8], position = jnt_value);

def arm_init(left_arm_group_handle, right_arm_group_handle):
	left_arm_group_handle.go(left_arm_torso_init_joint_value);
	right_arm_group_handle.go(right_arm_torso_init_joint_value);

def Save_traj(file_name,plan):
	
    print "saving ",file_name;    
    buf = StringIO();
    plan.serialize(buf);
    f = open(file_name,"w");
    f.write(buf.getvalue());
    f.close();

if __name__=='__main__':
  try:
	print ">>>> Initializing... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('Trajectory generator', anonymous=True);

	arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso");

	arm_left_group.allow_replanning(True);
	arm_right_group.allow_replanning(True);	
	arm_left_group.set_planner_id("RRTConnectkConfigDefault");
	arm_right_group.set_planner_id("RRTConnectkConfigDefault");
	arm_init(arm_left_group, arm_right_group);
	arm_left_group.set_planner_id("RRTstarkConfigDefault");
	arm_right_group.set_planner_id("RRTstarkConfigDefault");
	arm_left_group.set_planning_time(planning_time);
	arm_right_group.set_planning_time(planning_time);

	X_pos = default_Bin_X;
	Y_pos = default_Bin_Y;
	Z_pos = default_Bin_Z;
	print "No distance assigned, using default parameters: ",X_pos, Y_pos, Z_pos;
	Load_Bin_model(X_pos, Y_pos, Z_pos);

	print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
	print "Planning from Home to Drop";
	arm_left_group.set_joint_value_target(arm_left_drop);
	plan = arm_left_group.plan();
	if len(plan.joint_trajectory.points):
		rospy.sleep(5);
		arm_left_group.execute(plan);
		folder_name = os.path.join(os.path.dirname(__file__), "../../trajectories/");
		file_name = folder_name + "left_arm_home2drop";
		Save_traj(file_name,plan);
		# Plan from bin to drop
		print "----------------------------------------------";
		print "Planning from Drop to Home";
		arm_left_group.set_joint_value_target(arm_left_home);
		plan = arm_left_group.plan();
		if len(plan.joint_trajectory.points):
			rospy.sleep(5);
			arm_left_group.execute(plan);
			file_name = folder_name + "left_arm_drop2home";
			Save_traj(file_name,plan);
		else:
			print "Planning from Drop to Side Failed!";
			
	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()

  except rospy.ROSInterruptException:
    pass
