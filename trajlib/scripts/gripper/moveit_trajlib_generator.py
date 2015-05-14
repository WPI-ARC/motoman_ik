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

from apc_util.collision import attach_sphere

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Structure
from gripper_goal_pos_generate import Jnt_state_goal;
# Variable
from gripper_goal_pos_generate import left_arm_init_joint_value, right_arm_init_joint_value, left_arm_torso_init_joint_value, right_arm_torso_init_joint_value;
# Function
from gripper_goal_pos_generate import generate_goal_points, generate_left_arm_seed_state, generate_left_arm_torso_seed_state, generate_key_joint_state;

planning_time = 30;
using_torso = True;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;
default_Bin_X = X_pos;
default_Bin_Y = Y_pos;
default_Bin_Z = Z_pos;

topic = '/visualization_marker';
marker_publisher = rospy.Publisher(topic, Marker);
	
def Draw_GoalPnt(goal_pnts):
	
	goal_positions = MarkerArray();
	
	marker = Marker();
	
	for goal in goal_pnts:
		marker.header.frame_id = "base_link";
		marker.type = marker.SPHERE_LIST;
		marker.action = marker.ADD;
		marker.scale.x = 0.04;
		marker.scale.y = 0.04;
		marker.scale.z = 0.04;
		marker.pose.orientation.w = 1;
		
		marker.color.a = 1.0
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		
		showpnt = Point();
		showpnt.x = goal.x;
		showpnt.y = goal.y;
		showpnt.z = goal.z;
		marker.points.append(showpnt);
		
	marker_publisher.publish(marker);

def Get_current_state(group):
	joint_number = 7;
	if using_torso:
		joint_number = 8;
	current_state = JointState(name=group.get_joints()[:joint_number], position=group.get_current_joint_values());
	return	current_state

def Generate_joint_state_msg(group_handle,jnt_value):
	joint_number = 7;
	if using_torso:
		joint_number = 8;
	return JointState( name = group_handle.get_joints()[:joint_number], position = jnt_value);

def find_IK_solution(ik, target, seed, group_name):
    response = ik( GetPositionIKRequest(ik_request = PositionIKRequest( group_name = group_name,
                                                                        pose_stamped = PoseStamped( header = Header(frame_id="/base_link"),
                                                                                                    pose = target),
                                                                        robot_state = RobotState(joint_state = seed))
                                                                        ))
    return response

def torso_init(torso_group_handle):
	torso_init_rotation_angle = [0,0];
	torso_group_handle.set_start_state_to_current_state();
	torso_group.go(torso_init_rotation_angle);

def arm_init(left_arm_group_handle, right_arm_group_handle):
	
	if using_torso:
		left_arm_group_handle.go(left_arm_torso_init_joint_value);
		right_arm_group_handle.go(right_arm_torso_init_joint_value);
	else:
		left_arm_group_handle.go(left_arm_init_joint_value);
		right_arm_group_handle.go(right_arm_init_joint_value);

def Save_traj(file_name,plan):
	
    print "saving ",file_name;    
    buf = StringIO();
    plan.serialize(buf);
    f = open(file_name,"w");
    f.write(buf.getvalue());
    f.close();

def Copy_joint_value(group_name, joint_values):
    count = 0;
    Target_joint_value = [];
    if group_name == "arm_left":
        Target_joint_value = joint_values[1:8];
    elif group_name == "arm_left_torso":
        Target_joint_value = joint_values[0:8];
		
    elif group_name == "arm_right":
        Target_joint_value = joint_values[20:27];
    elif group_name == "arm_right_torso":
        Target_joint_value = joint_values[19:27];
        
    return Target_joint_value;

def calculateIK_solution(pnt, ik_handle, seed_config, group_handle):

	seed_state = Generate_joint_state_msg(group_handle,seed_config.jnt_val);
	
	target_pnt = geometry_msgs.msg.Pose();
	target_pnt.position.x = pnt.x;
	target_pnt.position.y = pnt.y;
	target_pnt.position.z = pnt.z;
	target_pnt.orientation.x = pnt.qx;
	target_pnt.orientation.y = pnt.qy;
	target_pnt.orientation.z = pnt.qz;
	target_pnt.orientation.w = pnt.qw;
	result = find_IK_solution(ik_handle, target_pnt, seed_state, group_handle.get_name());
	
	IK_solution = Jnt_state_goal();
	if result.error_code.val != 1:
		print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
		print "Can't find IK solution for Bin", pnt.bin_num, pnt.pnt_property;
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
		return IK_solution;	
	
	IK_solution.jnt_val = Copy_joint_value(group_handle.get_name(),result.solution.joint_state.position);
	IK_solution.bin_num = seed_config.bin_num;
	IK_solution.pos_property = pnt.pnt_property;
	
	return IK_solution;
	
	
def generate_configurationSet(target_pnt_set, seed_config_set,ik_handle,group_handle):

    arm_config = [];
    
    count = 1;
    for num in range(0,len(seed_config_set)):
		seed_config = seed_config_set[num];

		pnt = target_pnt_set[2*count -2];
		print "Solving IK for pnt:", pnt.bin_num,pnt.pnt_property;
		Solution_1 = calculateIK_solution(pnt,ik_handle,seed_config,group_handle);
		arm_config.append(Solution_1);
		
		pnt = target_pnt_set[2*count-1];
		print "Solving IK for pnt:", pnt.bin_num,pnt.pnt_property;
		Solution_2 = calculateIK_solution(pnt,ik_handle,seed_config,group_handle);
		arm_config.append(Solution_2);		
		count += 1;

    print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.."
    print "Configuration update complete!!"
    return arm_config;

def Generate_traj_for_key2pnt(key_config_set, goal_config_set, group_handle):
	success_num = 0;
	total_traj_num = len(key_config_set)*len(goal_config_set);
	print "Key_Config Num: ",len(key_config_set);
	print "Goal_Config Num: ",len(goal_config_set);
	print "Try to generate ", total_traj_num, "trajectories";
	
	start_pos = key_config_set[0];
	drop_pos = start_pos;

	# Goto start pnt
	group_handle.set_joint_value_target(start_pos.jnt_val);
	group_handle.go();
	count  = 0;
	
	planning_attemps = 0;
	while(count < len(goal_config_set)):
		
		# From start pnt to entrance pnt
		entrance_goal_config = goal_config_set[count];
		
		goal_jnt_value_msg = Generate_joint_state_msg(group_handle,entrance_goal_config.jnt_val)
		group_handle.set_joint_value_target(goal_jnt_value_msg);
		
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
		print "Now planning for bin",entrance_goal_config.bin_num;
		print "======================================================";
		print "Planning from start pos to bin",entrance_goal_config.bin_num, entrance_goal_config.pos_property;
		plan = group_handle.plan();
		
		if planning_attemps < 20:
			planning_attemps += 1;
		else:
			planning_attemps = 0;
			count += 2;
			
		if len(plan.joint_trajectory.points):
			rospy.sleep(5);
			group_handle.execute(plan);
			folder_name = os.path.join(os.path.dirname(__file__), "../../trajectories/bin") + entrance_goal_config.bin_num;
			file_name = folder_name + "/"+ "forward";
			Save_traj(file_name,plan);
			success_num += 1;
			
			# From entrance pnt to exit pnt
			count += 1;
			exit_goal_config = goal_config_set[count];
			goal_jnt_value_msg = Generate_joint_state_msg(group_handle,exit_goal_config.jnt_val)
			group_handle.set_joint_value_target(goal_jnt_value_msg);
			print "----------------------------------------------";
			print "From ",entrance_goal_config.bin_num, entrance_goal_config.pos_property," move to",exit_goal_config.bin_num, exit_goal_config.pos_property;
			group_handle.go();
			rospy.sleep(3);
			
			# Plan from exit pnt to drop
			print "----------------------------------------------";
			pose = PoseStamped()
			pose.header.frame_id = "/arm_left_link_7_t"
			pose.header.stamp = rospy.Time.now()
			pose.pose.position.x = 0
			pose.pose.position.y = 0
			pose.pose.position.z = -0.35
			pose.pose.orientation.x = 0
			pose.pose.orientation.y = 0
			pose.pose.orientation.z = 0
			pose.pose.orientation.w = 1
			attach_sphere("arm_left_link_7_t", "Object", pose, 0.17, ["hand_left_finger_1_link_2", "hand_left_finger_1_link_3", "hand_left_finger_1_link_3_tip", "hand_left_finger_2_link_2", "hand_left_finger_2_link_3", "hand_left_finger_2_link_3_tip", "hand_left_finger_middle_link_2", "hand_left_finger_middle_link_3", "hand_left_finger_middle_link_3_tip"]);
			print "Planning from bin",exit_goal_config.bin_num, exit_goal_config.pos_property, "to drop";
			group_handle.set_joint_value_target(drop_pos.jnt_val);
			plan = group_handle.plan();
			count += 1;
			
			if len(plan.joint_trajectory.points):
				rospy.sleep(5);
				group_handle.execute(plan);
				file_name = folder_name + "/"+ "drop";
				Save_traj(file_name,plan);
				success_num += 1;
			else:
				print "Planning from bin",exit_goal_config.bin_num, exit_goal_config.pos_property , "to Drop position Failed!";
				count = count - 2;
		else:
			print "Planning from Start Position to bin",entrance_goal_config.bin_num, entrance_goal_config.pos_property, " Failed!";
	
	print "Total success number: ",success_num,"/",total_traj_num;

def Generate_traj_for_pnt2pnt(goal_config_set, group_handle):
	success_num = 0;
	total_traj_num = len(goal_config_set)*len(goal_config_set);
	print "GoalConfigNum: ",len(goal_config_set), ", Try to generate ", total_traj_num, "trajectories";

if __name__=='__main__':
  try:
	print ">>>> Initializing... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('Trajectory generator', anonymous=True);

	if using_torso:
		arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");
		arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	else:
		arm_left_group = moveit_commander.MoveGroupCommander("arm_left");
		arm_right_group = moveit_commander.MoveGroupCommander("arm_right");		
		torso_group = moveit_commander.MoveGroupCommander("torso");
		torso_init(torso_group);  
		

	arm_left_group.allow_replanning(True);
	arm_right_group.allow_replanning(True);
	arm_left_group.set_planner_id("RRTConnectkConfigDefault");
	arm_right_group.set_planner_id("RRTConnectkConfigDefault");
	
	arm_init(arm_left_group, arm_right_group);
	
	#arm_left_group.set_planner_id("RRTstarkConfigDefault");
	#arm_right_group.set_planner_id("RRTstarkConfigDefault");
	arm_left_group.set_planning_time(planning_time);
	arm_right_group.set_planning_time(planning_time);

	print ">>>> Import Bin model, Generate Testing Targets >>>>"
	if len(sys.argv)>1:
		X_pos = float(sys.argv[1]);
		Y_pos = float(sys.argv[2]);
		Z_pos = float(sys.argv[3]);
	else:
		X_pos = default_Bin_X;
		Y_pos = default_Bin_Y;
		Z_pos = default_Bin_Z;
		print "No distance assigned, using default parameters: ",X_pos, Y_pos, Z_pos;

	Load_Bin_model(X_pos, Y_pos, Z_pos);

	print ">>>> Waiting for service `compute_ik` >>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);
    
	print "Generating Target Ponits..."
	Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, using_torso = using_torso);
	print "Total", len(Goal_points), "target points";
	Draw_GoalPnt(Goal_points);

	print "Generating Seed States..."
	if using_torso:
		left_arm_seed_states = generate_left_arm_torso_seed_state();
	else:
		left_arm_seed_states = generate_left_arm_seed_state();
		
	print "Total", len(left_arm_seed_states), "seed states";

	print "Updating Joint Configurations..."
	left_arm_config_set = generate_configurationSet(Goal_points, left_arm_seed_states, ik, arm_left_group);

	print "Updating Key Joint States..."
	key_joint_state = generate_key_joint_state(arm_left_group.get_name());

	print ">>>> Start Generating trajectory library (from KeyPos <--> Bin)>>>>";
	Generate_traj_for_key2pnt(key_joint_state, left_arm_config_set, arm_left_group);

	print ">>>> Start Generating trajectory library (from Bin <--> Bin)>>>>";
	Generate_traj_for_pnt2pnt(left_arm_config_set,arm_left_group);

	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()

  except rospy.ROSInterruptException:
    pass
