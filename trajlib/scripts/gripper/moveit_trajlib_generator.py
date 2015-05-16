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

from apc_util.collision import attach_sphere, remove_object

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
from gripper_goal_pos_generate import left_arm_torso_init_joint_value, right_arm_torso_init_joint_value;
# Function
from gripper_goal_pos_generate import generate_goal_points, generate_Pick_points, generate_left_arm_watch_config, generate_Scan_points, generate_left_arm_torso_seed_state, generate_key_joint_state;

planning_time = 30;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;
default_Bin_X = X_pos;
default_Bin_Y = Y_pos;
default_Bin_Z = Z_pos;

topic = '/visualization_marker';
marker_publisher = rospy.Publisher(topic, Marker);

# Utility
def Add_collision_ball():
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

def Draw_GoalPnt(goal_pnts, size = 0.04, color = [0,1,0]):
	
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
		marker.color.r = color[0];
		marker.color.g = color[1];
		marker.color.b = color[2];
		
		showpnt = Point();
		showpnt.x = goal.x;
		showpnt.y = goal.y;
		showpnt.z = goal.z;
		marker.points.append(showpnt);
		
	marker_publisher.publish(marker);

def Get_current_state(group):
	joint_number = 8;
	current_state = JointState(name=group.get_joints()[:joint_number], position=group.get_current_joint_values());
	return	current_state

def Generate_joint_state_msg(group_handle,jnt_value):
	joint_number = 8;
	return JointState( name = group_handle.get_joints()[:joint_number], position = jnt_value);

def find_IK_solution(ik, target, seed, group_name):
    response = ik( GetPositionIKRequest(ik_request = PositionIKRequest( group_name = group_name,
                                                                        pose_stamped = PoseStamped( header = Header(frame_id="/base_link"),
                                                                                                    pose = target),
                                                                        robot_state = RobotState(joint_state = seed))
                                                                        ))
    return response

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

	IK_solution = Jnt_state_goal();
	
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
	
	Calculationg_attempt = 0;
	while(result.error_code.val != 1 and Calculationg_attempt < 10):
		result = find_IK_solution(ik_handle, target_pnt, seed_state, group_handle.get_name());
		Calculationg_attempt += 1;		
	
	if result.error_code.val != 1:
		print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
		print "Can't find IK solution for Bin", pnt.bin_num, pnt.pnt_property;
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
		return IK_solution;	
	
	IK_solution.jnt_val = Copy_joint_value(group_handle.get_name(),result.solution.joint_state.position);
	IK_solution.bin_num = seed_config.bin_num;
	IK_solution.pos_property = pnt.pnt_property;
	
	return IK_solution;

# Function
def Update_seedstate(target_pnt_set, seed_config_set,ik_handle,group_handle):
	arm_config = [];
	count = 1;
	for num in range(0,len(seed_config_set)):
		seed_config = seed_config_set[num];		
		pnt = target_pnt_set[num];
		print "Solving IK for pnt:", pnt.bin_num,pnt.pnt_property;
		Solution = calculateIK_solution(pnt,ik_handle,seed_config,group_handle);
		arm_config.append(Solution);
		
	print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.."
	print "Configuration update complete!!"
	return arm_config;

def generate_PickingconfigurationSet(target_pnt_set, seed_config_set,ik_handle,group_handle):

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
	drop_pos = key_config_set[1];

	# Goto start pnt
	group_handle.set_joint_value_target(start_pos.jnt_val);
	group_handle.go();
	count  = 0;
	
	planning_attemps = 0;
	while(count < len(goal_config_set)):
		
		# From start pnt to entrance pnt
		entrance_goal_config = goal_config_set[count];
		
		#if entrance_goal_config.bin_num != "B":
		#	count += 2;
		#	continue;
		
		goal_jnt_value_msg = Generate_joint_state_msg(group_handle,entrance_goal_config.jnt_val)
		group_handle.set_joint_value_target(goal_jnt_value_msg);
		
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
		print "Now planning for bin",entrance_goal_config.bin_num;
		print "======================================================";
		print "Planning from start pos to bin",entrance_goal_config.bin_num, entrance_goal_config.pos_property;
		
		plan = group_handle.plan();
		
		planning_attemps = 1;
		while(len(plan.joint_trajectory.points) == 0 and planning_attemps <= 20):
			print "Planning Attempts:",planning_attemps;
			plan = group_handle.plan();
			planning_attemps += 1;
			
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
			Add_collision_ball();
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
			
			remove_object();
		else:
			print "Planning from Start Position to bin",entrance_goal_config.bin_num, entrance_goal_config.pos_property, " Failed!";
			count += 2;

	print "Total success number: ",success_num,"/",total_traj_num;

def Generate_traj_for_home2scan(home_pos, scan_config_set, group_handle):

	print "****************************************************************************";
	print "****************************************************************************";
	print "****************************************************************************";
	print ">>>> Start Generating trajectory library (from HOME --> SCAN)...";
	
	success_num = 0;
	print "Try to generate ", len(scan_config_set), "trajectories";

	# Goto start pnt
	home_jnt_value_msg = Generate_joint_state_msg(group_handle,home_pos.jnt_val)
	group_handle.set_joint_value_target(home_jnt_value_msg);
	group_handle.go();
	
	count  = 0;	
	planning_attemps = 0;
	while(count < len(scan_config_set)):
		
		scan_goal_config = scan_config_set[count];
		
		#if scan_goal_config.bin_num != "B":
		#	count += 2;
		#	continue;
		
		goal_jnt_value_msg = Generate_joint_state_msg(group_handle,scan_goal_config.jnt_val)
		group_handle.set_joint_value_target(goal_jnt_value_msg);
		
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
		print "Now planning for bin",scan_goal_config.bin_num, scan_goal_config.pos_property;
		print "======================================================";
		
		plan = group_handle.plan();
		
		planning_attemps = 1;
		while(len(plan.joint_trajectory.points) == 0 and planning_attemps <= 20):
			print "Planning Attempts:",planning_attemps;
			plan = group_handle.plan();
			planning_attemps += 1;
			
		if len(plan.joint_trajectory.points):
			rospy.sleep(5);
			group_handle.execute(plan);
			folder_name = os.path.join(os.path.dirname(__file__),"../../")+ "trajectories/bin" + scan_goal_config.bin_num;
			file_name = folder_name + "/"+ "scan";
			Save_traj(file_name,plan);
			success_num += 1;
			
			home_jnt_value_msg = Generate_joint_state_msg(group_handle,home_pos.jnt_val)
			group_handle.set_joint_value_target(home_jnt_value_msg);
			go_home_plan = group_handle.plan();
			planning_attemps = 1;
			while(len(go_home_plan.joint_trajectory.points) == 0 and planning_attemps <= 20):
				print "Try to get back home from bin,",scan_goal_config.bin_num, scan_goal_config.pos_property;
				go_home_plan = group_handle.plan();
				planning_attemps += 1;
			if len(go_home_plan.joint_trajectory.points) == 0 :
				group_handle.set_planner_id("RRTstarkConfigDefault");
				group_handle.set_planning_time(planning_time);
				go_home_plan = group_handle.plan();
			group_handle.execute(go_home_plan);
			group_handle.set_planner_id("RRTConnectkConfigDefault");
			
		else:
			print "Planning from Start Position to bin",scan_goal_config.bin_num, scan_goal_config.pos_property, " Failed!";
		
		count += 1;

	print "Total success number: ",success_num,"/", len(scan_config_set);

def Generate_traj_for_scan2enter(scan_config_set, enter_config_set, group_handle):

	print "****************************************************************************";
	print "****************************************************************************";
	print "****************************************************************************";
	print ">>>> Start Generating trajectory library (from SCAN --> ENTER)...";
	
	success_num = 0;
	print "Try to generate ", len(enter_config_set), "trajectories";
	
	start_pos = scan_config_set[0];
	count  = 0;
	
	planning_attemps = 0;
	while(count < len(scan_config_set)):
		
		# Goto A New ScanConfig
		current_scan_config = scan_config_set[count];		
		current_scan_value_msg = Generate_joint_state_msg(group_handle,current_scan_config.jnt_val)
		group_handle.set_joint_value_target(current_scan_value_msg);
		group_handle.go();
		enter_goal_config = enter_config_set[count];
		
		#if entrance_goal_config.bin_num != "B":
		#	count += 2;
		#	continue;
		
		goal_jnt_value_msg = Generate_joint_state_msg(group_handle,enter_goal_config.jnt_val)
		group_handle.set_joint_value_target(goal_jnt_value_msg);
		
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
		print "Now planning from",current_scan_config.bin_num, current_scan_config.pos_property, "to", enter_goal_config.bin_num, enter_goal_config.pos_property;
		print "======================================================";		
		plan = group_handle.plan();		
		planning_attemps = 1;
		
		while(len(plan.joint_trajectory.points) == 0 and planning_attemps <= 20):
			print "Planning Attempts:",planning_attemps;
			plan = group_handle.plan();
			planning_attemps += 1;
			
		if len(plan.joint_trajectory.points):
			rospy.sleep(5);
			group_handle.execute(plan);
			folder_name = os.path.join(os.path.dirname(__file__), "../../trajectories/bin") + current_scan_config.bin_num;
			file_name = folder_name + "/"+ "enter";
			Save_traj(file_name,plan);
			success_num += 1;
		else:
			print "Planning from", current_scan_config.bin_num, current_scan_config.pos_property, "to", enter_goal_config.bin_num, enter_goal_config.pos_property, " Failed!";
		
		count += 1;
	
	print "Total success number: ",success_num,"/",len(enter_config_set);

def Generate_traj_for_exit2drop(exit_config_set,drop_pos, group_handle):

	print "****************************************************************************";
	print "****************************************************************************";
	print "****************************************************************************";
	print ">>>> Start Generating trajectory library (from EXIT --> DROP)...";

	success_num = 0;
	print "Try to generate ", len(exit_config_set), "trajectories";
	
	count  = 0;	
	planning_attemps = 0;
	while(count < len(exit_config_set)):
		
		exit_goal_config = exit_config_set[count];
		
		#if exit_goal_config.bin_num != "B":
		#	count += 2;
		#	continue;		
		goal_jnt_value_msg = Generate_joint_state_msg(group_handle,exit_goal_config.jnt_val)
		group_handle.set_joint_value_target(goal_jnt_value_msg);
		plan = group_handle.plan();
		planning_attemps = 1;
		while(len(plan.joint_trajectory.points) == 0 and planning_attemps <= 20):
			print "Try to move to bin", exit_goal_config.bin_num, exit_goal_config.pos_property, "with No.",planning_attemps, "Attempts:";
			plan = group_handle.plan();
			planning_attemps += 1;
		
		if len(plan.joint_trajectory.points):
			group_handle.execute(plan);
			rospy.sleep(5);	
			print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
			print "Now planning from bin",exit_goal_config.bin_num, exit_goal_config.pos_property,"to drop";
			print "======================================================";
			
			drop_jnt_value_msg = Generate_joint_state_msg(group_handle,drop_pos.jnt_val)
			group_handle.set_joint_value_target(drop_jnt_value_msg);
			go_drop_plan = group_handle.plan();
			planning_attemps = 1;
			while(len(go_drop_plan.joint_trajectory.points) == 0 and planning_attemps <= 20):
				print "Try to plan to drop pos from bin,",exit_goal_config.bin_num, exit_goal_config.pos_property;
				go_drop_plan = group_handle.plan();
				planning_attemps += 1;
			if len(go_drop_plan.joint_trajectory.points) == 0 :
				print "Now will switch to RRT* planner, with",planning_time,"Seconds to plan";
				group_handle.set_planner_id("RRTstarkConfigDefault");
				group_handle.set_planning_time(planning_time);
				go_drop_plan = group_handle.plan();
			else:
				group_handle.execute(go_drop_plan);
				
			group_handle.set_planner_id("RRTConnectkConfigDefault");
			folder_name = os.path.join(os.path.dirname(__file__), "../../trajectories/bin") + exit_goal_config.bin_num;
			file_name = folder_name + "/"+ "drop";
			Save_traj(file_name,go_drop_plan);
			success_num += 1;
		else:
			print "Plan moving to bin",exit_goal_config.bin_num, exit_goal_config.pos_property, " Failed!";
		
		count += 1;

	print "Total success number: ",success_num,"/", len(exit_config_set);

def Generate_traj_for_pnt2pnt(goal_config_set, group_handle):
	success_num = 0;
	total_traj_num = len(goal_config_set)*len(goal_config_set);
	print "GoalConfigNum: ",len(goal_config_set), ", Try to generate ", total_traj_num, "trajectories";

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
	
	#arm_left_group.set_planner_id("RRTstarkConfigDefault");
	#arm_right_group.set_planner_id("RRTstarkConfigDefault");
	arm_left_group.set_planning_time(planning_time);
	arm_right_group.set_planning_time(planning_time);

	print ">>>> Import Bin model, Generate Testing Targets >>>>"
	if len(sys.argv)>1:
		X_pos = float(sys.argv[1]);
		Y_pos = float(sys.argv[2]);
		Z_pos = float(sys.argv[3]);
	
	X_pos = default_Bin_X;
	Y_pos = default_Bin_Y;
	Z_pos = default_Bin_Z;
	print "Current Shelf Position: ",X_pos, Y_pos, Z_pos;
	Load_Bin_model(X_pos, Y_pos, Z_pos);

	print ">>>> Waiting for service `compute_ik` >>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);

	print ">>>> Importing Init / Drop configurations..."
	key_joint_state = generate_key_joint_state(arm_left_group.get_name());

	# Generating SCANNING Configuration
	print ">>>> Generating SCANNING goal Ponits..."
	Scanning_points = generate_Scan_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, Extend_distance = 0.7);
	print "Total", len(Scanning_points), "SCAN points";
	print ">>>> Importing SCANNING seed States..."
	left_arm_scan_seed_set = generate_left_arm_watch_config();
	print "Total", len(left_arm_scan_seed_set), "SCAN seed states";
	print ">>>> Updating SCANNING Configurations...";
	LEFT_ARM_SCAN_CONFIG_SET = Update_seedstate(Scanning_points, left_arm_scan_seed_set, ik, arm_left_group);
	print "Total", len(LEFT_ARM_SCAN_CONFIG_SET), "SCANNING goal states";
	
	# Generating (ENTER / EXIT) Configuration
	print ">>>> Generating ENTER goal Ponits..."
	Enter_points = generate_Pick_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, Extend_distance = 0.5, pnt_property = 'EnterPnt');
	print "Total", len(Enter_points), "ENTER points";
	print ">>>> Generating EXIT goal Ponits..."
	Exit_points = generate_Pick_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, Extend_distance = 0.39, pnt_property = 'ExitPnt');
	print "Total", len(Exit_points), "EXIT points";
	print ">>>> Importing (ENTER / EXIT) seed States..."
	left_arm_Picking_seedstate_set = generate_left_arm_torso_seed_state();
	print "Total", len(left_arm_Picking_seedstate_set), "(ENTER / EXIT) seed states";
	
	print ">>>> Updating ENTER Configurations...";
	LEFT_ARM_ENTER_CONFIG_SET = Update_seedstate(Enter_points, left_arm_Picking_seedstate_set, ik, arm_left_group);
	print "Total", len(LEFT_ARM_ENTER_CONFIG_SET), "ENTER goal states";
	print ">>>> Updating EXIT Configurations...";
	LEFT_ARM_EXIT_CONFIG_SET = Update_seedstate(Exit_points, left_arm_Picking_seedstate_set, ik, arm_left_group);
	print "Total", len(LEFT_ARM_EXIT_CONFIG_SET), "EXIT goal states";
	
	#Draw_GoalPnt(Scanning_points, 0.06, [1,0,0]);
	#Generate_traj_for_home2scan(key_joint_state[0],LEFT_ARM_SCAN_CONFIG_SET,arm_left_group);

	#Draw_GoalPnt(Enter_points, 0.04, [1,1,0]);	
	#Generate_traj_for_scan2enter(LEFT_ARM_SCAN_CONFIG_SET, LEFT_ARM_ENTER_CONFIG_SET,arm_left_group);	

	Draw_GoalPnt(Exit_points, 0.04, [0,1,0]);
	Generate_traj_for_exit2drop(LEFT_ARM_EXIT_CONFIG_SET, key_joint_state[1],arm_left_group);

	#ScanPos --> PickPos Library
	#print ">>>> Generating PICK/DROP goal Ponits..."
	#Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, Extend_distance = 0.42);
	#print "Total", len(Goal_points), "TARGET points";
	#Draw_GoalPnt(Goal_points);
	#print ">>>> Importing PICK & DROP seed States..."
	#left_arm_seed_states = generate_left_arm_torso_seed_state();
	#print ">>>> Updating goal Configurations...";
	#left_arm_config_set = generate_PickingconfigurationSet(Goal_points, left_arm_seed_states, ik, arm_left_group);
	#print "Total", len(left_arm_config_set), "goal states";
	
	#print ">>>> Start Generating trajectory library (from KeyPos <--> Bin)>>>>";
	#Generate_traj_for_key2pnt(key_joint_state, left_arm_config_set, arm_left_group);
	#print ">>>> Start Generating trajectory library (from Bin <--> Bin)>>>>";
	#Generate_traj_for_pnt2pnt(left_arm_config_set,arm_left_group);

	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()

  except rospy.ROSInterruptException:
    pass
