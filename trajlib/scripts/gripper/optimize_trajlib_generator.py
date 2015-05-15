#!/usr/bin/env python

import os
import sys
import copy
import rospy
import StringIO
import moveit_commander
import moveit_msgs.msg
import geometry_msgs

# For save file
import xlsxwriter

from std_msgs.msg import String, Header,Int64
from StringIO import StringIO

from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Server msg structure
from optimize_planner.srv import PathPlan

# Structure
from gripper_goal_pos_generate import Jnt_state_goal;
# Variable
from gripper_goal_pos_generate import left_arm_init_joint_value, right_arm_init_joint_value, left_arm_torso_init_joint_value, right_arm_torso_init_joint_value;
# Function
from gripper_goal_pos_generate import generate_goal_points, generate_left_arm_seed_state, generate_left_arm_torso_seed_state, generate_key_joint_state;

from apc_util.collision import attach_sphere, remove_object

test_num = 1;
planning_time = 30

test_weight = 0.0;
using_torso = True;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"))
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;
default_Bin_X = X_pos;
default_Bin_Y = Y_pos;
default_Bin_Z = Z_pos;

flg_Save_property = False;

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
    
def property_table(work_sheet_hanle,row_num,traj_name, traj, traj_length, weight, planning_time):
	Column_1 = 'A'+str(row_num);
	work_sheet_hanle.write(Column_1,traj_name);
	
	points_num = len(traj.joint_trajectory.points);
	execution_time = (traj.joint_trajectory.points[points_num-1].time_from_start - traj.joint_trajectory.points[0].time_from_start);
	Column_2 = 'B'+str(row_num);
	work_sheet_hanle.write(Column_2,str(execution_time));	
	
	Column_3 = 'C'+str(row_num);
	work_sheet_hanle.write(Column_3,str(traj_length));	
	
	Column_4 = 'D'+str(row_num);
	work_sheet_hanle.write(Column_4,str(weight));	
	
	Column_5 = 'E'+str(row_num);
	work_sheet_hanle.write(Column_5,str(planning_time));	

def Save_Traj_property(file_name,traj_name, traj, traj_length, weight, planning_time):
	f = open(file_name,"r+");
	f.read();
	points_num = len(traj.joint_trajectory.points);
	execution_time = (traj.joint_trajectory.points[points_num-1].time_from_start - traj.joint_trajectory.points[0].time_from_start);
	Content = traj_name + " " + str(round(traj_length,3)) + " " + str(execution_time) + " " + str(weight) + " " + str(planning_time) + "\n";	
	f.write(Content);
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
		Solution_1 = calculateIK_solution(pnt,ik_handle,seed_config,group_handle);
		print "Solving IK for pnt:", Solution_1.bin_num,Solution_1.pos_property;
		arm_config.append(Solution_1);
		
		pnt = target_pnt_set[2*count-1];
		Solution_2 = calculateIK_solution(pnt,ik_handle,seed_config,group_handle);
		print "Solving IK for pnt:", Solution_2.bin_num,Solution_2.pos_property;
		arm_config.append(Solution_2);		
		count += 1;

    print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.."
    print "Configuration update complete!!"
    
    return arm_config;

def Generate_traj_for_key2pnt(key_config_set, goal_config_set, group_name, planner_handle, cost_weight, time_limit):
	success_num = 0;
	total_traj_num = len(key_config_set)*len(goal_config_set);
	print "KeyConfigNum: ",len(key_config_set), "GoalNum: ",len(goal_config_set),", Try to generate ", total_traj_num, "trajectories";
	
	# Using the same start & end point
	start_pos = key_config_set[0];
	drop_pos = start_pos;
	
	
	if flg_Save_property:
		sheet_file_name = os.path.join(os.path.dirname(__file__), "../optimize_trajectories_property") + ".xlsx";
		row_num = 1;
		workbook = xlsxwriter.Workbook(sheet_file_name);
		worksheet = workbook.add_worksheet();
	
	try:
		server_msg = PathPlan();
		server_msg.group_name = group_name;
		server_msg.cost_weight = cost_weight;
		server_msg.time_limit = time_limit;
		
		num = 0;
		while(num < len(goal_config_set)):
			print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
			current_goal_config = goal_config_set[num];
			# Will calculate 20 times for each trajectory
			print "Generating the trajectory from START --> bin", current_goal_config.bin_num,current_goal_config.pos_property;
			for count in range(0,test_num):
				# Plan from start to the bin
				server_msg.start_config = start_pos.jnt_val;
				server_msg.target_config = current_goal_config.jnt_val;
				response_forward = planner_handle(server_msg.group_name,
												  server_msg.start_config,
												  server_msg.target_config,
												  server_msg.cost_weight,
												  server_msg.time_limit,
												  0);
												  
				if response_forward.status == response_forward.SUCCESS:
					print "Weight:",server_msg.cost_weight," SUCCESS!";
					if(flg_Save_property):
						row_num += 1;
						traj_name = "Start-->bin" + current_goal_config.bin_num;
						property_table(worksheet,
									   row_num,
									   traj_name, 
									   response_forward.plan, 
									   response_forward.total_length, 
									   server_msg.cost_weight, 
									   server_msg.time_limit);
									   
					folder_name = os.path.join(os.path.dirname(__file__), "../../traj_updates/bin") + current_goal_config.bin_num;
					file_name = folder_name + "/"+ "forward";
					Save_traj(file_name,response_forward.plan);
					success_num += 1;
				else:
					print "Planning from START to bin",current_goal_config.bin_num, current_goal_config.pos_property, " FAILED!";
			
			print "-----------------------------------------------------------------------";
			
			num += 1;
			current_goal_config = goal_config_set[num];
			print "Generating the trajectory from bin", current_goal_config.bin_num,current_goal_config.pos_property, "--> DROP";
			for count in range(0,test_num):					
				server_msg.start_config = current_goal_config.jnt_val;
				server_msg.target_config = drop_pos.jnt_val;
				response_back = planner_handle(server_msg.group_name,
											   server_msg.start_config,
											   server_msg.target_config,
											   server_msg.cost_weight,
											   server_msg.time_limit,
											   0);
				if response_back.status == response_back.SUCCESS:
					print "Weight:",server_msg.cost_weight,"SUCCESS!";
					if(flg_Save_property):
						row_num += 1;
						traj_name = "bin" + current_goal_config.bin_num + "-->DROP";
						property_table(worksheet,
									   row_num,
									   traj_name, 
									   response_back.plan, 
									   response_back.total_length, 
									   server_msg.cost_weight, 
									   server_msg.time_limit);
									   
					folder_name = os.path.join(os.path.dirname(__file__), "../../traj_updates/bin") + current_goal_config.bin_num;  
					file_name = folder_name + "/"+ "drop";
					Save_traj(file_name,response_back.plan);
					success_num += 1;
				else:
					print "Planning from ",current_goal_config.bin_num, current_goal_config.pos_property,"to DROP , FAILED!";
				num += 1;
		
		if flg_Save_property:
			workbook.close();
		
		print "Total success ratio: ",success_num/total_traj_num;
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e;

def Generate_traj_for_pnt2pnt(goal_config_set, group_handle):
	success_num = 0;
	total_traj_num = len(goal_config_set)*len(goal_config_set);
	print "GoalConfigNum: ",len(goal_config_set), ", Try to generate ", total_traj_num, "trajectories";

if __name__=='__main__':
  try:
	print ">>>> Initializing... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('Optimize_trajectory_Generator', anonymous=True);

	remove_object();
	
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	arm_init(arm_left_group, arm_right_group);

	if len(sys.argv)>1:
		time_limit = float(sys.argv[1]);
		cost_weight = float(sys.argv[2]);
	else:
		time_limit = planning_time;
		cost_weight = test_weight;
		
	print "Current Cost weight is:",cost_weight, "and the time_limit is:",time_limit;
	print ">>>> Import Bin model, Generate Testing Targets >>>>"	
	X_pos = default_Bin_X;
	Y_pos = default_Bin_Y;
	Z_pos = default_Bin_Z;
	print "Default Bin_position: ",X_pos, Y_pos, Z_pos;
	Load_Bin_model(X_pos, Y_pos, Z_pos);

	print ">>>> Waiting for service `compute_ik` >>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);
	
	print "Generating Target Ponits...";
	Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, using_torso = using_torso);
	
	print "Total", len(Goal_points), "target points";
	Draw_GoalPnt(Goal_points);

	print "Generating Seed States..."
	if using_torso:
		left_arm_seed_states = generate_left_arm_torso_seed_state();
	else:
		left_arm_seed_states = generate_left_arm_seed_state();	
	print "Total", len(left_arm_seed_states), "seed states";
    
	print "Generating new Configurations..."
	left_arm_config_set = generate_configurationSet(Goal_points, left_arm_seed_states, ik, arm_left_group);
    
	print "Updating Key States(Drop/Init position)..."
	key_joint_state = generate_key_joint_state(arm_left_group.get_name());

	print ">>>> Waiting for service 'Optimize_plan_server' >>>>";
	rospy.wait_for_service('Optimize_plan_server');
	op_planner = rospy.ServiceProxy("Optimize_plan_server", PathPlan);

    
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

	print ">>>> Start Generating trajectory library (from KeyPos <--> Bin)>>>>";
	Generate_traj_for_key2pnt(key_joint_state, 
							  left_arm_config_set, 
							  arm_left_group.get_name(), 
							  op_planner,
							  cost_weight,
							  time_limit);

	#print ">>>> Start Generating trajectory library (from Bin <--> Bin)>>>>";
	#Generate_traj_for_pnt2pnt(left_arm_config_set,arm_left_group);

	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()

  except rospy.ROSInterruptException:
    pass

