
import os
import sys
import copy
import rospy
import StringIO

from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int64
from StringIO import StringIO

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tray_goal_pos_generator import generate_frontface_positions, seed_state_generator, arm_right_home, arm_left_home;

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotTrajectory

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"));
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;
default_Bin_X = X_pos;
default_Bin_Y = Y_pos;
default_Bin_Z = Z_pos;

# Constraint Planning
from moveit_msgs.msg import MotionPlanRequest;
from moveit_msgs.msg import WorkspaceParameters;
from moveit_msgs.msg import RobotState;
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint;
from geometry_msgs.msg import Quaternion;
from math import pi;
from moveit_msgs.msg import TrajectoryConstraints;

left_arm_init_joint_value = arm_left_home;
right_arm_init_joint_value = arm_right_home;

Planning_time = 600;

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
	joint_number = len(group_handle.get_joints());
	return JointState(name = group_handle.get_joints()[:joint_number], position = jnt_value);

def Get_current_state(group):
	return	JointState(name = group.get_joints()[0:8], 
					   position=group.get_current_joint_values());
    
def find_IK_solution(ik, target, seed, group_name):
    response = ik(GetPositionIKRequest(ik_request = PositionIKRequest( group_name = group_name,
																		pose_stamped = PoseStamped( header = Header(frame_id="base_link"),
																									pose = target),
																		robot_state = RobotState(joint_state=seed))
										) )
    return response

def pos_init(left_arm_group_handle, right_arm_group_handle):
	right_arm_group_handle.go(right_arm_init_joint_value);	
	left_arm_group_handle.go(left_arm_init_joint_value);

def Save_traj(plan, file_name):			

	print "saving trajectory to file",file_name;
	buf = StringIO();
	plan.serialize(buf);				
	f = open(file_name,"w");
	f.write(buf.getvalue());
	f.close();

def Generate_configuration_set(group_handle, pose_target_set, ik_handle):
	
	Arm_config_set = [];
	
	print ">>>> Generate Seed States >>>>"
	seed_state_set = seed_state_generator();
	
	for count_num in range(0,len(seed_state_set)):
		ik_solution = [];
		
		seed_state = seed_state_set[count_num];
		pose_target = pose_target_set[count_num];
		start_config = Generate_joint_state_msg(group_handle,seed_state.jnt_val);

		target = geometry_msgs.msg.Pose();
		target.position.x = pose_target.x;
		target.position.y = pose_target.y;
		target.position.z = pose_target.z;
		target.orientation.x = pose_target.qx;
		target.orientation.y = pose_target.qy;
		target.orientation.z = pose_target.qz;
		target.orientation.w = pose_target.qw;
		
		result = find_IK_solution(ik_handle, target, start_config, group_handle.get_name());
		
		if result.error_code.val != 1:
			print "Can't find IK solution for Bin", pose_target.bin_num, pose_target.pnt_property;		
		
		else:
			ik_solution = Copy_joint_value(group_handle.get_name(),result.solution.joint_state.position);
			#print "IK solution for Bin", pose_target.bin_num,"is:"
			#print ik_solution;
			#Arm_config_set.append(ik_solution);
			
			Arm_config_set.append(seed_state);
	
	return Arm_config_set;

def Copy_joint_value(group_name, joint_values):
	count = 0;
	Target_joint_value = [];
	if group_name == "arm_left_torso":
		Target_joint_value = joint_values[0:8];
	elif group_name == "arm_left":
		Target_joint_value = joint_values[1:8];
	elif group_name == "arm_right_torso":
		Target_joint_value = [8];
		Target_joint_value[0] = joint_values[27];
		Target_joint_value[1:8] = joint_values[20:27];
	elif group_name == "arm_right":
		Target_joint_value = joint_values[20:27];
	return Target_joint_value;

def pos_test(group_handle, IK_handle):

	print ">>>> Generate Goal Points >>>>"
	Goal_point_set = generate_frontface_positions(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos);
	#Draw_GoalPnt(Goal_point_set);
  # We have valid targets assigned
	if len(Goal_point_set):  

		print "Total target number:",len(Goal_point_set);
		success_number = 0;

		print ">>>> Generating IK solutions... >>>>"
		config_set = Generate_configuration_set(group_handle, Goal_point_set, IK_handle);
		print "Total ",len(config_set),"Ik solutions were found";

		for count in range(0,len(config_set)):
			group_handle.set_planner_id("RRTstarkConfigDefault");
			config = config_set[count];
			print ">>>> Planning Trajectory for bin",config.bin_num,"...";
			group_handle.set_start_state_to_current_state();
			group_handle.set_joint_value_target(config.jnt_val);
			Pick_plan = group_handle.plan();
			
			planning_attemps = 1;
			while(len(Pick_plan.joint_trajectory.points) == 0 and planning_attemps <= 5):
				print "Planning Attempts:",planning_attemps;
				Pick_plan = group_handle.plan();
				planning_attemps += 1;

			# Plan is valid, Save trajectory to file
			if len(Pick_plan.joint_trajectory.points):
				folder_name = os.path.join(os.path.dirname(__file__), "../../trajectories/bin") + config.bin_num;
				file_name = folder_name + "/"+ "Pick";	
				Save_traj(Pick_plan, file_name);
				print "Executing trajectory...";
				group_handle.execute(Pick_plan);
				rospy.sleep(5);
				# Go back to init_pos
				print "Try to go back to home...";
				group_handle.set_planner_id("RRTConnectkConfigDefault");
				group_handle.set_joint_value_target(right_arm_init_joint_value);
				plan = group_handle.plan();
				planning_attemps = 1;
				while(len(plan.joint_trajectory.points) == 0):
					group_handle.set_random_target();
					group_handle.go();
					print "Planning Attempts:",planning_attemps;
					group_handle.set_joint_value_target(right_arm_init_joint_value);
					plan = group_handle.plan();
					
				group_handle.execute(plan);
				
				rospy.sleep(5);
			else:
				print "Planning failed!";

	else:
	  print "No target Assigned, Exit!";
	  return False;

def setJointConstraints(goal_config, group_handle):
	constraints = [];
	names = group_handle.get_joints();
	for num in range(0,len(names)):
		jnt_constraint = JointConstraint();	
		jnt_constraint.joint_name = names[num];
		jnt_constraint.position = goal_config[num];
		jnt_constraint.tolerance_above = 0.0001;
		jnt_constraint.tolerance_below = 0.0001;
		jnt_constraint.weight = 1.0;
		constraints.append(jnt_constraint);
		
	return constraints;

def setOrientationConstraints(qw,qx,qy,qz, group_handle,weight):
	
	orient_constrain = OrientationConstraint();
	
	orient_constrain.link_name = group_handle.get_end_effector_link();
	
	orient_constrain.orientation = Quaternion();
	orient_constrain.orientation.w = qw;
	orient_constrain.orientation.x = qx;
	orient_constrain.orientation.y = qy;
	orient_constrain.orientation.z = qz;
	
	orient_constrain.absolute_x_axis_tolerance = 0.001;
	orient_constrain.absolute_y_axis_tolerance = 0.001;
	orient_constrain.absolute_z_axis_tolerance = pi;
	orient_constrain.weight = weight;

	return orient_constrain;

topic = '/move_group/motion_plan_request';
Planning_req = rospy.Publisher(topic, MotionPlanRequest);

def constraint_planner(start_robot_state, goal_config, group_handle, planner_name, planning_attemps, planning_time):
	
	planning_workspace = WorkspaceParameters();
	planning_workspace.header.frame_id = "/base_link";
	planning_workspace.header.stamp = rospy.Time.now();
	planning_workspace.min_corner.x = -1;
	planning_workspace.min_corner.y = -1;
	planning_workspace.min_corner.z = -1;	
	planning_workspace.max_corner.x = 1;
	planning_workspace.max_corner.y = 1;
	planning_workspace.max_corner.z = 1;

	# Set Start
	start_state = RobotState();
	start_state = start_robot_state;
	print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
	#print start_robot_state;
	
	# Set Goal
	goal_state = Constraints();
	Jnt_constraint = setJointConstraints(goal_config, group_handle);
	goal_state.joint_constraints = Jnt_constraint;
	#print Jnt_constraint;
	
	# Set Constraints
	des_w = -0.070099;
	des_x = 0.41382;
	des_y = -0.57302;
	des_z = 0.70391;
	rotation_constraints = setOrientationConstraints(des_w,des_x,des_y,des_z,group_handle, weight = 1.0);
	
	# Generating Request
	planningRequest = MotionPlanRequest();
	planningRequest.workspace_parameters = planning_workspace;
	
	planningRequest.start_state = start_robot_state;
	planningRequest.goal_constraints.append(goal_state);
	
	# Setting Constraint
	planningRequest.path_constraints.name = 'scoop_constraint';
	planningRequest.path_constraints.orientation_constraints.append(rotation_constraints);
	
	#traj_constraint = Constraints();
	#traj_constraint.orientation_constraints.append(rotation_constraints);
	#planningRequest.trajectory_constraints.constraints.append(traj_constraint);
	
	planningRequest.planner_id = "RRTConnectkConfigDefault";
	planningRequest.group_name = group_handle.get_name();
	planningRequest.num_planning_attempts = planning_attemps;
	planningRequest.allowed_planning_time = planning_time;
	planningRequest.max_velocity_scaling_factor = 1.0;
	
	Planning_req.publish(planningRequest);

if __name__=='__main__':
  try:
	
	print ">>>> Initializing... >>>>"
	rospy.init_node('IK_Solution_Test', anonymous=True);
	
	moveit_commander.roscpp_initialize(sys.argv);
	# Adding Bin
	scene = moveit_commander.PlanningSceneInterface();
	print ">>>> Import Bin model, Generate Testing Targets >>>>"
	if len(sys.argv)>1:
	  X_pos = float(sys.argv[1]);
	  Y_pos = float(sys.argv[2]);
	  Z_pos = float(sys.argv[3]);  
	else:
	  print "No distance assigned, using default parameters: X = ",default_Bin_X," Y = ",default_Bin_Y, " Z = ", default_Bin_Z;	  
	  #Load_Bin_model(default_Bin_X, default_Bin_Y, default_Bin_Z);
	
	print ">>>> Set Init Position >>>>"
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");	
	arm_left_group.set_planner_id("RRTConnectkConfigDefault");
	arm_left_group.allow_replanning(True);
	arm_left_group.set_goal_tolerance(0.001);

	arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso"); 
	arm_left_group.set_planner_id("RRTConnectkConfigDefault");
	arm_right_group.set_goal_tolerance(0.001);
	arm_right_group.allow_replanning(True);	
	#pos_init(arm_left_group, arm_right_group);	
	arm_left_group.set_planning_time(Planning_time);
	arm_right_group.set_planning_time(Planning_time);
	
	print ">>>> Waiting for service `compute_ik` >>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);

	print ">>>> Start Testing >>>>"
	#pos_test(arm_right_group, ik);
	motoman_sda10 = moveit_commander.RobotCommander();
	start_state = motoman_sda10.get_current_state();
	goal_state = [0.0, -0.5263604893956656, 1.6500163692522856, -0.5515519107755565, 1.1627465887780024, 2.26224630000971, -0.7075710170641819, 0.7939218847936328]
	
	#constraint_planner(start_state, goal_state, arm_right_group, "RRTConnectkConfigDefault", 15, 10);
	des_w = -0.070099;
	des_x = 0.41382;
	des_y = -0.57302;
	des_z = 0.70391;
	orient_constraint = setOrientationConstraints(des_w, des_x, des_y, des_z, arm_right_group, weight = 1.0);
	test_constraints = Constraints();
	test_constraints.orientation_constraints.append(orient_constraint);
	arm_right_group.set_path_constraints(test_constraints);
	arm_right_group.set_random_target();
	arm_right_group.plan();
	
	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
