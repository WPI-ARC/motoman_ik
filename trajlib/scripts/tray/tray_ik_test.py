
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

import moveit_commander
import moveit_msgs.msg
		
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.msg import RobotTrajectory
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tray_goal_pos_generator import generate_frontface_positions, seed_state_generator, arm_right_home, arm_left_home;

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../scripts"));
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;
default_Bin_X = X_pos;
default_Bin_Y = Y_pos;
default_Bin_Z = Z_pos;

left_arm_init_joint_value = arm_left_home;
right_arm_init_joint_value = arm_right_home;

Planning_time = 60;

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
	go_home(left_arm_group_handle);
	go_home(right_arm_group_handle);

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
	
def go_home(group_handle):
	print "Try to go back to home...";
	if group_handle.get_name() == "arm_right_torso":
		group_handle.set_joint_value_target(right_arm_init_joint_value);
	else:
		group_handle.set_joint_value_target(left_arm_init_joint_value);
	group_handle.set_planner_id("RRTConnectkConfigDefault");
	plan = group_handle.plan();
	planning_attemps = 1;
	while(len(plan.joint_trajectory.points) == 0):
		group_handle.set_random_target();
		group_handle.go();
		print "Planning Attempts:",planning_attemps;
		group_handle.set_joint_value_target(right_arm_init_joint_value);
		plan = group_handle.plan();					
	group_handle.execute(plan);
	
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
			#group_handle.set_planner_id("RRTConnectkConfigDefault");
			
			config = config_set[count];
			print ">>>> Planning Trajectory for bin",config.bin_num,"...";
			group_handle.set_start_state_to_current_state();
			group_handle.set_joint_value_target(config.jnt_val);
			Pick_plan = group_handle.plan();
			
			planning_attemps = 1;
			while(len(Pick_plan.joint_trajectory.points) == 0 and planning_attemps <= 100):
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
				go_home(group_handle);
				
				rospy.sleep(5);
			else:
				print "Planning failed!";

	else:
	  print "No target Assigned, Exit!";
	  return False;
	  
def rotation_path_generate(group_handle, horizontal_jnt_config):
	
	print ">>>> Generating IK solutions... >>>>"
	seed_state_set = seed_state_generator();
	print "Total ",len(seed_state_set),"seed state assigned";
	for count in range(0,len(seed_state_set)):		
		#group_handle.set_planner_id("RRTstarkConfigDefault");
		group_handle.set_planner_id("RRTConnectkConfigDefault");
		
		config = seed_state_set[count];
		print ">>>> Planning Trajectory for bin",config.bin_num,"...";
		group_handle.set_start_state_to_current_state();
		group_handle.set_joint_value_target(horizontal_jnt_config);
		Pick_plan = group_handle.plan();
		
		planning_attemps = 1;
		while(len(Pick_plan.joint_trajectory.points) == 0 and planning_attemps <= 100):
			print "Planning Attempts:",planning_attemps;
			Pick_plan = group_handle.plan();
			planning_attemps += 1;

		# Plan is valid, Save trajectory to file
		if len(Pick_plan.joint_trajectory.points):
			folder_name = os.path.join(os.path.dirname(__file__), "../../trajectories/bin") + config.bin_num;
			file_name = folder_name + "/"+ "Rotate";	
			Save_traj(Pick_plan, file_name);
			print "Executing trajectory...";
			group_handle.execute(Pick_plan);
			rospy.sleep(5);
			# Go back to init_pos
			go_home(group_handle);
			
			rospy.sleep(5);
		else:
			print "Planning failed!";
			
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
	  Load_Bin_model(default_Bin_X, default_Bin_Y, default_Bin_Z);
	
	print ">>>> Set Init Position >>>>"
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");	
	arm_left_group.set_planner_id("RRTConnectkConfigDefault");
	arm_left_group.allow_replanning(True);
	arm_left_group.set_goal_tolerance(0.01);

	arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso"); 
	arm_left_group.set_planner_id("RRTConnectkConfigDefault");
	arm_right_group.set_goal_tolerance(0.01);
	arm_right_group.allow_replanning(True);
	
	#pos_init(arm_left_group, arm_right_group);
	
	arm_left_group.set_planning_time(Planning_time);
	arm_right_group.set_planning_time(Planning_time);
	
	print ">>>> Waiting for service `compute_ik` >>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);

	print ">>>> Start Testing >>>>"
	#pos_test(arm_right_group, ik);
	
	# A		
	rotate_config = [2.608074188232422, -0.29658669233322144, 
					 0.8934586644172668, 1.7289633750915527, 
					 1.573803424835205, 1.2867212295532227, 
					 1.4699939489364624, 1.88583398];

	# G
	#rotate_config = [2.776894014401466,-0.639702221407827, 1.4694694444791978, -0.8327041049818699, 2.18906750147502, 2.2708616336783396, -1.232302856727945, 1.7149700732880278];
	
	rotation_path_generate(arm_right_group, rotate_config);
	
	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
