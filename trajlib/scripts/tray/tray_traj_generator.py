import os
import sys
import copy
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState

from tray_goal_pos_generator import start_pnt;
from tray_goal_pos_generator import generate_frontface_positions, generate_key_positions;

def pos_init(left_arm_group_handle, right_arm_group_handle):
	left_arm_init_joint_value = [0.0, 2.193856910816974, 1.1630675621113802, 0.852437058039672, 1.113211995331904, 0.8438088567310283, -1.0094747189949542, 0.24438780016629988];
	right_arm_init_joint_value = [0.0, -0.6780398983363556, -0.5308824828312927, 2.2170419900015057, -1.1857632029913627, -1.9757495514037438, 1.6338609473054155, -1.522792050619864];	

	right_arm_group_handle.set_start_state_to_current_state();	
	right_arm_group_handle.go(right_arm_init_joint_value);
	
	left_arm_group_handle.set_start_state_to_current_state();	
	left_arm_group_handle.go(left_arm_init_joint_value);
# Utility function

def Add_bin_model(bin_x, bin_y, bin_z):
	bin_pose = PoseStamped();
	bin_pose.pose.position.x = bin_x;
	bin_pose.pose.position.y = bin_y;
	bin_pose.pose.position.z = bin_z;
	bin_pose.pose.orientation.x = 0.5;
	bin_pose.pose.orientation.y = 0.5;
	bin_pose.pose.orientation.z = 0.5;
	bin_pose.pose.orientation.w = 0.5;
	scene = moveit_commander.PlanningSceneInterface();
	scene.attach_mesh( link =  "base_link",
						name = "kiva_pod",
						pose =  bin_pose,
						filename = os.path.join(os.path.dirname(__file__), "../../../apc_models/meshes/pod_lowres.stl"))

topic = '/visualization_marker';
marker_publisher = rospy.Publisher(topic, Marker);
def Draw_GoalPnt(goal_pnts , color):
	pnt_color = [];
	
	if color == "green":
		pnt_color.append(0);
		pnt_color.append(1);
		pnt_color.append(0);
	elif color == "red":
		pnt_color.append(1);
		pnt_color.append(0);
		pnt_color.append(0);
	elif color == "blue":
		pnt_color.append(0);
		pnt_color.append(0);
		pnt_color.append(1);
	else:
		pnt_color.append(1);
		pnt_color.append(1);
		pnt_color.append(0);
		
		
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
		marker.color.r = pnt_color[0]
		marker.color.g = pnt_color[1]
		marker.color.b = pnt_color[2]
		
		showpnt = Point();
		showpnt.x = goal.x;
		showpnt.y = goal.y;
		showpnt.z = goal.z;
		marker.points.append(showpnt);
		
	marker_publisher.publish(marker);

def generate_pos_pnt(pnt):
	test_pose = geometry_msgs.msg.Pose();
	test_pose.position.x = pnt.x;
	test_pose.position.y = pnt.y;
	test_pose.position.z = pnt.z;
	test_pose.orientation.x = pnt.qx;
	test_pose.orientation.y = pnt.qy;
	test_pose.orientation.z = pnt.qz;
	test_pose.orientation.w = pnt.qw;
	return test_pose;

def Generate_Cartesian_path_waypoint_set(key_pnt_set, test_pnt_set):
	test_path_set = [];
	
	level1_start_pnt = key_pnt_set[3];
	level1_test_pnt = test_pnt_set[9:12];
	
	for test_pnt in level1_test_pnt:
		path = [];
		path.append(generate_pos_pnt(start_pnt));
		path.append(generate_pos_pnt(level1_start_pnt));
		path.append(generate_pos_pnt(test_pnt));
		path.append(generate_pos_pnt(level1_start_pnt));
		path.append(generate_pos_pnt(start_pnt));		
		test_path_set.append(path);
		
	level2_start_pnt = key_pnt_set[2];
	level2_test_pnt = test_pnt_set[6:9];
	
	for test_pnt in level2_test_pnt:
		path = [];
		path.append(generate_pos_pnt(start_pnt));
		path.append(generate_pos_pnt(level2_start_pnt));
		path.append(generate_pos_pnt(test_pnt));
		path.append(generate_pos_pnt(level2_start_pnt));
		path.append(generate_pos_pnt(start_pnt));		
		test_path_set.append(path);
	
	level3_start_pnt = key_pnt_set[1];
	level3_test_pnt = test_pnt_set[3:6];
	
	for test_pnt in level3_test_pnt:
		path = [];
		path.append(generate_pos_pnt(start_pnt));
		path.append(generate_pos_pnt(level3_start_pnt));
		path.append(generate_pos_pnt(test_pnt));
		path.append(generate_pos_pnt(level3_start_pnt));
		path.append(generate_pos_pnt(start_pnt));		
		test_path_set.append(path);
	
	level4_start_pnt = key_pnt_set[0];
	level4_test_pnt = test_pnt_set[0:3];
	
	for test_pnt in level4_test_pnt:
		path = [];
		path.append(generate_pos_pnt(start_pnt));
		path.append(generate_pos_pnt(level4_start_pnt));
		path.append(generate_pos_pnt(test_pnt));
		path.append(generate_pos_pnt(level4_start_pnt));
		path.append(generate_pos_pnt(start_pnt));		
		test_path_set.append(path);		

	return test_path_set;
	
if __name__ == '__main__':
	
	try:
		print ">>>>>>>>>> Initializing... <<<<<<<<<<"
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('IK_test', anonymous = True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		print ">>>> Set Init Position >>>>"
		arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");	
		arm_left_group.set_planner_id("RRTstarkConfigDefault");
		arm_left_group.allow_replanning(True);
		arm_left_group.set_goal_tolerance(0.001);
		arm_left_group.set_planning_time(5);

		arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso"); 
		arm_right_group.set_planner_id("RRTstarkConfigDefault");
		arm_right_group.set_goal_tolerance(0.001);
		arm_right_group.allow_replanning(True);
		arm_right_group.set_planning_time(5);
		
		pos_init(arm_left_group, arm_right_group);
		
		if len(sys.argv)>1:
			X_pos = float(sys.argv[1])
			Y_pos = float(sys.argv[2])
			Z_pos = float(sys.argv[3])
		else:
			print "No distance assigned, using default parameters to add bin"
			X_pos = 1.40
			Y_pos = 0.08
			Z_pos = 0
			
		Add_bin_model(bin_x = X_pos,bin_y = Y_pos,bin_z = Z_pos);
		
		print "Generating front positions..."
		
		front_positions = generate_frontface_positions( X_pos, Y_pos, Z_pos)
		Draw_GoalPnt(front_positions, "green");
		
		key_positions = generate_key_positions(X_pos, Y_pos, Z_pos);
		Draw_GoalPnt(key_positions, "yellow");

		print ">>>>>>>>>> Generating Cartisian Path... "
		
		test_path_set = Generate_Cartesian_path_waypoint_set(key_positions, front_positions);
		
		for test_path in test_path_set:
			#print "Planning for bin:", test_path[1].pnt_property;
			print "Test Pnt:", test_path[2].position.x,test_path[2].position.y,test_path[2].position.z
			(plan, fraction) = arm_right_group.compute_cartesian_path(test_path, 0.01, 0.0);
			if len(plan.joint_trajectory.points):
				print "============ Waiting while RVIZ displays plan..."
				arm_right_group.execute(plan);
			else:
				print "Plan failed";
			rospy.sleep(5)
		
		print "********** Test End **********"
		moveit_commander.roscpp_shutdown()

	except rospy.ROSInterruptException:
		pass
