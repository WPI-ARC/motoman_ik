#!/usr/bin/env python

import os
import sys
import copy
import rospy
import StringIO
import moveit_msgs.msg
import moveit_commander

from std_msgs.msg import String, Header, Int64
from StringIO import StringIO

from trajlib.srv import *
from trajectory_verifier.srv import CheckTrajectoryValidity
from trajectory_verifier.msg import CheckTrajectoryValidityQuery, CheckTrajectoryValidityResult

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest
from trajectory_msgs.msg import JointTrajectoryPoint

from apc_util.collision import attach_sphere, remove_object
from geometry_msgs.msg import PoseStamped

from motoman_configuration import arm_left_home, arm_right_home;
from bin_loader import Load_Bin_model, X_pos, Y_pos, Z_pos;

from apc_util.shelf import bin_pose, add_shelf, remove_shelf, Shelf, get_shelf_pose

check_collision_server = rospy.ServiceProxy("/check_trajectory_validity", CheckTrajectoryValidity);

def check_collisions(query):
    for i in range(5):
        try:
            result = check_collision_server(query)
            return result, True
        except rospy.ServiceException as e:
            rospy.logwarn("Failure with check_collisions(<<query>>): %s" % (str(e)))
    rospy.logerr("Failed to check collisions")
    return None, False

robot = moveit_commander.RobotCommander();

def check_trajectory_validity(plan):
	collisions, success = check_collisions(CheckTrajectoryValidityQuery(initial_state=JointState(header=Header(stamp=rospy.Time.now()),
																								 name=robot.sda10f.get_joints(),
																								 position=robot.sda10f.get_current_joint_values()),
																		trajectory=plan.joint_trajectory,
																		check_type=CheckTrajectoryValidityQuery.CHECK_ENVIRONMENT_COLLISION,));
	if not success:
		return False

	if collisions.result.status != CheckTrajectoryValidityResult.SUCCESS:
		rospy.logerr("Can't execute path from trajectory library, status=%s" % collisions.result.status)
		rospy.loginfo("Planning path to drop")
		return False;
		
	else:
		return True;

def move_to_start_pos(plan, arm_group):
	first_point = plan.joint_trajectory.points[0];
	init_goal_joint_val = []
	for jnt_val in first_point.positions:
		init_goal_joint_val.append(jnt_val);
		
	print init_goal_joint_val;
	arm_group.set_planner_id("RRTConnectkConfigDefault");
	arm_group.set_joint_value_target(init_goal_joint_val);
	
	plan = arm_group.plan();
	arm_group.execute(plan);


def runTrajectory(req):

	print "---------------------------------"
	print req.task
	print " "
	print req.bin_num
	print " "
	print req.using_torso
	print "---------------------------------"

	# Get the trajectory
	file_root = os.path.join(os.path.dirname(__file__), "../trajectories");
	#file_root = os.path.join(os.path.dirname(__file__), "../traj_updates");
	traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	plan = RobotTrajectory();
	
# Gripper Hand
	if req.task == "use_tray":
		file_name = file_root + "/left_arm_drop2home";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");
	elif req.task == "use_gripper":
		file_name = file_root + "/left_arm_home2drop";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");
	elif req.task == "Forward":
		file_name = file_root+"/bin" + req.bin_num +"/forward";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");
	elif req.task == "Drop":
		file_name = file_root+"/bin" + req.bin_num+"/drop";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");
	elif req.task == "Scan":
		file_name = file_root+"/bin" + req.bin_num+"/scan";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_left_torso");

# Tray Hand
	elif req.task == "Pick":
		file_name = file_root+"/bin" + req.bin_num+"/Pick";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	elif req.task == "Dump":
		file_name = file_root+"/bin" + req.bin_num+"/Dump";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	elif req.task == "Lift":
		file_name = file_root+"/bin" + req.bin_num+"/Lift";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	elif req.task == "Home":
		file_name = file_root+"/bin" + req.bin_num+"/Home";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	elif req.task == "Rotate":
		file_name = file_root+"/bin" + req.bin_num+"/Rotate";
		traj_execute_group = moveit_commander.MoveGroupCommander("arm_right_torso");
	else :
		print "No plan selected, exit!";
		return GetTrajectoryResponse(plan,True);

	f = open(file_name,"r");
	buf = f.read();
	plan_file = RobotTrajectory();
	plan_file.deserialize(buf);    
	plan = copy.deepcopy(plan_file);
	
	move_to_start_pos(plan, traj_execute_group);
	
	if check_trajectory_validity(plan):
		print "Trajectory is valid";
		# Display Current Trajectory
		robot = moveit_commander.RobotCommander();
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
		display_trajectory = moveit_msgs.msg.DisplayTrajectory();
		display_trajectory.trajectory_start = robot.get_current_state()
		display_trajectory.trajectory.append(plan)
		display_trajectory_publisher.publish(display_trajectory);

		print "============ Waiting while", file_name, " is visualized (again)..."
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
		#attach_sphere("arm_left_link_7_t", "Object", pose, 0.17, ["hand_left_finger_1_link_2", "hand_left_finger_1_link_3", "hand_left_finger_1_link_3_tip", "hand_left_finger_2_link_2", "hand_left_finger_2_link_3", "hand_left_finger_2_link_3_tip", "hand_left_finger_middle_link_2", "hand_left_finger_middle_link_3", "hand_left_finger_middle_link_3_tip"]);
		traj_execute_group.execute(plan);
		print "Trajectory ", file_name, " finished!"
		f.close();
		remove_object();
		return GetTrajectoryResponse(plan,True);
		
	else:
		print "Trajectory is not valid! Test failed!";
		return GetTrajectoryResponse(plan,True);

def pos_init():
	
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso");	
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso");

	arm_left_group.set_start_state_to_current_state();
	arm_left_group.go(arm_left_home);
	
	arm_right_group.set_start_state_to_current_state();
	arm_right_group.go(arm_right_home);

def Start_server():

    rospy.init_node('trajectory_service');
    s = rospy.Service('trajectory_display', GetTrajectory, runTrajectory);	
    #Load_Bin_model(X_pos, Y_pos, Z_pos);
    add_shelf();
    check_collision_server.wait_for_service(0.5);
    # Might be a problem
    pos_init();    
    print " >>>>>>>>>>>>>> Waiting For trajectory request... >>>>>>>>>>>>>>>>>>>";
    print " Request format:";
    print " Task: <Forward/Drop> + BinCode: < A ... L > + UsingTorso < y/n >";
    print " Default: Forward + A + n";
    rospy.spin()

if __name__ == '__main__':
    Start_server()
