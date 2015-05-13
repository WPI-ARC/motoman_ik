import os
import sys
import copy
import rospy
import StringIO
from copy import deepcopy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int64
from StringIO import StringIO

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState, Constraints, OrientationConstraint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from sensor_msgs.msg import JointState
from waypoint_gen import generate_waypoints

global X_pos, Y_pos, Z_pos, bin_num
global arm_right_group, arm_left_group

def Save_traj(traj_property,plan):
	file_name = "Traj/bin_"+ str(bin_num) +"/"+ str(traj_property);		
	print "saving bin.",bin_num,"trajectory to file",file_name;
	buf = StringIO();
	plan.serialize(buf);				
	f = open(file_name,"w");
	f.write(buf.getvalue());
	f.close();

def plan_right_to_home():
	print "Moving Right arm to Home...."
	group_variable_values = arm_right_group.get_current_joint_values()
	group_variable_values[0] =  0.0
	group_variable_values[1] = -1.0030564513590334
	group_variable_values[2] = -1.49978651413566
	group_variable_values[3] =  0.457500317369117
	group_variable_values[4] = -2.1772162870743323
	group_variable_values[5] =  0.4509681667487428
	group_variable_values[6] = -1.2043397683221861
	group_variable_values[7] = -1.5581499385881046
	arm_right_group.set_joint_value_target(group_variable_values)

	pl1 = arm_right_group.plan()
	arm_right_group.execute(pl1)
	rospy.sleep(5)

	return pl1

def plan_left_to_home():	
	print "Moving Left arm to Home...."
	group_variable_values = arm_left_group.get_current_joint_values()
	group_variable_values[0] =  0.0
	group_variable_values[1] =  3.1161585602193242 #2.1987425554414965
	group_variable_values[2] =  1.5503676152322932 #-1.2856041261915816
	group_variable_values[3] = -1.6412882586977695 #-2.0572122610542043
	group_variable_values[4] = -1.4956238594895404 #-2.1844569809370875
	group_variable_values[5] = -3.129999281606637 #-0.8355313237647156
	group_variable_values[6] =  1.6841302015402857 #-0.6598546685936744
	group_variable_values[7] = -1.695842674210043 #2.244587644488423
	arm_left_group.set_joint_value_target(group_variable_values)

	pl2 = arm_left_group.plan()
	arm_left_group.execute(pl2)
	rospy.sleep(5)

	return pl2

def scoop_procedure():
	print "Executing Scoop..."

	scene.remove_world_object(
			name="bin");

	poses = [arm_right_group.get_current_pose().pose]

	# Forward
	poses[-1].position.x += 0.125
	poses.append(deepcopy(poses[-1]))

	# Down
	poses[-1].position.z += -0.0555
	poses.append(deepcopy(poses[-1]))

	# Scoop In
	poses[-1].position.x += 0.174
	poses[-1].position.z += -0.0810
	poses.append(deepcopy(poses[-1]))

	#Push In
	poses[-1].position.x += 0.1323
	poses.append(deepcopy(poses[-1]))

	#Tilt Up
	poses[-1].position.z += -0.1018
	poses.append(deepcopy(poses[-1]))

	#Lift Tray Up
	poses[-1].position.x +=  0.0059
	poses[-1].position.y +=  0.0155
	poses[-1].position.z += -0.0370
	poses[-1].orientation.x = -0.36665
	poses[-1].orientation.y = -0.64811
	poses[-1].orientation.z = 0.33362
	poses[-1].orientation.w = 0.57811
	poses.append(deepcopy(poses[-1]))

	#Up
	poses[-1].position.z += 0.1
	poses.append(deepcopy(poses[-1]))

	#Up and Out
	poses[-1].position.x += -0.4086
	poses[-1].position.z += 0.05
	poses.append(deepcopy(poses[-1]))

	#Go to Order bin

	poses[-1].position.x = 0.44726
	poses[-1].position.y = -0.373801
	poses[-1].position.z = 0.666238
	# poses[-1].orientation.x = -0.165242
	# poses[-1].orientation.y = 0.750189
	# poses[-1].orientation.z = -0.604833
	# poses[-1].orientation.w = -0.209972
	# poses.append(deepcopy(poses[-1]))

	# poses[-1].position.x = 0.327324
	# poses[-1].position.y = -0.446507
	# poses[-1].position.z = 0.62792
	poses[-1].orientation.x = -0.165242
	poses[-1].orientation.y = 0.750189
	poses[-1].orientation.z = -0.604833
	poses[-1].orientation.w = -0.209972
	poses.append(deepcopy(poses[-1]))

	#Dump Item
	poses[-1].position.x = 0.460695
	poses[-1].position.y = -0.363952
	poses[-1].position.z = 0.619981
	poses[-1].orientation.x =  0.11222
	poses[-1].orientation.y = -0.48567
	poses[-1].orientation.z =  0.840624
	poses[-1].orientation.w = 0.211856
	poses.append(deepcopy(poses[-1]))

	print "Planning Cartesian Path Dump....."

	(plan, fraction) = arm_right_group.compute_cartesian_path(
                             poses,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
	
	print "Executing Cartesian Path Dump....."
	arm_right_group.execute(plan)	
	Save_traj("Dump",plan)

	poses = [arm_right_group.get_current_pose().pose]

	#Lift tray up
	poses[-1].position.x = 0.37726
	poses[-1].position.y = -0.373801
	poses[-1].position.z = 0.80
	poses[-1].orientation.x = -0.165242
	poses[-1].orientation.y = 0.750189
	poses[-1].orientation.z = -0.604833
	poses[-1].orientation.w = -0.209972
	poses.append(deepcopy(poses[-1]))

	print "Planning Cartesian Path for Lift....."

	(plan1, fraction) = arm_right_group.compute_cartesian_path(
                             poses,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
	
	print "Executing Cartesian Path for Lift....."
	arm_right_group.execute(plan1)	
	Save_traj("Lift",plan1)
	
	rospy.sleep(10)
	add_collision_box()

	arm_right_group.set_planning_time(120)
	plan2 = plan_right_to_home()
	Save_traj("Home",plan2)

def add_dropOff():
	pose1 = PoseStamped()
	pose1.header.frame_id = "/base_link"
	pose1.header.stamp = rospy.Time.now()
	pose1.pose.position.x = 0.5 - 0.1905
	pose1.pose.position.y = 0.20
	pose1.pose.position.z = 0.2667
	pose1.pose.orientation.x = 0
	pose1.pose.orientation.y = 0
	pose1.pose.orientation.z = 0
	pose1.pose.orientation.w = 1
	scene.add_box(name="wall_1",
					pose=pose1,
					size=(0.02,0.635,0.533))

	pose2 = PoseStamped()
	pose2.header.frame_id = "/base_link"
	pose2.header.stamp = rospy.Time.now()
	pose2.pose.position.x = 0.5 + 0.1905
	pose2.pose.position.y = 0.20
	pose2.pose.position.z = 0.2667
	pose2.pose.orientation.x = 0
	pose2.pose.orientation.y = 0
	pose2.pose.orientation.z = 0
	pose2.pose.orientation.w = 1
	scene.add_box(name="wall_2",
					pose=pose2,
					size=(0.02,0.635,0.533))

	pose3 = PoseStamped()
	pose3.header.frame_id = "/base_link"
	pose3.header.stamp = rospy.Time.now()
	pose3.pose.position.x = 0.5
	pose3.pose.position.y = 0.20 - 0.3125
	pose3.pose.position.z = 0.2667
	pose3.pose.orientation.x = 0
	pose3.pose.orientation.y = 0
	pose3.pose.orientation.z = 0
	pose3.pose.orientation.w = 1
	scene.add_box(name="wall_3",
					pose=pose3,
					size=(0.381,0.02,0.533))

	pose4 = PoseStamped()
	pose4.header.frame_id = "/base_link"
	pose4.header.stamp = rospy.Time.now()
	pose4.pose.position.x = 0.5 
	pose4.pose.position.y = 0.20 + 0.3125
	pose4.pose.position.z = 0.2667
	pose4.pose.orientation.x = 0
	pose4.pose.orientation.y = 0
	pose4.pose.orientation.z = 0
	pose4.pose.orientation.w = 1
	scene.add_box(name="wall_4",
					pose=pose4,
					size=(0.381,0.02,0.533))





def add_collision_box():
	box_pose = PoseStamped()
	box_pose.header.frame_id = "/base_link"
	box_pose.header.stamp = rospy.Time.now()
	box_pose.pose.position.x = X_pos
	box_pose.pose.position.y = Y_pos
	box_pose.pose.position.z = Z_pos + 0.835
	box_pose.pose.orientation.x = 0
	box_pose.pose.orientation.y = 0
	box_pose.pose.orientation.z = 0
	box_pose.pose.orientation.w = 1
	scene.add_box(
		name="bin",
		pose=box_pose,
		size=(0.86, 0.8, 1.78));

if __name__ == '__main__':
	try:

		if len(sys.argv) <= 2:
			bin_num = int(sys.argv[1])
			X_pos = 1.40
			Y_pos = 0
			Z_pos = 0
			print "Bin number " + str(bin_num)
			print "Default Shelf Position"
		elif len(sys.argv)>2:
			bin_num = int(sys.argv[1])
			X_pos = float(sys.argv[2])
			Y_pos = float(sys.argv[3])
			Z_pos = float(sys.argv[4])
		else:
			print "No distance assigned, using default parameters"
			bin_num = 12
			X_pos = 1.40
			Y_pos = 0
			Z_pos = 0

		print ">>>>>>>>>> Initializing... <<<<<<<<<<"
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('IK_test', anonymous = True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

		arm_right_group = moveit_commander.MoveGroupCommander("arm_right_torso")
		arm_right_group.set_planner_id("RRTstarkConfigDefault")
		arm_right_group.set_planning_time(15)

		arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso")
		arm_left_group.set_planner_id("RRTstarkConfigDefault")
		arm_left_group.set_planning_time(15)

		print ">>>>>>>>>> Waiting for service 'compute_ik' <<<<<<<<<<"
		rospy.wait_for_service('compute_ik')
		ik = rospy.ServiceProxy("compute_ik", GetPositionIK)
		

		print ">>>>>>>>>> Loading Shelf model <<<<<<<<<<"
		bin_pose = PoseStamped()
		bin_pose.pose.position.x = X_pos;
		bin_pose.pose.position.y = Y_pos;
		bin_pose.pose.position.z = Z_pos;
		
		bin_pose.pose.orientation.x = 0.5;
		bin_pose.pose.orientation.y = 0.5;
		bin_pose.pose.orientation.z = 0.5;
		bin_pose.pose.orientation.w = 0.5;
		
		model_filepath = os.path.join(os.path.dirname(__file__), "Model/shelf.stl")
			
		scene.attach_mesh(link = "base_link",
						  name = "kiva_pod", 
						  pose = bin_pose,
						  filename = model_filepath);

		add_collision_box()
		add_dropOff()

		# arm_right_init = [0.0, -1.0030564513590334, -1.49978651413566, 0.457500317369117, -2.1772162870743323, 0.4509681667487428, -1.2043397683221861, -1.5581499385881046];

		p = plan_left_to_home()
		p1 = plan_right_to_home() 

		group_variable_values = arm_right_group.get_current_joint_values()
		group_variable_values[0] =  0.0
		group_variable_values[1] = -1.557
		group_variable_values[2] =  1.032 
		group_variable_values[3] =  0.0
		group_variable_values[4] = -1.448 
		group_variable_values[5] = -1.238
		group_variable_values[6] = -1.808 
		group_variable_values[7] = -0.087
		arm_right_group.set_joint_value_target(group_variable_values)

		plan1 = arm_right_group.plan()
		arm_right_group.execute(plan1)
		rospy.sleep(5)

		print ">>>>>>>>>> Testing <<<<<<<<<<"

		scoop_procedure()

		# path = [] 
		# path = generate_waypoints( X_pos, Y_pos, Z_pos , bin_num )
		# print path
		
		# (plan, fraction) = arm_right_group.compute_cartesian_path(
  #                            path,   # waypoints to follow
  #                            0.01,        # eef_step
  #                            0.0)         # jump_threshold

		# arm_right_group.execute(plan)
		
		# Save_traj(bin_num,"Pick",plan)

		# print "============ Waiting while RVIZ displays plan1..."
		# rospy.sleep(10)

		# plan1 = copy.deepcopy(plan)

		# print "******************************"
		# a = len(plan.joint_trajectory.points)

		# for i in range(1,a+1):
		# 	plan1.joint_trajectory.points[a-i].positions = copy.deepcopy(plan.joint_trajectory.points[i-1].positions)

		# print "============ Waiting while RVIZ displays plan1..."
		
		# display_trajectory = moveit_msgs.msg.DisplayTrajectory()

		# display_trajectory.trajectory_start = robot.get_current_state()
		# display_trajectory.trajectory.append(plan1)
		# display_trajectory_publisher.publish(display_trajectory);

		# Save_traj(bin_num,"Dump",plan1)

		# rospy.sleep(10)

		# arm_right_group.execute(plan1)

		print "********** Test End **********"
		moveit_commander.roscpp_shutdown()

	except rospy.ROSInterruptException:
		pass