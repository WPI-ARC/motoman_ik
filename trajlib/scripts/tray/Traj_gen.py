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

global X_pos, Y_pos, Z_pos, bin_num
global arm_right_group, arm_left_group

def Save_traj(traj_property,plan):

	model_filepath = os.path.join(os.path.dirname(__file__), "../../trajectories/")

	file_name = model_filepath + "bin"+ str(bin_num) +"/"+ str(traj_property);		
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

def goal_states(bin_num):

	if( bin_num == "A"):
		print "bin A"
		jnt_val = [-2.7414562000821596, 0.35908996927034464, -1.0338458209745651, 0.9155596477651657, 1.4354311690354857, 2.0361323271088922, 1.5042305571563939, 1.7114496138561022]

	elif( bin_num == "B"):
		print "bin B"
		jnt_val = [-0.081586918733629, 1.568263068581441, -1.5600065565278145, -0.12640853623635262, 1.3818779719788765, 1.8057370642492034, -1.6627886081928707, 0.33500689702609654]
	
	elif( bin_num == "C"):
		print "bin C"
		jnt_val = [-0.00017003233637660742, 0.9476867391620163, -1.0973046218304017, 0.8007416131485688, 1.6354262550391043, 2.375895961885445, -1.3475549255271067, -0.3187735148088192]
	
	elif( bin_num == "D"):
		print "bin D"
		jnt_val = [-2.9317089196761317, -1.3891042771603888, 1.5551417996298735, -1.795130814567031, -1.959660141829467, -1.1281119250820135, -0.42715715260501547, 0.2481895577719651]
	
	elif( bin_num == "E"):
		print "bin E"
		jnt_val = [2.9569502758554647, -2.359048008419167, 1.0030676671104295, -2.9133625875265268, 2.0038547415944508, -1.202117633059331, -1.8691892376098491, -2.4821946960131354]

	elif( bin_num == "F"):
		print "bin F"
		jnt_val = [-0.1354916601858345, -1.4024311652242984, 0.6585454937449481, -0.039352751542353334, -2.2773070429260507, 1.8020177560841422, 1.5190178837225605, -2.871546413266448]

	elif( bin_num == "G"):
		print "bin G"
		jnt_val = [-2.5798535224392536, 1.0697790242434855, -1.0839641149950956, -1.2201908276215852, 2.02966228699635, 0.5189577687608469, -0.3282565335080377, 2.2993732434898195]
	
	elif( bin_num == "H"):
		print "bin H"
		jnt_val = [-2.248761887096955, 1.3522287218551485, -1.7969524759519773, -0.6772191051315353, 2.1173732752930943, -0.044749839368110944, -0.5070960571918733, 2.1482518074388928]
	
	elif( bin_num == "I"):
		print "bin I"
		jnt_val = [1.786625748813978, -2.009663767489044, 1.6610497535117468, 2.3018214865330067, 1.9959024320028638, 1.092379101763545, 1.7286319777156305, -0.9795453621545195]

	elif( bin_num == "J"):
		print "bin J"
		jnt_val = [-2.9380471248618245, 1.5273729645602547, 1.0371530902300101, 1.5252995298122611, -2.0966592360817806, -0.44308936434507046, 0.8177707416074836, 1.9581798762937488]
	
	elif( bin_num == "K"):
		print "bin K"
		jnt_val = [1.6091060643563821, -0.42601474698429737, 1.8786813516173775, -1.9590250404127998, 2.143914460770546, 2.089120054792059, -1.5320915492899372, -1.3181846257921348]
	
	elif( bin_num == "L"):
		print "bin L"
		jnt_val = [2.956911493198829, -0.13811920555100166, -1.8429633508275745, -0.3930214380483685, 1.4840606133756926, -0.4428208584099841, -1.764811157007895, -3.125611904804895]
	
	return jnt_val

def scoop_procedure(arm_right_group, bin_num):
	print "Executing Scoop..."

	scene.remove_world_object(
			name="bin");

	poses = [arm_right_group.get_current_pose().pose]

	if( bin_num == "C"):
		# BIN C
		poses[-1].position.x = 0.337514
		poses[-1].position.y = -0.196057
		poses[-1].position.z = 1.63967
		poses[-1].orientation.x = -0.293131
		poses[-1].orientation.y = -0.512971
		poses[-1].orientation.z = 0.403546
		poses[-1].orientation.w = 0.698631
	elif ( bin_num == "B"):
		# BIN B
		poses[-1].position.x = 0.337514
		poses[-1].position.y += 0.310857
		poses[-1].position.z = 1.63967
		poses[-1].orientation.x = -0.293131
		poses[-1].orientation.y = -0.512971
		poses[-1].orientation.z = 0.403546
		poses[-1].orientation.w = 0.698631
	# BIN A
	# poses[-1].position.y += 0.270857
	# BIN F
	# poses[-1].position.z = 1.39467
	poses.append(deepcopy(poses[-1]))



	start_pose = [arm_right_group.get_current_pose().pose]

	if (bin_num == "C"):
		# BIN C
		start_pose[-1].position.x = 0.337514
		start_pose[-1].position.y = -0.196057
		start_pose[-1].position.z = 1.63967
		start_pose[-1].orientation.x = -0.293131
		start_pose[-1].orientation.y = -0.512971
		start_pose[-1].orientation.z = 0.403546
		start_pose[-1].orientation.w = 0.698631
	elif (bin_num == "B"):
		# BIN B
		start_pose[-1].position.x = 0.337514
		start_pose[-1].position.y += 0.310857
		start_pose[-1].position.z = 1.63967
		start_pose[-1].orientation.x = -0.293131
		start_pose[-1].orientation.y = -0.512971
		start_pose[-1].orientation.z = 0.403546
		start_pose[-1].orientation.w = 0.698631

	# BIN A
	# start_pose[-1].position.y += 0.541714

	

	# BIN F
	# start_pose[-1].position.z = 1.39467

	if (bin_num == "B"):
		poses[-1].position.x = 0.37066
		poses[-1].position.y += 0.14335
		poses[-1].position.z -= 0.0276
		poses[-1].orientation.x = 0.19924 
		poses[-1].orientation.y = -0.69387
		poses[-1].orientation.z = -0.14743
		poses[-1].orientation.w = 0.6761
		poses.append(deepcopy(poses[-1]))

		poses[-1].position.x += 0.12
		poses[-1].position.y += 0.03
		poses.append( deepcopy( poses[-1] ) )
        
		poses[-1].position.x += 0.25
		poses[-1].position.y += 0.01
		poses.append( deepcopy( poses[-1] ) )

		poses[-1].position.y -= 0.10
		poses.append( deepcopy( poses[-1] ) )

		poses[-1].position.x -= 0.4
		poses.append(deepcopy(poses[-1]))		

	elif (bin_num == "C"):
		poses[-1].position.x = 0.37066
		poses[-1].position.y += 0.04
		poses[-1].position.z -= 0.0276
		poses[-1].orientation.x = 0.19924 
		poses[-1].orientation.y = -0.69387
		poses[-1].orientation.z = -0.14743
		poses[-1].orientation.w = 0.6761
		poses.append(deepcopy(poses[-1]))

		poses[-1].position.x += 0.12
		poses[-1].position.y -= 0.03
		poses.append( deepcopy( poses[-1] ) )
        
		poses[-1].position.x += 0.25
		poses[-1].position.y -= 0.01
		poses.append( deepcopy( poses[-1] ) )

		poses[-1].position.y += 0.08
		poses.append( deepcopy( poses[-1] ) )

		poses[-1].position.x -= 0.4
		poses.append(deepcopy(poses[-1]))	



	# poses[-1].position.x = 0.337514
	# poses[-1].position.y = -0.196057
	# poses[-1].position.z = 1.63967
	# poses[-1].orientation.x = -0.293131
	# poses[-1].orientation.y = -0.512971
	# poses[-1].orientation.z = 0.403546
	# poses[-1].orientation.w = 0.698631
	poses.append(deepcopy(start_pose[-1]))

	# Forward
	poses[-1].position.x += 0.125
	poses.append(deepcopy(poses[-1]))

	# Down
	poses[-1].position.z += -0.0355
	poses.append(deepcopy(poses[-1]))

	# Scoop In
	poses[-1].position.x += 0.174
	poses[-1].position.z += -0.0710
	poses.append(deepcopy(poses[-1]))

	#Push In
	poses[-1].position.x += 0.1523
	poses.append(deepcopy(poses[-1]))

	#Tilt Up
	poses[-1].position.z += -0.1018
	poses.append(deepcopy(poses[-1]))

	#Lift Tray Up
	poses[-1].position.x +=  0.0059
	poses[-1].position.y +=  0.0
	poses[-1].position.z += -0.0370
	poses[-1].orientation.x = -0.36665
	poses[-1].orientation.y = -0.64811
	poses[-1].orientation.z = 0.33362
	poses[-1].orientation.w = 0.57811
	# poses[-1].orientation.x = 0.548548
	# poses[-1].orientation.y = -0.546171
	# poses[-1].orientation.z = -0.446286
	# poses[-1].orientation.w = 0.449022
	poses.append(deepcopy(poses[-1]))

	#Up
	poses[-1].position.z += 0.1
	poses.append(deepcopy(poses[-1]))

	#Up and Out
	poses[-1].position.x += -0.4586
	poses[-1].position.z += 0.03
	poses.append(deepcopy(poses[-1]))

	poses[-1].position.z += 0.03
	poses.append(deepcopy(poses[-1]))


	# To right side of shelf
	poses[-1].position.y = -0.602322
	poses[-1].position.z -= 0.05
	poses[-1].orientation.x = -0.36667
	poses[-1].orientation.y = -0.648119
	poses[-1].orientation.z = 0.333549	
	poses[-1].orientation.w = 0.578135
	poses.append(deepcopy(poses[-1]))

	#To order bin
	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses[-1].orientation.x = -0.164656 #-0.198328
	poses[-1].orientation.y = 0.766477 #0.759802
	poses[-1].orientation.z = -0.591483 #-0.598499
	poses[-1].orientation.w = -0.188543 #-0.158639
	poses.append(deepcopy(poses[-1]))

	#Tilt little by little

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses[-1].orientation.x = 0.143945
	poses[-1].orientation.y = -0.605741
	poses[-1].orientation.z = 0.757694
	poses[-1].orientation.w = 0.195594
	poses.append(deepcopy(poses[-1]))

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses[-1].orientation.x = 0.113945
	poses[-1].orientation.y = -0.525741
	poses[-1].orientation.z = 0.827694
	poses[-1].orientation.w = 0.215594
	poses.append(deepcopy(poses[-1]))

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses[-1].orientation.x = 0.112873
	poses[-1].orientation.y = -0.520793
	poses[-1].orientation.z = 0.819904
	poses[-1].orientation.w = 0.209268
	poses.append(deepcopy(poses[-1]))

	print "Planning Cartesian Path Dump....."

	(plan, fraction) = arm_right_group.compute_cartesian_path(
                             poses,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
	
	print "Executing Cartesian Path Dump....."
	arm_right_group.execute(plan)	
	Save_traj("Dump",plan)

	rospy.sleep(25)

	#Lift tray up
	poses1 = [arm_right_group.get_current_pose().pose]

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses1[-1].orientation.x = 0.112873
	poses1[-1].orientation.y = -0.520793
	poses1[-1].orientation.z = 0.819904
	poses1[-1].orientation.w = 0.209268
	poses1.append(deepcopy(poses1[-1]))

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses1[-1].orientation.x = 0.113945
	poses1[-1].orientation.y = -0.525741
	poses1[-1].orientation.z = 0.827694
	poses1[-1].orientation.w = 0.215594
	poses1.append(deepcopy(poses1[-1]))

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses1[-1].orientation.x = 0.143945
	poses1[-1].orientation.y = -0.605741
	poses1[-1].orientation.z = 0.757694
	poses1[-1].orientation.w = 0.195594
	poses1.append(deepcopy(poses1[-1]))

	poses[-1].position.x = 0.472985 #0.482178
	poses[-1].position.y = -0.351667 #-0.335627
	poses[-1].position.z = 0.753171 #0.706449
	poses1[-1].orientation.x = -0.179646
	poses1[-1].orientation.y = 0.685765
	poses1[-1].orientation.z = -0.68407
	poses1[-1].orientation.w = -0.17176
	poses1.append(deepcopy(poses1[-1]))

	poses1[-1].position.x = 0.30
	poses1[-1].position.y = -0.602322
	poses1[-1].position.z = 0.756449
	poses1[-1].orientation.x = -0.36667
	poses1[-1].orientation.y = -0.648119
	poses1[-1].orientation.z = 0.333549	
	poses1[-1].orientation.w = 0.578135
	poses1.append(deepcopy(poses1[-1]))

	print "Planning Cartesian Path for Lift....."

	(plan1, fraction) = arm_right_group.compute_cartesian_path(
                             poses1,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
	
	print "Executing Cartesian Path for Lift....."
	arm_right_group.execute(plan1)	
	Save_traj("Lift",plan1)
	
	rospy.sleep(10)
	add_collision_box()

	arm_right_group.set_planner_id("RRTstarkConfigDefault")
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
			bin_num = str(sys.argv[1])
			X_pos = 1.40
			Y_pos = 0
			Z_pos = 0
			print "Bin number " + str(bin_num)
			print "Default Shelf Position"
		elif len(sys.argv)>2:
			bin_num = str(sys.argv[1])
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
		arm_right_group.set_planner_id("RRTConnectkConfigDefault")
		# arm_right_group.set_planning_time(15)

		arm_left_group = moveit_commander.MoveGroupCommander("arm_left_torso")
		arm_left_group.set_planner_id("RRTConnectkConfigDefault")
		# arm_left_group.set_planning_time(15)

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
		
		model_filepath = os.path.join(os.path.dirname(__file__), "../../../apc_models/meshes/pod_lowres.stl")
			
		scene.attach_mesh(link = "base_link",
						  name = "kiva_pod", 
						  pose = bin_pose,
						  filename = model_filepath);

		add_collision_box()
		add_dropOff()

		print ">>>>>>>>>> Testing <<<<<<<<<<"

		scoop_procedure(arm_right_group,bin_num)

		print "********** Test End **********"
		moveit_commander.roscpp_shutdown()

	except rospy.ROSInterruptException:
		pass

