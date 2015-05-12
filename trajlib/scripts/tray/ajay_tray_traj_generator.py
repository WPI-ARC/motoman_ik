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

from sensor_msgs.msg import JointState
from goal_pos_generate import generate_goal_points


if __name__ == '__main__':
	try:

		if len(sys.argv)>1:
			X_pos = float(sys.argv[1])
			Y_pos = float(sys.argv[2])
			Z_pos = float(sys.argv[3])
		else:
			print "No distance assigned, using default parameters"
			X_pos = 1.32
			Y_pos = 0
			Z_pos = 0

		goal_points = generate_goal_points( Shelf_x = X_pos, Shelf_y = Y_pos, Shelf_z = Z_pos)

		print ">>>>>>>>>> Initializing... <<<<<<<<<<"
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('IK_test', anonymous = True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		arm_right_group = moveit_commander.MoveGroupCommander("arm_right")
		arm_right_group.set_planner_id("RRTstarkConfigDefault")
		arm_right_group.set_planning_time(30)

		print ">>>>>>>>>> Waiting for service 'compute_ik' <<<<<<<<<<"
		rospy.wait_for_service('compute_ik')
		ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

		goal_points = generate_goal_points(X_pos,Y_pos,Z_pos)

		print ">>>>>>>>>> Testing <<<<<<<<<<"
		path = []

		start = geometry_msgs.msg.Pose()
		start.position.x =  0.402135
		start.position.y =  0.156092
		start.position.z =  0.592701
		start.orientation.x =  0.689549
		start.orientation.y =  -0.141975
		start.orientation.z = 0.137415
		start.orientation.w =  0.696766

		mid = geometry_msgs.msg.Pose()
		mid.position.x =  0.40
		mid.position.y =  0
		mid.position.z =  1.3985
		mid.orientation.x =  0.689549
		mid.orientation.y =  -0.141975
		mid.orientation.z = 0.137415
		mid.orientation.w =  0.696766


		mid1 = geometry_msgs.msg.Pose()
		mid1.position.x =  0.40
		mid1.position.y =  0.275
		mid1.position.z =  1.3985
		mid1.orientation.x =  0.689549
		mid1.orientation.y =  -0.141975
		mid1.orientation.z = 0.137415
		mid1.orientation.w =  0.696766

		# mid2 = geometry_msgs.msg.Pose()
		# mid2.position.x =  0.40	
		# mid2.position.y =  0.0
		# mid2.position.z =  1.3985
		# mid2.orientation.x =  0.689549
		# mid2.orientation.y =  -0.141975
		# mid2.orientation.z = 0.137415
		# mid2.orientation.w =  0.696766

		mid21 = geometry_msgs.msg.Pose()
		mid21.position.x =  0.40	
		mid21.position.y =  0.0
		mid21.position.z =  1.3985
		mid21.orientation.x =  0.704678
		mid21.orientation.y =  0.00853603
		mid21.orientation.z = -0.0223218
		mid21.orientation.w =  0.709124

		mid3 = geometry_msgs.msg.Pose()
		mid3.position.x =  0.83
		mid3.position.y =  0.0
		mid3.position.z =  1.3985
		mid3.orientation.x =  0.704678
		mid3.orientation.y =  0.00853603
		mid3.orientation.z = -0.0223218
		mid3.orientation.w =  0.709124

		# mid3 = geometry_msgs.msg.Pose()
		# mid3.position.x =  0.29
		# mid3.position.y =  0.275
		# mid3.position.z =  1.6435
		# mid3.orientation.x =  0.704678
		# mid3.orientation.y =  0.00853603
		# mid3.orientation.z = -0.0223218
		# mid3.orientation.w =  0.709124

#		mid = geometry_msgs.msg.Pose()
#		mid.position.x =  0.245513
#		mid.position.y = -0.434602
#		mid.position.z =  0.568589
#		mid.orientation.x =  0.490285
#		mid.orientation.y = -0.492092
#		mid.orientation.z = -0.515267
#		mid.orientation.w =  0.501962

#		end = geometry_msgs.msg.Pose()
#		end.position.x =  0.29
#		end.position.y =  0
#		end.position.z =  1.6435
#		end.orientation.x =  0.701623
#		end.orientation.y =  0.00404495
#		end.orientation.z = -0.00479594
#		end.orientation.w =  0.712521

		path.append(copy.deepcopy(start))
		# path.append(copy.deepcopy(mid))
		path.append(copy.deepcopy(mid1))
		# path.append(copy.deepcopy(mid2))
		path.append(copy.deepcopy(mid21))
		path.append(copy.deepcopy(mid3))
#		path.append(copy.deepcopy(end))

		# arm_right_group.set_start_state_to_current_state ()

		(plan, fraction) = arm_right_group.compute_cartesian_path(
                             path,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

		print "============ Waiting while RVIZ displays plan..."
		rospy.sleep(5)

		#arm_right_group.go(wait=True)
		#rospy.sleep(5)

		print "********** Test End **********"
		moveit_commander.roscpp_shutdown()

	except rospy.ROSInterruptException:
		pass