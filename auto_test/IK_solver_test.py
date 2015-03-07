
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


def Get_current_state(group):
	return	JointState(
        name=group.get_joints()[:7],
        position=group.get_current_joint_values(),
    );
    
def find_IK_solution(ik, target, seed, group_name):
    response = ik( GetPositionIKRequest(ik_request = PositionIKRequest( group_name = group_name,
																		pose_stamped = PoseStamped( header = Header(frame_id=""),
																									pose = target),
																		robot_state = RobotState(joint_state=seed))
										) )	 
    return response
    
def Bin_number(count):
	
	if count<4:
		return 1;
		
	elif count<7 and count >3:
		return 2;
		
	elif count<10 and count >6:
		return 3;
		
	elif count<13 and count >9:
		return 4;
		
	elif count<16 and count >12:
		return 5;
		
	elif count<19 and count >15:
		return 6;
		
	elif count<22 and count >18:
		return 7;
		
	elif count<25 and count >19:
		return 8;
		
	elif count<28 and count >24:
		return 9;
		
	elif count<31 and count >27:
		return 10;
		
	elif count<34 and count >30:
		return 11;
	else:
		return 12;
		
def Copy_joint_value(group_name, joint_values):
	count = 0;
	Target_joint_value = [];
	for value in joint_values:	
		if group_name == "arm_left":
			if  count < 8:
				Target_joint_value.append(copy.deepcopy(value));
		elif group_name == "arm_right":
			if count > 19 and count < 27:
				Target_joint_value.append(copy.deepcopy(value));			
		count += 1;
					
	return Target_joint_value;
		
def pos_test(pose_targets, group_handle, IK_handle, animate_result = False):
  
  test_number = len(pose_targets);
  group = group_handle;
  print "============ Reference frame: %s" % group.get_end_effector_link()
  
  if IK_handle!=None:
	 ik = IK_handle;
  else:
	  print "No IK solver assigned! Exit!";
	  return False;  
  if len(pose_targets):
	count = 0;
	success_number = 0;
	for pose_target in pose_targets:
		count += 1;		
		#print "Start Planning No.", count, "Trajectory";		
		target = geometry_msgs.msg.Pose();		
		target.orientation.x = 0.50;
		target.orientation.y = -0.50;		
		target.orientation.z = -0.50;
		target.orientation.w = 0.507;		
		target.position.x = pose_target.x;
		target.position.y = pose_target.y;
		target.position.z = pose_target.z;	
		print target.position;	
		current_state = Get_current_state(group);
		result = find_IK_solution(ik, target, current_state, group_handle.get_name());		
		
		#print result.solution.joint_state;
		
		if result.error_code.val != 1:
			#print "Can't find solution for No.",count,"target";
			print "Bin ", Bin_number(count), "test failed!";
		else:
			#print result.solution.joint_state;
			success_number += 1;
			TargetJointValue = Copy_joint_value(group_handle.get_name(),result.solution.joint_state.position);
			#print "Target joint value:", TargetJointValue;			
			if animate_result:
				print ">>>>>>>>>>>>>>> Displaying No.", count, "trajectory >>>>>>>>>>>>>>>"
				if len(TargetJointValue):
					group.set_joint_value_target(TargetJointValue);
					#plan = group.plan();	
					#rospy.sleep(2);
					#print ">>>>>>>>>>>>> Actuate trajectory >>>>>>>>>>>>>"
					group.go();
					rospy.sleep(2);
					
	if success_number == test_number:
		print "Available for all position!";
		return True;
	else:
		print "Can't find solution for all target position!"
		print "Success ratio:", success_number, "/",test_number;
		return False;
  else:
	  print "No target Assigned, Exit!";
	  return False;
	

if __name__=='__main__':
  try:
	#Init_Right_Hand_Pos = [0.323265,-0.499221,1.07222, -0.4838, 0.515602, 0.5159, -0.48353];
	if len(sys.argv)>1:
	  print len(sys.argv);
	  X_pos = float(sys.argv[1]);
	  Y_pos = float(sys.argv[2]);
	  Z_pos = float(sys.argv[3]);
	else:
	  print "No distance assigned, using default parameters: 1.32m, 0m, 0m"
	  X_pos = 1.32;
	  Y_pos = 0;
	  Z_pos = 0;
	
	Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos, Test_Depth = 0.4);
	
	print "============ Initializing... ============="
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('IK_Solution_Test', anonymous=True);  
	robot = moveit_commander.RobotCommander();
	scene = moveit_commander.PlanningSceneInterface();
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left");
	
	arm_left_group.set_planner_id("RRTstarkConfigDefault");
	
	
	print ">>>>>>>>>>>>> Waiting for service `compute_ik` >>>>>>>>>>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);
		
	print ">>>>>>>>>>>>>> Start Testing >>>>>>>>>>>>>>"
	pos_test(Goal_points,arm_left_group, ik, animate_result = True)		
	
	print "**************** Test End ****************"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
