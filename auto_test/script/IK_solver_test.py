
import sys
import copy
import rospy
import StringIO
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int64
from StringIO import StringIO

import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.msg import RobotTrajectory
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

def pos_init(left_arm_group_handle, right_arm_group_handle):
	
	right_arm_init_pos = geometry_msgs.msg.Pose();	
	left_arm_init_pos = geometry_msgs.msg.Pose();
				
	right_arm_init_pos.orientation.x = -0.50;
	right_arm_init_pos.orientation.y = -0.50;		
	right_arm_init_pos.orientation.z = 0.50;
	right_arm_init_pos.orientation.w = 0.50;	
	right_arm_init_pos.position.x = 0.1;
	right_arm_init_pos.position.y = -0.5;
	right_arm_init_pos.position.z = 0.68;
	
	left_arm_init_pos.orientation.x = -0.50;
	left_arm_init_pos.orientation.y = -0.50;		
	left_arm_init_pos.orientation.z = 0.50;
	left_arm_init_pos.orientation.w = 0.50;	
	left_arm_init_pos.position.x = 0.1;
	left_arm_init_pos.position.y = 0.5;
	left_arm_init_pos.position.z = 0.68;	
	
	left_arm_group_handle.set_pose_target(left_arm_init_pos);
	right_arm_group_handle.set_pose_target(right_arm_init_pos);	
	
	left_arm_group_handle.go();
	right_arm_group_handle.go();	
	
		
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
		target.orientation.x = pose_target.qx;
		target.orientation.y = pose_target.qy;		
		target.orientation.z = pose_target.qz;
		target.orientation.w = pose_target.qw;	
		target.position.x = pose_target.x;
		target.position.y = pose_target.y;
		target.position.z = pose_target.z;
		
		print target.position;	
		current_state = Get_current_state(group);
		result = find_IK_solution(ik, target, current_state, group_handle.get_name());		
		
		if result.error_code.val != 1:
			print "Bin ", Bin_number(count), "test failed!";
			
		else:
			success_number += 1;	
					
			TargetJointValue = Copy_joint_value(group_handle.get_name(),result.solution.joint_state.position);
			
			if animate_result:
				
				print ">>>>>>>>>>>>>>> Displaying No.", count, "trajectory >>>>>>>>>>>>>>>"
				
				if len(TargetJointValue):
					group.set_joint_value_target(TargetJointValue);
					plan = group.plan();
										
					# Save trajectory to file			
					buf = StringIO();
					plan.serialize(buf);					
					file_name = "traj_file"+str(count);	
					print "saving No.",count,"trajectory to file",file_name;					
					f = open(file_name,"w");
					f.write(buf.getvalue());
					f.close();
									
					group.go();
					
	if success_number == test_number:
		print "Available for all position!";
		return True;
	else:
		print "Can't find IK solution for all target position!"
		print "Success ratio:", success_number, "/",test_number;
		return False;
  else:
	  print "No target Assigned, Exit!";
	  return False;

if __name__=='__main__':
  try:
	
	print ">>>> Initializing... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('IK_Solution_Test', anonymous=True);  
	robot = moveit_commander.RobotCommander();
	scene = moveit_commander.PlanningSceneInterface();	
	
	print ">>>> Import Bin model, Generate Testing Targets >>>>"
	if len(sys.argv)>1:
	  X_pos = float(sys.argv[1]);
	  Y_pos = float(sys.argv[2]);
	  Z_pos = float(sys.argv[3]);	  
	else:
	  print "No distance assigned, using default parameters"
	  X_pos = 1.32;
	  Y_pos = 0;
	  Z_pos = 0;
	  
	Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos);
	print "Total", len(Goal_points), "targets need to be test";
	
	bin_pose = PoseStamped();
	bin_pose.pose.position.x = X_pos;
	bin_pose.pose.position.y = Y_pos;
	bin_pose.pose.position.z = Z_pos;
	
	bin_pose.pose.orientation.x = 0.5;	
	bin_pose.pose.orientation.y = 0.5;	
	bin_pose.pose.orientation.z = 0.5;	
	bin_pose.pose.orientation.w = 0.5;
		
	scene.attach_mesh(link = "base_link", 
					  name = "kiva_pod", 
					  pose = bin_pose,
					  filename = "Model/pod_lowres.stl");
	
	print ">>>> Set Init Position >>>>"
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left");	
	arm_left_group.set_planner_id("RRTstarkConfigDefault");	
	arm_left_group.allow_replanning(True);	
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right"); 
	arm_right_group.set_planner_id("RRTstarkConfigDefault");	
	arm_right_group.allow_replanning(True);	
	pos_init(arm_left_group, arm_right_group);
	
	print ">>>> Waiting for service `compute_ik` >>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);
					  
	print ">>>> Start Testing >>>>"
	pos_test(Goal_points,arm_left_group, ik, animate_result = True)		
	
	print "**** Test End ****"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
