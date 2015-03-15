
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
		
def pos_test(group_handle):

  for i in range(1,2):
      file_name = "Traj/traj_file "+str(i)
      f = open(file_name,"r")
      plan = RobotTrajectory()
      buf = f.read()
      plan.deserialize(buf)
      #print plan
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = robot.get_current_state()
      display_trajectory.trajectory.append(plan)
      display_trajectory_publisher.publish(display_trajectory);
      rospy.sleep(5)
      print "============ Waiting while", file_name, " is visualized (again)..."
      f.close()
      
def import_bin_model(planning_scene_handle, sys):
	
	print ">>>> Import Bin model, Generate Testing Targets >>>>"
	if len(sys.argv)>1:
	  X_pos = float(sys.argv[1]);
	  Y_pos = float(sys.argv[2]);
	  Z_pos = float(sys.argv[3]);	  
	else:
	  print "No distance assigned, using default parameters"
	  X_pos = 1.35;
	  Y_pos = 0;
	  Z_pos = 0;

	bin_pose = PoseStamped();
	bin_pose.pose.position.x = X_pos;
	bin_pose.pose.position.y = Y_pos;
	bin_pose.pose.position.z = Z_pos;
	
	bin_pose.pose.orientation.x = 0.5;	
	bin_pose.pose.orientation.y = 0.5;	
	bin_pose.pose.orientation.z = 0.5;	
	bin_pose.pose.orientation.w = 0.5;
	
	planning_scene_handle.attach_mesh(link = "base_link", 
									  name = "kiva_pod", 
									  pose = bin_pose,
									  filename = "Model/pod_lowres.stl");



def pos_init(left_arm_group_handle, right_arm_group_handle):		
	left_arm_init_joint_value = [0.0, -0.6039528242412353, -1.6264234108138371, 1.0221685537999192, -0.9819728428472593,-0.3411106568963133, -1.675219368757607, 1.2102858416139277];
	right_arm_init_joint_value = [0.0, 2.5794765930828296, 1.3620727097356629, 1.3831275005664025, 0.7845256389316293, -3.057076564078304, -1.7625990915019676, 1.3096307216010097];		
	left_arm_group_handle.go(left_arm_init_joint_value);
	rospy.sleep(5);
	right_arm_group_handle.go(right_arm_init_joint_value);	
	rospy.sleep(5);
	
if __name__=='__main__':
  try:

	print "============ Initializing... ============="
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('IK_Solution_Test', anonymous=True); 
	scene = moveit_commander.PlanningSceneInterface();	
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory);

	scene = moveit_commander.PlanningSceneInterface();	
	import_bin_model(scene,sys);
	
	print ">>>>>>>>>>>>>> Start Testing >>>>>>>>>>>>>>" 
	robot = moveit_commander.RobotCommander();	
	arm_right_group = moveit_commander.MoveGroupCommander("arm_right"); 
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left");	
	pos_init(arm_left_group,arm_right_group);

	pos_test(arm_left_group);	
	
	print "**************** Test End ****************"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
