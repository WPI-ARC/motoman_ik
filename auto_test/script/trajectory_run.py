
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

#global trajectory
#trajectory = []

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
		
def pos_test(group_handle):

  for i in range(1,12):
      file_name = "traj_file"+str(i)  
      f = open(file_name,"r")
      plan = RobotTrajectory()
      buf = f.read()
      plan.deserialize(buf)
      #print plan
      display_trajectory = moveit_msgs.msg.DisplayTrajectory()
      display_trajectory.trajectory_start = robot.get_current_state()
      display_trajectory.trajectory.append(plan)
      display_trajectory_publisher.publish(display_trajectory);
      print "============ Waiting while plan1 is visualized (again)..."
      rospy.sleep(5)
      f.close()
	

if __name__=='__main__':
  try:

	print "============ Initializing... ============="
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('IK_Solution_Test', anonymous=True);  
	robot = moveit_commander.RobotCommander();
	scene = moveit_commander.PlanningSceneInterface();
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left");
	arm_left_group.set_planner_id("RRTstarkConfigDefault");
	
	display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
	print ">>>>>>>>>>>>> Waiting for service `compute_ik` >>>>>>>>>>>>";
	rospy.wait_for_service('compute_ik');
	ik = rospy.ServiceProxy("compute_ik", GetPositionIK);
		
	print ">>>>>>>>>>>>>> Start Testing >>>>>>>>>>>>>>"
	pos_test(arm_left_group)		
	
	print "**************** Test End ****************"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
