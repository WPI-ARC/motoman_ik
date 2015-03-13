
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

  for i in range(1,25):
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
      rospy.sleep(5)
      print "============ Waiting while", file_name, " is visualized (again)..."
      f.close()

if __name__=='__main__':
  try:

	print "============ Initializing... ============="
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('IK_Solution_Test', anonymous=True); 
	scene = moveit_commander.PlanningSceneInterface();
	
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory)
		
	print ">>>>>>>>>>>>>> Start Testing >>>>>>>>>>>>>>" 
	robot = moveit_commander.RobotCommander();
	arm_left_group = moveit_commander.MoveGroupCommander("arm_left");

	pos_test(arm_left_group);	
	
	print "**************** Test End ****************"
	moveit_commander.roscpp_shutdown()
    
  except rospy.ROSInterruptException:
    pass
