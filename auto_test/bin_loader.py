import os
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
  try:
	
	print ">>>> Initializing... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('IK_Solution_Test', anonymous=True);  
	
	scene = moveit_commander.PlanningSceneInterface();	
	print ">>>> Import Bin model, Generate Testing Targets >>>>"
	if len(sys.argv)>1:
	  X_pos = float(sys.argv[1]);
	  Y_pos = float(sys.argv[2]);
	  Z_pos = float(sys.argv[3]);	  
	else:
	  print "No distance assigned, using default parameters"
	  X_pos = 1.4;
	  Y_pos = 0.08;
	  Z_pos = 0;
	  
	#Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos);
	#print "Total", len(Goal_points), "targets need to be test";
	
	bin_pose = PoseStamped();
	bin_pose.pose.position.x = X_pos;
	bin_pose.pose.position.y = Y_pos;
	bin_pose.pose.position.z = Z_pos;
	
	bin_pose.pose.orientation.x = 0.5;	
	bin_pose.pose.orientation.y = 0.5;	
	bin_pose.pose.orientation.z = 0.5;	
	bin_pose.pose.orientation.w = 0.5;
	
	model_filepath = os.path.join(os.path.dirname(__file__), "../apc_models/meshes/pod_lowres.stl")
		
	scene.attach_mesh(link = "base_link", 
					  name = "kiva_pod", 
					  pose = bin_pose,
					  filename = model_filepath);

    
  except rospy.ROSInterruptException:
    pass
