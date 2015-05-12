
import os
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

X_pos = 1.39;
Y_pos = 0.08;
Z_pos = 0;


def Load_Bin_model(bin_x, bin_y, bin_z):
	try:
		scene = moveit_commander.PlanningSceneInterface();
		
		bin_pose = PoseStamped();
		
		bin_pose.header.frame_id = "/base_link"
		bin_pose.header.stamp = rospy.Time.now()
		
		bin_pose.pose.position.x = bin_x;
		bin_pose.pose.position.y = bin_y;
		bin_pose.pose.position.z = bin_z;
		
		bin_pose.pose.orientation.x = 0.5;
		bin_pose.pose.orientation.y = 0.5;
		bin_pose.pose.orientation.z = 0.5;
		bin_pose.pose.orientation.w = 0.5;
		
		model_filepath = os.path.join(os.path.dirname(__file__), "../../apc_models/meshes/pod_lowres.stl")
			
		scene.attach_mesh(link = "base_link",
						  name = "kiva_pod", 
						  pose = bin_pose,
						  filename = model_filepath);
		
		box_pose = PoseStamped()
		box_pose.header.frame_id = "/base_link"
		box_pose.header.stamp = rospy.Time.now()
		box_pose.pose.position.x = bin_x
		box_pose.pose.position.y = bin_y
		box_pose.pose.position.z = bin_z + 0.835
		box_pose.pose.orientation.x = 0
		box_pose.pose.orientation.y = 0
		box_pose.pose.orientation.z = 0
		box_pose.pose.orientation.w = 1
		scene.add_box(
			name="bin",
			pose=box_pose,
			size=(0.86, 0.8, 1.78));
			
		pose = PoseStamped()
		pose.header.frame_id = "/base_link"
		pose.header.stamp = rospy.Time.now()
		pose.pose.position.x = -0.505
		pose.pose.position.y = 0
		pose.pose.position.z = 1.5
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		scene.add_box(
			name="wall",
			pose=pose,
			size=(0.01, 6, 3));
        
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	print ">>>> Loading Bin Model... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('Bin loader', anonymous=True);
	Load_Bin_model(X_pos, Y_pos, Z_pos);
