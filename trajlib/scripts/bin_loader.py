
import os
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

X_pos = 1.39;
Y_pos = 0.08;
Z_pos = -0.05;


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
			size=(0.96, 0.96, 2.5));
		
		drop_size_x = 0.381;
		drop_size_y = 0.635;
		drop_size_z = 0.533;
		dropbin_pose = PoseStamped()
		dropbin_pose.header.frame_id = "/base_link"
		dropbin_pose.header.stamp = rospy.Time.now()
		dropbin_pose.pose.position.x = 0.67
		dropbin_pose.pose.position.y = 0.2
		dropbin_pose.pose.position.z = drop_size_z/2
		dropbin_pose.pose.orientation.x = 0
		dropbin_pose.pose.orientation.y = 0
		dropbin_pose.pose.orientation.z = 0
		dropbin_pose.pose.orientation.w = 1
		scene.add_box( name="dropbox",
					   pose=dropbin_pose,
					   size=(drop_size_x, drop_size_y, drop_size_z));
        
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	print ">>>> Loading Bin Model... >>>>"
	moveit_commander.roscpp_initialize(sys.argv);
	rospy.init_node('Bin loader', anonymous=True);
	Load_Bin_model(X_pos, Y_pos, Z_pos);
