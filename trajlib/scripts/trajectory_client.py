#!/usr/bin/env python

from trajlib.srv import *

import sys
import rospy
import moveit_msgs.msg
import moveit_commander

from moveit_msgs.msg import DisplayTrajectory

def request_a_traj(current_task, bin_num, using_torso):
	try:
		traj_server = rospy.ServiceProxy('trajectory_display', GetTrajectory);
		
		resp1 = traj_server(current_task, bin_num, using_torso);
		
		return GetTrajectoryResponse(resp1.plan, True);

	except rospy.ServiceException, e:
		print "Service call failed: %s" %e;

if __name__ == '__main__':
	
	rospy.init_node('trajectory_client');
	rospy.wait_for_service('trajectory_display');
	
	if len(sys.argv) == 4:
		current_task = sys.argv[1];
		bin_num = sys.argv[2];
		using_torso = sys.argv[3];
		
	elif len(sys.argv) == 3:
		current_task = sys.argv[1];
		bin_num = sys.argv[2];
		using_torso = "n";
	
	elif len(sys.argv) == 2:
		current_task = sys.argv[1];
		bin_num = "";
		using_torso = "n";	

	else:
		current_task = "Forward";
		bin_num = "A";
		using_torso = "n";
		
	request_a_traj(current_task, bin_num, using_torso);
