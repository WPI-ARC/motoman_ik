#!/usr/bin/env python

from trajlib.srv import *

import sys
import rospy

def request_a_traj(current_task,bin_num, using_torso):
	
	rospy.wait_for_service('trajectory_execute');
	
	try:		
		traj_server = rospy.ServiceProxy('trajectory_execute', task);
		resp1 = traj_server(current_task, bin_num, using_torso);
		return taskResponse(True);
		
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e;

if __name__ == '__main__':
	if len(sys.argv) == 4:
		current_task = sys.argv[1];
		bin_num = sys.argv[2];
		using_torso = sys.argv[3];
		
	elif len(sys.argv) == 3:
		current_task = sys.argv[1];
		bin_num = sys.argv[2];
		using_torso = "n";
		
	else:
		current_task = "Forward";
		bin_num = "A";
		using_torso = "n";
		
	request_a_traj(current_task, bin_num, using_torso);
