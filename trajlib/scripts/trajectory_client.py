#!/usr/bin/env python

from trajectory_srv.srv import *

import sys
import rospy

def request_a_traj(current_task,bin_num):
	
	rospy.wait_for_service('trajectory_execute');
	
	try:		
		traj_server = rospy.ServiceProxy('trajectory_execute', task);
		resp1 = traj_server(current_task,bin_num);
		return taskResponse(True);
		
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e;

if __name__ == '__main__':
    if len(sys.argv) > 1:
		current_task = sys.argv[1];
		bin_num = sys.argv[2];
    else:
		current_task = "Forward";
		bin_num = "A";
    request_a_traj(current_task,bin_num);
