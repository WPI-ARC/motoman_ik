#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    rospy.sleep(10)
    rospy.init_node('publish_mesh', anonymous=True);
    scene = moveit_commander.PlanningSceneInterface();

    name, stl = sys.argv[1], sys.argv[2]
    pose = PoseStamped();
    pose.pose.position.x = float(sys.argv[3]);
    pose.pose.position.y = float(sys.argv[4]);
    pose.pose.position.z = float(sys.argv[5]);
    pose.pose.orientation.x = float(sys.argv[6]);	
    pose.pose.orientation.y = float(sys.argv[7]);	
    pose.pose.orientation.z = float(sys.argv[8]);	
    pose.pose.orientation.w = float(sys.argv[9]);
        
    scene.attach_mesh(link = "base_link", 
                      name = name, 
                      pose = pose,
                      filename = stl);
        
    print "Published "+name
    moveit_commander.roscpp_shutdown()
