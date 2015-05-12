#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    #rospy.sleep(15)
    scene = moveit_commander.PlanningSceneInterface();
    rospy.init_node('publish_box', anonymous=True);

    name = sys.argv[1]
    pose = PoseStamped();
    pose.pose.position.x = float(sys.argv[2]);
    pose.pose.position.y = float(sys.argv[3]);
    pose.pose.position.z = float(sys.argv[4]);
    pose.pose.orientation.x = float(sys.argv[5]);	
    pose.pose.orientation.y = float(sys.argv[6]);	
    pose.pose.orientation.z = float(sys.argv[7]);	
    pose.pose.orientation.w = float(sys.argv[8]);
    size = (float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]))
        
    scene.attach_box(link = "base_link", 
                      name = name, 
                      pose = pose,
                      size = size);
        
    print "Published "+name
    moveit_commander.roscpp_shutdown()
