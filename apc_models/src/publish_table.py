#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy

import moveit_commander
from moveit_msgs.msg import PlanningScene, PlanningSceneWorld

from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    scene = moveit_commander.PlanningSceneInterface()
    scene_topic = rospy.Publisher("planning_scene", PlanningScene)
    rospy.init_node('publish_boxshelf', anonymous=True)
    # while scene._pub_co.get_num_connections() == 0:
    #     rospy.sleep(1)
    rospy.sleep(10)

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0
    pose.pose.position.y = 1
    pose.pose.position.z = 0.33
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    scene.add_box(
        name="table",
        pose=pose,
        size=(1, 0.5, 0.6)
    )
    print "Published table"
    # rospy.spin()
    moveit_commander.roscpp_shutdown()
