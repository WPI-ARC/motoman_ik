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
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(1)

    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 1.28
    pose.pose.position.y = 0
    pose.pose.position.z = 1.25
    pose.pose.orientation.x = 0.5
    pose.pose.orientation.y = 0.5
    pose.pose.orientation.z = 0.5
    pose.pose.orientation.w = 0.5

    # print dir(scene)
    # box = scene._PlanningSceneInterface__make_box(
    #     name="shelf",
    #     pose=pose,
    #     size=(1, 3, 1)
    # )
    # box.header.frame_id = "/base_link"
    # box.header.stamp = rospy.Time.now()
    # print box
    # scene._pub_co.publish(box)
    # print dir(scene._pub_co)
    # print scene._pub_co.name
    # print scene._pub_co.resolved_name
    # print scene._pub_co.type
    # print scene._pub_co.reg_type
    # print scene._pub_co.md5sum
    # print scene._pub_co.get_num_connections()

    # diff = PlanningScene(
    #     world=PlanningSceneWorld(collision_objects=[box]),
    #     is_diff=True,
    # )
    # print diff
    # scene_topic.publish(diff)

    scene.add_box(
        name="shelf",
        pose=pose,
        size=(1, 2.5, 1)
    )

    print "Published shelf"
    # rospy.spin()
    moveit_commander.roscpp_shutdown()
