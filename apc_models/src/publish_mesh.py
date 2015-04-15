#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy

import moveit_commander

from geometry_msgs.msg import PoseStamped


if __name__=='__main__':
    scene = moveit_commander.PlanningSceneInterface()
    rospy.init_node('publish_mesh', anonymous=True)
    rospy.sleep(10)

    name, stl = sys.argv[1], sys.argv[2]
    pose = PoseStamped()
    pose.header.frame_id = "/base_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = float(sys.argv[3])
    pose.pose.position.y = float(sys.argv[4])
    pose.pose.position.z = float(sys.argv[5])
    pose.pose.orientation.x = float(sys.argv[6])
    pose.pose.orientation.y = float(sys.argv[7])
    pose.pose.orientation.z = float(sys.argv[8])
    pose.pose.orientation.w = float(sys.argv[9])

    # mesh = scene._PlanningSceneInterface__make_mesh(
    #     name="shelf",
    #     pose=pose,
    #     filename=stl
    # )
    # mesh.header.frame_id = "/base_link"
    # mesh.header.stamp = rospy.Time.now()
    # scene._pub_co.publish(mesh)

    scene.add_mesh(
        name=name,
        pose=pose,
        filename=stl
    )

    print "Published "+name
    moveit_commander.roscpp_shutdown()
