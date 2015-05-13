#!/usr/bin/python

import roslib; roslib.load_manifest('apc_models')

import sys
import rospy
from copy import deepcopy

import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from geometry_msgs.msg import Pose


if __name__=='__main__':
    scene = moveit_commander.PlanningSceneInterface()
    rospy.init_node('publish_scoop', anonymous=True)
    while scene._pub_co.get_num_connections() == 0:
        rospy.sleep(0.01)

    handle_pose = Pose()
    handle_pose.position.x = 0.73
    handle_pose.position.y = 0.03
    handle_pose.position.z = 1.22
    handle_pose.orientation.x = 0
    handle_pose.orientation.y = 0
    handle_pose.orientation.z = 0
    handle_pose.orientation.w = 1
    surface_pose = deepcopy(handle_pose)
    surface_pose.position.x += 0.25
    surface_pose.position.z += 0.11
    handle_size = [0.02, 0.155, 0.16]
    surface_size = [0.48, 0.23, 0.08]

    co = CollisionObject()
    co.operation = CollisionObject.ADD
    co.id = "Scoop"
    co.header.frame_id = "/base_link"
    co.header.stamp = rospy.Time.now()
    handle = SolidPrimitive()
    handle.type = SolidPrimitive.BOX
    handle.dimensions = handle_size
    surface = SolidPrimitive()
    surface.type = SolidPrimitive.BOX
    surface.dimensions = surface_size
    co.primitives = [handle, surface]
    co.primitive_poses = [handle_pose, surface_pose]

    scene._pub_co.publish(co)

    print "Published scoop"
    # rospy.spin()
    moveit_commander.roscpp_shutdown()
