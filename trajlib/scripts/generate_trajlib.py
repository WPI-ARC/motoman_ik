#!/usr/bin/env python

import os
import sys
import copy
import rospy
import StringIO
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Int64
from StringIO import StringIO

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from sensor_msgs.msg import JointState

# Structure
from goal_pos_generate import Jnt_state_goal;
# Variable
from goal_pos_generate import left_arm_init_joint_value, right_arm_init_joint_value, torso_init_rotation_angle;
# Function
from goal_pos_generate import generate_goal_points, generate_left_arm_seed_state, generate_key_joint_state;

def Get_current_state(group):
    return	JointState(
        name=group.get_joints()[:7],
        position=group.get_current_joint_values(),
    );

def Generate_joint_state_msg(group_handle,jnt_value):
    return JointState( name = group_handle.get_joints()[:7],
                       position = jnt_value);

def find_IK_solution(ik, target, seed, group_name):
    response = ik( GetPositionIKRequest(ik_request = PositionIKRequest( group_name = group_name,
                                                                        pose_stamped = PoseStamped( header = Header(frame_id="/base_link"),
                                                                                                    pose = target),
                                                                        robot_state = RobotState(joint_state = seed))
                                                                        ))
    return response

def torso_init(torso_group_handle):
    torso_group_handle.set_start_state_to_current_state();
    torso_group.go(torso_rotation_angle);

def arm_init(left_arm_group_handle, right_arm_group_handle):
    left_arm_group_handle.set_start_state_to_current_state();
    left_arm_group_handle.go(left_arm_joint_value);

    right_arm_group_handle.set_start_state_to_current_state();
    right_arm_group_handle.go(right_arm_init_joint_value);

def Save_traj(file_name,plan):
	
    print "saving ",file_name;    
    buf = StringIO();
    plan.serialize(buf);
    f = open(file_name,"w");
    f.write(buf.getvalue());
    f.close();

def Copy_joint_value(group_name, joint_values):
    count = 0;
    Target_joint_value = [];
    if group_name == "arm_left":
        Target_joint_value = joint_values[1:8];
    elif group_name == "arm_right":
        Target_joint_value = joint_values[20:27];
    return Target_joint_value;

def Add_bin_model(bin_x, bin_y, bin_z):
	bin_pose = PoseStamped();
	bin_pose.pose.position.x = bin_x;
	bin_pose.pose.position.y = bin_y;
	bin_pose.pose.position.z = bin_z;
	bin_pose.pose.orientation.x = 0.5;
	bin_pose.pose.orientation.y = 0.5;
	bin_pose.pose.orientation.z = 0.5;
	bin_pose.pose.orientation.w = 0.5;
	scene = moveit_commander.PlanningSceneInterface();
	scene.attach_mesh( link =  "base_link",
						name = "kiva_pod",
						pose =  bin_pose,
						filename = os.path.join(os.path.dirname(__file__), "../../apc_models/meshes/pod_lowres.stl"))
	scene.add_box(name = "shelf_box",
					pose =  bin_pose,
					size = (0.87, 0.87, 1.87));
    
def generate_configurationSet(target_pnt_set, seed_config_set,ik_handle,group_handle):

    arm_config = [];
    if len(target_pnt_set) != len(seed_config_set):
        print "WARNNING!! target_pnt number is not equal to seed_config_set number, Exit!";
        return arm_config;

    for num in range(0,len(target_pnt_set)):

        pnt = target_pnt_set[num];
        target_pnt = geometry_msgs.msg.Pose();
        target_pnt.position.x = pnt.x;
        target_pnt.position.y = pnt.y;
        target_pnt.position.z = pnt.z;
        target_pnt.orientation.x = pnt.qx;
        target_pnt.orientation.y = pnt.qy;
        target_pnt.orientation.z = pnt.qz;
        target_pnt.orientation.w = pnt.qw;

        seed_config = seed_config_set[num];
        seed_state = Generate_joint_state_msg(group_handle,seed_config.jnt_val);

        result = find_IK_solution(ik_handle, target_pnt, seed_state, group_handle.get_name());

        if result.error_code.val != 1:
            attempt = 0;
            Success = False;
            while attempt < 10:
                attempt += 1;
                result = find_IK_solution(ik_handle, target_pnt, seed_state, group_handle.get_name());
                if result.error_code.val == 1:
                    Success = True;
                    break;
            if Success is not True:
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
                print "Can't find IK solution for Bin", pnt.bin_num, pnt.pnt_property;
                print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
                continue;

        #print "IK solution: ", result.solution.joint_state.position;
        IK_solution = Jnt_state_goal();
        IK_solution.jnt_val = Copy_joint_value(group_handle.get_name(),result.solution.joint_state.position);
        IK_solution.bin_num = seed_config.bin_num;

        #print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
        #print "Seed State: ", seed_state.position;
        #print "IK solution ", IK_solution.jnt_val;

        arm_config.append(IK_solution);

    print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.."
    print "Configuration update complete!!"

    return arm_config;

def Generate_traj_for_key2pnt(key_config_set, goal_config_set, group_handle):
	success_num = 0;
	total_traj_num = len(key_config_set)*len(goal_config_set);
	print "KeyConfigNum: ",len(key_config_set), "GoalNum: ",len(goal_config_set),", Try to generate ", total_traj_num, "trajectories";
	
	start_pos = key_config_set[0];
	drop_pos = start_pos;
	
	group_handle.set_joint_value_target(start_pos.jnt_val);
	group_handle.go();
	for num in range(0,len(goal_config_set)):
		
		current_goal_config = goal_config_set[num];		
		# Plan from start to the bin
		#group_handle.set_start_state_to_current_state();
		goal_jnt_value_msg = Generate_joint_state_msg(group_handle,current_goal_config.jnt_val)
		group_handle.set_joint_value_target(goal_jnt_value_msg);
		plan = group_handle.plan();
		
		if len(plan.joint_trajectory.points):
			rospy.sleep(5);
			#group_handle.go();
			group_handle.execute(plan);
			folder_name = os.path.join(os.path.dirname(__file__), "../trajectories/bin") + current_goal_config.bin_num;
			file_name = folder_name + "/"+ "forward";
			Save_traj(file_name,plan);
			success_num += 1;
			# Plan from bin to drop
			group_handle.set_start_state_to_current_state();
			group_handle.set_joint_value_target(drop_pos.jnt_val);
			plan = group_handle.plan();
			if len(plan.joint_trajectory.points):
				rospy.sleep(5);
				#group_handle.go();
				group_handle.execute(plan);
				file_name = folder_name + "/"+ "drop";
				Save_traj(file_name,plan);
				success_num += 1;
			#for point in plan.joint_trajectory.points:
			#	point.velocities = [0]*len(point.velocities);
			#	point.accelerations = [0]*len(point.accelerations);
			else:
				print "Planning from bin",current_goal_config.bin_num, "to Drop position Failed!";
		else:
			print "Planning from Start Position to bin",current_goal_config.bin_num, " Failed!";
	
	print "Total success number: ",success_num;

def Generate_traj_for_pnt2pnt(goal_config_set, group_handle):
	success_num = 0;
	total_traj_num = len(goal_config_set)*len(goal_config_set);
	print "GoalConfigNum: ",len(goal_config_set), ", Try to generate ", total_traj_num, "trajectories";
	
if __name__=='__main__':
  try:
    print ">>>> Initializing... >>>>"
    moveit_commander.roscpp_initialize(sys.argv);
    rospy.init_node('IK_Solution_Test', anonymous=True);
    
    planning_time = 60;
    
    arm_left_group = moveit_commander.MoveGroupCommander("arm_left");
    arm_left_group.set_planner_id("RRTstarkConfigDefault");
    arm_left_group.allow_replanning(True);
    
    arm_right_group = moveit_commander.MoveGroupCommander("arm_right");
    arm_right_group.set_planner_id("RRTstarkConfigDefault");
    arm_right_group.allow_replanning(True);
    
    arm_init(arm_left_group, arm_right_group);

    arm_left_group.set_planning_time(planning_time);
    arm_right_group.set_planning_time(planning_time);

    #torso_group = moveit_commander.MoveGroupCommander("torso");
    #torso_init(torso_group);

    print ">>>> Import Bin model, Generate Testing Targets >>>>"
    if len(sys.argv)>1:
      X_pos = float(sys.argv[1]);
      Y_pos = float(sys.argv[2]);
      Z_pos = float(sys.argv[3]);
    else:
      X_pos = 1.29;
      Y_pos = 0;
      Z_pos = 0;
      print "No distance assigned, using default parameters: ",X_pos, Y_pos, Z_pos;

<<<<<<< HEAD
    Add_bin_model(X_pos, Y_pos, Z_pos);
    # Generate 
=======
    bin_pose = PoseStamped();
    bin_pose.pose.position.x = X_pos;
    bin_pose.pose.position.y = Y_pos;
    bin_pose.pose.position.z = Z_pos;
    bin_pose.pose.orientation.x = 0.5;
    bin_pose.pose.orientation.y = 0.5;
    bin_pose.pose.orientation.z = 0.5;
    bin_pose.pose.orientation.w = 0.5;

    scene = moveit_commander.PlanningSceneInterface();
    scene.attach_mesh(link = "base_link",
                      name = "kiva_pod",
                      pose =  bin_pose,
                      filename = os.path.join(os.path.dirname(__file__), "../../apc_models/meshes/pod_lowres.stl"))

    scene.add_box(link = "base_link",
                      name = "shelf_box",
                      pose =  bin_pose,
                      size = (1.77, 0.87, 0.87))

>>>>>>> origin/test_version
    Goal_points = generate_goal_points(Bin_base_x = X_pos, Bin_base_y = Y_pos, Bin_base_z = Z_pos);
    print "Total", len(Goal_points), "target points";

    left_arm_seed_states = generate_left_arm_seed_state();
    print "Total", len(left_arm_seed_states), "seed states";

    print ">>>> Waiting for service `compute_ik` >>>>";
    rospy.wait_for_service('compute_ik');
    ik = rospy.ServiceProxy("compute_ik", GetPositionIK);
    
    print "Updating Joint Configurations..."
    left_arm_config_set = generate_configurationSet(Goal_points, left_arm_seed_states, ik, arm_left_group);
    
    print "Updating Key Joint States..."
    key_joint_state = generate_key_joint_state(arm_left_group.get_name());

    print ">>>> Start Generating trajectory library (from KeyPos <--> Bin)>>>>";
    Generate_traj_for_key2pnt(key_joint_state, left_arm_config_set, arm_left_group);

    print ">>>> Start Generating trajectory library (from Bin <--> Bin)>>>>";
    Generate_traj_for_pnt2pnt(left_arm_config_set,arm_left_group);

    print "**** Test End ****"
    moveit_commander.roscpp_shutdown()

  except rospy.ROSInterruptException:
    pass
