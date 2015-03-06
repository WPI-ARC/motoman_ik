# motoman_ik

IK solvers for the Motoman robot

## Installation

`motoman_ik` includes a number of standard ROS packages, so
installation from source is straightforward.

1. `cd <<your-catkin-workspace>>/src`
2. `git clone git@github.com:WPI-ARC/motoman_ik.git`
3. `cd ..`
4. `catkin_make`

## Usage

The kinematics plugin is loaded within a MoveIt group called `arms`,
it can be controlled by the standard MoveIt interface. When specifying
poses, first specify the left pose, then the right pose.

For basic usage, launch with `roslaunch motoman_wpi_moveit_config
demo.launch`.

## Packages

- `generalik_kinematics_plugin`: A MoveIt kinematics plugin for the
  motoman sda10f based on GeneralIK. It allows IK to be solved for
  both end-effectors simultaneously using an iterative jacobian
  method.
- `motoman_wpi_moveit_config`: A MoveIt configuration for the motoman
  sda10f that uses the `generalik_kinematics_plugin` and has a few
  other tweaks in the srdf. It is a fork of
  `motoman_sda10f_moveit_config`.
