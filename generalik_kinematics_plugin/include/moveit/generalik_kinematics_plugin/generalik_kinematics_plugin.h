/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#ifndef GENERALIK_KINEMATICS_PLUGIN_
#define GENERALIK_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// System
#include <boost/shared_ptr.hpp>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace generalik_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using KDL. This version can be used with any robot.
 */
  class GeneralIKKinematicsPlugin : public kinematics::KinematicsBase
  {
    public:

    /**
     *  @brief Default constructor
     */
    GeneralIKKinematicsPlugin();

    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
                                  const moveit::core::RobotState* context_state = NULL) const;

    virtual bool getPositionFK(const std::vector<std::string> &link_names,
                               const std::vector<double> &joint_angles,
                               std::vector<geometry_msgs::Pose> &poses) const;

    virtual bool initialize(const std::string &robot_description,
                            const std::string &group_name,
                            const std::string &base_name,
                            const std::string &tip_name,
                            double search_discretization) {
      logError("moveit.generalik_kinematics_plugin: Requires multiple tips.");
      return false;
    }

    virtual bool initialize(const std::string& robot_description,
                            const std::string& group_name,
                            const std::string& base_frame,
                            const std::vector<std::string>& tip_frames,
                            double search_discretization);

    /**
     * @brief  Return all the joint names in the order they are used internally
     */
    const std::vector<std::string>& getJointNames() const;

    /**
     * @brief  Return all the link names in the order they are represented internally
     */
    const std::vector<std::string>& getLinkNames() const;
    
    virtual const bool supportsGroup(const moveit::core::JointModelGroup *jmg,
                                     std::string* error_text_out) const;
  protected:

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param check_consistency Set to true if consistency check needs to be performed
   * @param redundancy The index of the redundant joint
   * @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   * @return True if a valid solution was found, false otherwise
   */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const std::vector<double> &consistency_limits,
                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;
    
    virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices);

  private:
    
    void getRandomConfiguration(std::vector<double>& jnt_array) const;

    void getRandomConfiguration(const std::vector<double> &seed_state,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &jnt_array) const;
      
    bool checkConsistency(const std::vector<double>& seed_state,
                          const std::vector<double> &consistency_limits,
                          const std::vector<double>& solution) const;

    bool timedOut(const ros::WallTime &start_time, double duration) const;

    bool solvePositionIK(const std::vector<double> &ik_seed_state,
                         const std::vector<geometry_msgs::Pose> &ik_poses,
                         std::vector<double> &solution) const;
                         // TODO: double timeout, (?)

    Eigen::VectorXd singleStep(const Eigen::VectorXd& qcur, const Eigen::VectorXd& xdes) const;

    Eigen::VectorXd getPose(const Eigen::VectorXd& qcur) const;
    
    Eigen::MatrixXd getJacobian(const Eigen::VectorXd& qcur) const;

    Eigen::MatrixXd pinv( const Eigen::MatrixXd &b, double rcond ) const;

    int getJointIndex(const std::string &name) const;

    bool isRedundantJoint(unsigned int index) const;

    bool active_; /** Internal variable that indicates whether solvers are configured and ready */

    moveit_msgs::KinematicSolverInfo ik_chain_info_; /** Stores information for the inverse kinematics solver */

    moveit_msgs::KinematicSolverInfo fk_chain_info_; /** Store information for the forward kinematics solver */

    unsigned int dimension_; /** Dimension of the group */

    mutable random_numbers::RandomNumberGenerator random_number_generator_;

    robot_model::RobotModelPtr robot_model_;

    robot_state::RobotStatePtr state_, state_2_;

    int num_possible_redundant_joints_;
    std::vector<unsigned int> redundant_joints_map_index_;

    // Storage required for when the set of redundant joints is reset
    bool position_ik_; //whether this solver is only being used for position ik
    robot_model::JointModelGroup* joint_model_group_;
    double max_solver_iterations_;
    double epsilon_;
  };
}

#endif
