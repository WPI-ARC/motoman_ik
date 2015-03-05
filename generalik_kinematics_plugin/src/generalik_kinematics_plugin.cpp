
#include <moveit/generalik_kinematics_plugin/generalik_kinematics_plugin.h>
#include <class_loader/class_loader.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

// Logging
#include "log4cxx/logger.h"

//register GeneralIKKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(generalik_kinematics_plugin::GeneralIKKinematicsPlugin, kinematics::KinematicsBase)

namespace generalik_kinematics_plugin
{

GeneralIKKinematicsPlugin::GeneralIKKinematicsPlugin():active_(false) {
  log4cxx::LoggerPtr my_logger =  log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
}

void GeneralIKKinematicsPlugin::getRandomConfiguration(std::vector<double>& jnt_array) const
{
  std::vector<double> jnt_array_vector(dimension_, 0.0);
  state_->setToRandomPositions(joint_model_group_);
  state_->copyJointGroupPositions(joint_model_group_, &jnt_array_vector[0]);
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array[i] = jnt_array_vector[i];
  }
}

void GeneralIKKinematicsPlugin::getRandomConfiguration(const std::vector<double> &seed_state,
                                                 const std::vector<double> &consistency_limits,
                                                 std::vector<double> &jnt_array) const
{
  std::vector<double> values(dimension_, 0.0);
  std::vector<double> near(dimension_, 0.0);
  for (std::size_t i = 0 ; i < dimension_; ++i)
    near[i] = seed_state[i];

  joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), values, near, std::vector<double>());
  
  for (std::size_t i = 0; i < dimension_; ++i)
  {
    jnt_array[i] = values[i];
  }
}

bool GeneralIKKinematicsPlugin::checkConsistency(const std::vector<double>& seed_state,
                                                 const std::vector<double>& consistency_limits,
                                                 const std::vector<double>& solution) const
{
  for (std::size_t i = 0; i < dimension_; ++i)
    if (fabs(seed_state[i] - solution[i]) > consistency_limits[i])
      return false;
  return true;
}

bool GeneralIKKinematicsPlugin::initialize(const std::string &robot_description,
                                     const std::string& group_name,
                                     const std::string& base_frame,
                                     const std::vector<std::string>& tip_frames,
                                     double search_discretization)
{
  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  ROS_WARN_STREAM_NAMED("generalik","Initializing "<<group_name<<", "<<base_frame<<", "<<tip_frames.size());
  ROS_WARN_STREAM_NAMED("generalik","Initializing "<<group_name<<", "<<base_frame<<", "<<tip_frames[0]);
  ROS_WARN_STREAM_NAMED("generalik","Initializing "<<group_name<<", "<<base_frame<<", "<<tip_frames[1]);

  ros::NodeHandle private_handle("~");
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("generalik","URDF and SRDF must be loaded for GeneralIK kinematics solver to work.");
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group)
    return false;
  
  if(!joint_model_group->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("generalik","Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  dimension_ = joint_model_group->getActiveJointModels().size() + joint_model_group->getMimicJointModels().size();
  for (std::size_t i=0; i < joint_model_group->getJointModels().size(); ++i)
  {
    if(joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE || joint_model_group->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
    {
      ik_chain_info_.joint_names.push_back(joint_model_group->getJointModelNames()[i]);
      ROS_DEBUG_STREAM_NAMED("generalik","Joint[" << i <<"]: "
                             << joint_model_group->getJointModelNames()[i]);
      const std::vector<moveit_msgs::JointLimits> &jvec = joint_model_group->getJointModels()[i]->getVariableBoundsMsg();
      ik_chain_info_.limits.insert(ik_chain_info_.limits.end(), jvec.begin(), jvec.end());
    }
  }

  fk_chain_info_.joint_names = ik_chain_info_.joint_names;
  fk_chain_info_.limits = ik_chain_info_.limits;

  for (std::size_t i=0; i < getTipFrames().size(); ++i)
  {
    if(!joint_model_group->hasLinkModel(getTipFrames()[i]))
    {
      ROS_ERROR_NAMED("generalik","Could not find tip name in joint group '%s'", group_name.c_str());
      return false;
    }
    ik_chain_info_.link_names.push_back(getTipFrames()[0]);
  }
  fk_chain_info_.link_names = joint_model_group->getLinkModelNames();

  // joint_min_.resize(ik_chain_info_.limits.size());
  // joint_max_.resize(ik_chain_info_.limits.size());

  // for(unsigned int i=0; i < ik_chain_info_.limits.size(); i++)
  // {
  //   joint_min_(i) = ik_chain_info_.limits[i].min_position;
  //   joint_max_(i) = ik_chain_info_.limits[i].max_position;
  // }

  // Get Solver Parameters
  int max_solver_iterations;
  double epsilon;
  bool position_ik;

  private_handle.param("max_solver_iterations", max_solver_iterations, 500);
  private_handle.param("epsilon", epsilon, 1e-5);
  private_handle.param(group_name+"/position_only_ik", position_ik, false);
  ROS_DEBUG_NAMED("generalik","Looking in private handle: %s for param name: %s",
            private_handle.getNamespace().c_str(),
            (group_name+"/position_only_ik").c_str());

  if(position_ik)
    ROS_INFO_NAMED("generalik","Using position only ik");

  // Setup the joint state groups that we need
  state_.reset(new robot_state::RobotState(robot_model_));
  state_2_.reset(new robot_state::RobotState(robot_model_));

  // Store things for when the set of redundant joints may change
  position_ik_ = position_ik;
  joint_model_group_ = joint_model_group;
  max_solver_iterations_ = max_solver_iterations;
  epsilon_ = epsilon;

  active_ = true;
  ROS_DEBUG_NAMED("generalik","GeneralIK solver initialized");
  return true;
}

bool GeneralIKKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int> &redundant_joints)
{
  logError("moveit.generalik_kinematics_plugin: Does not support redundant joints.");
  return false;
}

int GeneralIKKinematicsPlugin::getJointIndex(const std::string &name) const
{
  for (unsigned int i=0; i < ik_chain_info_.joint_names.size(); i++) {
    if (ik_chain_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool GeneralIKKinematicsPlugin::timedOut(const ros::WallTime &start_time, double duration) const
{
  return ((ros::WallTime::now()-start_time).toSec() >= duration);
}

bool GeneralIKKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool GeneralIKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool GeneralIKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool GeneralIKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool GeneralIKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool GeneralIKKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const std::vector<double> &consistency_limits,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);
  return searchPositionIK(ik_poses,
                          ik_seed_state,
                          timeout,
                          consistency_limits,
                          solution,
                          solution_callback,
                          error_code,
                          options);
}
  
bool GeneralIKKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options,
                                           const moveit::core::RobotState* context_state) const
{
  ROS_WARN_STREAM_NAMED("generalik","Searching for "<<ik_poses.size()<<" solutions.");
  // geometry_msgs::Pose ik_pose = ik_poses[0];
  
  ros::WallTime n1 = ros::WallTime::now();
  if(!active_)
  {
    ROS_ERROR_NAMED("generalik","kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("generalik","Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("generalik","Consistency limits be empty or must have size " << dimension_ << " instead of size " << consistency_limits.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  
  solution.resize(dimension_);

  ROS_DEBUG_STREAM_NAMED("generalik","searchPositionIK2: Position request pose[0] is " <<
                   ik_poses[0].position.x << " " <<
                   ik_poses[0].position.y << " " <<
                   ik_poses[0].position.z << " " <<
                   ik_poses[0].orientation.x << " " <<
                   ik_poses[0].orientation.y << " " <<
                   ik_poses[0].orientation.z << " " <<
                   ik_poses[0].orientation.w);
  ROS_DEBUG_STREAM_NAMED("generalik","searchPositionIK2: Position request pose[1] is " <<
                   ik_poses[1].position.x << " " <<
                   ik_poses[1].position.y << " " <<
                   ik_poses[1].position.z << " " <<
                   ik_poses[1].orientation.x << " " <<
                   ik_poses[1].orientation.y << " " <<
                   ik_poses[1].orientation.z << " " <<
                   ik_poses[1].orientation.w);  

  // Do the IK
  std::vector<double> ik_in_state = ik_seed_state;
  std::vector<double> ik_out_state(dimension_);

  unsigned int counter(0);
  while(1)
  {
    ROS_DEBUG_NAMED("generalik","Iteration: %d, time: %f, Timeout: %f",counter,(ros::WallTime::now()-n1).toSec(), timeout);
    counter++;
    if(timedOut(n1,timeout))
    {
      ROS_DEBUG_NAMED("generalik","IK timed out");
      error_code.val = error_code.TIMED_OUT;
      return false;
    }
    // bool ik_valid = false;
    bool ik_valid = solvePositionIK(ik_in_state, ik_poses, ik_out_state);
    ROS_DEBUG_NAMED("generalik","IK valid: %d", ik_valid);
    if(!consistency_limits.empty())
    {
      getRandomConfiguration(ik_in_state, consistency_limits, ik_out_state);
      if( (!ik_valid && !options.return_approximate_solution) || !checkConsistency(ik_in_state, consistency_limits, ik_out_state))
      {
        ROS_DEBUG_NAMED("generalik","Could not find IK solution: does not match consistency limits");
        continue;
      }
    }
    else
    {
      getRandomConfiguration(ik_in_state);
      ROS_DEBUG_NAMED("generalik","New random configuration");
      for(unsigned int j=0; j < dimension_; j++)
        ROS_DEBUG_NAMED("generalik","%d %f", j, ik_in_state[j]);

      if(!ik_valid && !options.return_approximate_solution)
      {
        ROS_DEBUG_NAMED("generalik","Could not find IK solution");
        continue;
      }
    }
    ROS_DEBUG_NAMED("generalik","Found IK solution");
    for(unsigned int j=0; j < dimension_; j++)
      solution[j] = ik_out_state[j];
    if(!solution_callback.empty())
      solution_callback(ik_poses[0],solution,error_code);
    else
      error_code.val = error_code.SUCCESS;

    if(error_code.val == error_code.SUCCESS)
    {
      ROS_DEBUG_STREAM_NAMED("generalik","Solved after " << counter << " iterations");
      for(unsigned int j=0; j < dimension_; j++)
        ROS_DEBUG_NAMED("generalik","%d %f", j, solution[j]);
      return true;
    }
  }
  ROS_DEBUG_NAMED("generalik","An IK that satisfies the constraints and is collision free could not be found");
  error_code.val = error_code.NO_IK_SOLUTION;
  return false; 
}

bool GeneralIKKinematicsPlugin::solvePositionIK(const std::vector<double> &ik_seed_state,
                                                const std::vector<geometry_msgs::Pose> &ik_poses,
                                                std::vector<double> &solution) const
{
  // TODO: Pull out
  int nb_steps_ = 100;
  double tolerance_ = 0.002;
  bool check_joint_limits_ = false;

  // ROS_DEBUG_NAMED("generalik","SolvePositionIK()");
  
  Eigen::VectorXd xdes(12);

  // Left Arm
  xdes(0) = ik_poses[0].position.x;
  xdes(1) = ik_poses[0].position.y;
  xdes(2) = ik_poses[0].position.z;
  tf::Quaternion q;
  tf::quaternionMsgToTF(ik_poses[0].orientation, q);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  xdes(3) = roll;
  xdes(4) = pitch;
  xdes(5) = yaw;

  // Right Arm
  xdes(6) = ik_poses[1].position.x;
  xdes(7) = ik_poses[1].position.y;
  xdes(8) = ik_poses[1].position.z;
  tf::quaternionMsgToTF(ik_poses[1].orientation, q);
  tf::Matrix3x3 m2(q);
  m2.getRPY(roll, pitch, yaw);
  xdes(9) = roll;
  xdes(10) = pitch;
  xdes(11) = yaw;
  
  ROS_DEBUG_STREAM_NAMED("generalik","Target Pose: " << std::endl << xdes);
  
  // Probably don't need
  // Move3D::confPtr_t q_proj = robot_->getCurrentPos();
  // robot_->setAndUpdate(*q_proj);

  // ROS_DEBUG_NAMED("generalik","Preparing for IK loop");    
  Eigen::VectorXd qcur(ik_seed_state.size());
  for(unsigned int i=0; i < dimension_; i++) {
    qcur(i) = ik_seed_state[i];
  }
  double dist=.0;
  bool succeed = false;
  Eigen::VectorXd qnew;
  ROS_DEBUG_STREAM_NAMED("generalik","Initial Pose: " << std::endl << getPose(qcur));

  // ROS_DEBUG_NAMED("generalik","Running IK loop");  
  for( int i=0; i<nb_steps_; i++) // IK LOOP
  {
    if( check_joint_limits_ )
      // TODO: implement
      // q_new = q + single_step_joint_limits( xdes );
      continue;
    else
      qnew = qcur + singleStep(qcur, xdes);
    qcur = qnew;
    // Eigen::Affine3d pose = getPose(qcur);
    // Eigen::Quaterniond q(pose.rotation());
    // dist = fabs(q.dot(quat_dest));
    ROS_DEBUG_STREAM_NAMED("generalik","Pose: " << std::endl << getPose(qcur));
    dist = (xdes-getPose(qcur)).norm();
    ROS_DEBUG_STREAM_NAMED("generalik","diff = " << dist);
    if( dist < tolerance_ ){
      ROS_DEBUG_STREAM_NAMED("generalik","success (" << i << "), diff = " << dist);
      succeed = true;
      break;
    }
  }

  for(unsigned int i=0; i < dimension_; i++) {
    solution[i] = qcur(i);
  }
  return true;
  // return succeed;
}

Eigen::VectorXd GeneralIKKinematicsPlugin::singleStep(const Eigen::VectorXd& qcur,
                                                      const Eigen::VectorXd& xdes) const
{
  // TODO: Pull out
  double magnitude_ = 0.1;

  // ROS_DEBUG_NAMED("generalik","Running single step");  
  // Eigen::Affine3d xcur_aff = getPose(qcur);
  // Eigen::VectorXd xcur(6);
  // Eigen::Quaterniond q(xcur_aff.rotation());
  // xcur << xcur_aff.translation(), q.coeffs();
  Eigen::VectorXd xcur = getPose(qcur);
  Eigen::VectorXd x_error = xdes - xcur;
  Eigen::MatrixXd J = getJacobian(qcur);
  Eigen::MatrixXd Jplus = pinv(J, 1e-3);
  // ROS_DEBUG_STREAM_NAMED("generalik","J^t: " << std::endl << Jplus);
  Eigen::VectorXd dq = Jplus * magnitude_ * x_error;
  // ROS_DEBUG_STREAM_NAMED("generalik","dq: " << dq);
  return dq;
}

Eigen::VectorXd GeneralIKKinematicsPlugin::getPose(const Eigen::VectorXd& qcur) const
{
  // ROS_DEBUG_NAMED("generalik","Getting Pose");  
  // TODO: optimize by making global?
  moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));
  std::vector<double> qvec(qcur.data(), qcur.data() + qcur.size());
  kinematic_state->setVariablePositions(getJointNames(), qvec);
  const Eigen::Affine3d &left_state = kinematic_state->getGlobalLinkTransform(getTipFrames()[0]);
  const Eigen::Affine3d &right_state = kinematic_state->getGlobalLinkTransform(getTipFrames()[1]);
  
  Eigen::VectorXd pose(12);

  // Left arm
  pose(0) = left_state.translation()(0);
  pose(1) = left_state.translation()[1];
  pose(2) = left_state.translation()(2);
  tf::Matrix3x3 m(left_state.rotation()(0,0), left_state.rotation()(0,1), left_state.rotation()(0,2),
                  left_state.rotation()(1,0), left_state.rotation()(1,1), left_state.rotation()(1,2),
                  left_state.rotation()(2,0), left_state.rotation()(2,1), left_state.rotation()(2,2));
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose(3) = roll;
  pose(4) = pitch;
  pose(5) = yaw;
  
  pose(6) = right_state.translation()(0);
  pose(7) = right_state.translation()[1];
  pose(8) = right_state.translation()(2);
  tf::Matrix3x3 m2(right_state.rotation()(0,0), right_state.rotation()(0,1), right_state.rotation()(0,2),
                   right_state.rotation()(1,0), right_state.rotation()(1,1), right_state.rotation()(1,2),
                   right_state.rotation()(2,0), right_state.rotation()(2,1), right_state.rotation()(2,2));
  m2.getRPY(roll, pitch, yaw);
  pose(9) = roll;
  pose(10) = pitch;
  pose(11) = yaw;
  
  return pose;
}

Eigen::MatrixXd GeneralIKKinematicsPlugin::getJacobian(const Eigen::VectorXd& qcur) const
{
  // ROS_DEBUG_NAMED("generalik","Getting Jacobian");  
  // TODO: optimize by making global?
  moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model_));
  std::vector<double> qvec(qcur.data(), qcur.data() + qcur.size());
  kinematic_state->setVariablePositions(getJointNames(), qvec);
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd J, J_left, J_right;
  // Eigen::MatrixXd J, J_left, J_right, J_torso_left, J_torso_right;
  bool success = kinematic_state->getJacobian(robot_model_->getJointModelGroup("arm_left"), // TODO: Abstract out group
                                              kinematic_state->getLinkModel(getTipFrames()[0]),
                                              reference_point_position,
                                              J_left);
  if (!success) {
    ROS_ERROR_STREAM_NAMED("generalik","Error calculating J_left: " << std::endl << J_left);
  }
  success = kinematic_state->getJacobian(robot_model_->getJointModelGroup("arm_right"), // TODO: Abstract out group
                                         kinematic_state->getLinkModel(getTipFrames()[1]),
                                         reference_point_position,
                                         J_right);
  if (!success) {
    ROS_ERROR_STREAM_NAMED("generalik","Error calculating J_right: " << std::endl << J_right);
  }

  J.resize(12, 15);
  J << J_left.block<3,1>(0, 0), J_left.block<3,7>(0, 1), Eigen::MatrixXd::Zero(3, 7),
       J_left.block<3,1>(3, 0), J_left.block<3,7>(3, 1), Eigen::MatrixXd::Zero(3, 7),
      J_right.block<3,1>(0, 0), Eigen::MatrixXd::Zero(3, 7), J_right.block<3,7>(0, 1),
      J_right.block<3,1>(3, 0), Eigen::MatrixXd::Zero(3, 7), J_right.block<3,7>(3, 1);
  // ROS_DEBUG_STREAM_NAMED("generalik","J: " << std::endl << J);
  return J;
}

// Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
// see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
Eigen::MatrixXd GeneralIKKinematicsPlugin::pinv( const Eigen::MatrixXd &b, double rcond ) const
{
  // TODO: Figure out why it wants fewer rows than columns
  // if ( a.rows()<a.cols() )
  // return false;
  bool flip = false;
  Eigen::MatrixXd a;
  if( a.rows() < a.cols() )
  {
    a = b.transpose();
    flip = true;
  }
  else
    a = b;

  // SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svdA;
  svdA.compute( a, Eigen::ComputeFullU | Eigen::ComputeThinV );

  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType vSingular = svdA.singularValues();

  // Build a diagonal matrix with the Inverted Singular values
  // The pseudo inverted singular matrix is easy to compute :
  // is formed by replacing every nonzero entry by its reciprocal (inversing).
  Eigen::VectorXd vPseudoInvertedSingular( svdA.matrixV().cols() );

  for (int iRow=0; iRow<vSingular.rows(); iRow++)
  {
    if ( fabs(vSingular(iRow)) <= rcond ) // Todo : Put epsilon in parameter
    {
      vPseudoInvertedSingular(iRow)=0.;
    }
    else
      vPseudoInvertedSingular(iRow)=1./vSingular(iRow);
  }

  // A little optimization here
  Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block( 0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols() );

  // Pseudo-Inversion : V * S * U'
  Eigen::MatrixXd a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;

  if( flip )
  {
    a = a.transpose();
    a_pinv = a_pinv.transpose();
  }

  return a_pinv;
}
  
bool GeneralIKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
  // ros::WallTime n1 = ros::WallTime::now();
  // if(!active_)
  // {
  //   ROS_ERROR_NAMED("generalik","kinematics not active");
  //   return false;
  // }
  // poses.resize(link_names.size());
  // if(joint_angles.size() != dimension_)
  // {
  //   ROS_ERROR_NAMED("generalik","Joint angles vector must have size: %d",dimension_);
  //   return false;
  // }

  // KDL::Frame p_out;
  // geometry_msgs::PoseStamped pose;
  // tf::Stamped<tf::Pose> tf_pose;

  // KDL::JntArray jnt_pos_in(dimension_);
  // for(unsigned int i=0; i < dimension_; i++)
  // {
  //   jnt_pos_in(i) = joint_angles[i];
  // }

  // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

  // bool valid = true;
  // for(unsigned int i=0; i < poses.size(); i++)
  // {
  //   ROS_DEBUG_NAMED("generalik","End effector index: %d",getKDLSegmentIndex(link_names[i]));
  //   if(fk_solver.JntToCart(jnt_pos_in,p_out,getKDLSegmentIndex(link_names[i])) >=0)
  //   {
  //     tf::poseKDLToMsg(p_out,poses[i]);
  //   }
  //   else
  //   {
  //     ROS_ERROR_NAMED("generalik","Could not compute FK for %s",link_names[i].c_str());
  //     valid = false;
  //   }
  // }
  // return valid;
}

const std::vector<std::string>& GeneralIKKinematicsPlugin::getJointNames() const
{
  return ik_chain_info_.joint_names;
}

const std::vector<std::string>& GeneralIKKinematicsPlugin::getLinkNames() const
{
  return ik_chain_info_.link_names;
}

const bool GeneralIKKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
                                              std::string* error_text_out) const
{
  // Default implementation for legacy solvers:
  // if (!jmg->isChain())
  // {
  //   if(error_text_out)
  //   {
  //     *error_text_out = "This plugin only supports joint groups which are chains";
  //   }
  //   return false;
  // }

  return true;
}

} // namespace
