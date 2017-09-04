/**
 @file    nlp_optimizer_node.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 21, 2016
 @brief   Defines the ROS node that initializes/calls the NLP optimizer.
 */

#include <xpp/ros/nlp_optimizer_node.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/transport_hints.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Int32.h>

#include <kindr/rotations/Rotation.hpp>

#include <xpp/optimization_parameters.h>
#include <xpp/state.h>
#include <xpp/ros/ros_conversions.h>
#include <xpp/ros/topic_names.h>
#include <xpp/height_map.h>

namespace xpp {
namespace ros {


NlpOptimizerNode::NlpOptimizerNode ()
{
  ::ros::NodeHandle n;

  user_command_sub_ = n.subscribe(xpp_msgs::user_command, 1,
                                &NlpOptimizerNode::UserCommandCallback, this);

  current_state_sub_ = n.subscribe(xpp_msgs::curr_robot_state_real,
                                    1, // take only the most recent information
                                    &NlpOptimizerNode::CurrentStateCallback, this,
                                    ::ros::TransportHints().tcpNoDelay());

  cart_trajectory_pub_  = n.advertise<xpp_msgs::RobotStateCartesianTrajectory>
                                          (xpp_msgs::robot_trajectory_cart, 1);

  opt_parameters_pub_  = n.advertise<xpp_msgs::OptParameters>
                                    (xpp_msgs::opt_parameters, 1);

  dt_          = RosConversions::GetDoubleFromServer("/xpp/trajectory_dt");
  rosbag_name_ = RosConversions::GetStringFromServer("/xpp/rosbag_name") + ".bag";
}

void
NlpOptimizerNode::CurrentStateCallback (const xpp_msgs::RobotStateCartesian& msg)
{
  auto curr_state = RosConversions::RosToXpp(msg);
  SetInitialState(curr_state);

//  ROS_INFO_STREAM("Received Current Real State");
//  std::cout << curr_state.GetBase() << std::endl;
//  for (auto p : curr_state.GetEEPos().ToImpl()) {
//    std::cout << p << std::endl;
//  }

//  OptimizeMotion();
//  PublishTrajectory();
}

void
NlpOptimizerNode::SetInitialState (const RobotStateCartesian& initial_state)
{
  motion_optimizer_.initial_ee_W_ = initial_state.GetEEPos();

  motion_optimizer_.inital_base_     = State3dEuler(); // zero
  motion_optimizer_.inital_base_.lin = initial_state.base_.lin;

  kindr::RotationQuaternionD quat(initial_state.base_.ang.q);
  kindr::EulerAnglesZyxD euler(quat);
  euler.setUnique(); // to express euler angles close to 0,0,0, not 180,180,180 (although same orientation)
  motion_optimizer_.inital_base_.ang.p_ = euler.toImplementation().reverse();
  // assume zero euler rates and euler accelerations
}

void
NlpOptimizerNode::UserCommandCallback(const xpp_msgs::UserCommand& msg)
{
  motion_optimizer_.terrain_ = opt::HeightMap::MakeTerrain(static_cast<opt::HeightMap::ID>(msg.terrain_id));
  motion_optimizer_.model_->SetInitialGait(msg.gait_id);
  motion_optimizer_.params_->SetTotalDuration(msg.total_duration);

  motion_optimizer_.nlp_solver_ = msg.use_solver_snopt ? MotionOptimizerFacade::Snopt : MotionOptimizerFacade::Ipopt;
  motion_optimizer_.SetFinalState(RosConversions::RosToXpp(msg.goal_lin), RosConversions::RosToXpp(msg.goal_ang));


  user_command_msg_ = msg;

  opt_parameters_pub_.publish(BuildOptParametersMsg());

  if (msg.optimize) {
    OptimizeMotion();
//    cart_trajectory_pub_.publish(BuildTrajectoryMsg());
    SaveOptimizationAsRosbag();
  }

  if (msg.replay_trajectory || msg.optimize) {
    // PublishTrajectory();
    // play back the rosbag hacky like this, as I can't find appropriate C++ API.
    system(("rosbag play --quiet " + rosbag_name_).c_str());
  }
}

void
NlpOptimizerNode::OptimizeMotion ()
{
  try {
    motion_optimizer_.SolveProblem();
  } catch (const std::runtime_error& e) {
    ROS_ERROR_STREAM("Optimization failed, not sending. " << e.what());
  }
}

xpp_msgs::RobotStateCartesianTrajectory
NlpOptimizerNode::BuildTrajectoryMsg () const
{
  auto cart_traj = motion_optimizer_.GetTrajectory(dt_);
  return RosConversions::XppToRos(cart_traj);
}

xpp_msgs::OptParameters
NlpOptimizerNode::BuildOptParametersMsg() const
{
  xpp_msgs::OptParameters params_msg;
  auto max_dev_xyz = motion_optimizer_.model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = RosConversions::XppToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = motion_optimizer_.model_->GetNominalStanceInBase();
  auto ee_names = motion_optimizer_.model_->GetEndeffectorNames();
  params_msg.ee_names = ee_names;
  for (auto ee : nominal_B.ToImpl()) {
    params_msg.nominal_ee_pos.push_back(RosConversions::XppToRos<geometry_msgs::Point>(ee));
  }

  params_msg.base_mass = motion_optimizer_.model_->GetMass();

  return params_msg;
}

void
NlpOptimizerNode::SaveOptimizationAsRosbag () const
{
  rosbag::Bag bag;
  bag.open(rosbag_name_, rosbag::bagmode::Write);
  ::ros::Time t0(0.001);

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::opt_parameters, t0, BuildOptParametersMsg());
  bag.write(xpp_msgs::user_command+"_saved", t0, user_command_msg_);

  // save the trajectory of each iteration
  auto trajectories = motion_optimizer_.GetIntermediateSolutions(dt_);
  int n_iterations = trajectories.size();
  for (int i=0; i<n_iterations; ++i)
    SaveTrajectoryInRosbag(bag, trajectories.at(i), xpp_msgs::nlp_iterations_name + std::to_string(i));

  // save number of iterations the optimizer took
  std_msgs::Int32 m;
  m.data = n_iterations;
  bag.write(xpp_msgs::nlp_iterations_count, t0, m);

  // save the final trajectory
  SaveTrajectoryInRosbag(bag, trajectories.back(), xpp_msgs::robot_state);

  // optional: save the entire trajectory as one message
  bag.write(xpp_msgs::robot_trajectory_cart, t0, BuildTrajectoryMsg());

  bag.close();
}

void
NlpOptimizerNode::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                          const std::vector<RobotStateCartesian>& traj,
                                          const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ +1e-6); // to avoid t=0.0

    auto state_msg = ros::RosConversions::XppToRos(state);
    bag.write(topic, timestamp, state_msg);
  }
}


} /* namespace ros */
} /* namespace xpp */
