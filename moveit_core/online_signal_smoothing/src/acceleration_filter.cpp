/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

#include <moveit/online_signal_smoothing/acceleration_filter.hpp>
#include <rclcpp/logging.hpp>

// Disable -Wold-style-cast because all _THROTTLE macros trigger this
#pragma GCC diagnostic ignored "-Wold-style-cast"

namespace online_signal_smoothing
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.core.acceleration_limited_plugin");
}

bool AccelerationLimitedPlugin::initialize(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
                                           size_t num_joints)
{
  node_ = node;
  num_joints_ = num_joints;
  robot_model_ = robot_model;

  auto param_listener = online_signal_smoothing::ParamListener(node_);
  params_ = param_listener.get_params();

  auto joint_model_group = robot_model_->getJointModelGroup(params_.planning_group_name);
  auto joint_bounds = joint_model_group->getActiveJointModelsBounds();
  min_acceleration_limits_ = Eigen::VectorXd::Zero(num_joints);
  max_acceleration_limits_ = Eigen::VectorXd::Zero(num_joints);
  size_t ind = 0;
  for (const auto& joint_bound : joint_bounds)
  {
    for (const auto& variable_bound : *joint_bound)
    {
      if (variable_bound.acceleration_bounded_)
      {
        min_acceleration_limits_[ind] = variable_bound.min_acceleration_;
        max_acceleration_limits_[ind] = variable_bound.max_acceleration_;
      }
      else
      {
        RCLCPP_ERROR(getLogger(), "The robot must have acceleration joint limits specified for all joints to "
                                  "use AccelerationLimitedPlugin.");
        return false;
      }
    }
    ind++;
  }

  return true;
}

bool AccelerationLimitedPlugin::doSmoothing(Eigen::VectorXd& positions, Eigen::VectorXd& velocities,
                                            Eigen::VectorXd& /* unused */)
{
  const size_t num_positions = velocities.size();
  if (num_positions != num_joints_)
  {
    RCLCPP_ERROR_THROTTLE(
        getLogger(), *node_->get_clock(), 1000,
        "The length of the joint positions parameter is not equal to the number of joints, expected %zu got %zu.",
        num_joints_, num_positions);
    return false;
  }
  else if (last_positions_.size() != positions.size())
  {
    RCLCPP_ERROR_THROTTLE(getLogger(), *node_->get_clock(), 1000,
                          "The length of the last joint positions not equal to the current, expected %zu got %zu. Make "
                          "sure the reset was called.",
                          last_positions_.size(), positions.size());
    return false;
  }

  const double dt = params_.update_period;

  // Find the uniform scaling factor from the most-constrained joint
  // so all joints scale together, preserving Cartesian direction.
  double scale = 1.0;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    double desired_accel = (velocities[i] - last_velocities_[i]) / dt;
    if (desired_accel != 0.0)
    {
      double clamped = std::clamp(desired_accel, min_acceleration_limits_[i], max_acceleration_limits_[i]);
      scale = std::min(scale, std::abs(clamped / desired_accel));
    }
  }

  for (size_t i = 0; i < num_joints_; ++i)
  {
    double desired_accel = (velocities[i] - last_velocities_[i]) / dt;
    double scaled_accel = desired_accel * scale;
    velocities[i] = last_velocities_[i] + scaled_accel * dt;
    positions[i] = last_positions_[i] + velocities[i] * dt;
  }

  last_velocities_ = velocities;
  last_positions_ = positions;

  return true;
}

bool AccelerationLimitedPlugin::reset(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                                      const Eigen::VectorXd& /* unused */)
{
  last_velocities_ = velocities;
  last_positions_ = positions;
  return true;
}

}  // namespace online_signal_smoothing

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(online_signal_smoothing::AccelerationLimitedPlugin, online_signal_smoothing::SmoothingBaseClass)
