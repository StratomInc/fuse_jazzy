/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <fuse_models/parameters/parameter_base.h>

#include <fuse_core/loss.h>
#include <fuse_core/parameter.h>
#include <fuse_variables/orientation_2d_stamped.h>
#include <fuse_variables/position_2d_stamped.h>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace fuse_models
{

namespace parameters
{

/**
 * @brief Defines the set of parameters required by the Gnss class
 */
struct GnssParams : public ParameterBase
{
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] node - The ROS node handle with which to load parameters
   */
  void loadFromROS(    fuse_core::node_interfaces::NodeInterfaces<
      fuse_core::node_interfaces::Base,
      fuse_core::node_interfaces::Logging,
      fuse_core::node_interfaces::Parameters
    > interfaces,
    const std::string & ns) final
  {
    position_indices =
        loadSensorConfig<fuse_variables::Position2DStamped>(interfaces, fuse_core::joinParameterName(
        ns,
        "position_dimensions"));
    
    // node->declare_parameter<bool>("differential", differential);
    differential = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "differential"),
      differential);

    // node->declare_parameter<bool>("disable_checks", disable_checks);
        disable_checks =
      fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "disable_checks"),
      disable_checks);

    // node->declare_parameter<int>("queue_size", queue_size);
    queue_size = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "queue_size"),
      queue_size);

    // node->declare_parameter<bool>("tcp_no_delay", tcp_no_delay);
    tcp_no_delay = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "tcp_no_delay"),
      tcp_no_delay);

    // node->declare_parameter<std::string>("reference_heading_topic", reference_heading_topic);
    reference_heading_topic = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "reference_heading_topic"),
      reference_heading_topic);
    
    // node->declare_parameter<bool>("use_reference_heading", use_reference_heading);
    use_reference_heading = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "use_reference_heading"),
      use_reference_heading);

    
    // node->declare_parameter<double>("magnetic_declination_radians", magnetic_declination_radians);
    magnetic_declination_radians = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "magnetic_declination_radians"),
      magnetic_declination_radians);

    
    // node->declare_parameter<double>("yaw_offset", yaw_offset);
    yaw_offset = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "yaw_offset"),
      yaw_offset);

    

    // node->get_parameter("differential", differential);
    // node->get_parameter("disable_checks", disable_checks);
    // node->get_parameter("queue_size", queue_size);
    // node->get_parameter("tcp_no_delay", tcp_no_delay);
    // node->get_parameter("reference_heading_topic", reference_heading_topic);
    // node->get_parameter("use_reference_heading", use_reference_heading);
    // node->get_parameter("magnetic_declination_radians", magnetic_declination_radians);
    // node->get_parameter("yaw_offset", yaw_offset);

    
    fuse_core::getPositiveParam(interfaces, "tf_timeout", tf_timeout, false);

    fuse_core::getPositiveParam(interfaces, "throttle_period", throttle_period, false);
    // nh.getParam("throttle_use_wall_time", throttle_use_wall_time);
    throttle_use_wall_time = true;

    fuse_core::getParamRequired(interfaces,  fuse_core::joinParameterName(
        ns,
        "topic"), topic);
    // node->declare_parameter<std::string>("target_frame", target_frame);
    // node->get_parameter("target_frame", target_frame);

    target_frame = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "target_frame"),
      target_frame);

    if (differential)
    {
    //   node->declare_parameter<bool>("independent", independent);
    //   node->get_parameter("independent", independent);

    independent = fuse_core::getParam(
      interfaces, fuse_core::joinParameterName(
        ns,
        "independent"),
      independent);

      if (!independent)
      {
        minimum_pose_relative_covariance = fuse_core::getCovarianceDiagonalParam<3>(
            interfaces, "minimum_pose_relative_covariance_diagonal", 0.0);
      }
    }

    loss = fuse_core::loadLossConfig(interfaces, "loss");
  }

  std::string reference_heading_topic{"imu/data"};
  double magnetic_declination_radians{ 0.0 };
  double yaw_offset{ 0.0 };
  bool differential{ false };
  bool disable_checks{ false };
  bool independent{ true };
  bool use_reference_heading{ false };
  fuse_core::Matrix3d minimum_pose_relative_covariance;  //!< Minimum pose relative covariance matrix
  int queue_size{ 10 };
  bool tcp_no_delay{
    false
  };  //!< Whether to use TCP_NODELAY, i.e. disable Nagle's algorithm, in the subscriber
      //!< socket or not. TCP_NODELAY forces a socket to send the data in its buffer,
      //!< whatever the packet size. This reduces delay at the cost of network congestion,
      //!< specially if the payload of a packet is smaller than the TCP header data. This is
      //!< true for small ROS messages like geometry_msgs::AccelWithCovarianceStamped
  rclcpp::Duration tf_timeout{
    0, 0
  };  //!< The maximum time to wait for a transform to become available
  rclcpp::Duration throttle_period{ 0, 0 };  //!< The throttle period duration in seconds
  bool throttle_use_wall_time{ false };      //!< Whether to throttle using ros::WallTime or not
  std::string topic{};
  std::string target_frame{};
  std::vector<size_t> position_indices;
  std::vector<size_t> orientation_indices;
  fuse_core::Loss::SharedPtr loss;
};

}  // namespace parameters

}  // namespace fuse_models