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

#include <fuse_models/parameters/gnss_params.h>

#include <fuse_core/async_sensor_model.h>
#include <fuse_core/throttled_callback.h>
#include <fuse_core/uuid.h>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

namespace fuse_models
{

/**
 * @brief An adapter-type sensor that produces absolute or relative pose constraints from
 * information published by another node.
 *
 * This sensor subscribes to a geometry_msgs::PoseWithCovarianceStamped topic and converts each
 * received message into an absolute or relative pose constraint. If the \p differential parameter
 * is set to false (the default), the measurement will be treated as an absolute constraint. If it
 * is set to true, consecutive measurements will be used to generate relative pose constraints.
 *
 * Parameters:
 *  - device_id (uuid string, default: 00000000-0000-0000-0000-000000000000) The device/robot ID to
 * publish
 *  - device_name (string) Used to generate the device/robot ID if the device_id is not provided
 *  - queue_size (int, default: 10) The subscriber queue size for the pose messages
 *  - topic (string) The topic to which to subscribe for the pose messages (required if \p subscribe
 * is true)
 *  - differential (bool, default: false) Whether we should fuse measurements absolutely, or to
 * create relative pose constraints using consecutive measurements.
 *
 * Subscribes:
 *  - \p topic (geometry_msgs::PoseWithCovarianceStamped) Absolute pose information at a given
 * timestamp
 */
class Gnss : public fuse_core::AsyncSensorModel
{
public:
  FUSE_SMART_PTR_DEFINITIONS(Gnss)
  using ParameterType = parameters::GnssParams;

  /**
   * @brief Default constructor
   */
  Gnss();

  /**
   * @brief Destructor
   */
  virtual ~Gnss() = default;

  /**
   * @brief Callback for pose messages
   * @param[in] msg - The pose message to process
   */
  void process(const sensor_msgs::msg::NavSatFix & msg);

protected:
  fuse_core::UUID device_id_;  //!< The UUID of this device


  void initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string & name,
    fuse_core::TransactionCallback transaction_callback) override;

  /**
   * @brief Perform any required initialization for the sensor model
   *
   * This could include things like reading from the parameter server or subscribing to topics. The
   * class's node handles will be properly initialized before SensorModel::onInit() is called.
   * Spinning of the callback queue will not begin until after the call to SensorModel::onInit()
   * completes.
   */
  void onInit() override;

  /**
   * @brief Subscribe to the input topic to start sending transactions to the optimizer
   */
  void onStart() override;

  /**
   * @brief Unsubscribe from the input topic to stop sending transactions to the optimizer
   */
  void onStop() override;

  /**
   * @brief Process a pose message in differential mode
   *
   * @param[in] pose - The pose message to process in differential mode
   * @param[in] validate - Whether to validate the pose or not
   * @param[out] transaction - The generated variables and constraints are added to this transaction
   */
  void processDifferential(const sensor_msgs::msg::NavSatFix& navsatfix, const bool validate,
                           fuse_core::Transaction& transaction);


  fuse_core::node_interfaces::NodeInterfaces<
    fuse_core::node_interfaces::Base,
    fuse_core::node_interfaces::Clock,
    fuse_core::node_interfaces::Logging,
    fuse_core::node_interfaces::Parameters,
    fuse_core::node_interfaces::Topics,
    fuse_core::node_interfaces::Services,
    fuse_core::node_interfaces::Waitables
  > interfaces_;

  ParameterType params_;

  sensor_msgs::msg::NavSatFix::SharedPtr previous_gnss_msg_{nullptr};
  sensor_msgs::msg::NavSatFix::SharedPtr init_gnss_msg_{nullptr};
  sensor_msgs::msg::Imu::SharedPtr imu_msg_{nullptr};

  std::shared_ptr<rclcpp::Node> heading_sub_node_{nullptr};
  void fromLLServiceCallback(robot_localization::srv::FromLL::Request::SharedPtr request,
  robot_localization::srv::FromLL::Response::SharedPtr response);

  bool ignition_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<robot_localization::srv::FromLL>::SharedPtr from_ll_service_;

  using PoseThrottledCallback = fuse_core::ThrottledMessageCallback<sensor_msgs::msg::NavSatFix>;
  PoseThrottledCallback throttled_callback_;
};

}  // namespace fuse_models