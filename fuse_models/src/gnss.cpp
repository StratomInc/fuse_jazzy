/*
 *  Copyright (c) 2022 Stratom Icc
 *  All rights reserved.
 */
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_models/common/sensor_proc.h>
#include <fuse_models/gnss.h>

#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <utility>
#include <string>

// Register this sensor model with ROS as a plugin.
PLUGINLIB_EXPORT_CLASS(fuse_models::Gnss, fuse_core::SensorModel)

namespace fuse_models
{

Gnss::Gnss()
  : fuse_core::AsyncSensorModel(1)
  , device_id_(fuse_core::uuid::NIL)
  , throttled_callback_(std::bind(&Gnss::process, this, std::placeholders::_1))
{
}

void Gnss::initialize(
    fuse_core::node_interfaces::NodeInterfaces<ALL_FUSE_CORE_NODE_INTERFACES> interfaces,
    const std::string& name, fuse_core::TransactionCallback transaction_callback)
{
  interfaces_ = interfaces;

  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Starting initialize");
  fuse_core::AsyncSensorModel::initialize(interfaces, name, transaction_callback);

  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Finished initialize");
}

void Gnss::onInit()
{
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Starting onInit");
  tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(interfaces_.get_node_clock_interface()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Read settings from the parameter sever
  device_id_ = fuse_variables::loadDeviceId(interfaces_);

  params_.loadFromROS(interfaces_, name_);

  throttled_callback_.setThrottlePeriod(params_.throttle_period);

  if (params_.use_reference_heading)
  {
    // We need to get the heading from the IMU, but our node won't spin until we return from onInit.
    // Create a sub node to get the IMU message and use it to get the Earth referenced heading
    // Then destroy the sub node
    heading_sub_node_ = std::make_shared<rclcpp::Node>("heading_sub_node");
    imu_sub_ = heading_sub_node_->create_subscription<sensor_msgs::msg::Imu>(
        params_.reference_heading_topic, 1,
        [this](const sensor_msgs::msg::Imu& msg) {
          imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>(msg);
        }

        // [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_msg_ = msg; }
    );
  }
  if (params_.position_indices.empty() && params_.orientation_indices.empty())
  {
    // RCLCPP_WARN_STREAM(interfaces_.get_node_logging_interface()->get_logger(), "No dimensions
    // were specified. Data from topic "
    //                                             << node_->get_namespace() << "/" << params_.topic
    //                                             << " will be ignored.");
  }
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Finished onInit");
}

void Gnss::onStart()
{
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Starting onStart");
  // Get an IMU message and use it to get the Earth referenced heading
  rclcpp::Duration wait_duration = rclcpp::Duration(10, 1e7);
  auto end_time = interfaces_.get_node_clock_interface()->get_clock()->now() + wait_duration;
  auto prev_timestamp = interfaces_.get_node_clock_interface()->get_clock()->now();
  while (imu_msg_ == nullptr)
  {
    if (params_.use_reference_heading)
    {
      rclcpp::spin_some(heading_sub_node_);
    }
    auto time = interfaces_.get_node_clock_interface()->get_clock()->now();
    if (time - prev_timestamp > rclcpp::Duration(1, 0))
    {
      // Timestamp jump caused by starting the node; disregard older calculated end time and retry
      end_time = time + wait_duration;
    }
    if (time > end_time)
    {
      RCLCPP_WARN(interfaces_.get_node_logging_interface()->get_logger(),
                  "Unable to retrieve heading from %s", params_.reference_heading_topic.c_str());
      break;
    }
    prev_timestamp = time;
  }
  imu_sub_.reset();

  // Not sure what this was for...
  //   if (!params_.position_indices.empty() || !params_.orientation_indices.empty())
  if (true)
  {
    // subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    //     params_.topic, params_.queue_size, std::bind(&Gnss::process, this, std::placeholders::_1));

    subscriber_ = rclcpp::create_subscription<sensor_msgs::msg::NavSatFix>(
        interfaces_, params_.topic, params_.queue_size,
        std::bind(&Gnss::process, this, std::placeholders::_1));
    RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
                "Subscribed to gnss topic ");
  }
  else
  {
    RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
                "Didn't subscribe to gnss topic ");
  }
  //   from_ll_service_ = node_->create_service<robot_localization::srv::FromLL>(
  //       "/fromLL",
  //       std::bind(&Gnss::fromLLServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  const std::string service_name = "/fromLL";
  from_ll_service_ = rclcpp::create_service<robot_localization::srv::FromLL>(
      interfaces_.get_node_base_interface(), interfaces_.get_node_services_interface(),
      service_name,
      std::bind(&Gnss::fromLLServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), cb_group_);
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Finished onStart");
}

void Gnss::onStop()
{
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Starting onStop");
  // subscriber_.shutdown();

  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Finished onStop");
}

void Gnss::process(const sensor_msgs::msg::NavSatFix& msg)
{
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Starting process");
  // Create a transaction object
  auto transaction = fuse_core::Transaction::make_shared();
  transaction->stamp(msg.header.stamp);
  const bool validate = !params_.disable_checks;
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "%lf", msg.latitude);
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "%lf", msg.longitude);

  if (!ignition_)
  {
    RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Set init gnss msg");
    ignition_ = true;
    init_gnss_msg_ = std::make_shared<sensor_msgs::msg::NavSatFix>(msg);
    // [this](sensor_msgs::msg::NavSatFix::SharedPtr msg) { init_gnss_msg_ = msg; };
    return;
  }
  if (params_.differential)
  {
    RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Processing diff");
    processDifferential(msg, validate, *transaction);
  }
  else
  {
    RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Processing navsat");
    common::processNavSat(name(), device_id_, msg, params_.loss, params_.target_frame,
                          params_.position_indices, tf_buffer_, validate, *transaction, imu_msg_,
                          params_.use_reference_heading, init_gnss_msg_, params_.tf_timeout);
  }

  // Send the transaction object to the plugin's parent
  sendTransaction(transaction);
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(), "Finished process");
}

void Gnss::fromLLServiceCallback(robot_localization::srv::FromLL::Request::SharedPtr request,
                                 robot_localization::srv::FromLL::Response::SharedPtr response)
{
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
              "Starting fromLLServiceCallback");
  // Take data
  // Pack into NavSatFix
  auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
  msg->latitude = request->ll_point.latitude;
  msg->longitude = request->ll_point.longitude;
  msg->altitude = request->ll_point.altitude;

  // Convert to UTM
  std::string utm_zone;
  auto pose_ptr = std::make_unique<geometry_msgs::msg::Pose>();

  if (!common::preprocessNavSat(msg.get(), imu_msg_, params_.use_reference_heading, init_gnss_msg_,
                                params_.magnetic_declination_radians, params_.yaw_offset,
                                pose_ptr.get()))
  {
    (interfaces_.get_node_logging_interface()->get_logger(), "Cannot preprocess NavSatFix message");
    return;
  }
  response->map_point.x = pose_ptr->position.x;
  response->map_point.y = pose_ptr->position.y;
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
              "Converted from lat: %f, lon: %f, alt: %f to x: %f, y: %f", msg->latitude,
              msg->longitude, msg->altitude, response->map_point.x, response->map_point.y);
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
              "Finished fromLLServiceCallback");
  return;
}

void Gnss::processDifferential(const sensor_msgs::msg::NavSatFix& nav_sat_fix, const bool validate,
                               fuse_core::Transaction& transaction)
{
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
              "Starting processDifferential");
  RCLCPP_INFO(interfaces_.get_node_logging_interface()->get_logger(),
              "Finished processDifferential");
  return;  // TODO(ccharland Nov 2022): Revisit Differential config for the GNSS sensors
  // auto transformed_pose = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  // transformed_pose->header.frame_id =
  //     params_.target_frame.empty() ? nav_sat_fix.header.frame_id : params_.target_frame;

  // if (!common::transformMessage(tf_buffer_, nav_sat_fix, *transformed_pose))
  // {
  //   RCLCPP_WARN_STREAM(interfaces_.get_node_logging_interface()->get_logger(), "Cannot transform
  //   pose message with stamp "
  //                                               << nav_sat_fix.header.stamp.sec << " to target frame "
  //                                               << params_.target_frame);
  //   return;
  // }

  // if (previous_gnss_msg_)
  // {
  //   common::processDifferentialPoseWithCovariance(
  //       name(), device_id_, *previous_gnss_msg_, *transformed_pose, params_.independent,
  //       params_.minimum_pose_relative_covariance, params_.loss, params_.position_indices,
  //       params_.orientation_indices, validate, transaction);
  // }

  // previous_gnss_msg_ = std::move(transformed_pose);
}

}  // namespace fuse_models
