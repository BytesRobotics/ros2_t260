//
// Created by michael on 7/16/20.
//

#ifndef ROS2_T260_T260_H
#define ROS2_T260_T260_H

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "map_msgs/srv/save_map.hpp"

// ros2 param set /t260_node calib_odom_file "/home/michael/Github/br_core/ws/src/deps/ros2_t260/config/calibration_odometry.json"
class T260 : public rclcpp_lifecycle::LifecycleNode {

  const std::string virtual_object_guid_ = "node0";

  std::mutex mutex_;
  rs2::context ctx_; // Create librealsense context for managing devices
  rs2::pipeline pipe_;
  rs2::config cfg_;
  std::shared_ptr<rs2::pose_sensor> tm_sensor_;
  rs2::pipeline_profile pipe_profile_;
  std::shared_ptr<rs2::wheel_odometer> wheel_odometer_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener transform_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  /// Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> relocalization_pub_;

  // See (https://answers.ros.org/question/312870/ros2-node-pointer-from-a-lifecyclenode/) for issue
  // image_transport::ImageTransport left_it_, right_it_;

  /// Subscribers
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> odom_in_sub_;
  bool use_odom_in_{false};

  /// Services
  rclcpp::Service<map_msgs::srv::SaveMap>::SharedPtr save_map_srv_, load_map_srv_;

  /// Dynamically reconfigurable parameters
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  /// Parameters
  bool hardware_reset_{}, enable_fisheye_streams_, enable_pose_stream_;
  bool enable_mapping_, enable_pose_jumping_, enable_relocalization_, enable_dynamic_calibration_,
    enable_map_preservation_;
  std::string serial_num_;
  std::string odom_frame_, base_frame_, camera_frame_;
  bool publish_odom_, publish_tf_;
  double pose_cov_, rotation_cov_;
  std::string calib_odom_file_; //https://github.com/IntelRealSense/librealsense/pull/3462

  void notifications_cb(const rs2::notification &n);

  void odom_in_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

  void main_cb(const rs2::frame &frame);

  void save_map_cb(std::shared_ptr<map_msgs::srv::SaveMap::Request> request,
                   std::shared_ptr<map_msgs::srv::SaveMap::Response> response);

  void load_map_cb(std::shared_ptr<map_msgs::srv::SaveMap::Request> request,
                   std::shared_ptr<map_msgs::srv::SaveMap::Response> response);

  void configure_params();

  inline void rs2_pose_to_transform(rs2_pose& rs2_pose, tf2::Transform& transform);

  inline void get_odom_to_base_tf(tf2::Transform& odom_to_camera, tf2::Transform& odom_to_base);

public:
  explicit T260(const std::string &node_name = "t260_node", bool intra_process_comms = false);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state) override;

};

#endif //ROS2_T260_ROS_T260_H
