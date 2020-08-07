/*
MIT License

Copyright (c) 2020 Bytes Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <string>
#include <vector>
#include <memory>

#include "ros2_t260/t260.hpp"
#include <fstream>

/**
 * Helper functions for handling saving and loading maps from
 * https://github.com/IntelRealSense/librealsense/blob/master/examples/ar-advanced/rs-ar-advanced.cpp
 */

void raw_file_from_bytes(const std::string & filename, const std::vector<uint8_t> & bytes)
{
  std::ofstream file(filename, std::ios::binary | std::ios::trunc);
  if (!file.good()) {
    throw std::runtime_error(
            "Invalid binary file specified. Verify the target path and location permissions");
  }
  file.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
}

std::vector<uint8_t> bytes_from_raw_file(const std::string & filename)
{
  std::ifstream file(filename.c_str(), std::ios::binary);
  if (!file.good()) {
    throw std::runtime_error(
            "Invalid binary file specified. Verify the source path and location permissions");
  }

  // Determine the file length
  file.seekg(0, std::ios_base::end);
  std::size_t size = file.tellg();
  if (!size) {
    throw std::runtime_error("Invalid binary file -zero-size");
  }
  file.seekg(0, std::ios_base::beg);

  // Create a vector to store the data
  std::vector<uint8_t> v(size);

  // Load the data
  file.read(reinterpret_cast<char *>(&v[0]), size);

  return v;
}

rs2_pose identity_pose()
{
  // Return an identity pose (no translation, no rotation)
  rs2_pose pose;
  pose.translation.x = 0;
  pose.translation.y = 0;
  pose.translation.z = 0;
  pose.rotation.x = 0;
  pose.rotation.y = 0;
  pose.rotation.z = 0;
  pose.rotation.w = 1;
  return pose;
}

/**
 * T260 node definitions
 * @param node_name
 * @param intra_process_comms
 */

T260::T260(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(
      intra_process_comms)),
  transform_listener_(tf_buffer_),
  tf_broadcaster_(this)
{
  configure_params();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring T260 Node");

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS());
  relocalization_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "relocalization", rclcpp::SystemDefaultsQoS().transient_local().reliable());
  odom_in_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/wheel/odom", rclcpp::SensorDataQoS(),
    std::bind(&T260::odom_in_cb, this, std::placeholders::_1));

//    image_transport::Publisher left_pub_ = left_it_.advertise("left/image", 1);
//    image_transport::Publisher right_pub_ = right_it_.advertise("right/image", 1);

  save_map_srv_ = this->create_service<map_msgs::srv::SaveMap>(
    std::string(this->get_name()) + "/save_map",
    std::bind(&T260::save_map_cb, this, std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = this->create_service<map_msgs::srv::SaveMap>(
    std::string(this->get_name()) + "/load_map",
    std::bind(&T260::load_map_cb, this, std::placeholders::_1, std::placeholders::_2));

  std::vector<std::string> serials;
  bool device_available{false};
  for (auto && dev : ctx_.query_devices(RS2_PRODUCT_LINE_T200)) {
    auto serial_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

    std::stringstream ss;
    ss << "T200 series device detected" <<
      "\nDevice Serial No: " << serial_num <<
      "\nDevice physical port: " << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) <<
      "\nDevice FW version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) <<
      "\nDevice Product ID: 0x%s" << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
    RCLCPP_INFO(this->get_logger(), ss.str());

    if (std::strcmp(serial_num_.c_str(), serial_num) == 0 ||
      std::strcmp(serial_num_.c_str(), "") == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Connecting to device with serial number: %s", serial_num);
      serial_num_ = serial_num;
      if (hardware_reset_) {
        dev.hardware_reset();
        RCLCPP_INFO(this->get_logger(), "Hardware reset");
      }
      /// Setup wheel odom input if calibration file is provided
      if (!calib_odom_file_.empty()) {
        std::ifstream calibrationFile(calib_odom_file_);
        if (!calibrationFile) {
          RCLCPP_FATAL_STREAM(
            this->get_logger(),
            "calibration_odometry file not found. calib_odom_file = " <<
              calib_odom_file_);
          throw std::runtime_error("calibration_odometry file not found");
        }
        wheel_odometer_ = std::make_shared<rs2::wheel_odometer>(dev.first<rs2::wheel_odometer>());
        const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
          std::istreambuf_iterator<char>());
        const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

        if (!wheel_odometer_->load_wheel_odometery_config(wo_calib)) {
          RCLCPP_FATAL_STREAM(
            this->get_logger(),
            "Format error in calibration_odometry file: " << calib_odom_file_);
          throw std::runtime_error("Format error in calibration_odometry file");
        }
        use_odom_in_ = true;
      } else {
        RCLCPP_INFO(this->get_logger(), "No calibration file provided, odom input is disabled!");
      }
      device_available = true;
    }
  }

  if (device_available) {
    cfg_.enable_device(serial_num_);

    if (enable_fisheye_streams_) {
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
      cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
    }
    if (enable_pose_stream_) {
      cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    }

    if (!enable_mapping_ && (enable_pose_jumping_ || enable_relocalization_)) {
      RCLCPP_WARN(this->get_logger(), "Mapping disabled may conflict with other configurations");
      enable_pose_jumping_ = false;
      enable_relocalization_ = false;
    }
    tm_sensor_ = std::make_shared<rs2::pose_sensor>(
      cfg_.resolve(pipe_).get_device().first<rs2::pose_sensor>());
    tm_sensor_->set_option(RS2_OPTION_ENABLE_MAPPING, enable_mapping_);
    tm_sensor_->set_option(RS2_OPTION_ENABLE_POSE_JUMPING, enable_pose_jumping_);
    tm_sensor_->set_option(RS2_OPTION_ENABLE_RELOCALIZATION, enable_relocalization_);
    tm_sensor_->set_option(RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, enable_dynamic_calibration_);
    tm_sensor_->set_option(RS2_OPTION_ENABLE_MAP_PRESERVATION, enable_map_preservation_);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_activate(const rclcpp_lifecycle::State &)
{
  odom_pub_->on_activate();
  relocalization_pub_->on_activate();
  tm_sensor_->set_notifications_callback(
    std::bind(&T260::notifications_cb, this, std::placeholders::_1));
  pipe_profile_ = pipe_.start(cfg_, std::bind(&T260::main_cb, this, std::placeholders::_1));
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_deactivate(const rclcpp_lifecycle::State &)
{
  odom_pub_->on_deactivate();
  relocalization_pub_->on_deactivate();
  pipe_.stop();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_cleanup(const rclcpp_lifecycle::State &)
{
  tf_buffer_.clear();
  odom_pub_.reset();
  save_map_srv_.reset();
  load_map_srv_.reset();
  tm_sensor_.reset();
  relocalization_pub_.reset();
  wheel_odometer_.reset();
  odom_in_sub_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void T260::save_map_cb(
  const std::shared_ptr<map_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<map_msgs::srv::SaveMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Saving map to: %s", request->filename.data.c_str());
  /// Set static node for relocalizing on the map when reloaded
  rs2_pose pose = identity_pose();
  tm_sensor_->set_static_node(virtual_object_guid_, pose.translation, pose.rotation);
  /// Export map to a raw file
  auto out_map_filepath = request->filename.data.data();
  raw_file_from_bytes(out_map_filepath, tm_sensor_->export_localization_map());
}

void T260::odom_in_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (use_odom_in_) {
    rs2_vector velocity{-static_cast<float>(msg->twist.twist.linear.y),
      static_cast<float>(msg->twist.twist.linear.z),
      -static_cast<float>(msg->twist.twist.linear.x)};
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "Add odom: " << velocity.x << ", " <<
        velocity.y << ", " << velocity.z);
    wheel_odometer_->send_wheel_odometry(0, 0, velocity);
  }
}


void T260::load_map_cb(
  const std::shared_ptr<map_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<map_msgs::srv::SaveMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", request->filename.data.c_str());
  pipe_.stop();
  tm_sensor_->import_localization_map(bytes_from_raw_file(request->filename.data));
  pipe_profile_ = pipe_.start(cfg_, std::bind(&T260::main_cb, this, std::placeholders::_1));
}

void T260::main_cb(const rs2::frame & frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto now = this->now();
  if (auto fp = frame.as<rs2::pose_frame>()) {
    if (tf_buffer_.canTransform(base_frame_, camera_frame_, tf2::TimePointZero)) {
      auto pose_data = fp.get_pose_data();

      double cov_pose(pose_cov_ * pow(10, 3 - static_cast<int>(pose_data.tracker_confidence)));
      double cov_twist(rotation_cov_ * pow(10, 1 - static_cast<int>(pose_data.tracker_confidence)));

      tf2::Transform odom_to_camera;
      rs2_pose_to_transform(pose_data, odom_to_camera);

      tf2::Transform odom_to_base;
      get_odom_to_base_tf(odom_to_camera, odom_to_base);

      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.transform.translation.x = odom_to_base.getOrigin().x();
      transform_msg.transform.translation.y = odom_to_base.getOrigin().y();
      transform_msg.transform.translation.z = odom_to_base.getOrigin().z();
      transform_msg.transform.rotation.x = odom_to_base.getRotation().x();
      transform_msg.transform.rotation.y = odom_to_base.getRotation().y();
      transform_msg.transform.rotation.z = odom_to_base.getRotation().z();
      transform_msg.transform.rotation.w = odom_to_base.getRotation().w();
      transform_msg.header.stamp = now;
      transform_msg.header.frame_id = odom_frame_;
      transform_msg.child_frame_id = base_frame_;
      if (publish_tf_) {
        tf_broadcaster_.sendTransform(transform_msg);
      }

      if (publish_odom_) {
        geometry_msgs::msg::Vector3Stamped v_msg;
        v_msg.vector.x = -pose_data.velocity.z;
        v_msg.vector.y = -pose_data.velocity.x;
        v_msg.vector.z = pose_data.velocity.y;
        tf2::Vector3 tfv;
        tfv.setX(v_msg.vector.x);
        tfv.setY(v_msg.vector.y);
        tfv.setZ(v_msg.vector.z);
        tf2::Quaternion q(-transform_msg.transform.rotation.x, -transform_msg.transform.rotation.y,
          -transform_msg.transform.rotation.z, transform_msg.transform.rotation.w);
        tfv = tf2::quatRotate(q, tfv);
        v_msg.vector.x = tfv.getX();
        v_msg.vector.y = tfv.getY();
        v_msg.vector.z = tfv.getZ();

        geometry_msgs::msg::Vector3Stamped om_msg;
        om_msg.vector.x = -pose_data.angular_velocity.z;
        om_msg.vector.y = -pose_data.angular_velocity.x;
        om_msg.vector.z = pose_data.angular_velocity.y;
        tfv.setX(om_msg.vector.x);
        tfv.setY(om_msg.vector.y);
        tfv.setZ(om_msg.vector.z);
        tfv = tf2::quatRotate(q, tfv);
        om_msg.vector.x = tfv.getX();
        om_msg.vector.y = tfv.getY();
        om_msg.vector.z = tfv.getZ();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        tf2::toMsg(odom_to_base, odom_msg.pose.pose);
        odom_msg.pose.covariance = {
          cov_pose, 0, 0, 0, 0, 0,
          0, cov_pose, 0, 0, 0, 0,
          0, 0, cov_pose, 0, 0, 0,
          0, 0, 0, cov_twist, 0, 0,
          0, 0, 0, 0, cov_twist, 0,
          0, 0, 0, 0, 0, cov_twist
        };
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        odom_msg.twist.covariance = {
          cov_pose, 0, 0, 0, 0, 0,
          0, cov_pose, 0, 0, 0, 0,
          0, 0, cov_pose, 0, 0, 0,
          0, 0, 0, cov_twist, 0, 0,
          0, 0, 0, 0, cov_twist, 0,
          0, 0, 0, 0, 0, cov_twist
        };
        odom_pub_->publish(odom_msg);
      }
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Unable to get transform between %s and %s.",
        odom_frame_.c_str(),
        camera_frame_.c_str());
    }
  } else if (auto fs = frame.as<rs2::frameset>()) {
//            cv::Mat right(cv::Size(fs.get_fisheye_frame(1).get_width(),
//                    fs.get_fisheye_frame(1).get_height()), CV_8UC1,
//                            (void*)fs.get_fisheye_frame(1).get_data());
//            cv::imshow("Right Image", right);
//            cv::waitKey(1);
//            cv::Mat left(cv::Size(fs.get_fisheye_frame(2).get_width(),
//                    fs.get_fisheye_frame(2).get_height()), CV_8UC1,
//                            (void*)fs.get_fisheye_frame(2).get_data());
//            cv::imshow("Left Image", left);
//            cv::waitKey(1);
  }
}

void T260::notifications_cb(const rs2::notification & n)
{
  if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
    RCLCPP_INFO(this->get_logger(), "Relocalization event detected");
    rs2_pose pose_transform;
    // Get static node if available
    if (tm_sensor_->get_static_node(
        virtual_object_guid_, pose_transform.translation, pose_transform.rotation))
    {
      while (!tf_buffer_.canTransform(base_frame_, camera_frame_, tf2::TimePointZero)) {
        RCLCPP_ERROR(
          this->get_logger(), "Unable to get transform between %s and %s.",
          odom_frame_.c_str(),
          camera_frame_.c_str());
      }

      geometry_msgs::msg::PoseStamped pose_msg, transformed_pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = odom_frame_;

      tf2::Transform odom_to_camera;
      rs2_pose_to_transform(pose_transform, odom_to_camera);

      tf2::Transform odom_to_base;
      get_odom_to_base_tf(odom_to_camera, odom_to_base);

      tf2::toMsg(odom_to_base, pose_msg.pose);
      relocalization_pub_->publish(pose_msg);
    }
  }
}

void T260::configure_params()
{
  hardware_reset_ = this->declare_parameter("hardware_reset", true);
  serial_num_ = this->declare_parameter("serial_number", "");
  enable_fisheye_streams_ = this->declare_parameter("enable_fisheye_streams", true);
  enable_pose_stream_ = this->declare_parameter("enable_pose_stream", true);

  enable_mapping_ = this->declare_parameter("enable_mapping", true);
  enable_dynamic_calibration_ = this->declare_parameter("enable_dynamic_calibration", true);
  enable_relocalization_ = this->declare_parameter("enable_relocalization", true);
  enable_pose_jumping_ = this->declare_parameter("enable_pose_jumping", true);
  enable_map_preservation_ = this->declare_parameter("enable_map_preservation", false);

  publish_odom_ = this->declare_parameter("publish_odom", true);
  publish_tf_ = this->declare_parameter("publish_tf", true);

  odom_frame_ = this->declare_parameter("odom_frame", "odom");
  base_frame_ = this->declare_parameter("base_frame", "base_link");
  camera_frame_ = this->declare_parameter("camera_frame", "t260_link");

  pose_cov_ = this->declare_parameter("position_covariance", 0.1);
  rotation_cov_ = this->declare_parameter("rotation_covariance", 0.1);

  calib_odom_file_ = this->declare_parameter("calib_odom_file", "");

  /// Update parameters dynamically
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    this->get_node_base_interface(), this->get_node_topics_interface(),
    this->get_node_graph_interface(), this->get_node_services_interface());
  auto on_parameter_event_callback = [this](
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
      std::stringstream ss;
      ss << "\nParameter event:\n changed parameters:";
      for (auto & changed_parameter : event->changed_parameters) {
        ss << "\n  " << changed_parameter.name;
        if (changed_parameter.name == "hardware_reset") {
          hardware_reset_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "serial_number") {
          serial_num_ = changed_parameter.value.string_value;
        } else if (changed_parameter.name == "enable_fisheye_streams") {
          enable_fisheye_streams_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "enable_pose_stream") {
          enable_pose_stream_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "enable_mapping") {
          enable_mapping_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "enable_dynamic_calibration") {
          enable_dynamic_calibration_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "enable_relocalization") {
          enable_relocalization_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "enable_pose_jumping") {
          enable_pose_jumping_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "enable_map_preservation") {
          enable_map_preservation_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "publish_odom") {
          publish_odom_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "publish_tf") {
          publish_tf_ = changed_parameter.value.bool_value;
        } else if (changed_parameter.name == "odom_frame") {
          odom_frame_ = changed_parameter.value.string_value;
        } else if (changed_parameter.name == "child_frame") {
          base_frame_ = changed_parameter.value.string_value;
        } else if (changed_parameter.name == "mounted_frame") {
          camera_frame_ = changed_parameter.value.string_value;
        } else if (changed_parameter.name == "position_covariance") {
          pose_cov_ = changed_parameter.value.double_value;
        } else if (changed_parameter.name == "rotation_covariance") {
          rotation_cov_ = changed_parameter.value.double_value;
        } else if (changed_parameter.name == "save_map") {
          rotation_cov_ = changed_parameter.value.double_value;
        } else if (changed_parameter.name == "calib_odom_file") {
          calib_odom_file_ = changed_parameter.value.string_value;
        }
        ss << "\n";
        RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
      }
    };

  /// Setup callback for changes to parameters.
  parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
}

void T260::rs2_pose_to_transform(rs2_pose & rs2_pose, tf2::Transform & transform)
{
  transform.setOrigin({-rs2_pose.translation.z, -rs2_pose.translation.x, rs2_pose.translation.y});
  transform.setRotation(
    {-rs2_pose.rotation.z, -rs2_pose.rotation.x, rs2_pose.rotation.y, rs2_pose.rotation.w});
}

void T260::get_odom_to_base_tf(tf2::Transform & odom_to_camera, tf2::Transform & odom_to_base)
{
  auto base_to_camera_msg = tf_buffer_.lookupTransform(
    base_frame_, camera_frame_, tf2::TimePointZero);
  tf2::Stamped<tf2::Transform> base_to_camera;
  tf2::fromMsg(base_to_camera_msg, base_to_camera);
  odom_to_base = odom_to_camera * base_to_camera.inverse();
}
