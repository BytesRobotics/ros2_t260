//
// Created by michael on 7/16/20.
//

#include "t260.h"

T260::T260(const std::string& node_name, bool intra_process_comms):
rclcpp_lifecycle::LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
transform_listener_(tf_buffer_),
tf_broadcaster_(this)
{
    configure_params();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_configure(const rclcpp_lifecycle::State &){

    RCLCPP_INFO(this->get_logger(), "Configuring T260 Node");

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
//    image_transport::Publisher left_pub_ = left_it_.advertise("left/image", 1);
//    image_transport::Publisher right_pub_ = right_it_.advertise("right/image", 1);


    std::vector<std::string> serials;
    bool device_available{false};
    for (auto&& dev : ctx_.query_devices(RS2_PRODUCT_LINE_T200)){
        auto serial_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

        std::stringstream ss;
        ss << "T200 series device detected" <<
        "\nDevice Serial No: " << serial_num <<
        "\nDevice physical port: " << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) <<
        "\nDevice FW version: " << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) <<
        "\nDevice Product ID: 0x%s" << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
        RCLCPP_INFO(this->get_logger(), ss.str());

        if(std::strcmp(serial_num_.c_str(), serial_num) == 0 || std::strcmp(serial_num_.c_str(), "") == 0){
            RCLCPP_INFO(this->get_logger(), "Connecting to device with serial number: %s", serial_num);
            serial_num_ = serial_num;
            if(hardware_reset_){
                dev.hardware_reset();
                RCLCPP_INFO(this->get_logger(), "Hardware reset");
            }
            wheel_odometer_  = std::make_shared<rs2::wheel_odometer>(dev.first<rs2::wheel_odometer>());
            device_available = true;
        }
    }

    if(device_available){
        cfg_.enable_device(serial_num_);

        if(enable_fisheye_streams_){
            cfg_.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
            cfg_.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
        }
        if(enable_pose_stream_){
            cfg_.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        }

        if(!enable_mapping_ && (enable_pose_jumping_ || enable_relocalization_)){
            RCLCPP_WARN(this->get_logger(), "Mapping disabled may conflict with other configurations");
        }
        auto tm_sensor = cfg_.resolve(pipe_).get_device().first<rs2::pose_sensor>();
        tm_sensor.set_option(RS2_OPTION_ENABLE_MAPPING, enable_mapping_);
        tm_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, enable_pose_jumping_);
        tm_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, enable_relocalization_);
        tm_sensor.set_option(RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, enable_dynamic_calibration_);
        tm_sensor.set_option(RS2_OPTION_ENABLE_MAP_PRESERVATION, enable_map_preservation_);

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_activate(const rclcpp_lifecycle::State &){
    odom_pub_->on_activate();

    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = this->get_clock()->now();
        if (auto fp = frame.as<rs2::pose_frame>()) {
            auto pose_data = fp.get_pose_data();

            double cov_pose(pose_cov_ * pow(10, 3-(int)pose_data.tracker_confidence));
            double cov_twist(rotation_cov_ * pow(10, 1-(int)pose_data.tracker_confidence));

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.pose.position.x = -pose_data.translation.z;
            pose_msg.pose.position.y = -pose_data.translation.x;
            pose_msg.pose.position.z = pose_data.translation.y;
            pose_msg.pose.orientation.x = -pose_data.rotation.z;
            pose_msg.pose.orientation.y = -pose_data.rotation.x;
            pose_msg.pose.orientation.z = pose_data.rotation.y;
            pose_msg.pose.orientation.w = pose_data.rotation.w;

            geometry_msgs::msg::TransformStamped transform_msg;
            transform_msg.header.stamp = now;
            transform_msg.header.frame_id = odom_frame_;
            transform_msg.child_frame_id = child_frame_;
            transform_msg.transform.translation.x = pose_msg.pose.position.x;
            transform_msg.transform.translation.y = pose_msg.pose.position.y;
            transform_msg.transform.translation.z = pose_msg.pose.position.z;
            transform_msg.transform.rotation.x = pose_msg.pose.orientation.x;
            transform_msg.transform.rotation.y = pose_msg.pose.orientation.y;
            transform_msg.transform.rotation.z = pose_msg.pose.orientation.z;
            transform_msg.transform.rotation.w = pose_msg.pose.orientation.w;

            if(publish_tf_){
                tf_broadcaster_.sendTransform(transform_msg);
            }

            if(publish_odom_){
                geometry_msgs::msg::Vector3Stamped v_msg;
                v_msg.vector.x = -pose_data.velocity.z;
                v_msg.vector.y = -pose_data.velocity.x;
                v_msg.vector.z = pose_data.velocity.y;
                tf2::Vector3 tfv;
                tfv.setX(v_msg.vector.x);
                tfv.setY(v_msg.vector.y);
                tfv.setZ(v_msg.vector.z);
                tf2::Quaternion q(-transform_msg.transform.rotation.x,-transform_msg.transform.rotation.y,
                        -transform_msg.transform.rotation.z,transform_msg.transform.rotation.w);
                tfv=tf2::quatRotate(q,tfv);
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
                tfv=tf2::quatRotate(q,tfv);
                om_msg.vector.x = tfv.getX();
                om_msg.vector.y = tfv.getY();
                om_msg.vector.z = tfv.getZ();

                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = now;
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = child_frame_;
                odom_msg.pose.pose = pose_msg.pose;
                odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                            0, cov_pose, 0, 0, 0, 0,
                                            0, 0, cov_pose, 0, 0, 0,
                                            0, 0, 0, cov_twist, 0, 0,
                                            0, 0, 0, 0, cov_twist, 0,
                                            0, 0, 0, 0, 0, cov_twist};
                odom_msg.twist.twist.linear = v_msg.vector;
                odom_msg.twist.twist.angular = om_msg.vector;
                odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                                            0, cov_pose, 0, 0, 0, 0,
                                            0, 0, cov_pose, 0, 0, 0,
                                            0, 0, 0, cov_twist, 0, 0,
                                            0, 0, 0, 0, cov_twist, 0,
                                            0, 0, 0, 0, 0, cov_twist};
                odom_pub_->publish(odom_msg);
            }
        }
        else if (auto fs = frame.as<rs2::frameset>()) {
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
    };
    rs2::pipeline_profile pipe_profile = pipe_.start(cfg_, callback);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_deactivate(const rclcpp_lifecycle::State &){
    odom_pub_->on_deactivate();
    std::cout << "deactivating" << std::endl;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_cleanup(const rclcpp_lifecycle::State &){
    odom_pub_.reset();
    std::cout << "cleaning up" << std::endl;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_shutdown(const rclcpp_lifecycle::State & state){
    odom_pub_.reset();
    std::cout << "shutting down" << std::endl;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void T260::configure_params() {
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
    publish_tf_ = this->declare_parameter("publish_tf", false);

    odom_frame_ = this->declare_parameter("odom_frame", "odom");
    child_frame_ = this->declare_parameter("child_frame", "base_link");
    mounted_frame_ = this->declare_parameter("mounted_frame", "t260_link"); // Not implemented yet

    pose_cov_ = this->declare_parameter("position_covariance", 0.1);
    rotation_cov_ = this->declare_parameter("rotation_covariance", 0.1);


    /// Update parameters dynamically
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            this->get_node_base_interface(),this->get_node_topics_interface(),
            this->get_node_graph_interface(), this->get_node_services_interface());
    auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void {
        std::stringstream ss;
        ss << "\nParameter event:\n changed parameters:";
        for (auto &changed_parameter : event->changed_parameters) {
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
            }  else if (changed_parameter.name == "publish_odom") {
                publish_odom_ = changed_parameter.value.bool_value;
            } else if (changed_parameter.name == "publish_tf") {
                publish_tf_ = changed_parameter.value.bool_value;
            } else if (changed_parameter.name == "odom_frame") {
                odom_frame_ = changed_parameter.value.string_value;
            } else if (changed_parameter.name == "child_frame") {
                child_frame_ = changed_parameter.value.string_value;
            } else if (changed_parameter.name == "mounted_frame") {
                mounted_frame_ = changed_parameter.value.string_value;
            } else if (changed_parameter.name == "position_covariance") {
                pose_cov_ = changed_parameter.value.double_value;
            } else if (changed_parameter.name == "rotation_covariance") {
                rotation_cov_ = changed_parameter.value.double_value;
            }

            ss << "\n";
            RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
        }
    };

    /// Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
}
