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
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto fp = frame.as<rs2::pose_frame>()) {
            auto pose_data = fp.get_pose_data();

            if(publish_odom_){
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = this->get_clock()->now();
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = child_frame_;

            }



//            std::cout << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
//                      pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;
//            std::cout << "Device Orientation: " << std::setprecision(3) << std::fixed << pose_data.rotation.x << " " <<
//                      pose_data.rotation.y << " " << pose_data.rotation.z << " " << pose_data.rotation.w << " (quaternion)" << std::endl;
        }
        else if (auto fs = frame.as<rs2::frameset>()) {

            //    _encoding[RS2_STREAM_FISHEYE] = sensor_msgs::image_encodings::MONO8; // ROS message type

            cv::Mat right(cv::Size(fs.get_fisheye_frame(1).get_width(), fs.get_fisheye_frame(1).get_height()), CV_8UC1, (void*)fs.get_fisheye_frame(1).get_data());
            cv::imshow("Right Image", right);
            cv::waitKey(1);
            cv::Mat left(cv::Size(fs.get_fisheye_frame(2).get_width(), fs.get_fisheye_frame(2).get_height()), CV_8UC1, (void*)fs.get_fisheye_frame(2).get_data());
            cv::imshow("Left Image", left);
            cv::waitKey(1);
        }
    };
    rs2::pipeline_profile pipe_profile = pipe_.start(cfg_, callback);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_deactivate(const rclcpp_lifecycle::State &){
    std::cout << "deactivating" << std::endl;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_cleanup(const rclcpp_lifecycle::State &){
    std::cout << "cleaning up" << std::endl;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
T260::on_shutdown(const rclcpp_lifecycle::State & state){
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
            }

            ss << "\n";
            RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
        }
    };

    /// Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
}
