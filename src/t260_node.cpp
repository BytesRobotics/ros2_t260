//
// Created by michael on 7/16/20.
//

#include "t260.h"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto t260_node = std::make_shared<T260>();
    rclcpp::spin(t260_node->get_node_base_interface());
    rclcpp::shutdown();


    // Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
//    auto callback = [&](const rs2::frame& frame)
//    {
//        std::lock_guard<std::mutex> lock(mutex);
//        if (auto fp = frame.as<rs2::pose_frame>()) {
//            auto pose_data = fp.get_pose_data();
////            std::cout << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
////                      pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;
////            std::cout << "Device Orientation: " << std::setprecision(3) << std::fixed << pose_data.rotation.x << " " <<
////                      pose_data.rotation.y << " " << pose_data.rotation.z << " " << pose_data.rotation.w << " (quaternion)" << std::endl;
//        }
//        else if (auto fs = frame.as<rs2::frameset>()) {
//
//            //    _encoding[RS2_STREAM_FISHEYE] = sensor_msgs::image_encodings::MONO8; // ROS message type
//
//            cv::Mat right(cv::Size(fs.get_fisheye_frame(1).get_width(), fs.get_fisheye_frame(1).get_height()), CV_8UC1, (void*)fs.get_fisheye_frame(1).get_data());
//            cv::imshow("Right Image", right);
//            cv::waitKey(1);
//            cv::Mat left(cv::Size(fs.get_fisheye_frame(2).get_width(), fs.get_fisheye_frame(2).get_height()), CV_8UC1, (void*)fs.get_fisheye_frame(2).get_data());
//            cv::imshow("Left Image", left);
//            cv::waitKey(1);
//        }
//    };
//
//    rs2::context                          ctx;        // Create librealsense context for managing devices
//
//    std::vector<rs2::pipeline>            pipelines;
//
//    // Capture serial numbers before opening streaming
//    std::vector<std::string>              serials;
//    for (auto&& dev : ctx.query_devices()){
//        dev.hardware_reset();
//        serials.emplace_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//        std::cout << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
//        if(strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense T265") == 0){
//            std::cout << "Is T265!" << std::endl;
//            rs2::wheel_odometer wheel_odometer_(dev.first<rs2::wheel_odometer>());
//        }
//        std::cout << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
//    }
//
//
//    // Start a streaming pipe per each connected device
//    rs2::pipeline pipe;
//    rs2::config cfg;
//
//    std::cout << serials[0] << std::endl;
//    cfg.enable_device(serials[0]); //948422110355 T265
//
////    // Declare RealSense pipeline, encapsulating the actual device and sensors.
////    rs2::pipeline pipe;
////
////    // Create a configuration for configuring the pipeline with a non default profile
////    rs2::config cfg;
//    // Note: It is not currently possible to enable only one
//    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
//    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
//    // Add pose stream
//    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
//
//
//
//
//    ///////////////////////////////////////////
//    const std::string virtual_object_guid = "node0";
//
//    // Load raw map on request
//    auto tm_sensor = cfg.resolve(pipe).get_device().first<rs2::pose_sensor>();
//    tm_sensor.set_option(RS2_OPTION_ENABLE_MAPPING, 1);
//    tm_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 1);
//    tm_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 1);
//    tm_sensor.set_option(RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, 1);
//    tm_sensor.set_option(RS2_OPTION_ENABLE_MAP_PRESERVATION, 0);
//
//
//
//    auto in_map_filepath = "/home/michael/Desktop/map_saved";
//    try {
//        tm_sensor.import_localization_map(bytes_from_raw_file(in_map_filepath));
//        std::cout << "Map loaded from " << in_map_filepath << std::endl;
//    }
//    catch (std::runtime_error& e) { std::cout << e.what() << std::endl; }
//
//    // Add relocalization callback
//
//    tm_sensor.set_notifications_callback([&](const rs2::notification& n) {
//        if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
//            std::cout << "Relocalization Event Detected." << std::endl;
//
//            rs2_pose pose_transform;
//
//            // Get static node if available
//            if (tm_sensor.get_static_node(virtual_object_guid, pose_transform.translation, pose_transform.rotation)) {
//                std::cout << "Virtual object loaded:  " << pose_transform.translation << std::endl;
////                object_pose_in_world_initialized = true;
//            }
//        }
//    });
//
//////////////////////////////
//
//
//    // Start pipeline with chosen configuration
//    rs2::pipeline_profile pipe_profile = pipe.start(cfg, callback);
//
//    rs2::stream_profile fisheye_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
//    rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
//    rs2_extrinsics pose_to_fisheye_extrinsics = pipe_profile.get_stream(RS2_STREAM_POSE).get_extrinsics_to(fisheye_stream);
//    std::cout << "Device got. Streaming data" << std::endl;
//    std::cout << intrinsics.fx << std::endl;
//    std::cout << pose_to_fisheye_extrinsics.translation[0] << std::endl;
//
//    // Start streaming through the callback with default recommended configuration
//    // The default video configuration contains Depth and Color streams
//    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
//    //
////    rs2::pipeline_profile profiles = pipe.start(callback);
//
//    std::this_thread::sleep_for(std::chrono::seconds(30));
//    rs2_pose pose = identity_pose();
//    tm_sensor.set_static_node(virtual_object_guid, pose.translation, pose.rotation);
//    pipe.stop();
//
//    // Export map to a raw file
//    auto out_map_filepath = "/home/michael/Desktop/map_saved";
//    raw_file_from_bytes(out_map_filepath, tm_sensor.export_localization_map());
//    std::cout << "Saved map to " << out_map_filepath << std::endl;
//
//
//
//    return EXIT_SUCCESS;
}