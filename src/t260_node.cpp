//
// Created by michael on 7/16/20.
//

#include "t260.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto t260_node = std::make_shared<T260>();
    rclcpp::spin(t260_node);
    rclcpp::shutdown();

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
                  pose_data.translation.y << " " << pose_data.translation.z << " (meters)" << std::endl;
    }

    return EXIT_SUCCESS;
}