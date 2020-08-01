# ros2_t260

ROS2 node for specialized for the T260 series realsense cameras. Key features of this package include:
1. Reduced code base by focusing on the T200 series sensors alone.
2. Map saving and loading from the T200 series camera using services.
3. Relocalization on loaded maps to solve the kidnapped robot problem.
4. Wheel odometry input for improved VIO from T200 series camera
5. Built on a lifecycle node for easier integration into larger projects and more control over camera state.

For build, installation, and setup guidance please reference the `.travis.yaml` file in this repository.

## Build and Code Status

#### Release
[![Build Status](https://travis-ci.org/BytesRobotics/ros2_t260.svg?branch=release)](https://travis-ci.org/BytesRobotics/ros2_t260)
[![CodeFactor](https://www.codefactor.io/repository/github/bytesrobotics/ros2_t260/badge/release)](https://www.codefactor.io/repository/github/bytesrobotics/ros2_t260/overview/release)

#### Master
[![Build Status](https://travis-ci.org/BytesRobotics/ros2_t260.svg?branch=master)](https://travis-ci.org/BytesRobotics/ros2_t260)
[![CodeFactor](https://www.codefactor.io/repository/github/bytesrobotics/ros2_t260/badge/master)](https://www.codefactor.io/repository/github/bytesrobotics/ros2_t260/overview/master)

#### Development
[![Build Status](https://travis-ci.org/BytesRobotics/ros2_t260.svg?branch=development)](https://travis-ci.org/BytesRobotics/ros2_t260)
[![CodeFactor](https://www.codefactor.io/repository/github/bytesrobotics/ros2_t260/badge/development)](https://www.codefactor.io/repository/github/bytesrobotics/ros2_t260/overview/development)

## Topics

## Services

## Parameters

|Name|Default|Description|
|---|---|---|
|hardware_reset | true | Whether or not to perform a hardware reset when configuring the sensor|
|serial_number | "" | The serial number of the camera to connect to. If left empty it will connect to the first detected T200 series camera on machine.|
| enable_fisheye_streams | true | Whether or not to enable the fisheye camera stream.
| enable_pose_stream | true | Whether or not to enable the position and odometry streams.
| enable_mapping | true | Whether the camera should use an on device map (recommended)
| enable_dynamic_calibration | true | Whether the camera should apply dynamic calibration (recommended)
| enable_relocalization | true | Use appearance based mapping (depends on enable_mapping being true)
| enable_pose_jumping | true | Allow pose jumping (depends on enable_mapping being true)
| enable_map_preservation | false | Preserve the map from previous runs as if it was loaded
| publish_odom | true | Whether or not to publish nav_msgs::msg::Odometry based on sensor readings
| publish_tf | true | Whether or not to broadcast tf from sensor data as odometry_frame -> base_frame
| odom_frame | "odom" | The frame to use as the robot's odometry frame for purposes of the T200 sensor
| base_frame | "base_link" | The base frame of the robot
| camera_frame | "t260_link" | The link that represents the mounting location of the T200 series sensor.
| position_covariance | 0.1 | Value for scaling the position covariance.
| rotation_covariance | 0.1 | Value for scaling the rotational covariance.
| calib_odom_file | "" | Absolute file path to a config file, such as the one in the config directory, that details the transformation and specification of the wheel odometry input.

## Troubleshooting
1. Problem: The transitioning of the node into the configured state fails with the following output:
    ```
    [INFO] [1596037403.052517939] [t260_node]: Configuring T260 Node
    [INFO] [1596037403.145169185] [t260_node]: T200 series device detected
    Device Serial No: 905312110841
    Device physical port: 2-1.1-9
    Device FW version: 0.2.0.951
    Device Product ID: 0x0B37
    [INFO] [1596037403.145514918] [t260_node]: Connecting to device with serial number: 905312110841
    [INFO] [1596037403.191280128] [t260_node]: Hardware reset
    [INFO] [1596037403.191655134] [t260_node]: No calibration file provided, odom is disabled!
    [ERROR] [1596037403.282707847] []: Caught exception in callback for transition 10
    [ERROR] [1596037403.282893495] []: Original error: No device connected
    [WARN] [1596037403.283088873] []: Error occurred while doing error handling.
    ```
    Solution: Set the hardware reset parameter to false before calling for the configuration of the node on your machine.