from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    harwdare_reset = LaunchConfiguration('hardware_reset', default='true')
    hardware_reset_cmd = DeclareLaunchArgument(
        'harwdare_reset',
        default_value='true',
        description='Whether to configure a hardware reset on the realsense camera when starting')

    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    odom_frame_cmd = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odom frame')

    base_frame = LaunchConfiguration('base_frame', default='t260_link')
    base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='t260_link',
        description='Base frame. The child frame for odom and TF.')

    camera_frame = LaunchConfiguration('camera_frame', default='t260_link')
    camera_frame_cmd = DeclareLaunchArgument(
        'camera_frame',
        default_value='t260_link',
        description='Camera frame')

    calib_odom_file = LaunchConfiguration('calib_odom_file', default='')
    calib_odom_file_cmd = DeclareLaunchArgument(
        'calib_odom_file',
        default_value='',
        description='Calibration file for using input odometry source')

    publish_tf = LaunchConfiguration('publish_tf', default='true')
    publish_tf_cmd = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether or not to publish /tf')

    t260_node_cmd = Node(
        package='ros2_t260',
        executable='t260_node',
        name='t260_node',
        parameters=[{'hardware_reset': harwdare_reset, "odom_frame": odom_frame,
                     "base_frame": base_frame, "camera_frame": camera_frame,
                     "calib_odom_file": calib_odom_file, "publish_tf": publish_tf}])

    t260_lifecycle_client_cmd = Node(
        package='ros2_t260',
        executable='lifecycle_client',
        name='t260_lifecycle_client')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(hardware_reset_cmd)
    ld.add_action(odom_frame_cmd)
    ld.add_action(base_frame_cmd)
    ld.add_action(camera_frame_cmd)
    ld.add_action(calib_odom_file_cmd)
    ld.add_action(publish_tf_cmd)
    ld.add_action(t260_node_cmd)
    ld.add_action(t260_lifecycle_client_cmd)

    return ld
