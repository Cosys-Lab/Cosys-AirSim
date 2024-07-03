import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    output = DeclareLaunchArgument(
        "output",
        default_value='log')

    publish_clock = DeclareLaunchArgument(
        "publish_clock",
        default_value='False')

    is_vulkan = DeclareLaunchArgument(
        "is_vulkan",
        default_value='True')

    host_ip = DeclareLaunchArgument(
        "host_ip",
        default_value='localhost')

    host_port = DeclareLaunchArgument(
        "host_port",
        default_value='41451')
  
    airsim_node = Node(
            package='airsim_ros_pkgs',
            executable='airsim_node',
            name='airsim_node',
            output='screen',
            parameters=[{
                'is_vulkan': True,
                'update_airsim_img_response_every_n_sec': 0.05,
                'update_airsim_control_every_n_sec': 0.01,
                'update_lidar_every_n_sec': 0.01,
                'publish_clock': LaunchConfiguration('publish_clock'),
                'host_ip': LaunchConfiguration('host_ip'),
                'host_port': LaunchConfiguration('host_port'),
                'enable_api_control': False
            }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(output)
    ld.add_action(publish_clock)
    ld.add_action(is_vulkan)
    ld.add_action(host_ip)
    ld.add_action(host_port)
    ld.add_action(airsim_node)

    return ld
