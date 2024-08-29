import os
import pathlib


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    nav2_bringup_path = get_package_share_directory('nav2_bringup')
    slam_toolbox_path = get_package_share_directory('slam_toolbox')

    rviz = LaunchConfiguration('rviz')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Whether to start RViz')
    
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_path, '/launch/navigation_launch.py']))

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_path, '/launch/online_async_launch.py']))

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_path, '/launch/rviz_launch.py']),
        condition=IfCondition(rviz))    

    return LaunchDescription([
        declare_rviz_cmd,
        stretch_driver_launch,
        rplidar_launch,
        navigation_launch,
        slam_launch,
        rviz_launch,
    ])