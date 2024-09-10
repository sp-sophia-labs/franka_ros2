#!/usr/bin/env python
import os.path 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'),'launch'),
                '/rs_launch.py']),
            launch_arguments = {
                'pointcloud.enable': 'True',
            }.items(),
            )
    rs_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["1.44", "0", "0.3", "3.14", "0", "0", "fr3_link0", "camera_link"]) # parent child yaw(z), pitch(y), roll(x)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
    )

    launch_description = []
    launch_description.extend([realsense_launch, rs_tf])

    return LaunchDescription(launch_description)
