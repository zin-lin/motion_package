"""
Author: Zin Lin Htun
class: Launch
"""

# import necessaries
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# import descriptions
from launch_ros.actions import Node

# constants
PKG_SRC = 'motion_package'

def generate_launch_description():

    odometry_node = Node(
        package='motion_package',
        executable='odometry',
        output='both',  # both means both log files and terminal
    )


    # empty launch_des
    launch_description = LaunchDescription()
    launch_description.add_action(odometry_node) # adding vcu control unit


    return launch_description
