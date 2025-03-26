import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #注意要改名字 change name
    package_name = 'chassis25_ros2'

    ld = LaunchDescription()
    


    mav_ros_bridge = Node(
        package=package_name,
        executable='mav_ros_bridge',
        name='mav_ros_bridge'
    )

    ld.add_action(mav_ros_bridge)
    return ld