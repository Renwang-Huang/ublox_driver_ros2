import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    #ublox_driver项目默认位于用户主目录
    default_config_path = os.path.expanduser('~/ublox_driver_ros2/config/driver_config.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value = default_config_path,
            description = 'Path to the driver config file'
        ),
        
        Node(
            package = 'ublox_driver_ros2',  
            executable = 'ublox_driver_ros2',  
            name = 'ublox_driver_ros2',  
            output = 'screen',  
            parameters = [{'config_file': LaunchConfiguration('config_path')}] 
        )
    ])