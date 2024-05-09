import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    
    # slam_toolbox
    start_slam_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_path("slam_toolbox"), 'launch', 'online_async_launch.py')]))
    
    
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(start_slam_cmd)
    
    
    return ld