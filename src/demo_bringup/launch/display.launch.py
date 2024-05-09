import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rviz_config_path = os.path.join(get_package_share_path('demo_bringup'), 'rviz', 'demo_robot.rviz')
    
    start_robot_description = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path("demo_description"), "launch", "robot_state_publisher.launch.py")
    ))
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        start_robot_description,
        joint_state_publisher_gui_node,
        rviz_node
    ])