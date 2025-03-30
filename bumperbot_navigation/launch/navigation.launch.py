import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directories
    pkg_bumperbot_navigation = get_package_share_directory('bumperbot_navigation')
    pkg_bumperbot_description = get_package_share_directory('bumperbot_description')
    
    # Define paths for configuration files
    config_dir = os.path.join(pkg_bumperbot_navigation, 'config')
    map_file = os.path.join(config_dir, 'small_house.yaml')  # Ensure this is correct
    param_file = os.path.join(config_dir, 'nav2_params.yaml')  # Ensure correct param file exists
    rviz_config_dir = os.path.join(config_dir, 'navigation.rviz')  # Add RViz config if available

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Gazebo with the robot
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(pkg_bumperbot_description, 'launch', 'gazebo.launch.py')),
        # ),

        # Launch Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': param_file,
                'use_sim_time': use_sim_time
            }.items(),
        ),

        # RViz for Navigation
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen',
            parameters=[{"use_sim_time": use_sim_time}]
        ),
    ])
