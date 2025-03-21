import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_mapping = get_package_share_directory("bumperbot_mapping")
    pkg_bringup = get_package_share_directory("bumperbot_bringup")
    pkg_description = get_package_share_directory("bumperbot_description")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time"
    )

    declare_map_file = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_mapping, "maps", "map.yaml"),
        description="Full path to map file"
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_bringup, "config", "nav2_default.rviz"),
        description="Full path to RViz config file"
    )

    # Robot State Publisher (for URDF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Static Transform Publisher (for LIDAR frame)
    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_frame"],
    )

    # Navigation2 Bringup
    nav2_bringup = Node(
        package="nav2_bringup",
        executable="bringup_launch.py",
        name="nav2_bringup",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"map": LaunchConfiguration("map")},
            os.path.join(pkg_bringup, "config", "nav2_params.yaml"),
        ],
    )

    # RViz for visualization
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_rviz_config_file,
        robot_state_publisher,
        static_tf_publisher,
        nav2_bringup,
        rviz2,
    ])
