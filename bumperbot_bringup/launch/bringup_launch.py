from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Set TurtleBot3 model as an environment variable
    turtlebot_model = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle")

    # Include the launch files
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py"
            ])
        )
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_bringup"),
                "launch",
                "robot.launch.py"
            ])
        )
    )

    online_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py"
            ])
        )
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            ])
        )
    )

    # Create LaunchDescription and add all actions
    ld = LaunchDescription()
    ld.add_action(turtlebot_model)
    ld.add_action(navigation_launch)
    ld.add_action(robot_launch)
    ld.add_action(online_launch)
    ld.add_action(rosbridge_launch)

    return ld
