import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rover_description = get_package_share_directory('rover_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_drive_node = LaunchConfiguration('use_drive_node')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # default world path
    default_world_path = os.path.join(pkg_rover_description, 'worlds', 'empty.world')

    # path to xacro file and processing
    xacro_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # path to RViz config
    rviz_config_path = os.path.join(pkg_rover_description, 'rviz', 'default.rviz')

    # 1. start gazebo server & client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # 2. publish robot state / TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. spawn D.A.V.E in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'dave_rover',
            '-z', '0.25'  # spawn slightly above ground to prevent clipping
        ],
        output='screen'
    )

    # 4. RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # 5. D.A.V.E. drive node (from drive_package)
    drive_node = Node(
        package='drive_package',
        executable='drive_node',
        name='drive_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_drive_node)
    )

    # 6. simulation drive bridge (bridges /motor/* -> /cmd_vel_sim)
    sim_drive_bridge = Node(
        package='rover_description',
        executable='sim_drive_bridge.py',
        name='sim_drive_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_drive_node)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock from Gazebo'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 along with Gazebo'
        ),
        DeclareLaunchArgument(
            'use_drive_node',
            default_value='true',
            description='Start drive_node and simulation bridge to control the rover'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Path to Gazebo world file'
        ),

        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        drive_node,
        sim_drive_bridge
    ])
