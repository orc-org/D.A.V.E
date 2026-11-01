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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_drive_node = LaunchConfiguration('use_drive_node')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # default world path (sdf for modern gazebo)
    default_world_path = os.path.join(pkg_rover_description, 'worlds', 'empty.sdf')

    # path to xacro file and processing into urdf string
    xacro_path = os.path.join(pkg_rover_description, 'urdf', 'rover.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # path to rviz config
    rviz_config_path = os.path.join(pkg_rover_description, 'rviz', 'default.rviz')

    # start modern gazebo (gz_sim) with the world unpaused (-r)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # publish robot state and tf
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

    # spawn entity in modern gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dave_rover',
            '-z', '0.25'
        ],
        output='screen'
    )

    # modern gazebo to ros 2 communication bridge
    # maps gazebo transport messages to standard ros 2 message types
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ros to gazebo: velocity commands
            '/cmd_vel_sim@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # gazebo to ros: odometry, clock, transforms, joint states
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # rviz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # drive node (from drive_package)
    drive_node = Node(
        package='drive_package',
        executable='drive_node',
        name='drive_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_drive_node)
    )

    # simulation drive bridge (bridges /motor/* to /cmd_vel_sim)
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
            description='Path to Modern Gazebo SDF world file'
        ),

        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        rviz_node,
        drive_node,
        sim_drive_bridge
    ])
