import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_rover_description = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_drive_node = LaunchConfiguration('use_drive_node')
    use_sim_stack = LaunchConfiguration('use_sim_stack')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gz_flags = LaunchConfiguration('gz_flags')

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
        # gz_flags: '-r' runs with the GUI; use '-r -s --headless-rendering' for a server-only run
        # (no window), which is how the sensors are tested without a display
        launch_arguments={'gz_args': [gz_flags, ' ', world]}.items()
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
            '-string', robot_description_raw,
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
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # simulated perception sensors (urdf/sensors.xacro), renamed to the SLAM sensor contract
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/lidar/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # simulator ground truth: the true pose of every model (not the wheel odometry)
            '/world/default/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        # the rgbd camera publishes colour, depth and one camera_info; the contract has separate names
        remappings=[
            ('/camera/image', '/camera/color/image'),
            ('/camera/depth_image', '/camera/depth/image'),
            ('/camera/camera_info', '/camera/color/camera_info'),
            ('/world/default/pose/info', '/sim/ground_truth_poses'),
        ],
        output='screen'
    )

    # the depth image shares the colour camera's intrinsics (same rgbd sensor), so the same Gazebo
    # camera_info is bridged a second time under the depth name (avoids a topic_tools dependency)
    depth_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_info_bridge',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
        remappings=[('/camera/camera_info', '/camera/depth/camera_info')],
        output='screen'
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

    # simulation drive bridge (reads /motor/* when drive_node runs, or /cmd_vel directly if use_drive_node:=false)
    sim_drive_bridge = Node(
        package='rover_description',
        executable='sim_drive_bridge.py',
        name='sim_drive_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_cmd_vel': ParameterValue(
                PythonExpression(["'false' if '", use_drive_node, "'.lower() == 'true' else 'true'"]),
                value_type=bool,
            ),
        }],
    )

    # optional simulated autonomy sensor/state stack (sensor bridge -> rover_ekf -> NAVnode)
    sim_sensor_bridge = Node(
        package='rover_description',
        executable='sim_sensor_bridge.py',
        name='sim_sensor_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_radius': 0.15,
            'wheel_separation': 0.60,
            'datum_latitude': 51.46,
            'datum_longitude': -112.70,
        }],
        condition=IfCondition(use_sim_stack)
    )

    ekf_sim_node = Node(
        package='rover_ekf',
        executable='ekf_node',
        name='rover_ekf_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_radius': 0.15,
            'imu_yaw_offset_deg': 0.0,
            'datum_latitude': 51.46,
            'datum_longitude': -112.70,
            # Gazebo DiffDrive publishes odom -> base_footprint -> base_link; disable EKF TF broadcast in sim
            'publish_tf': False,
        }],
        condition=IfCondition(use_sim_stack)
    )

    nav_sim_node = Node(
        package='navigation_system',
        executable='NAVnode',
        name='NAVnode',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_gps_fix_topic': True,
            'gps_fix_topic': '/gps/fix',
        }],
        condition=IfCondition(use_sim_stack)
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
            'use_sim_stack',
            default_value='false',
            description='Start sim_sensor_bridge, rover_ekf (publish_tf:=false), and NAVnode (use_gps_fix_topic:=true)'
        ),
        DeclareLaunchArgument(
            'gz_flags',
            default_value='-r',
            description="Gazebo flags: '-r' (GUI) or '-r -s --headless-rendering' (server only)"
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
        depth_info_bridge,
        rviz_node,
        drive_node,
        sim_drive_bridge,
        sim_sensor_bridge,
        ekf_sim_node,
        nav_sim_node,
    ])
