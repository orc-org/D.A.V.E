from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    # declare arguments
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='publish synthetic IMU data if true (never on the rover); if false and the sensor fails, nothing is published'
    )
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='i2c bus number for bno085'
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='telemetry publish rate in hz'
    )
    use_magnetometer_arg = DeclareLaunchArgument(
        'use_magnetometer',
        default_value='false',
        description='use 9-DoF magnetometer fusion instead of 6-DoF game rotation vector (off by default to resist motor EM distortion)'
    )
    run_kalman_filter_arg = DeclareLaunchArgument(
        'run_kalman_filter',
        default_value='false',
        description='launch redundant 15-state AUKF node (off by default; rover_ekf consumes /imu/euler and /imu/data_raw directly)'
    )

    # telemetry node
    telemetry_node = Node(
        package='imu',
        executable='telemetry',
        name='imu_telemetry',
        output='screen',
        parameters=[{
            'sim_mode': LaunchConfiguration('sim_mode'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_magnetometer': LaunchConfiguration('use_magnetometer'),
        }]
    )

    # kalman filter node (opt-in diagnostic/standalone filter)
    kalman_filter_node = Node(
        package='imu',
        executable='kalman_filter',
        name='imu_kalman_filter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_kalman_filter')),
        parameters=[{
            # filter step must match the telemetry publish period
            'dt': ParameterValue(PythonExpression(['1.0 / ', LaunchConfiguration('publish_rate')]), value_type=float),
            'topic_raw_imu': '/imu/data_raw',
            'topic_filtered_imu': '/imu/data_filtered',
            'topic_odom': '/imu/odometry',
        }]
    )

    return LaunchDescription([
        sim_mode_arg,
        i2c_bus_arg,
        publish_rate_arg,
        use_magnetometer_arg,
        run_kalman_filter_arg,
        telemetry_node,
        kalman_filter_node,
    ])
