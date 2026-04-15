import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """SLAM with real hardware: motor odometry + RPLidar + slam_toolbox.

    Requires robot_bringup to be running first, OR launches standalone
    with motor_control + rplidar + rsp included.
    """
    pkg_share = get_package_share_directory('amr')

    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_real.yaml')
    motor_params = os.path.join(pkg_share, 'config', 'motor_control_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_rviz.rviz')

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    arduino_port = LaunchConfiguration('arduino_port')

    # Robot state publisher (URDF TFs: base_link->chassis->laser_frame etc.)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # Motor control + odometry (publishes odom->base_link TF)
    motor_control = Node(
        package='motor_control',
        executable='motor_control_node',
        name='motor_control_node',
        output='screen',
        parameters=[motor_params, {
            'serial_port': arduino_port,
        }],
    )

    # RPLidar
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': '',
        }],
        output='screen',
    )

    # slam_toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),
        rsp,
        motor_control,
        rplidar_node,
        slam_toolbox,
        rviz,
    ])
