import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Nav2 navigation with real hardware: motor odometry + RPLidar + saved map.

    Uses AMCL for localization (works properly now with real odometry).
    """
    pkg_share = get_package_share_directory('amr')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params_real.yaml')
    motor_params = os.path.join(pkg_share, 'config', 'motor_control_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'nav2_rviz.rviz')

    map_file = LaunchConfiguration('map')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    arduino_port = LaunchConfiguration('arduino_port')

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # Motor control + odometry
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

    # Nav2 full bringup (map_server + AMCL + costmaps + planner + controller)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'map': map_file,
        }.items(),
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
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/my_map.yaml')),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0'),
        rsp,
        motor_control,
        rplidar_node,
        nav2_bringup,
        rviz,
    ])
