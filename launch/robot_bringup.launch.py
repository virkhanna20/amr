import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Unified real robot bringup: motors + LIDAR + camera.

    Launch this first, then SLAM or Nav on top:
      ros2 launch amr robot_bringup.launch.py
      ros2 launch amr slam_real.launch.py   # for mapping
      ros2 launch amr nav_real.launch.py     # for navigation
    """
    pkg_share = get_package_share_directory('amr')
    motor_params = os.path.join(pkg_share, 'config', 'motor_control_params.yaml')

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    arduino_port = LaunchConfiguration('arduino_port')
    video_device = LaunchConfiguration('video_device')

    # Robot state publisher (URDF TFs)
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

    # Camera (monitoring only)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': video_device,
            'image_size': [640, 480],
            'time_per_frame': [1, 15],
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='RPLidar serial port'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0',
                              description='Arduino serial port'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        rsp,
        motor_control,
        rplidar_node,
        camera_node,
    ])
