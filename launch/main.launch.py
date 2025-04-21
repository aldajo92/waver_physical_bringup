from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    display_config_file = os.path.join(
        get_package_share_directory('waver_physical_bringup'),
        'params',
        'display_config.yaml'
    )
    
    ekf_config_file = os.path.join(
        get_package_share_directory('waver_physical_bringup'),
        'params',
        'ekf.yaml'
    )
    return LaunchDescription([
        # Include the teleop_twist_joy launch file with a custom parameter file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('teleop_twist_joy'),
                    'launch',
                    'teleop-launch.py'
                ])
            ),
            launch_arguments={
                'config_filepath': PathJoinSubstitution([
                    FindPackageShare('waver_physical_bringup'),
                    'params',
                    'teleop_twist_joy.yaml'
                ])
            }.items()
        ),
        # Launcher for the Lidar: ros2 launch ldlidar_stl_ros2 ld06.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('ldlidar_stl_ros2'),
        #             'launch',
        #             'ld06.launch.py'
        #         ])
        #     )
        # ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD06',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD06'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'port_name': '/dev/ttyUSB0'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0},
                {'publish_frequency': 50}
            ]
        ),
        # base_link to base_laser tf node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_ld06',
            # arguments=['0','0','0.14','1.5708','0','0','base_footprint','base_laser']
            arguments=['0.04','0','0.10','1.5708','0','0','base_footprint','base_laser']
        ),
        # Node for the battery monitor
        Node(
            package='ina219_battery',
            executable='battery_monitor_node',
            name='battery_monitor_node',
        ),
        # Node for the SSD1306 OLED display
        Node(
            package='ssd1306_display',
            executable='display_node',
            name='oled_display_node',
            parameters=[display_config_file]
        ),
        # Node for the i2c motor driver
        Node(
            package='i2c_motor_driver',
            executable='motor_node',
            name='motor_node',
        ),
        # Node for the velocity sensor
        Node(
            package='i2c_motor_driver',
            executable='velocity_node',
            name='velocity_node',
        ),
        # Node for Odometry
        Node(
            package='waver_odometry',
            executable='odometry_node',
            name='odometry_node'
        ),
        # ros2 run imu_bno055 bno055_i2c_node
        Node(
            package='imu_bno055',
            executable='bno055_i2c_node',
            name='imu_node',
            namespace='imu',
            parameters=[
                {'frame_id': 'imu_frame'},
                {'rate': 10.0},
            ]
        ),
        # Node for the IMU tf node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0','0','0.14','1.5708','0','0','base_footprint','imu_frame']
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf_config_file],
        # )
    ])
