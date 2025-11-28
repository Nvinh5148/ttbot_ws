import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Khai báo Argument nhận từ bên ngoài
    # Mặc định là /dev/ttyUSB1 nếu không ai truyền vào
    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttbot_imu',
        description='Serial port for IMU sensor'
    )
    
    # 2. Biến hứng giá trị
    imu_port = LaunchConfiguration('imu_port')

    return LaunchDescription([
        imu_port_arg, # Nhớ thêm dòng này vào return

        # ---------------------------------------------------------
        # 1. DRIVER NODE (ADIS16488)
        # ---------------------------------------------------------
        Node(
            package='adis16488_driver',
            executable='adis16488_node',
            name='adis16488_node',
            output='screen',
            parameters=[
                {'port': imu_port},        # <--- SỬA Ở ĐÂY: Dùng biến thay vì cứng
                {'baudrate': 460800},
                {'frame_id': 'imu_link'}
            ]
        ),

        # ---------------------------------------------------------
        # 2. FILTER NODE (Madgwick)
        # ---------------------------------------------------------
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[
                {'use_mag': True},
                {'publish_tf': False},
                {'world_frame': 'enu'},
                {'fixed_frame': 'odom'},
                {'gain': 0.1},
            ],
            remappings=[
                ('imu/data_raw', '/imu/data_raw'),
                ('imu/mag', '/imu/mag'),
                ('imu/data', '/imu/data_filtered') 
            ]
        )
    ])