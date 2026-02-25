import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Đường dẫn các gói ---
    fast_lio_pkg = get_package_share_directory('fast_lio')
    xsens_pkg = get_package_share_directory('xsens_mti_ros2_driver')
    velodyne_pkg = get_package_share_directory('velodyne')

    # --- 2. Các file cấu hình mặc định ---
    default_fastlio_config = os.path.join(fast_lio_pkg, 'config', 'velodyne.yaml')
    default_rviz_config = os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')
    xsens_params_path = os.path.join(xsens_pkg, 'param', 'xsens_mti_node.yaml')

    # --- 3. Khởi tạo Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    fast_lio_config = LaunchConfiguration('fast_lio_config')
    rviz_use = LaunchConfiguration('rviz')

    # --- 4. KHAI BÁO ARGUMENTS (CHUẨN FAST-LIO) ---
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )
    declare_fast_lio_config = DeclareLaunchArgument(
        'fast_lio_config', default_value=default_fastlio_config,
        description='Path to Fast-LIO configuration yaml file'
    )
    declare_rviz_use = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to run without RViz (e.g. on robot computer)'
    )

    # --- 5. Khai báo các Node Driver ---

    # A. Driver Xsens MTi
    xsens_mti_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[xsens_params_path, {'use_sim_time': use_sim_time}]
    )

    # B. Driver Velodyne VLP-16
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(velodyne_pkg, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
        ])
    )

    # C. Fast-LIO Mapping Node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            fast_lio_config, # Dùng biến cấu hình linh hoạt
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # D. RViz2 (Bật config sẵn và có thể bật/tắt từ xa)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(rviz_use),
        output='screen'
    )

    # --- 6. Tổng hợp Launch Description ---
    ld = LaunchDescription()

    # Thêm môi trường
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Thêm arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_fast_lio_config)
    ld.add_action(declare_rviz_use)

    # Thêm các thành phần
    ld.add_action(xsens_mti_node)
    ld.add_action(velodyne_launch)
    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld