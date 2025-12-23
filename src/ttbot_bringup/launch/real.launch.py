import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ====================================================
    # 1. KHAI BÁO PATH
    # ====================================================
    pkg_description = get_package_share_directory('ttbot_description')
    pkg_localization = get_package_share_directory('ttbot_localization')
    pkg_controller = get_package_share_directory('ttbot_controller')
    pkg_imu_driver = get_package_share_directory('adis16488_driver')

    # ====================================================
    # 2. CẤU HÌNH THAM SỐ (ARGUMENTS)
    # ====================================================
    
    # [QUAN TRỌNG] Port phần cứng (Theo Udev Rules của bạn)
    imu_port = LaunchConfiguration('imu_port')
    arg_imu = DeclareLaunchArgument(
        'imu_port', 
        default_value='/dev/ttbot_imu', # <-- Khớp với ảnh bạn gửi
        description='Port for IMU'
    )
    
    stm32_port = LaunchConfiguration('stm32_port')
    arg_stm32 = DeclareLaunchArgument(
        'stm32_port', 
        default_value='/dev/ttbot_stm32', # <-- Khớp với ảnh bạn gửi
        description='Port for STM32 Micro-ROS'
    )

    # Các tham số khác
    use_sim_time = LaunchConfiguration('use_sim_time')
    arg_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    controller_type = LaunchConfiguration('controller_type')
    arg_controller = DeclareLaunchArgument(
        'controller_type', default_value='mpc',
        description='Choose controller: "stanley" or "mpc"'
    )

    run_path = LaunchConfiguration('run_path')
    arg_run_path = DeclareLaunchArgument('run_path', default_value='true')
    
    path_file = LaunchConfiguration('path_file')
    arg_path_file = DeclareLaunchArgument('path_file', default_value='path_l.csv')

    # ====================================================
    # 3. KÍCH HOẠT PHẦN CỨNG (HARDWARE LAYER)
    # ====================================================

    # 3.1. ROBOT STATE PUBLISHER
    # Cung cấp cây TF tĩnh (base_link -> imu_link, gps_link)
    xacro_file = os.path.join(pkg_description, 'urdf', 'ttbot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' is_ignition:=false']), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # 3.2. IMU DRIVER
    # Gọi file launch của driver IMU với port đã định
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_imu_driver, 'launch', 'imu_system.launch.py')),
        launch_arguments={'imu_port': imu_port}.items()
    )

    # 3.3. MICRO-ROS AGENT (Kết nối STM32)
    # Node này giúp STM32 (chạy micro-ros) giao tiếp Serial với máy tính
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', stm32_port, '-b', '115200']
    )

    # 3.4. ACKERMANN KINEMATICS
    # Node này nhận cmd_vel -> tính toán -> gửi lệnh xuống STM32 (qua topic mà Agent subscribe)
    ackermann_node = Node(
        package='ttbot_controller',
        executable='ackermann_controller',
        name='ackermann_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wheel_radius': 0.15,
            'wheel_base': 0.65
        }]
    )

    # ====================================================
    # 4. THUẬT TOÁN (ALGORITHMS LAYER)
    # ====================================================

    # [Delay 3s] LOCALIZATION (EKF)
    # Chạy sau khi IMU và STM32 (Odom source) đã lên
    localization_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_localization, 'launch', 'global_localization.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # [Delay 5s] PATH PUBLISHER (Nếu run_path=true)
    path_pub_launch = GroupAction(
        condition=IfCondition(run_path),
        actions=[
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'path_publisher.launch.py')),
                        launch_arguments={'use_sim_time': use_sim_time, 'path_file': path_file}.items()
                    )
                ]
            )
        ]
    )

    # [Delay 6s] HIGH-LEVEL CONTROLLER (Stanley / MPC)
    # >> MPC Group
    mpc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'mpc'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'mpc.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # >> Stanley Group
    stanley_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'stanley'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_controller, 'launch', 'stanley.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    high_level_control = TimerAction(
        period=6.0,
        actions=[stanley_group, mpc_group]
    )

    # ====================================================
    # 5. RETURN
    # ====================================================
    return LaunchDescription([
        # Args
        arg_sim_time, arg_controller, arg_run_path, arg_path_file, 
        arg_imu, arg_stm32, # 2 Arg quan trọng

        # Hardware Nodes
        robot_state_publisher,
        imu_launch,
        micro_ros_agent,  # Node cầu nối STM32
        ackermann_node,   # Node tính toán hình học xe
        
        # Algorithm Nodes
        localization_launch,
        path_pub_launch,
        high_level_control
    ])