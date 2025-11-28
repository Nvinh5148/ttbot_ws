import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # 1. THÊM ARGUMENT NÀY (BẮT BUỘC)
    # Mặc định là 'true' để test mô phỏng không bị lỗi, nhưng cho phép sửa thành 'false'
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Lấy giá trị biến
    use_sim_time = LaunchConfiguration('use_sim_time')


    # ---------- Tham số cho controller ----------
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.15"
    )

    wheel_base_arg = DeclareLaunchArgument(
        "wheel_base",
        default_value="0.65"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_base   = LaunchConfiguration("wheel_base")


    # ---------- Spawner joint_state_broadcaster ----------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ---- Spawner cho Steering Controller (position) ----
    front_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "front_steering_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ---- Spawner cho Rear Wheel Controller (velocity) ----
    rear_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # ---------- Node C++ AckermannController ----------
    ackermann_controller_node = Node(
        package="ttbot_controller",          
        executable="ackermann_controller",   
        parameters=[
            {"wheel_radius": wheel_radius},
            {"wheel_base": wheel_base},
            # SỬA DÒNG NÀY: Dùng biến thay vì số True cứng
            {"use_sim_time": use_sim_time}  
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg, # Nhớ return thêm cái này
        wheel_radius_arg,
        wheel_base_arg,

        joint_state_broadcaster_spawner,
        front_steering_controller_spawner,
        rear_wheel_controller_spawner,
        ackermann_controller_node,
    ])