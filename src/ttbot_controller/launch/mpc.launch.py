import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # 1. System Params
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation time (Gazebo)"
    )

    # 2. Path File Params (Optional usage)
    pkg_share = get_package_share_directory('ttbot_controller')
    default_csv = os.path.join(pkg_share, 'path', 'path_l.csv')
    path_file_arg = DeclareLaunchArgument("path_file", default_value=default_csv)

    # 3. MPC & Vehicle Params
    desired_speed_arg = DeclareLaunchArgument("desired_speed", default_value="1.5")
    wheel_base_arg    = DeclareLaunchArgument("wheel_base", default_value="0.65")
    max_steer_deg_arg = DeclareLaunchArgument("max_steer_deg", default_value="30.0")
    goal_tol_arg      = DeclareLaunchArgument("goal_tolerance", default_value="0.3")

    # 4. MPC Weights & Horizon - ĐÃ TUNE LẠI
    # Tăng tầm nhìn lên 30 bước (3 giây) để đường đi mượt hơn, giảm lắc
    
    dt_mpc_arg  = DeclareLaunchArgument("dt_mpc", default_value="0.1")
    Np_arg      = DeclareLaunchArgument("N_p", default_value="30") 

    # 3. TRỌNG SỐ "ĐIỂM NGỌT"
    
    # Q_ey: Giữ ở mức cao vừa phải.
    Q_ey_arg    = DeclareLaunchArgument("Q_ey", default_value="60.0")   
    
    # Q_epsi: Quan trọng. Giữ chặt hướng đầu xe.
    Q_epsi_arg  = DeclareLaunchArgument("Q_epsi", default_value="30.0")
    
    # R_delta: ĐIỂM CÂN BẰNG
    # 0.5 thì giật, 6.0 thì văng -> Chọn 2.5 hoặc 3.0
    # Mức này đủ để kìm hãm dao động nhưng vẫn cho phép bẻ lái nhanh.
    R_delta_arg = DeclareLaunchArgument("R_delta", default_value="3.0")
    R_delta_arg = DeclareLaunchArgument("R_delta", default_value="6.0")

    # 5. Node Definition
    mpc_node = Node(
        package="ttbot_controller",
        executable="mpc_controller",
        name="mpc_controller",
        output="screen",
        parameters=[{
            "use_sim_time":   LaunchConfiguration("use_sim_time"),
            
            "desired_speed":  LaunchConfiguration("desired_speed"),
            "wheel_base":     LaunchConfiguration("wheel_base"),
            "max_steer_deg":  LaunchConfiguration("max_steer_deg"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),

            "N_p":            LaunchConfiguration("N_p"),
            "dt_mpc":         LaunchConfiguration("dt_mpc"),
            "Q_ey":           LaunchConfiguration("Q_ey"),
            "Q_epsi":         LaunchConfiguration("Q_epsi"),
            "R_delta":        LaunchConfiguration("R_delta"),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        path_file_arg,
        desired_speed_arg,
        wheel_base_arg,
        max_steer_deg_arg,
        goal_tol_arg,
        Np_arg,
        dt_mpc_arg,
        Q_ey_arg,
        Q_epsi_arg,
        R_delta_arg,
        mpc_node
    ])