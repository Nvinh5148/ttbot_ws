import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # === Lấy đường dẫn tới các package ===
    controller_pkg = get_package_share_directory("ttbot_controller")
    description_pkg = get_package_share_directory("ttbot_description")
    localization_pkg = get_package_share_directory("ttbot_localization")

    # === Include launch mô phỏng (Gazebo + robot_state_publisher) ===
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, "launch", "gazebo.launch.py")
        )
    )

    # === Include launch bộ điều khiển ===
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, "launch", "controller.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "use_simple_controller": "True",
            "use_python": "False",
        }.items(),
    )

    # === Include launch bộ lọc EKF (localization) ===
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, "launch", "local_localization.launch.py")
        ),
        launch_arguments={
            "use_python": "False",
        }.items(),
    )

    # === Trả về launch tổng hợp ===
    return LaunchDescription([
        simulation_launch,
        controller_launch,
        localization_launch,
    ])
