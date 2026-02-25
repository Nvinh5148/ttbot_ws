import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    fast_lio_dir = get_package_share_directory('fast_lio')
    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox') 
    ttbot_mapping_dir = get_package_share_directory('ttbot_mapping')

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_lio_dir, 'launch', 'fast_lio_sim.launch.py')),
        launch_arguments={
            'config_file': os.path.join(fast_lio_dir, 'config', 'velodyne_sim.yaml'),
            'use_sim_time': 'true'}.items()
    )
    pointcloud_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pointcloud_to_laserscan_dir, 'launch', 'pointcloud_to_laserscan.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    mapper_params_file = os.path.join(ttbot_mapping_dir, 'config', 'mapper_params.yaml')
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': mapper_params_file,
            'use_sim_time': 'true'   
        }.items()
    )

    return LaunchDescription([
        fast_lio_launch,
        pointcloud_to_laserscan_launch,
        slam_toolbox_launch
    ])