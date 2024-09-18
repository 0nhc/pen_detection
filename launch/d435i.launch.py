from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    enable_pointcloud_arg = DeclareLaunchArgument(
        'pointcloud.enable',
        default_value='true',
        description='Enable D435i point cloud processing'
    )
    
    enable_depth_alignment_arg = DeclareLaunchArgument(
        'align_depth.enable',
        default_value='true',
        description='Enable D435i depth alignment processing'
    )
    
    d435i_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(d435i_launch_dir, 'rs_launch.py')),
        launch_arguments={'pointcloud.enable': LaunchConfiguration('pointcloud.enable'),
                          'align_depth.enable': LaunchConfiguration('align_depth.enable')}.items(),
    )
    
    return LaunchDescription([
        enable_pointcloud_arg,
        enable_depth_alignment_arg,
        
        d435i_launch
    ])