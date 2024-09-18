from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    timer_frequency_arg = DeclareLaunchArgument(
        'timer_frequency',
        default_value='100.0',
        description='Frequency of the timer callback in Hz'
    )
    
    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='0.001', # mm
        description='Depth scale of the camera'
    )
    
    pen_detection_package_launch_dir = os.path.join(get_package_share_directory('pen_detection'), 'launch')
    
    d435i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pen_detection_package_launch_dir, 'd435i.launch.py')),
        launch_arguments={'timer_frequency': LaunchConfiguration('timer_frequency')}.items()
    )
    pen_detection_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pen_detection_package_launch_dir, 'pen_detection.launch.py')),
        launch_arguments={'depth_scale': LaunchConfiguration('depth_scale')}.items()
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('pen_detection'),
        'rviz',
        'pen_detection.rviz'
        )
    
    return LaunchDescription([
        timer_frequency_arg,
        depth_scale_arg,
        
        d435i_launch,
        pen_detection_node_launch,
        
        Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
        ),
    ])