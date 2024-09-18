from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    timer_frequency_arg = DeclareLaunchArgument(
        'timer_frequency',
        default_value='10.0',
        description='Frequency of the timer callback in Hz'
    )
    
    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='1.0',
        description='Depth scale of the camera'
    )

    return LaunchDescription([
        timer_frequency_arg,
        depth_scale_arg,
        
        Node(
            package='pen_detection',
            executable='pen_detection_node',
            name='pen_detection',
            output = 'screen',
            parameters=[{
            'timer_frequency': LaunchConfiguration('timer_frequency'),
            'depth_scale': LaunchConfiguration('depth_scale')
            }]
        ),
    ])