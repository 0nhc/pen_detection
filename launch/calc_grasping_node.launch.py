from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    timer_frequency_arg = DeclareLaunchArgument(
        'timer_frequency',
        default_value='100.0',
        description='Frequency of the timer callback in Hz'
    )

    return LaunchDescription([
        timer_frequency_arg,
        
        Node(
            package='pen_detection',
            executable='calc_grasping_node',
            name='calc_grasping_node',
            output = 'screen',
            parameters=[{
            'timer_frequency': LaunchConfiguration('timer_frequency')
            }]
        ),
    ])