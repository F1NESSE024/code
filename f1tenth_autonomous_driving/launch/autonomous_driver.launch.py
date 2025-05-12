from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_autonomous_driving',
            executable='autonomous_driver',
            name='autonomous_driver',
            parameters=[{
                'max_speed': 2.0,
                'max_steering_angle': 0.34,
                'safety_distance': 0.5,
                'lookahead_distance': 1.0
            }],
            output='screen'
        )
    ]) 