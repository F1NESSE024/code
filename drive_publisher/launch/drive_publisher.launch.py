from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('drive_publisher')
    
    # Create the config file path
    config_file = os.path.join(pkg_share, 'config', 'drive_publisher.yaml')
    
    # Create the drive publisher node
    drive_publisher_node = Node(
        package='drive_publisher',
        executable='drive_publisher',
        name='drive_publisher',
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        drive_publisher_node
    ]) 