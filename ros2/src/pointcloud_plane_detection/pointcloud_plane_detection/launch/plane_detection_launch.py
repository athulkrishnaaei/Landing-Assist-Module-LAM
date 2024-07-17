import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='prosac', description='Select segmentation method to perform'),

        Node(
            package='pointcloud_plane_detection',
            executable='plane_detection',
            name='plane_detection',
            output='screen',
            parameters=[{'method': LaunchConfiguration('method')}],
            arguments=[]
        )
    ])
