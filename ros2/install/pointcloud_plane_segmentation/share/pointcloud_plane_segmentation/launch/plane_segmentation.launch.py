import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='ransac', description='segmentation_method to perform'),
       

        Node(
            package='pointcloud_plane_segmentation',
            executable='plane_segmentation',
            name='plane_segmentation',
            output='screen',
            parameters=[],
            arguments=[
                LaunchConfiguration('method')
            ]
        )
    ])
