from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='true',
            description='Enable or disable the terrain layer'
        ),
        
        DeclareLaunchArgument(
            'map_frame_id',
            default_value='base_link',
            description='Frame ID for the grid map'
        ),
        
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.05',
            description='Resolution of the grid map in meters'
        ),
        
        DeclareLaunchArgument(
            'map_width',
            default_value='10.0',
            description='Width of the grid map in meters'
        ),
        
        DeclareLaunchArgument(
            'map_height',
            default_value='10.0',
            description='Height of the grid map in meters'
        ),
        
        Node(
            package='terra_sense',
            executable='terrain_layer_node',
            name='terrain_layer',
            parameters=[{
                'enabled': LaunchConfiguration('enabled'),
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_width': LaunchConfiguration('map_width'),
                'map_height': LaunchConfiguration('map_height')
            }],
            output='screen'
        )
    ])
