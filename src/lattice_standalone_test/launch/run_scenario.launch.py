import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lattice_standalone_test')
    config_path = os.path.join(pkg_share, 'config', 'configs.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='1',
            description='Scenario: 1=Lateral offset recovery, 2=Static obstacle avoidance, 3=Lane change'
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(pkg_share, 'rviz', 'lattice_test.rviz'),
            description='RViz config file path'
        ),

        SetEnvironmentVariable('LATTICE_CONFIG_PATH', config_path),

        Node(
            package='lattice_standalone_test',
            executable='lattice_test_node',
            name='lattice_test_node',
            output='screen',
            parameters=[{'scenario': LaunchConfiguration('scenario')}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen'
        ),
    ])
