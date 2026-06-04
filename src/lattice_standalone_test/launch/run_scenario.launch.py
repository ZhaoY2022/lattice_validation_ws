import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def get_simulator_params(context):
    scenario = int(LaunchConfiguration('scenario').perform(context))
    positions = {
        1: (0.0, 0.5, 0.0),
        2: (0.0, 0.0, 0.0),
        3: (0.0, 0.0, 0.0),
        5: (0.0, 0.0, 0.0),
    }
    x, y, h = positions.get(scenario, (0.0, 0.0, 0.0))
    return [
        Node(
            package='lattice_standalone_test',
            executable='lattice_simulator_node',
            name='lattice_simulator_node',
            output='screen',
            parameters=[{'init_x': x, 'init_y': y, 'init_heading': h}]
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('lattice_standalone_test')
    config_path = os.path.join(pkg_share, 'config', 'configs.yaml')
    dds_profile_path = os.path.join(pkg_share, 'config', 'fastdds_no_shm.xml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'lattice_car.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'lattice_test.rviz')

    with open(urdf_path, 'r') as f:
        robot_description_text = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='1',
            description='Scenario: 1=Lateral offset recovery, 2=Static obstacle avoidance, 3=Lane change, 5=4.2km S-curve highway'
        ),
        DeclareLaunchArgument(
            'persist_history', default_value='false',
            description='Enable history trajectory persistence (lifetime=infinite)'
        ),
        DeclareLaunchArgument(
            'history_frame_interval', default_value='5',
            description='Frames between history snapshots (5 = 0.5s)'
        ),

        SetEnvironmentVariable('LATTICE_CONFIG_PATH', config_path),
        # Use CycloneDDS — no shared memory residue, better throughput for MarkerArray
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        # Fallback Fast-DDS (no shared memory):
        # SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        # SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', dds_profile_path),

        # Kill any leftover lattice processes from previous runs BEFORE nodes start.
        # Runs immediately; nodes are delayed 0.5s via TimerAction below so cleanup
        # has time to complete and stale DDS memory is purged.
        ExecuteProcess(
            cmd=[os.path.join(pkg_share, 'scripts', 'cleanup_stale.sh')],
            output='screen',
            name='cleanup_stale_processes',
        ),

        # All nodes delayed by 0.5s to let cleanup finish first.
        TimerAction(period=0.5, actions=[
            Node(
                package='lattice_standalone_test',
                executable='lattice_test_node',
                name='lattice_test_node',
                output='screen',
                parameters=[{
                    'scenario': LaunchConfiguration('scenario'),
                    'persist_history': LaunchConfiguration('persist_history'),
                    'history_frame_interval': LaunchConfiguration('history_frame_interval'),
                }]
            ),

            OpaqueFunction(function=get_simulator_params),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description_text,
                }]
            ),

            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_path],
                output='screen'
            ),
        ]),
    ])
