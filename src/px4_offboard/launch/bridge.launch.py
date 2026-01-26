from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    server_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.31.101',
        description='VRPN Server IP Address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='VRPN Server Port'
    )

    # MicroXRCEAgent
    # Command: MicroXRCEAgent udp4 -p 8888
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # VRPN Client
    # Command: ros2 launch vrpn_mocap client.launch.yaml ...
    vrpn_launch_path = PathJoinSubstitution([
        FindPackageShare('vrpn_mocap'), 'launch', 'client.launch.yaml'
    ])

    vrpn_client = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(vrpn_launch_path),
        launch_arguments={
            'server': LaunchConfiguration('server'),
            'port': LaunchConfiguration('port')
        }.items()
    )

    return LaunchDescription([
        server_arg,
        port_arg,
        micro_xrce_agent,
        vrpn_client
    ])