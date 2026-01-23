from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    server_ip_arg = DeclareLaunchArgument(
        'server',
        default_value='192.168.31.101',
        description='VRPN Server IP'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='3883',
        description='VRPN Server Port'
    )
    
    rigid_body_arg = DeclareLaunchArgument(
        'rigid_body',
        default_value='zu',
        description='Name of the rigid body to track'
    )

    # VRPN Client Node
    vrpn_node = Node(
        package='vrpn_mocap',
        executable='client_node',
        namespace='vrpn_client',
        name='vrpn_client',
        output='screen',
        parameters=[{
            'server': LaunchConfiguration('server'),
            'port': LaunchConfiguration('port')
        }]
    )

    # Mocap Bridge Node
    # Topic: /vrpn_client/<rigid_body>/pose
    pose_topic_expression = PythonExpression([
        "'/vrpn_client/' + '", LaunchConfiguration('rigid_body'), "' + '/pose'"
    ])

    mocap_bridge_node = Node(
        package='px4_offboard',
        executable='mocap_bridge',
        name='mocap_bridge',
        output='screen',
        parameters=[{
            'pose_topic': pose_topic_expression
        }]
    )

    # Offboard Control Node
    offboard_control_node = Node(
        package='px4_offboard',
        executable='offboard_control',
        name='offboard_control',
        output='screen'
    )

    return LaunchDescription([
        server_ip_arg,
        port_arg,
        rigid_body_arg,
        vrpn_node,
        mocap_bridge_node,
        offboard_control_node
    ])
