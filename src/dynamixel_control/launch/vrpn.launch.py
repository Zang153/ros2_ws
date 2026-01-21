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

    # Display Node
    # Topic format usually: /<node_name>/<rigid_body_name>/pose
    # Here node_name is 'vrpn_client'
    topic_name_expression = PythonExpression([
        "'/vrpn_client/' + '", LaunchConfiguration('rigid_body'), "' + '/pose'"
    ])

    display_node = Node(
        package='dynamixel_control',
        executable='vrpn_display_node',
        name='vrpn_display',
        output='screen',
        parameters=[{
            'topic_name': topic_name_expression
        }]
    )

    return LaunchDescription([
        server_ip_arg,
        port_arg,
        rigid_body_arg,
        vrpn_node,
        display_node
    ])
