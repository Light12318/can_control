from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='can_control',
        executable='can_control',
        name='can_control',
        output='screen',
        # remap common teleop topic to /cmd_vel so keyboard teleop works without extra remap
        remappings=[
            ('/teleop_twist_keyboard/cmd_vel', '/cmd_vel'),
            ('teleop_twist_keyboard/cmd_vel', '/cmd_vel')
        ],
        # you may add parameters here, e.g. {'total_power_limit_w': 20.0}
        parameters=[]
    )

    return LaunchDescription([node])