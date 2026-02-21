from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    autonomous_state_machine = Node(
        package='autonomous_state_machine',
        executable='autonomous_state_machine',
        name='autonomous_state_machine',
        output='screen',
    )

    return LaunchDescription([autonomous_state_machine])