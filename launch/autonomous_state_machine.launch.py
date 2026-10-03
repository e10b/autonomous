from launch import LaunchDescription
from launch_ros.actions import Node
from autonomous_state_machine.config import SIMULATION

OAKD_DEPTH_IMAGE_TOPIC = "/autonomous/obstacle_avoidance/oakd/depth/image_rect"
OAKD_DEPTH_CAMERA_INFO_TOPIC = "/autonomous/obstacle_avoidance/oakd/depth/camera_info"
OAKD_POINTCLOUD_TOPIC = "/autonomous/obstacle_avoidance/oakd/pointcloud"

def generate_launch_description():

    pointcloud_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="oakd_pointcloud_node",
        output="screen",
        remappings=[
            ("image_rect", OAKD_DEPTH_IMAGE_TOPIC),
            ("camera_info", OAKD_DEPTH_CAMERA_INFO_TOPIC),
            ("points", OAKD_POINTCLOUD_TOPIC),
        ],
        parameters=[
            {
                "queue_size": 1,
            }
        ],
    )

    autonomous_state_machine = Node(
        package='autonomous_state_machine',
        executable='autonomous_state_machine',
        name='autonomous_state_machine',
        output='screen',
    )

    if SIMULATION: 
        return LaunchDescription([autonomous_state_machine])

    return LaunchDescription([pointcloud_node, autonomous_state_machine])
