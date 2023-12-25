from launch import LaunchDescription
from launch_ros.actions import Node

pkg_name1 = 'truck_teleop'

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
        package=pkg_name1,
        executable='JoyStick'
    )

    node2 = Node(
        package=pkg_name1,
        executable='Jetson'
    )

    ld.add_action(node1)
    ld.add_action(node2)

    return ld