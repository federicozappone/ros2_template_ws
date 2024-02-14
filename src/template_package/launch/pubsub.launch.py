from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    publisher = Node(
        package="template_package",
        name="subscriber",
        executable="subscriber",
        # respawn=True,
        # respawn_delay=4,
    )

    subscriber = Node(
        package="template_package",
        name="publisher",
        executable="publisher",
        # respawn=True,
        # respawn_delay=4,
    )

    ld.add_action(publisher)
    ld.add_action(subscriber)
    return ld
