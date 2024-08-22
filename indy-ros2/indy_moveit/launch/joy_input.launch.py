import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='joy_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='indy_moveit',
                    plugin='moveit_servo::JoyToServoPub',
                    name='servo_joy')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])