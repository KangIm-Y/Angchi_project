import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



moveit_demo = get_package_share_directory('mani') + '/launch/demo.launch.py'
mani_rs485 = get_package_share_directory('rs485_mani') + '/mani_rs485.launch.py'
tf_joint_states = get_package_share_directory('rs485_mani') + '/tf_joint_states.launch.py'


def generate_launch_description():


    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_demo)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mani_rs485)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_joint_states)
        ),

        launch_ros.actions.Node(
            package='mani_moveit',
            executable='mani_moveit',
            name='mani_moveit',
            output='screen'
            ),
  ])
