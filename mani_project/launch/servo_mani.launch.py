import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


mani_rs485 = get_package_share_directory('rs485_mani') + '/mani_rs485.launch.py'
mani_indy = get_package_share_directory('indy_moveit') + '/launch/indy_moveit_gazebo.launch.py'
tf_joint_states = get_package_share_directory('rs485_mani') + '/tf_joint_states_indy.launch.py'
joint = get_package_share_directory('indy_moveit') + '/launch/joy_input.launch.py'

def generate_launch_description():


    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mani_rs485)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mani_indy),
            launch_arguments={'indy_type': 'indy7', 'servo_mode': 'true'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tf_joint_states)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joint)
        ),

        launch_ros.actions.Node(
            package='mani_moveit',
            executable='mani_moveit',
            name='mani_moveit',
            output='screen'
            ),
  ])
