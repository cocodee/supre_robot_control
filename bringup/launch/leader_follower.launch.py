from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('supre_robot_control')
    
    # --- Leader Launch ---
    leader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'common_robot_control.launch.py'])
        ),
        launch_arguments={
            'namespace': 'supre_robot_leader',
            'robot_prefix': 'leader_',
            'can_device_index': '0',
            'serial_device': '/dev/ttyTHS1',
            'serial_slave_id': '10'
        }.items()
    )


    return LaunchDescription([
        leader_launch,
    ])