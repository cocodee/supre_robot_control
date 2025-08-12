import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node,PushRosNamespace
from launch_ros.subxstitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('supre_robot_control')

    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Get URDF file path
    urdf_file = os.path.join(pkg_share,'urdf','leader_follower_supre_robot.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {"robot_description": robot_description_content}

    # Get controller config file path
    leader_controllers_file = os.path.join(pkg_share, 'config', 'leader_robot_controllers.yaml')
    
    leader_stack = GroupAction(
        actions=[
            PushRosNamespace('supre_robot_leader'),
            
            # Leader Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description]
            ),
            
            # Leader Controller Manager
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[robot_description, leader_controllers_file]
            ),
            
            # Spawners for Leader
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_arm_controller', '-c', '/controller_manager'],
            ),
    # Right arm controller spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller", "--controller-manager", '/controller_manager'],
            ),

        # Spawner for the main gripper controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["misumi_gripper_controller", "--controller-manager", '/controller_manager'],
                output="screen",
            )    
        ]
    )

    leader_stack = GroupAction(
        actions=[
            PushRosNamespace('supre_robot_leader'),
            
            # Leader Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description]
            ),
            
            # Leader Controller Manager
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[robot_description, leader_controllers_file]
            ),
            
            # Spawners for Leader
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_arm_controller', '-c', '/controller_manager'],
            ),
    # Right arm controller spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller", "--controller-manager", '/controller_manager'],
            ),

        # Spawner for the main gripper controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["misumi_gripper_controller", "--controller-manager", '/controller_manager'],
                output="screen",
            )    
        ]
    )

    return LaunchDescription(
        declared_arguments +
        [
            leader_stack
        ]
    )
