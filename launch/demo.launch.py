from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sixdofarm", package_name="sixdofarm_moveit_config").to_moveit_configs()
    
    # ros2_control_node の追加
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': moveit_config.robot_description},
            os.path.join(
                moveit_config.package_path, 'config', 'ros2_controllers.yaml'
            )
        ],
        output='screen'
    )
    
    # Start controllers after the demo launch
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # Make sure the joint_state_broadcaster is started before the sixdofarm_controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sixdofarm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # Ensure correct startup order
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        generate_demo_launch(moveit_config)
    ])
