from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Launch argumentを追加：Gazeboを使用するかどうか
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Set to true to use Gazebo simulation'
    )
    
    use_gazebo = LaunchConfiguration('use_gazebo')
    
    # MoveIt設定の構築
    moveit_config = MoveItConfigsBuilder("sixdofarm", package_name="sixdofarm_moveit_config").to_moveit_configs()
    
    # === Gazebo関連のノード ===
    
    # Gazeboの起動
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
        condition=IfCondition(use_gazebo)
    )
    
    # ロボットのURDF準備（Gazebo用）
    def get_robot_description_gazebo():
        moveit_config_pkg = get_package_share_directory('sixdofarm_moveit_config')
        robot_description_config = xacro.process_file(
            os.path.join(moveit_config_pkg, 'config', 'sixdofarm.urdf.xacro')
        )
        return {'robot_description': robot_description_config.toxml()}
    
    # robot_state_publisherの起動（Gazebo用）
    robot_state_publisher_gazebo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[get_robot_description_gazebo(), {"use_sim_time": True}],
        condition=IfCondition(use_gazebo)
    )
    
    # Gazeboにロボットをスポーン
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'sixdofarm', '-z', '0.5'],
        condition=IfCondition(use_gazebo)
    )
    
    # ros2_control nodeの起動（Gazebo用）
    ros2_control_node_gazebo = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[
                    get_robot_description_gazebo(),
                    os.path.join(get_package_share_directory('sixdofarm_moveit_config'), 'config', 'ros2_controllers_gazebo.yaml'),
                    {"use_sim_time": True}
                ],
                output='screen'
            )
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # Joint state broadcasterのスポーン（Gazebo用）
    joint_state_broadcaster_spawner_gazebo = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ],
        condition=IfCondition(use_gazebo)
    )

    # Robot controllerのスポーン（Gazebo用）
    robot_controller_spawner_gazebo = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["sixdofarm_controller", "-c", "/controller_manager"],
                parameters=[{"use_sim_time": True}],
                output="screen"
            )
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # MoveGroupのノード（Gazebo用）
    move_group_node_gazebo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    get_robot_description_gazebo(),
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.trajectory_execution,
                    moveit_config.move_group_capabilities,
                    moveit_config.planning_scene_monitor,
                    {"use_sim_time": True},
                ]
            )
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # Rvizノード（Gazebo用）
    rviz_node_gazebo = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_moveit",
                output="log",
                arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
                parameters=[
                    get_robot_description_gazebo(),
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    {"use_sim_time": True},
                ]
            )
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # === Mock hardware関連のノード ===
    
    # 基本のMoveIt設定（Mock hardware用）
    demo_nodes = generate_demo_launch(moveit_config)
    
    # Mock hardware用にros2_control_nodeを追加
    ros2_control_node_mock = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory('sixdofarm_moveit_config'), 'config', 'ros2_controllers.yaml')
        ],
        output='screen',
        condition=UnlessCondition(use_gazebo)
    )
    
    # Mock hardware用のコントローラースポーナー
    joint_state_broadcaster_spawner_mock = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                output="screen"
            )
        ],
        condition=UnlessCondition(use_gazebo)
    )
    
    robot_controller_spawner_mock = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["sixdofarm_controller", "-c", "/controller_manager"],
                output="screen"
            )
        ],
        condition=UnlessCondition(use_gazebo)
    )
    
    # LaunchDescriptionを構築
    return LaunchDescription([
        use_gazebo_arg,
        # Gazebo関連（条件付き）
        gazebo_launch,
        robot_state_publisher_gazebo,
        spawn_entity,
        ros2_control_node_gazebo,
        joint_state_broadcaster_spawner_gazebo,
        robot_controller_spawner_gazebo,
        move_group_node_gazebo,
        rviz_node_gazebo,
        # Mock hardware用（Gazeboを使わない場合）
        ros2_control_node_mock,
        joint_state_broadcaster_spawner_mock,
        robot_controller_spawner_mock,
        demo_nodes
    ])
