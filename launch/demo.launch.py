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
    moveit_config = MoveItConfigsBuilder("sixdofarm", package_name="sixdofarm_moveit_config").to_moveit_configs()
    
    # Launch argumentを追加：Gazeboを使用するかどうか
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Set to true to use Gazebo simulation'
    )
    
    use_gazebo = LaunchConfiguration('use_gazebo')
    
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
            os.path.join(moveit_config_pkg, 'config', 'sixdofarm_gazebo.urdf.xacro')
        )
        return {'robot_description': robot_description_config.toxml()}
    
    # robot_state_publisherの起動（Gazebo用）
    robot_state_publisher_gazebo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[get_robot_description_gazebo()],
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
    
    # Gazebo用のコントローラー起動（遅延あり）
    joint_state_broadcaster_spawner_gazebo = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                output="screen"
            )
        ],
        condition=IfCondition(use_gazebo)
    )

    robot_controller_spawner_gazebo = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["sixdofarm_controller", "-c", "/controller_manager"],
                output="screen"
            )
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # Gazebo用のMoveItノード（move_group）
    move_group_gazebo = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.move_group_capabilities,
            moveit_config.planning_scene_monitor,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # Gazebo用のRviz
    rviz_gazebo = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_gazebo)
    )
    
    # 基本のMoveIt設定を取得（Gazeboを使わない場合のみ）
    demo_launch = generate_demo_launch(moveit_config)
    
    # LaunchDescriptionを構築
    ld = LaunchDescription([
        use_gazebo_arg,
        # Gazebo関連（条件付き）
        gazebo_launch,
        robot_state_publisher_gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner_gazebo,
        robot_controller_spawner_gazebo,
        move_group_gazebo,
        rviz_gazebo,
    ])
    
    # MoveItのデモ起動を追加（Gazeboを使わない場合のみ）
    # この部分は複雑なので、簡単にするため削除し、必要に応じて別途対応
    # for action in demo_launch.entities:
    #     action.condition = UnlessCondition(use_gazebo)
    #     ld.add_action(action)
    
    return ld
