import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Gazeboを起動
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # MoveIt設定パッケージのパスを取得
    moveit_config_pkg = get_package_share_directory('sixdofarm_moveit_config')

    # XACROからURDFを生成（Gazebo専用）
    robot_description_config = xacro.process_file(
        os.path.join(moveit_config_pkg, 'config', 'sixdofarm_gazebo.urdf.xacro')
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # robot_state_publisherの起動
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Gazeboにロボットをスポーン
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'sixdofarm', '-z', '0.5']
    )

    # ros2_controlコントローラをロード（Gazeboプラグインが直接管理）
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    sixdofarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sixdofarm_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],  # spawn_entity終了後にコントローラーを起動
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[sixdofarm_controller_spawner],
            )
        ),
    ])