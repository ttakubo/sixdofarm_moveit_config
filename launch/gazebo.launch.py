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

    # XACROからURDFを生成
    robot_description_config = xacro.process_file(
        os.path.join(moveit_config_pkg, 'config', 'sixdofarm.urdf.xacro')
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # robot_state_publisherの起動
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # ros2_controlノードの起動
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')],
        output='screen',
        namespace='sixdofarm'  # 追加: namespaceをsixdofarmに
    )

    # Gazeboにロボットをスポーン
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'sixdofarm', '-z', '0.5']
    )

    # ros2_controlコントローラをロード
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/sixdofarm/controller_manager"],
    )

    sixdofarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sixdofarm_controller", "-c", "/sixdofarm/controller_manager"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[ros2_control_node],  # spawn_entity終了後にros2_control_nodeを起動
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ros2_control_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[sixdofarm_controller_spawner],
            )
        ),
    ])