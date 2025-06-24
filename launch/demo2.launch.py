import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="sixdofarm",
            description="Robot name for the URDF.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "package_name",
            default_value="sixdofarm_moveit_config",
            description="Name of the MoveIt config package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("package_name")),
                "config",
                "initial_positions.yaml",
            ]),
            description="YAML file with initial joint positions for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_controller_yaml", # この引数は今回は使わないが、一般的なMoveIt!デモに存在
            default_value=PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("package_name")),
                "config",
                "fake_controllers.yaml",
            ]),
            description="YAML file with fake controller setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ros2_controllers_path", # コントローラー設定YAMLのパス
            default_value=PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("package_name")),
                "config",
                "ros2_controllers.yaml",
            ]),
            description="Path to the ros2_controllers.yaml file.",
        )
    )

    # Get arguments
    robot_name = LaunchConfiguration("robot_name")
    package_name = LaunchConfiguration("package_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    fake_controller_yaml = LaunchConfiguration("fake_controller_yaml")
    ros2_controllers_path = LaunchConfiguration("ros2_controllers_path")


    # Robot Description (URDF/XACRO)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(package_name),
                "config",
                "sixdofarm.urdf.xacro",
            ]),
            " initial_positions_file:=", initial_positions_file,
            " use_fake_hardware:=false", # これも重要: Gazeboを使う場合はfalse
            " fake_sensor_connections:=false", # これも重要
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Semantic Robot Description (SRDF)
    # robot_description_semantic_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution([
    #             FindPackageShare(package_name),
    #             "config",
    #             "sixdofarm.srdf", # SRDFもxacroの場合
    #         ]),
    #     ]
    # )
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    robot_description_semantic_content = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "sixdofarm.srdf",
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    # Kinematics config
    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "kinematics.yaml",
    ])

    # Planning pipeline config
    planning_pipeline_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "ompl_planning.yaml", # または他のプランナー
    ])

    # Joint limits config
    joint_limits_yaml = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "joint_limits.yaml",
    ])

    # OMPL planning config
    ompl_planning_yaml = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "ompl_planning.yaml",
    ])

    # RViz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        "rviz",
        "moveit.rviz",
    ])

    # --- Gazebo (Ignition) related setup ---
    # Gazebo server node
    # Note: Use ign_gazebo_ros_demos for an example of how to launch Gazebo with ros2_control
    # For now, let's assume Gazebo is launched separately or via a wrapper launch file.
    # The ros2_control_node needs the robot_description *after* xacro expansion.

    # ros2_control_node: This is where your hardware interface (GazeboSystem) is loaded
    # and controllers are started by the spawner.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, # URDF
            ros2_controllers_path, # ros2_controllers.yaml
            {"use_sim_time": use_sim_time}, # シミュレーション時間
        ],
        output="screen",
        # デバッグログを詳細にする
        arguments=[
            "--ros-args",
            "--log-level", "info", # 全体のログレベルをINFOに
            "--log-level", "controller_manager:=debug", # controller_manager のログをDEBUGに
            "--log-level", "rclpy:=debug",
            "--log-level", "xacro:=debug", # xacroの展開に関するデバッグログ
        ],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            planning_pipeline_config,
            joint_limits_yaml,
            ompl_planning_yaml,
            ros2_controllers_path, # ここでもコントローラー設定YAMLを読み込む
            {"use_sim_time": use_sim_time},
        ],
        arguments=[
            "--ros-args",
            "--log-level", "info", # 全体のログレベルをINFOに
            "--log-level", "moveit_ros_move_group:=debug", # move_group のログをDEBUGに
            "--log-level", "moveit.plugins.moveit_simple_controller_manager:=debug", # コントローラー認識のデバッグログ
        ],
    )

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_yaml,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Spawners (These will now be part of your main launch or a separate launch)
    # The default demo.launch.py generated by setup_assistant usually includes these.
    # Ensure these are executed AFTER ros2_control_node.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    sixdofarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sixdofarm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            ros2_control_node,
            move_group_node,
            rviz_node,
            joint_state_broadcaster_spawner,
            sixdofarm_controller_spawner,
        ]
    )