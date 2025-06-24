# sixdofarm_moveit_config
## 確認環境
2025年6月24日時点

環境：Ubuntu22.04 on Windows11(WSL)

ROS 2 version : Humble

ロボットモデル： <a href="https://github.com/ttakubo/csv2urdf">csv2urdf</a>で作成した6自由度ロボットアーム

## ROS 2 HumbleとGazebo FortressによるMoveIt!マニピュレータシミュレーションガイド

ROS 2 Humbleを使用して、マニピュレータの動作計画ライブラリであるMoveIt! 2のシミュレーション環境を構築する手順を解説します。

---

### 1. はじめに

今回の練習でやること。

-   自分で作成したURDFのロボットモデルを、MoveIt!セットアップアシスタントを使ってロボットのMoveIt!設定を生成する。


**前提条件**:
-   Ubuntu 22.04 LTS
-   ROS 2 Humble Hawksbillがインストール済みであること。
-   `colcon`ビルドツールがインストール済みであること。

---

### 2. 環境構築

まず、シミュレーションに必要なパッケージをインストールし、ワークスペースを作成します。

#### 2.1 必要なパッケージのインストール

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-xacro
```

-   `moveit`: MoveIt! 2本体
-   `gazebo-ros-pkgs`: GazeboとROS 2を連携させるための基本パッケージ
-   `ros2_control`, `ros2_controllers`: ハードウェア/シミュレーションを抽象化する制御フレームワーク
-   `ros-gz`: Gazebo FortressとROS 2を連携させるための新世代パッケージ
-   `xacro`: XMLマクロを使ってURDFを記述するためのツール

#### 2.2 ワークスペースの作成

```bash
mkdir -p ~/moveit_ws/src
cd ~/moveit_ws/src
```

この`src`ディレクトリに、後ほど作成するパッケージを配置します。

---

### 3. MoveIt!セットアップアシスタントによる設定

MoveIt!セットアップアシスタントは、ロボットのURDFファイルからMoveIt!で必要となる設定ファイル（SRDFや各種YAMLファイル）をGUIで簡単に生成できるツール。。。のはずだが難しい。

#### 3.1 セットアップアシスタントの起動

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

#### 3.2 設定手順

1.  **Create New MoveIt Configuration Package** をクリックします。
2.  **Load Robot Model**: `Browse`をクリックし、自分で作ったロボットのURDFファイルを選択します。

    2025年6月24日時点でhomeディレクトリに置いたURDFが何故か読み込めないため、自分で作ったURDFファイルを下記コマンドでmoveit_setup_assistantのinstallされているディレクトリ (ex: /opt/ros/humble/share/moveit_setup_assistant) へコピーしておく。

        cp sixdofarm.urdf /opt/ros/humble/share/moveit_setup_assistant

    上記のコマンドでコピーしたURDFのパスをファイルダイアログで指定します。（例: `/opt/ros/humble/share/moveit_setup_assistant/sixdofarm.urdf`）

    `Load Files`をクリックすると、RVizウィンドウに自分で作成したロボットのモデルが表示されます。

3.  **Self-Collisions**: `Generate Collision Matrix`をクリックします。これにより、ロボットのどのリンク同士が衝突しうるかを計算し、プランニング時に考慮しない組み合わせ（隣接リンクなど）を無効化します。デフォルトの設定で問題ありません。

4.  **Virtual Joints**: ロボットがワールドにどのように固定されるかを定義します。`Add Virtual Joint`をクリックし、以下の通り設定します。
    -   Joint Name: `virtual_joint`
    -   Child Link: `base_link` （ロボットのベースリンク）
    -   Parent Frame Name: `world`
    -   Joint Type: `fixed`

    ただし、今回の例ではcsv2urdfで作成したsixdofarm.urdfは初めからworldも付けてモデル化しているのでここのステップはスキップしてOK。

下記途中：ここまで

5.  **Planning Groups**: 動作計画の対象となる関節グループを定義します。
    -   **アームの追加**:
        -   `Add Group`をクリック。
        -   Group Name: `panda_arm`
        -   Kinematic Solver: `KDLKinematicsPlugin`
        -   `Add Joints`をクリックし、`panda_joint1` から `panda_joint7` までを選択して `>` で追加します。
    -   **ハンドの追加**:
        -   `Add Group`をクリック。
        -   Group Name: `panda_hand`
        -   `Add Joints`をクリックし、`panda_finger_joint1` と `panda_finger_joint2` を選択して追加します。

6.  **Robot Poses**: 事前定義されたポーズを追加します。
    -   `Add Pose`をクリック。
    -   Pose Name: `home`
    -   Planning Group: `panda_arm`
    -   スライダーを動かして、ロボットを直立に近い姿勢にし、`Save`します。

7.  **End Effectors**: エンドエフェクタ（ハンド）を定義します。
    -   `Add End Effector`をクリック。
    -   End Effector Name: `hand`
    -   End Effector Group: `panda_hand` （先ほど作成したハンドのグループ）
    -   Parent Link: `panda_link8` （ハンドが取り付けられているリンク）
    -   Parent Group: `panda_arm`

8.  **ROS 2 Control**: シミュレーションや実機を制御するための設定です。
    -   `Add Controller`をクリック。
    -   Controller Name: `panda_arm_controller`
    -   Controller Type: `joint_trajectory_controller/JointTrajectoryController`
    -   `Controller Joints`タブに移動し、`panda_joint1`から`panda_joint7`までを追加します。
    -   同様に、`panda_hand_controller` (`gripper_controllers/GripperActionController`) も追加します。

9.  **Gazebo Simulation**: Gazebo用のURDFを生成します。
    -   `Generate Gazebo-compatible URDF`をクリック。
    -   `File Path`がワークスペースの`src`ディレクトリを指していることを確認します。

10. **Configuration Files**: 最後に設定ファイルを生成します。
    -   `Configuration Files`タブに移動します。
    -   `Setup package path`で、`~/moveit_ws/src` を指定します。
    -   `Package Name`を `panda_moveit_config` とします。
    -   `Generate Package`をクリックします。成功すると、`~/moveit_ws/src/panda_moveit_config` に設定ファイル一式が生成されます。

アシスタントを閉じてください。

---

### 4. Gazeboとの連携とシミュレーション実行

生成されたファイルだけではGazebo Fortressとの連携が不十分なため、いくつかファイルを追加・修正します。

#### 4.1 `ros2_control`の設定

Gazeboが`ros2_control`経由でロボットを制御できるように設定ファイルを作成します。

1.  **コントローラ設定ファイル (`controllers.yaml`) の修正**
    `~/moveit_ws/src/panda_moveit_config/config/panda_controllers.yaml` を開き、以下のように修正します。`joint_state_broadcaster`を追加することが重要です。

    ```yaml
    controller_manager:
      ros__parameters:
        update_rate: 100

        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster

        panda_arm_controller:
          type: joint_trajectory_controller/JointTrajectoryController

        panda_hand_controller:
          type: gripper_controllers/GripperActionController

    panda_arm_controller:
      ros__parameters:
        joints:
          - panda_joint1
          - panda_joint2
          - panda_joint3
          - panda_joint4
          - panda_joint5
          - panda_joint6
          - panda_joint7
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity

    panda_hand_controller:
      ros__parameters:
        joint: panda_finger_joint1
    ```

2.  **`ros2_control`用URDF (`panda.ros2_control.xacro`) の作成**
    Gazeboのシミュレーションプラグインを読み込むためのファイルを作成します。
    `~/moveit_ws/src/panda_moveit_config/config/panda.ros2_control.xacro` を新規作成します。

    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">

        <xacro:macro name="panda_ros2_control" params="name">
            <ros2_control name="${name}" type="system">
                <hardware>
                    <plugin>gz_ros2_control/GazeboSystem</plugin>
                </hardware>
                <joint name="panda_joint1">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_joint2">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_joint3">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_joint4">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_joint5">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_joint6">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_joint7">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_finger_joint1">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
                <joint name="panda_finger_joint2">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
            </ros2_control>
        </xacro:macro>
    </robot>
    ```

3.  **メインのURDF (`panda.urdf.xacro`) の修正**
    元のURDFを読み込み、`ros2_control`の定義を追加します。
    `~/moveit_ws/src/panda_moveit_config/config/panda.urdf.xacro` を開き、以下のように修正します。

    ```xml
    <?xml version="1.0"?>
    <robot name="panda" xmlns:xacro="http://www.ros.org/wiki/xacro">
        <!-- Import panda urdf -->
        <xacro:include filename="$(find moveit_resources_panda_description)/urdf/panda.urdf" />
        
        <!-- Import ros2_control description -->
        <xacro:include filename="$(find panda_moveit_config)/config/panda.ros2_control.xacro" />
        <xacro:panda_ros2_control name="GazeboSystem"/>
    </robot>
    ```

#### 4.2 シミュレーション用のLaunchファイル作成

GazeboとMoveIt!を連携させて起動するためのLaunchファイルを作成します。

`~/moveit_ws/src/panda_moveit_config/launch/gazebo.launch.py` を新規作成します。

```python
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    moveit_config_pkg = FindPackageShare('panda_moveit_config')

    # XACROからURDFを生成
    robot_description_config = xacro.process_file(
        os.path.join(moveit_config_pkg, 'config', 'panda.urdf.xacro')
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
        arguments=['-topic', '/robot_description', '-name', 'panda', '-z', '0.5']
    )

    # ros2_controlコントローラをロード
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[panda_arm_controller_spawner],
            )
        ),
    ])
```

#### 4.3 ビルドと実行

1.  **ビルド**
    ワークスペースのルートディレクトリに戻り、ビルドします。

    ```bash
    cd ~/moveit_ws
    colcon build
    ```

2.  **実行**
    ターミナルを2つ開きます。それぞれのターミナルでワークスペースをsourceします。
    ```bash
    source ~/moveit_ws/install/setup.bash
    ```
    -   **ターミナル1: Gazeboシミュレーションの起動**
        ```bash
        ros2 launch panda_moveit_config gazebo.launch.py
        ```
        Gazeboが起動し、Pandaロボットが表示されます。

    -   **ターミナル2: MoveIt! (RViz) の起動**
        ```bash
        ros2 launch panda_moveit_config demo.launch.py
        ```
        RVizが起動します。`MotionPlanning`パネルが表示されていることを確認します。

#### 4.4 動作確認

1.  RVizの`MotionPlanning`パネルを開きます。
2.  `Planning`タブの`Query`セクションで、`Planning Group`が`panda_arm`になっていることを確認します。
3.  3Dビュー内で、エンドエフェクタ（水色の球体）の先にあるインタラクティブマーカーをドラッグして、目標の姿勢（Goal State）を設定します。
4.  `Plan and Execute`ボタンをクリックします。
5.  RVizとGazeboの両方で、ロボットが計画された軌道に沿って動けば成功です。

---

### 5. デプスカメラを用いた障害物回避マニピュレーション

次に、デプスカメラで検出した物体を障害物としてMoveIt!に認識させ、それを回避しながら動作するアプリケーションを作成します。

#### 5.1 URDFへのデプスカメラの追加

`~/moveit_ws/src/panda_moveit_config/config/panda.urdf.xacro` を編集し、デプスカメラの定義を追加します。`</robot>` タグの直前に追加してください。

```xml
        <!-- ... 既存のxacro:include と xacro:panda_ros2_control ... -->

        <!-- Depth Camera Link -->
        <link name="camera_link">
            <visual>
                <geometry>
                    <box size="0.05 0.05 0.05"/>
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <box size="0.05 0.05 0.05"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="0.1"/>
                <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
            </inertial>
        </link>

        <!-- Depth Camera Joint -->
        <joint name="camera_joint" type="fixed">
            <parent link="world"/>
            <child link="camera_link"/>
            <origin xyz="0.8 0.0 0.6" rpy="0 0.5 3.14"/>
        </joint>

        <!-- Gazebo sensor plugin for depth camera -->
        <gazebo reference="camera_link">
            <sensor name="depth_camera" type="depth_camera">
                <update_rate>10.0</update_rate>
                <camera>
                    <horizontal_fov>1.047</horizontal_fov>
                    <image>
                        <width>640</width>
                        <height>480</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.1</near>
                        <far>10.0</far>
                    </clip>
                </camera>
                <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                    <ros>
                        <namespace>/_</namespace> <!-- No namespace -->
                        <argument>--ros-args -r image_raw:=/depth_camera/image_raw -r depth_image_raw:=/depth_camera/depth/image_raw -r camera_info:=/depth_camera/camera_info -r points:=/depth_camera/points</argument>
                    </ros>
                    <frame_name>camera_link</frame_name>
                </plugin>
            </sensor>
        </gazebo>
```

#### 5.2 Gazeboワールドの準備

障害物となる箱を配置したGazeboワールドファイルを作成します。

`~/moveit_ws/src/panda_moveit_config/worlds/obstacle.sdf` を新規作成します。

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="default">
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    
    <model name="obstacle_box">
      <pose>0.4 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.5 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.5 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

`gazebo.launch.py`のGazebo起動部分を修正して、このワールドを読み込むようにします。

```python
# ~/moveit_ws/src/panda_moveit_config/launch/gazebo.launch.py の一部を修正

    # Gazeboを起動
    world_file = os.path.join(moveit_config_pkg, 'worlds', 'obstacle.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items() # worldファイルを指定
    )
```

#### 5.3 障害物認識ノードの実装

デプスカメラからの点群データ (`PointCloud2`) を購読し、それをMoveIt!のプランニングシーンに`CollisionObject`として追加するPythonノードを作成します。

1.  **Pythonパッケージの作成**
    ```bash
    cd ~/moveit_ws/src
    ros2 pkg create --build-type ament_python obstacle_detector --dependencies rclpy sensor_msgs moveit_commander sensor_msgs_py
    ```

2.  **ノードのコード作成**
    `~/moveit_ws/src/obstacle_detector/obstacle_detector/obstacle_node.py` を作成します。

    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    import moveit_commander
    from geometry_msgs.msg import PoseStamped
    import numpy as np
    import time

    class ObstacleDetector(Node):
        def __init__(self):
            super().__init__('obstacle_detector')
            self.subscription = self.create_subscription(
                PointCloud2,
                '/depth_camera/points',
                self.listener_callback,
                10)
            
            # MoveIt Commanderの初期化
            moveit_commander.roscpp_initialize([])
            # PlanningSceneInterfaceが利用可能になるまで待機
            time.sleep(5.0) 
            self.scene = moveit_commander.PlanningSceneInterface()
            self.get_logger().info("Obstacle Detector Node Started and connected to Planning Scene")

        def listener_callback(self, msg):
            OBSTACLE_NAME = "detected_obstacle"
            
            # 既存の障害物を削除
            # get_known_object_names()でオブジェクトが存在するか確認してから削除
            if OBSTACLE_NAME in self.scene.get_known_object_names():
                self.scene.remove_world_object(OBSTACLE_NAME)

            points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if not points:
                self.get_logger().info("No points in point cloud to form an obstacle.")
                return

            points_np = np.array(points)

            # 点群からバウンディングボックスを計算
            min_bound = np.min(points_np, axis=0)
            max_bound = np.max(points_np, axis=0)
            
            center = (min_bound + max_bound) / 2.0
            size = max_bound - min_bound

            # 小さすぎる障害物は無視
            if np.any(size < 0.02):
                 return

            # Planning Sceneに箱として追加
            obstacle_pose = PoseStamped()
            obstacle_pose.header.frame_id = msg.header.frame_id
            obstacle_pose.pose.position.x = center[0]
            obstacle_pose.pose.position.y = center[1]
            obstacle_pose.pose.position.z = center[2]
            obstacle_pose.pose.orientation.w = 1.0
            
            self.scene.add_box(OBSTACLE_NAME, obstacle_pose, size=(float(size[0]), float(size[1]), float(size[2])))
            
            self.get_logger().info(f"Added/Updated obstacle '{OBSTACLE_NAME}'")

    def main(args=None):
        rclpy.init(args=args)
        obstacle_detector_node = ObstacleDetector()
        rclpy.spin(obstacle_detector_node)
        obstacle_detector_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **`setup.py`の編集**
    `~/moveit_ws/src/obstacle_detector/setup.py` に、実行可能ファイルとしてノードを追加します。

    ```python
    # ...
    entry_points={
        'console_scripts': [
            'obstacle_node = obstacle_detector.obstacle_node:main',
        ],
    },
    # ...
    ```

#### 5.4 マニピュレーション実行ノードの実装

障害物を回避して目標位置に移動する指令を出すノードを作成します。

1.  **Pythonパッケージの作成**
    ```bash
    cd ~/moveit_ws/src
    ros2 pkg create --build-type ament_python motion_commander --dependencies rclpy moveit_commander geometry_msgs
    ```

2.  **ノードのコード作成**
    `~/moveit_ws/src/motion_commander/motion_commander/motion_node.py` を作成します。

    ```python
    import rclpy
    from rclpy.node import Node
    import moveit_commander
    from geometry_msgs.msg import Pose
    import time
    import sys

    class MotionCommanderNode(Node):
        def __init__(self):
            super().__init__('motion_commander_node')
            moveit_commander.roscpp_initialize(sys.argv)
            
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.group_name = "panda_arm"
            self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
            self.move_group.set_planning_time(10.0) # 計画時間を延長

            self.get_logger().info("Motion Commander Node Started")

        def go_to_pose_goal(self, x, y, z):
            self.get_logger().info(f"Going to pose: x={x}, y={y}, z={z}")
            
            pose_goal = Pose()
            pose_goal.orientation.w = 0.924
            pose_goal.orientation.y = -0.383
            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z

            self.move_group.set_pose_target(pose_goal)

            success = self.move_group.go(wait=True)
            
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            
            if success:
                self.get_logger().info("Motion executed successfully!")
            else:
                self.get_logger().warn("Motion execution failed!")
            return success

    def main(args=None):
        rclpy.init(args=args)
        motion_node = MotionCommanderNode()
        
        # MoveItが完全に起動するまで待機
        time.sleep(5)

        # 障害物の向こう側へ移動
        motion_node.go_to_pose_goal(0.4, -0.3, 0.5)

        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **`setup.py`の編集**
    `~/moveit_ws/src/motion_commander/setup.py` を編集します。

    ```python
    # ...
    entry_points={
        'console_scripts': [
            'motion_node = motion_commander.motion_node:main',
        ],
    },
    # ...
    ```

#### 5.5 統合シミュレーションの実行

1.  **ビルド**
    新しいパッケージをビルドします。
    ```bash
    cd ~/moveit_ws
    colcon build
    ```

2.  **実行**
    ターミナルを4つ開きます。それぞれでワークスペースをsourceしてください。
    `source ~/moveit_ws/install/setup.bash`

    -   **ターミナル1: Gazeboシミュレーション**
        ```bash
        ros2 launch panda_moveit_config gazebo.launch.py
        ```
        Gazeboにロボットと赤い障害物、そしてカメラが表示されます。

    -   **ターミナル2: MoveIt! (RViz)**
        ```bash
        ros2 launch panda_moveit_config demo.launch.py
        ```
        RVizが起動します。

    -   **ターミナル3: 障害物認識ノード**
        ```bash
        ros2 run obstacle_detector obstacle_node
        ```
        しばらくすると、RVizの`Planning Scene` -> `Scene Geometry`に`detected_obstacle`が表示されます。これはデプスカメラがGazeboの赤い箱を認識して生成したものです。

    -   **ターミナル4: マニピュレーション実行ノード**
        障害物がRVizに表示されたことを確認してから、以下のコマンドを実行します。
        ```bash
        ros2 run motion_commander motion_node
        ```
        ロボットが、RViz上で表現されている`detected_obstacle`を**回避**して、目標位置（障害物の向こう側）へ移動する様子がGazeboとRVizで確認できます。

---

### 6. まとめと今後の展望

このガイドでは、ROS 2 HumbleとGazebo Fortress、MoveIt! 2を用いて、マニピュレータのシミュレーション環境をゼロから構築しました。さらに、デプスカメラからの情報を使って動的に障害物を認識し、それを回避する動作計画を実行する一連の流れを実装しました。

**今後の展望**:
-   **高度な物体認識**: 点群クラスタリング（DBSCANなど）を用いて、複数の障害物を個別に認識する。
-   **把持 (Grasping)**: 認識した物体の姿勢に合わせてハンドを動かし、物体を掴む動作を実装する。
-   **タスクプランニング**: より複雑なタスク（例：「箱を掴んで別の場所に置く」）を状態遷移やビヘイビアツリーで管理する。
-   **実機への応用**: `ros2_control`のハードウェアインターフェースを実機用に書き換えることで、このシミュレーション環境で開発したコードを実物のロボットで動かす。