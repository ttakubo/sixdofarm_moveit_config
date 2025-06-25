# sixdofarm_moveit_config
## 確認環境
2025年6月24日時点

環境：Ubuntu22.04 on Windows11(WSL)

ROS 2 version : Humble

ロボットモデル： <a href="https://github.com/ttakubo/csv2urdf">csv2urdf</a>で作成した6自由度ロボットアーム

## ROS 2 HumbleによるMoveIt!マニピュレータシミュレーションガイド

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
-   `ros-gz`: Gazebo FortressとROS 2を連携させるためのパッケージ
-   `xacro`: XMLマクロを使ってURDFを記述するためのツール

#### 2.2 ワークスペースの作成(必要なら)
いつも使っている~/ros2_ws/srcでOK。
別のワークスペースにしたい場合は下記の要領で作成してください。

```bash
mkdir -p ~/moveit_ws/src
cd ~/moveit_ws/src
```

この`src`ディレクトリに、後ほど作成するパッケージを配置します。

新規に作ったワークスペースやもともとのワークスペースを使うにせよ、buildのたびにワークスペース直下にできるinstallディレクトリ内のsetup.bashに対してsourceコマンドで更新する必要があります。

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

    <b>2025年6月24日時点</b>でhomeディレクトリに置いたURDFが何故か読み込めないため、自分で作ったURDFファイルを下記コマンドでmoveit_setup_assistantのinstallされているディレクトリ (ex: /opt/ros/humble/share/moveit_setup_assistant) へコピーしておく。管理者権限ではないとコピーできない場所なのでsudoを使うこと。

        sudo cp sixdofarm.urdf /opt/ros/humble/share/moveit_setup_assistant

    上記のコマンドでコピーしたURDFのパスをファイルダイアログで指定します。（例: `/opt/ros/humble/share/moveit_setup_assistant/sixdofarm.urdf`）

    `Load Files`をクリックすると、RVizウィンドウに自分で作成したロボットのモデルが表示されます。

3.  **Self-Collisions**: `Generate Collision Matrix`をクリックします。これにより、ロボットのどのリンク同士が衝突しうるかを計算し、プランニング時に考慮しない組み合わせ（隣接リンクなど）を無効化します。デフォルトの設定で問題ありません。

4.  **Virtual Joints**: ロボットがワールドにどのように固定されるかを定義します。`Add Virtual Joint`をクリックし、以下の通り設定します。
    -   Joint Name: `virtual_joint`
    -   Child Link: `base_link` （ロボットのベースリンク）
    -   Parent Frame Name: `world`
    -   Joint Type: `fixed`

    ただし、今回の例ではcsv2urdfで作成したsixdofarm.urdfは初めからworldも付けてモデル化しているのでここのステップはスキップしてOK。


5.  **Planning Groups**: 動作計画の対象となる関節グループを定義します。
    -   **アームの追加**:
        -   `Add Group`をクリック。
        -   Group Name: `sixdofarm`
        -   Kinematic Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
        - `OMPL Planning`のGroup Default PlannerにトグルからRRTを選択。
        -   `Add Joints`をクリックし、`Link1_2_Link2` から `Link6_2_Link7` までの6つの関節を選択して `>` で追加して、下にある`Save`ボタンをクリック。
        -   `Links`を選択して、下にあるボタンの`Edit selected`をクリック。
        -   `Link1` から `Link7` までの7つのリンクを選択して `>` で追加。        
        -   `Chain`を選択して、下にあるボタンの`Edit selected`をクリック。
        -   ツリーを展開後に`Link1` を選択して、下にあるBase Linkの右横にある`Choose selected`をクリックして割り当てる。同じく `Link7`を選択して、Top Linkの右横にある`Choose selected`をクリックして割り当てる。 選択が終わったら下の`Save`ボタンをクリック。

    -   **ハンドがある場合はここで追加しておく(今回の例ではいらない）**:
        -   `Add Group`をクリック。
        -   Group Name: `***_hand`
        -   `Add Joints`をクリックし、`***_finger_joint1` と `***_finger_joint2` を選択して追加。

6.  **Robot Poses**: 事前定義されたポーズを追加します。
    -   `Add Pose`をクリック。
    -   Pose Name: `home`
    -   Planning Group: `sixdofarm`
    -   スライダーを動かして、ロボットが良く使う姿勢にし、`Save`します。

7.  **End Effectors**: エンドエフェクタ（ハンド）を定義するとき利用（今回は必要なし）。
    -   `Add End Effector`をクリック。
    -   End Effector Name: `hand`
    -   End Effector Group: `***_hand` （先ほど作成したハンドのグループ）
    -   Parent Link: `panda_link8` （ハンドが取り付けられているリンク）
    -   Parent Group: `panda_arm`

8.  **Passive Joints**: 今回はスキップ

9.  **ros2_control URDF Modifications**: シミュレーションや実機を制御するためのros2_controlの設定をURDFに自動で追加してくれる。
    -   `Command interfaces`と`State interfaces`で必要となる項目にチェックを付ける。デフォルトでは、`Command interfaces`はposition、`State interfaces`はpositionとvelocityにチェックがついている。
    -   `Add interfaces`をクリック。

10.  **ROS 2 Controllers**: ros2_controlの設定。
    -   `Auto Add joint Trajectory Controller Controllers For Each Planning Group`をクリック。
    自動で下記が追加される
    -   Controller Name: `sixdofarm_controller`
    -   Controller Type: `joint_trajectory_controller/JointTrajectoryController`
    -   `Joints`に、`Link1_2_Link2`から`Link6_2_Link7`までの6関節が追加されている。

11.  **Moveit Controllers**: moveit controllerの設定。
    -   `Auto Add Followjoints Trajectrory Controllers For Each Planning Group`をクリック。
    自動で下記が追加される
    -   Controller Name: `sixdofarm_controller`
    -   Controller Type: `FollowJointTrajectory`
    -   `Joints`に、`Link1_2_Link2`から`Link6_2_Link7`までの6関節が追加されている。

12. **Perception**: カメラなどの認識。今回はスキップ

13. **Launch Files**: 生成されるLaunch Fileのリスト。全部チェックマーク入っていてよい。

14. **Auther Information**: 名前とメールアドレスを入力。なにも入れないとエラーが起きるので適当に記載しておくこと。

15. **Configuration Files**: 最後に設定ファイルを生成します。
    -   `Configuration Files`タブに移動します。
    -   `Configuration Package Save Path`で、Browsボタンから`~/moveit_ws/src` を指定します(~/ros2_ws/srcでもOK)。
    -   表示されたパスの最後にパッケージの名前を追記します。今回は、`sixdofarm_moveit_config` とします。（ex:/home/takubo/ros2_ws/src/sixdofarm_moveit_config)
    -   `Generate Package`をクリックします。成功すると、`~/moveit_ws/src/sixdof_moveit_config` に設定ファイル一式が生成されます。

アシスタントを閉じてください。

---

### 4. Rviz上でのシミュレーション実行

生成されたファイルだけではなぜかシミュレーションの設定が不十分なため、いくつかファイルを追加・修正します。

#### 4.1 `ros2_control`の設定

Gazeboが`ros2_control`経由でロボットを制御できるように設定ファイルを作成します。

1.  **コントローラ設定ファイル (`sixdofarm_controllers.yaml`) の追加**
    `~/moveit_ws/src/sixdofarm_moveit_config/config/sixdofarm_controllers.yaml` を作成し、以下の内容を記載します。

    ```yaml
    controller_manager:
    ros__parameters:
        update_rate: 100  # Hz

        sixdofarm_controller:
        type: joint_trajectory_controller/JointTrajectoryController


        joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

    sixdofarm_controller:
    ros__parameters:
        joints:
        - Link1_2_Link2
        - Link2_2_Link3
        - Link3_2_Link4
        - Link4_2_Link5
        - Link5_2_Link6
        - Link6_2_Link7
        command_interfaces:
        - position
        state_interfaces:
        - position
        - velocity
    ```

2.  **Moveitのコントローラ(`moveit_controllers.yaml`) の修正**

    `~/moveit_ws/src/sixdofarm_moveit_config/config/moveit_controllers.yaml` を修正します。<br>
    sixdofarm_controllerのtype:の下に、action_ns:を追加する。

    ```xml
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

    moveit_simple_controller_manager:
    controller_names:
        - sixdofarm_controller

    sixdofarm_controller:
        type: FollowJointTrajectory
        action_ns: follow_joint_trajectory
        default: true
        joints:
        - Link1_2_Link2
        - Link2_2_Link3
        - Link3_2_Link4
        - Link4_2_Link5
        - Link5_2_Link6
        - Link6_2_Link7
    ```

3.  **ros2_controller(`ros2_controllers.yaml`) の修正**

    `~/moveit_ws/src/sixdofarm_moveit_config/config/ros2_controllers.yaml` を修正

    ```yaml
    # This config file is used by ros2_control
    controller_manager:
    ros__parameters:
        update_rate: 100  # Hz

        sixdofarm_controller:
        type: joint_trajectory_controller/JointTrajectoryController


        joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

    sixdofarm_controller:
    ros__parameters:
        joints:
        - Link1_2_Link2
        - Link2_2_Link3
        - Link3_2_Link4
        - Link4_2_Link5
        - Link5_2_Link6
        - Link6_2_Link7
        command_interfaces:
        - position
        state_interfaces:
        - position
        - velocity
        constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        allow_partial_joints_goal: true
    ```

4.  **demo.launchファイル (`demo.launch.py`) の修正**
    まずros2_control_nodeを起動、次にjoint_state_broadcasterを起動、その後、sixdofarm_controllerを起動、最後にMoveItのデモコンポーネントを起動するように修正する。

    ```python
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
    ```


#### 4.3 ビルドと実行

1.  **ビルド**
    ワークスペースのルートディレクトリに戻り、ビルドします。

    ```bash
    cd ~/moveit_ws
    colcon build
    ```

2.  **実行**
    ターミナルでワークスペースをsourceでビルドした内容を反映する。
    ```bash
    source ~/moveit_ws/install/setup.bash
    ```
    <!-- -   **ターミナル1: Gazeboシミュレーションの起動**
        ```bash
        ros2 launch sixdofarm_moveit_config gazebo.launch.py
        ```
        Gazeboが起動し、sixdofarmロボットが表示されます。 -->

    -   **ターミナル: MoveIt! (RViz) の起動**
        ```bash
        ros2 launch sixdofarm_moveit_config demo.launch.py
        ```
        RVizが起動します。`MotionPlanning`パネルが表示されていることを確認します。

#### 4.4 動作確認

1.  RVizの`MotionPlanning`パネルを開きます。
2.  `Planning`タブの`Query`セクションで、`Planning Group`が`sixdofarm`になっていることを確認します。
3.  3Dビュー内で、エンドエフェクタ（水色の球体）の先にあるインタラクティブマーカーをドラッグして、目標の姿勢（Goal State）を設定します。
4.  `Plan and Execute`ボタンをクリックします。
5.  RVizとGazeboの両方で、ロボットが計画された軌道に沿って動けば成功。

---

### 5. まとめと今後の展望

このガイドでは、ROS 2 Humbleと MoveIt! 2を用いて、マニピュレータのシミュレーション環境をゼロから構築しました。次は、動力学シミュレータIGN Gazeboと接続してシミュレーションを行います。さらに、デプスカメラからの情報を使って動的に障害物を認識し、それを回避する動作計画を実行する一連の流れを実装を行う予定です。
