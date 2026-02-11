# STORY-011 オドメトリ・EKF トラブルシューティング

## TSB-EKF-001: robot_localization インストール失敗（404 Not Found）

### 症状

```
E: http://packages.ros.org/.../ros-jazzy-robot-localization_... の取得に失敗しました  404  Not Found
```

### 原因

aptのパッケージインデックスが古い。

### 対策

```bash
sudo apt-get update
sudo apt install ros-jazzy-robot-localization
```

---

## TSB-EKF-002: EKFノードが起動するがTFが配信されない

### 症状

- `ekf_filter_node` が起動ログは出るが、`ros2 run tf2_ros tf2_echo odom base_footprint` でTFが見えない
- `/odometry/filtered` にメッセージが来ない

### 確認手順

1. IMUデータが配信されているか確認:
   ```bash
   ros2 topic hz /imu/data_raw
   ```
   → 0Hz の場合、IMUノードが起動していない

2. EKFのログを確認:
   ```bash
   ros2 launch zeuscar_bringup odometry.launch.py
   # 出力にWarningやErrorがないか確認
   ```

### 考えられる原因と対策

| 原因 | 対策 |
|---|---|
| IMUノードが未起動 | `use_imu:=true` を確認、IMU接続を確認 |
| QoS不整合 | IMUはBEST_EFFORT、EKFのデフォルトと不一致の場合あり。下記参照 |
| imu0_config が全てfalse | `ekf_params.yaml` の角速度フラグ（インデックス9-11）を確認 |
| TFツリー不完全 | `base_footprint` と `imu_link` がTFで接続されているか確認 |

---

## TSB-EKF-003: QoS不整合による受信失敗

### 症状

EKFノードのログに以下が表示される:
```
[WARN] New subscription discovered on topic '/imu/data_raw', requesting incompatible QoS
```

### 原因

- `imu_node` は `BEST_EFFORT` QoSで配信
- `robot_localization` のデフォルトは `RELIABLE` の場合がある

### 対策

`ekf_params.yaml` でQoS設定を追加（必要な場合のみ）:

```yaml
imu0_queue_size: 10
```

それでも解決しない場合、`imu_node` のQoSを `RELIABLE` に変更するか、
`robot_localization` のソースを確認してQoS設定オプションを探す。

> **注記**: `robot_localization` 3.x系ではセンサーのQoSは自動マッチングされることが多い。
> この問題が発生した場合は、バージョンを確認すること。

---

## TSB-EKF-004: ヨー角が大きくドリフトする

### 症状

ロボット静止状態でも `/odometry/filtered` のヨー角が連続的に変化する。

### 確認手順

```bash
ros2 topic echo /odometry/filtered --field pose.pose.orientation
```

### 考えられる原因と対策

| 原因 | 対策 |
|---|---|
| IMUジャイロのバイアス | IMU静止時のgyro_zが0に近いか確認。バイアスが大きい場合はキャリブレーション |
| プロセスノイズが大きすぎる | `process_noise_covariance` の角速度成分（インデックス9-11の対角）を小さくする |
| フィルタ周波数が低い | `frequency` を上げる（IMUの配信レート以上に） |

---

## TSB-EKF-005: slam_toolbox が TF タイムアウトで起動失敗

### 症状

```
[ERROR] [slam_toolbox]: Failed to lookup transform from base_footprint to odom
```

### 原因

EKFノードが `slam_toolbox` より遅く起動し、TFがまだ配信されていない。

### 対策

1. `zeuscar.launch.py` の起動順序を確認:
   - EKFは `robot_base` と同時起動（遅延なし）
   - SLAMは `use_slam:=true` で条件付き起動
   - IMUデータが来てからEKFがTF配信を開始するため、数秒のラグがある

2. `slam_params.yaml` の `transform_timeout` を確認:
   ```yaml
   transform_timeout: 0.2  # 現在の設定
   ```
   必要に応じて大きくする（例: 1.0）

3. SLAMの起動をTimerActionで遅延させる（最終手段）

---

## TSB-EKF-006: 「odom」フレームが2つ配信される

### 症状

TFツリーに `odom → base_footprint` が2つ表示される。

### 原因

`ekf_filter_node` が2つ起動している（`odometry.launch.py` を直接起動 + `zeuscar.launch.py` でも起動）。

### 対策

起動方法を統一する:
- **推奨**: `zeuscar.launch.py use_ekf:=true` で統合起動
- `odometry.launch.py` は単体テスト用

---

## TSB-EKF-007: マップが生成されない / /map トピックにデータがない

### 症状

`ros2 topic echo /map` で何も表示されない。

### 確認手順

1. SLAM起動確認:
   ```bash
   ros2 node list | grep slam
   ```

2. 必要なトピックの確認:
   ```bash
   ros2 topic hz /scan           # LiDARデータ
   ros2 topic hz /odometry/filtered  # オドメトリ
   ```

3. TFチェーン確認:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   → `map → odom → base_footprint → base_link → laser_frame` が完成しているか

### 考えられる原因と対策

| 原因 | 対策 |
|---|---|
| `use_slam:=true` を忘れている | コマンドに `use_slam:=true` を追加 |
| LiDARが起動していない | `/scan` トピックのHz確認、デバイス接続確認 |
| TFチェーンが不完全 | `view_frames` でチェーン切れを確認 |
| ロボットが移動していない | slam_toolboxはmimimum_travel_distanceだけ移動しないとマップ更新しない |

---

## TSB-EKF-008: slam_toolbox が unconfigured 状態で停止する（Jazzy固有）

### 症状

slam_toolbox が「Node using stack size ...」のログ出力後に停止し、`/map` や `map→odom` TFが配信されない。

```
[async_slam_toolbox_node-3] [INFO] [slam_toolbox]: Node using stack size 40000000
```

ライフサイクル状態を確認すると `unconfigured [1]` のまま:
```bash
ros2 lifecycle get /slam_toolbox
# → unconfigured [1]
```

### 原因

ROS 2 Jazzy版の `slam_toolbox`（`async_slam_toolbox_node`）は**ライフサイクルノード**として実装されている。起動時は `unconfigured` 状態で待機し、外部から `configure → activate` 遷移を行わないとスキャン処理を開始しない。

ROS 2 Humble版では通常のノードだったため、この変更はJazzy移行時の破壊的変更である。

### 対策

`slam.launch.py` で `Node` の代わりに `LifecycleNode` を使用し、自動的に configure→activate 遷移を行う:

```python
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

# LifecycleNodeとして起動
slam_toolbox_node = LifecycleNode(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    namespace='',  # 必須（TSB-EKF-009参照）
    ...
)

# configure遷移イベント
configure_event = EmitEvent(
    event=ChangeState(
        lifecycle_node_matcher=lambda node: True,
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    )
)

# inactive→activate自動遷移
activate_event_handler = RegisterEventHandler(
    OnStateTransition(
        target_lifecycle_node=slam_toolbox_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda node: True,
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )),
        ],
    )
)
```

### 確認方法

```bash
ros2 lifecycle get /slam_toolbox
# → active [3] であれば正常
```

---

## TSB-EKF-009: LifecycleNode に namespace 引数が必要

### 症状

slam.launch.py を LifecycleNode に変更後、以下のエラーで起動失敗:

```
TypeError: LifecycleNode.__init__() missing 1 required keyword-only argument: 'namespace'
```

### 原因

`launch_ros.actions.LifecycleNode` は `Node` と異なり、`namespace` がキーワード必須引数。

### 対策

`namespace=''`（空文字列 = ルート名前空間）を明示的に指定:

```python
slam_toolbox_node = LifecycleNode(
    ...
    namespace='',
    ...
)
```

---

## TSB-EKF-010: RPi4 メモリ圧迫による電源断・プロセス強制終了

### 症状

統合テスト中にすべてのROS 2プロセスが exit code -9（SIGKILL）で終了。
ロボットの電源が落ちることもある。

```
[rplidar_composition-4]: process has finished cleanly [pid XXXXX] (exit code -9)
[imu_node-5]: process has finished cleanly [pid XXXXX] (exit code -9)
```

### 原因

- slam_toolbox のデフォルト `stack_size_to_use: 40000000`（40MB）がRPi4の3.7GB RAMに対して過大
- 複数ノード同時起動時にOOM Killerが発動

### 対策

`slam_params.yaml` でリソース削減:

```yaml
stack_size_to_use: 10000000    # 40MB → 10MB
tf_buffer_duration: 15.0       # 30s → 15s（メモリ削減）
enable_interactive_mode: false  # RViz不要なら無効化
```

EKF周波数の削減（`ekf_params.yaml`）:

```yaml
frequency: 30.0  # 50Hz → 30Hz（RPi4では処理が追いつかない）
```

---

## TSB-EKF-011: EKF 更新レート未達警告

### 症状

```
[ERROR] [ekf_filter_node]: Failed to meet update rate! Took 0.043seconds.
Try decreasing the rate, limiting sensor output frequency, or limiting the number of sensors.
```

### 原因

RPi4のCPU性能では50Hz更新が間に合わない（1サイクル24〜43ms）。

### 対策

`ekf_params.yaml` の `frequency` を 30.0 に下げる。30Hzでもオドメトリ精度は実用十分。

---

## TSB-EKF-012: rplidar_composition のノード名変更で /scan パブリッシャーが消失する

### 症状

`lidar.launch.py` で `name='rplidar_node'` とノード名を変更すると、`/scan` トピックのパブリッシャーが登録されない。

```bash
# name='rplidar_node' の場合（我々のlaunch）
ros2 node info /rplidar_node
# → Publishers: /parameter_events, /rosout のみ（/scan なし）
ros2 topic info /scan
# → Publisher count: 0

# name='rplidar_composition' の場合（公式launch）
ros2 node info /rplidar_composition
# → Publishers: /parameter_events, /rosout, /scan（/scan あり）
ros2 topic info /scan
# → Publisher count: 1
```

### 検証手順と結果

1. **我々の `lidar.launch.py`**（`name='rplidar_node'`）で起動:
   - ログ: SDK初期化OK、S/N取得、health OK、「Start」、scan mode表示
   - しかし `ros2 node info` に `/scan` パブリッシャーが**存在しない**
   - CPU 93%消費（スキャンデータ取得自体は行っている模様）

2. **公式 `rplidar.launch.py`**（`name='rplidar_composition'`）で起動:
   - ログ: 同上（正常に初期化・スキャン開始）
   - `ros2 node info` に `/scan` パブリッシャーが**存在する**（Publisher count: 1）
   - `/start_motor`、`/stop_motor` サービスも正常に登録

### 環境

- rplidar_ros: v2.1.0（ros-jazzy-rplidar-ros、arm64）
- 実行ファイル: `rplidar_composition`（パッケージ内唯一の実行ファイル）
- デバイス: /dev/rplidar → /dev/ttyUSB0（シンボリックリンク）
- RMW: rmw_fastrtps_cpp（デフォルト）

### 原因（推定）

rplidar_ros v2.1.0 の `rplidar_composition` は ROS 2 コンポーネントノードとして実装されている。
`--ros-args -r __node:=rplidar_node` によるノード名リマップが、内部のコンポーネント登録と整合しなくなり、パブリッシャーの作成が失敗する可能性がある。

### 対策

`lidar.launch.py` のノード名を公式に合わせて `rplidar_composition` に変更する:

```python
rplidar_node = Node(
    package='rplidar_ros',
    executable='rplidar_composition',
    name='rplidar_composition',  # 'rplidar_node' ではなく公式名を使用
    output='screen',
    parameters=[{...}],
)
```

### ステータス

- **ノード名修正**: 適用済み（`lidar.launch.py` を修正、`colcon build` 完了）
- 修正後、`/scan` パブリッシャーが正しく登録されることを確認（Publisher count: 1）
- **根本原因**: rplidar_ros v2.1.0 のコンポーネントノード実装がノード名リマップに対応していない

---

## TSB-EKF-013: FastRTPS 共有メモリ通信障害（RPi4、Jazzy）【解決済み】

### 症状

ROS 2ノード間でDDSデータ通信が成立しない。ノード発見（discovery）は正常に動作するが、**トピックデータの送受信が一切できない**。

```bash
# ノード発見は正常
ros2 node list
# → /rplidar_composition（正しく発見される）

ros2 topic info /scan
# → Publisher count: 1（パブリッシャーも発見される）

# しかしデータ受信は不可
ros2 topic echo /scan --once
# → タイムアウト（データが来ない）

ros2 topic hz /scan
# → 出力なし（0Hz）

# パラメータ取得も不可
ros2 param get /rplidar_composition serial_port
# → タイムアウト
```

### 影響範囲

rplidar固有の問題ではなく、**全てのROS 2トピック通信に影響**する:

```bash
# 基本的なpub/subテストも失敗
ros2 topic pub /test_topic std_msgs/msg/String "data: hello" -r 5 &
ros2 topic echo /test_topic --once
# → タイムアウト（std_msgs/Stringすら受信不可）

# パブリッシャーは正しく登録される
ros2 topic info /test_topic
# → Publisher count: 1, Subscription count: 0
```

### 環境

| 項目 | 値 |
|---|---|
| プラットフォーム | Raspberry Pi 4 Model B (4GB) |
| OS | Ubuntu 24.04 (Linux 6.8.0-1044-raspi, arm64) |
| ROS 2 | Jazzy Jalisco |
| RMW | rmw_fastrtps_cpp 8.4.3 |
| ネットワーク | wlan0 (192.168.11.20) のみ、eth0 未接続 |

### 原因

**FastRTPS（rmw_fastrtps_cpp）の共有メモリ（SHM）トランスポート機能の障害**。

Raspberry Pi 4 / Ubuntu 24.04 環境において、FastRTPSがデフォルトで使用する共有メモリ通信が正常に機能せず、トピックデータの転送が遮断されていた。UDP通信のみを使用するように強制することで回復したため、共有メモリ関連の権限、リソース、あるいは実装上のバグが原因と断定。

### 実施した対策と結果

1. **FastRTPS共有メモリトランスポートの無効化（UDP専用プロファイル作成）**
   
   共有メモリを使用せず、UDPのみを使用するXMLプロファイルを作成して適用したところ、通信が正常に復旧した。

   **設定ファイル**: `/home/pi/fastrtps_udp_only.xml`

   ```xml
   <?xml version="1.0" encoding="UTF-8" ?>
   <dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
     <profiles>
       <transport_descriptors>
         <transport_descriptor>
           <transport_id>udpv4_transport</transport_id>
           <type>UDPv4</type>
         </transport_descriptor>
       </transport_descriptors>
       <participant profile_name="disable_shm" is_default_profile="true">
         <rtps>
           <userTransports>
             <transport_id>udpv4_transport</transport_id>
           </userTransports>
           <useBuiltinTransports>false</useBuiltinTransports>
         </rtps>
       </participant>
     </profiles>
   </dds>
   ```

2. **恒久的な適用設定**

   ユーザーの `.bashrc` に以下の環境変数を追記し、常にこの設定が有効になるようにした。

   ```bash
   # ROS 2 Jazzy FastDDS SHM Workaround (TSB-EKF-013)
   export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastrtps_udp_only.xml
   ```

### ステータス

**解決済み**

回答者: AI Coding Assistant (Cursor)

### 適用確認結果

TSB-EKF-013の対策適用後、基本的なpub/subテスト（`std_msgs/String`）で通信復旧を確認:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastrtps_udp_only.xml
ros2 topic pub /test_topic std_msgs/msg/String "data: hello_udp" -r 5 &
ros2 topic echo /test_topic --once
# → data: hello_udp（受信成功）
```

> **注意**: `.bashrc` に設定済みだが、非対話シェル（スクリプト等）では `.bashrc` の
> 非対話ガード（`case $- in *i*) ;; *) return;; esac`）により環境変数が読み込まれない。
> スクリプトやsystemdサービスから実行する場合は、明示的に `export` するか、
> `/etc/environment` や `.profile` にも設定すること。

---

## TSB-EKF-014: LiDARシリアルポートの不安定化（killall -9後）

### 症状

`killall -9 rplidar_composition` でプロセスを強制終了した後、次回起動時にLiDARの初期化がSDKバージョン表示で停止し、S/N取得やスキャン開始に進まない。

```
[rplidar_composition]: RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '1.12.0'
（以降のログなし — S/N、Firmware、health、Startが出ない）
```

正常時は以下のように進行する:
```
[rplidar_composition]: RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '1.12.0'
[rplidar_composition]: RPLIDAR S/N: 62E1ED93C0EA98C7A0E69BF5F23F4560
[rplidar_composition]: Firmware Ver: 1.29
[rplidar_composition]: Hardware Rev: 7
[rplidar_composition]: RPLidar health status : '0'
[rplidar_composition]: Start
[rplidar_composition]: current scan mode: Sensitivity, max_distance: 12.0 m, ...
```

### 原因

`SIGKILL（-9）`はプロセスにクリーンアップの機会を与えないため、シリアルポート（`/dev/ttyUSB0`）が不正な状態のまま残る。次のプロセスが同じポートを開いても、LiDARのファームウェアが前回の通信セッションの中断を認識できず応答しない。

### 対策

1. **USBケーブルの物理的な抜き差し**（最も確実）
   - LiDARのUSBを抜き、3秒待ってから再接続
   - `ls /dev/rplidar` でデバイス復帰を確認

2. **プロセス停止時は `SIGTERM`（kill -15）を優先**
   ```bash
   kill $(pgrep rplidar_composition)  # SIGTERM（正常終了）
   sleep 3
   # 停止しない場合のみ SIGKILL
   kill -9 $(pgrep rplidar_composition) 2>/dev/null
   ```

3. **USB デバイスのソフトウェアリセット**（物理抜き差し不要な場合）
   ```bash
   # USBポートの電源をリセット
   sudo usbreset /dev/bus/usb/XXX/YYY  # lsusb で確認
   ```

### ステータス

**対策確立済み** — 次回テスト時にUSB再接続で復旧予定

> **2026-02-10追記**: 前回テストから時間が経過し、デバイスリセットが自然に行われた模様。
> USB再接続なしで `/dev/rplidar` → `ttyUSB0` が正常認識され、LiDAR起動・スキャン取得に成功。

---

## TSB-EKF-015: slam_toolbox save_map が result=255 で失敗する

### 症状

slam_toolbox の `save_map` サービスを呼び出すと `result=255`（失敗）が返る:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/pi/zeuscar_map'}}"
# → response: slam_toolbox.srv.SaveMap_Response(result=255)
```

ログに以下のメッセージ:
```
[slam_toolbox]: SlamToolbox: Saving map as /home/pi/zeuscar_map.
Package 'nav2_map_server' not found
```

### 原因

slam_toolbox の `save_map` サービスは内部的に `nav2_map_server` の `map_saver_cli` を呼び出す。
`ros-jazzy-nav2-map-server` がインストールされていないと、マップファイルの書き出しに失敗する。

### 対策

```bash
sudo apt-get install -y ros-jazzy-nav2-map-server
```

インストール後に再度 `save_map` サービスを呼び出すと `result=0`（成功）となる。

### ステータス

**解決済み** — `ros-jazzy-nav2-map-server` インストールで恒久的に解決

---

## 更新履歴

| 日付 | 内容 |
|---|---|
| 2026-02-09 | 初版作成（STORY-011実装時） |
| 2026-02-09 | TSB-EKF-008〜012追加（実機テストで発見した問題を記録） |
| 2026-02-09 | TSB-EKF-012を調査完了に更新、TSB-EKF-013追加（DDS通信障害、未解決） |
| 2026-02-09 | TSB-EKF-013を解決済みに更新（FastDDS共有メモリ無効化対応） |
| 2026-02-09 | TSB-EKF-012ステータス更新（修正適用済み）、TSB-EKF-013適用確認追記、TSB-EKF-014追加 |
| 2026-02-10 | TSB-EKF-014追記（自然復旧確認）、TSB-EKF-015追加（save_map失敗→nav2-map-server導入で解決） |
