# ZeusCar 統合Bringup実装ガイド

## 目的

このガイドは、ZeusCar ROS 2システムの統合起動（bringup）の設計思想と実装手順を、初心者にもわかりやすく解説します。

---

## 1. 統合Bringupとは何か

### 1.1 Bringupの役割

ROS 2プロジェクトにおける「bringup」パッケージは、システム全体を起動するためのエントリポイントです。

**具体的には:**
- 複数のノードをまとめて起動する
- 起動順序を制御する
- パラメータを一元管理する
- 条件付き起動（センサーのON/OFF等）を実現する

**一般的なロボットシステムの起動構成:**
```
個別パッケージ起動（開発・テスト時）:
  ros2 launch zeuscar_lidar lidar.launch.py
  ros2 launch zeuscar_imu imu.launch.py
  ros2 launch zeuscar_motor motor.launch.py
  ↓
統合起動（運用時）:
  ros2 launch zeuscar_bringup zeuscar.launch.py
```

### 1.2 なぜ統合Bringupが必要か

**利点:**
1. **運用効率化**: 一つのコマンドでシステム全体を起動
2. **起動順序保証**: TF確立後にセンサー起動等、依存関係を管理
3. **柔軟な構成**: Launch Argumentsで機能のON/OFF切り替え
4. **保守性向上**: 起動構成が一箇所に集約、変更が容易

**ZeusCarでの具体例:**
```bash
# センサーのみ起動（モーター動作確認不要時）
ros2 launch zeuscar_bringup zeuscar.launch.py use_motor:=false

# SLAM有効化（マッピング作業時）
ros2 launch zeuscar_bringup zeuscar.launch.py use_slam:=true

# 最小構成（TFのみ、デバッグ時）
ros2 launch zeuscar_bringup zeuscar.launch.py use_lidar:=false use_imu:=false use_motor:=false
```

---

## 2. ZeusCarシステムの構成要素

### 2.1 パッケージの役割

ZeusCarは以下の機能別パッケージで構成されています。

| パッケージ名 | 役割 | 主要ノード |
|---|---|---|
| zeuscar_description | ロボットの形状・TF定義 | robot_state_publisher |
| zeuscar_lidar | LiDARセンサー制御 | rplidar_node |
| zeuscar_imu | IMUセンサー制御 | imu_node |
| zeuscar_motor | モーター制御（Arduino通信） | motor_controller_node |
| zeuscar_slam | SLAM（地図生成・自己位置推定） | slam_toolbox |
| zeuscar_bringup | 統合起動制御 | （launchファイルのみ） |

### 2.2 ノードとトピックの関係

```
[robot_state_publisher]
    ↓ /tf_static (座標系定義)
    ├─ [rplidar_node] → /scan (LiDARデータ)
    ├─ [imu_node] → /imu/data_raw (IMUデータ)
    └─ [slam_toolbox] → /map (地図)

[motor_controller_node] ← /cmd_vel (移動指令)
```

**重要なポイント:**
- TF（Transform）はロボットの各部位の位置関係を定義する座標系
- すべてのセンサーはTFを参照して、データがどこから来たかを示す
- そのため、`robot_state_publisher`は最初に起動する必要がある

### 2.3 TFツリーの理解

TF（Transform Frame）は、ロボットの各部品の位置関係を表す座標系です。

**ZeusCarのTFツリー:**
```
base_footprint (地面投影点)
 └─ base_link (ロボット本体の中心)
     ├─ laser_frame (LiDARの位置)
     └─ imu_link (IMUの位置)
```

**各フレームの意味:**
- `base_footprint`: ロボットの地面への投影点（Z=0）
- `base_link`: ロボット本体の中心（実際の高さを持つ）
- `laser_frame`: LiDARスキャン面の中心
- `imu_link`: IMUセンサーの位置

**なぜこの構造にするのか:**
- REP-103（ROS座標系標準）に準拠
- SLAMやNavigationツールがこの構造を前提としている
- 他のROS 2パッケージとの互換性を保つため

---

## 3. 統合Launch設計の考え方

### 3.1 階層化戦略

ZeusCarの統合launchは3階層構造を採用しています。

```
階層1: 個別パッケージlaunch（単体テスト用）
  ├─ zeuscar_lidar/launch/lidar.launch.py
  ├─ zeuscar_imu/launch/imu.launch.py
  ├─ zeuscar_motor/launch/motor.launch.py
  └─ zeuscar_slam/launch/slam.launch.py

階層2: 機能グループlaunch（部分統合）
  ├─ zeuscar_bringup/launch/robot_base.launch.py (TF + Motor)
  └─ zeuscar_bringup/launch/sensors.launch.py (LiDAR + IMU)

階層3: 統合launchlaunch（全体統合）
  └─ zeuscar_bringup/launch/zeuscar.launch.py
```

**この構造の利点:**
1. **段階的テスト**: 下層から順に動作確認
2. **再利用性**: 中間層launchを他のプロジェクトでも使用可能
3. **保守性**: 変更時の影響範囲が明確

### 3.2 起動順序制御

ROS 2のlaunchシステムは、基本的に並行起動を前提としています。
しかし、依存関係がある場合は順序制御が必要です。

**ZeusCarの起動順序:**
```
1. robot_state_publisher (TF確立)
   ↓ 待機なし（並行起動OK）
2. rplidar_node, imu_node, motor_controller_node
   ↓ TFとトピック準備完了後
3. slam_toolbox（SLAM使用時のみ）
```

**実装方法:**
- `RegisterEventHandler`を使用してノード起動完了イベントを監視
- または、各ノード内でTF待機処理を実装（推奨）
- ZeusCarでは後者を採用（各ノードがTF確立を待つ）

### 3.3 条件付き起動

Launch Argumentsを使用して、起動するノードを動的に制御します。

**例: LiDARの条件付き起動**
```python
use_lidar_arg = DeclareLaunchArgument(
    'use_lidar',
    default_value='true',
    description='Enable LiDAR sensor'
)

lidar_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([...]),
    condition=IfCondition(LaunchConfiguration('use_lidar'))
)
```

**利用シーン:**
- デバッグ時に特定のセンサーのみ起動
- ハードウェア不具合時の切り分け
- シミュレーション環境での選択的起動

---

## 4. パラメータ管理戦略

### 4.1 パラメータの配置方針

ZeusCarでは、パラメータを以下のように分類管理します。

| パラメータ種類 | 配置場所 | 例 |
|---|---|---|
| パッケージ固有 | 各パッケージのconfig/*.yaml | LiDAR設定、モーター設定 |
| システム共通 | zeuscar_bringup/config/*.yaml | use_sim_time等 |
| 環境依存 | Launch Arguments | デバイスパス、IPアドレス |

**設計判断の理由:**
1. **独立性**: 各パッケージを単体で動作可能に保つ
2. **移植性**: 他のプロジェクトでパッケージを再利用しやすい
3. **柔軟性**: 環境ごとの違いをLaunch Argumentsで吸収

### 4.2 パラメータの優先順位

ROS 2では、同じパラメータが複数箇所で定義された場合、以下の優先順位で適用されます。

```
優先度高: Launch Arguments（コマンドライン）
    ↓
優先度中: Launchファイル内のparameters=[...]
    ↓
優先度低: YAMLファイル
```

**実例: serial_portの上書き**
```bash
# デフォルト（motor_params.yaml）: /dev/ttyACM0
ros2 launch zeuscar_bringup zeuscar.launch.py

# Launch Argumentsで上書き
ros2 launch zeuscar_bringup zeuscar.launch.py serial_port_motor:=/dev/ttyACM1
```

---

## 5. 実装時の重要ポイント

### 5.1 IncludeLaunchDescriptionの使い方

既存のlaunchファイルを統合launchから呼び出す方法です。

**基本パターン:**
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# パッケージのパスを取得
lidar_pkg = get_package_share_directory('zeuscar_lidar')

# launchファイルのパス
lidar_launch = os.path.join(lidar_pkg, 'launch', 'lidar.launch.py')

# Include実行
lidar_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(lidar_launch),
    launch_arguments={
        'serial_port': LaunchConfiguration('serial_port_lidar'),
    }.items()
)
```

**ポイント:**
- `get_package_share_directory()`: インストール後のshareディレクトリを取得
- `launch_arguments`: 引数を渡す（辞書型→items()で変換）
- パス結合は`os.path.join()`を使用（プラットフォーム非依存）

### 5.2 GroupActionによるグループ化

複数のアクションをまとめて制御する方法です。

**例: センサーグループの条件付き起動**
```python
from launch.actions import GroupAction
from launch.conditions import IfCondition

sensors_group = GroupAction(
    actions=[
        lidar_include,
        imu_include,
    ],
    condition=IfCondition(LaunchConfiguration('use_sensors'))
)
```

**利点:**
- 複数ノードをまとめてON/OFF制御
- namespace適用時に一括設定可能

### 5.3 Launch Argumentsの命名規則

ZeusCarで採用している命名規則です。

| 型 | 命名規則 | 例 |
|---|---|---|
| boolean | use_* | use_lidar, use_slam |
| デバイスパス | *_port | serial_port_motor, serial_port_lidar |
| 数値（単位あり） | *_hz, *_sec | publish_rate_hz, timeout_sec |
| ファイルパス | *_file | params_file, urdf_file |

**理由:**
- 型推測が容易（コード可読性向上）
- 他のROS 2パッケージとの統一性
- タイポ防止

### 5.4 エラーハンドリング

**シリアルデバイス未接続時の動作:**
- 各ノード内でエラーログを出力
- 他のノードは起動継続（部分的な動作を許容）

**TF確立失敗時の動作:**
- robot_state_publisher起動失敗 → システム全体停止
- 理由: TFはすべてのノードが依存する基盤のため

**I2C権限不足時の動作:**
- imu_node起動失敗 → エラーログと対策メッセージ表示
- 対策: `sudo usermod -aG i2c $USER` + 再ログイン

---

## 6. 段階的な実装手順

### Phase 1: URDF修正（imu_link追加）

**必要性:**
- IMUノードは`frame_id: imu_link`を使用している
- 現在のURDFに`imu_link`が未定義
- TFツリーに不整合が発生

**修正箇所: zeuscar_description/urdf/zeuscar.urdf.xacro**
```xml
<!-- imu_link追加（base_linkの子フレーム） -->
<link name="imu_link"/>

<joint name="base_link_to_imu" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>  <!-- 実測値で調整 -->
</joint>
```

**確認方法:**
```bash
# URDF文法チェック
ros2 run xacro xacro zeuscar.urdf.xacro

# TFツリー確認
ros2 launch zeuscar_description description.launch.py
ros2 run tf2_tools view_frames
```

### Phase 2: robot_base.launch.py作成

**役割: TF + Motorの統合起動**

**実装ポイント:**
1. zeuscar_description/launch/description.launch.pyをInclude
2. zeuscar_motor/launch/motor.launch.pyをInclude
3. use_motorフラグで条件付き起動

**ファイル構成:**
```
zeuscar_bringup/
├─ launch/
│  └─ robot_base.launch.py
```

**動作確認:**
```bash
# TFのみ起動
ros2 launch zeuscar_bringup robot_base.launch.py use_motor:=false

# TF + Motor起動
ros2 launch zeuscar_bringup robot_base.launch.py
```

### Phase 3: sensors.launch.py作成

**役割: LiDAR + IMUの統合起動**

**実装ポイント:**
1. zeuscar_lidar/launch/lidar.launch.pyをInclude
2. zeuscar_imu/launch/imu.launch.pyをInclude
3. 個別ON/OFFフラグ（use_lidar, use_imu）

**動作確認:**
```bash
# 両センサー起動
ros2 launch zeuscar_bringup sensors.launch.py

# LiDARのみ
ros2 launch zeuscar_bringup sensors.launch.py use_imu:=false

# トピック確認
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /imu/data_raw
```

### Phase 4: zeuscar.launch.py作成

**役割: システム全体の統合起動**

**実装ポイント:**
1. robot_base.launch.pyをInclude
2. sensors.launch.pyをInclude
3. slam.launch.pyをInclude（use_slam=trueの場合）
4. RViz起動（use_rviz=trueの場合）

**Launch Arguments:**
- use_sim_time
- use_lidar, use_imu, use_motor
- use_slam, use_rviz
- serial_port_motor, serial_port_lidar

**TSB-INT-003対策: センサー遅延起動**

sensors.launch.py の起動を TimerAction で3秒遅延させている。これにより robot_base（TF + Motor）が先に起動し安定した後にセンサー系が起動する。

遅延時間は `sensor_startup_delay` Launch Argument でカスタマイズ可能:
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py sensor_startup_delay:=5.0
```

**動作確認:**
```bash
# 全機能起動（SLAM除く）
ros2 launch zeuscar_bringup zeuscar.launch.py

# SLAM有効化
ros2 launch zeuscar_bringup zeuscar.launch.py use_slam:=true

# ノード確認
ros2 node list
```

### Phase 5: slam.launch.py作成（オプション）

**役割: zeuscar_slam/launch/slam.launch.pyのラッパー**

**必要性:**
- 統合launchからのパラメータ上書き
- 将来のカスタマイズ余地

**動作確認:**
```bash
# 前提: LiDAR + TFが起動済み
ros2 launch zeuscar_bringup slam.launch.py

# マップ確認
ros2 topic echo /map
```

---

## 7. デバッグ手法

### 7.1 起動確認チェックリスト

**Step 1: ノード起動確認**
```bash
ros2 node list
# 期待値: robot_state_publisher, rplidar_node, imu_node等
```

**Step 2: トピック確認**
```bash
ros2 topic list
# 期待値: /scan, /imu/data_raw, /cmd_vel, /tf, /tf_static
```

**Step 3: TFツリー確認**
```bash
ros2 run tf2_tools view_frames
# frames.pdfが生成される → 開いてツリー構造確認
```

**Step 4: トピックデータ確認**
```bash
ros2 topic echo /scan --once
ros2 topic echo /imu/data_raw --once
```

### 7.2 よくあるエラーと対策

**エラー1: `[ERROR] Failed to open serial port`**
- 原因: デバイスが接続されていない、または権限不足
- 対策: デバイス接続確認、udev rules設定、ユーザー権限追加

**エラー2: `Lookup would require extrapolation into the future`**
- 原因: TFツリーが不完全、またはタイムスタンプ不一致
- 対策: robot_state_publisher起動確認、use_sim_time統一

**エラー3: `Package 'zeuscar_bringup' not found`**
- 原因: ワークスペースのsource不足、またはビルド不足
- 対策: `colcon build`, `. install/setup.bash`再実行

**エラー4: `Frame [imu_link] does not exist`**
- 原因: URDFにimu_link未定義
- 対策: Phase 1（URDF修正）実施

---

## 8. 運用上のベストプラクティス

### 8.1 開発時の起動パターン

**パターン1: センサー単体テスト**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py \
  use_motor:=false use_slam:=false
```

**パターン2: モーター単体テスト**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py \
  use_lidar:=false use_imu:=false use_slam:=false
```

**パターン3: SLAM動作確認**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py \
  use_slam:=true use_rviz:=true
```

**パターン4: 最小構成（TFのみ）**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py \
  use_lidar:=false use_imu:=false use_motor:=false
```

### 8.2 ログレベル調整

**通常時:**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py
```

**デバッグ時:**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py \
  --ros-args --log-level DEBUG
```

**特定ノードのみDEBUG:**
```bash
ros2 launch zeuscar_bringup zeuscar.launch.py \
  --ros-args --log-level rplidar_node:=DEBUG
```

### 8.3 systemdサービス化（自動起動）

**運用フェーズでの推奨設定**

```bash
# サービスファイル作成
sudo nano /etc/systemd/system/zeuscar.service
```

```ini
[Unit]
Description=ZeusCar ROS2 Bringup
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/project/zeuscar-ros2-jazzy-rpi/ros2_ws
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch zeuscar_bringup zeuscar.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
# サービス有効化
sudo systemctl enable zeuscar.service
sudo systemctl start zeuscar.service
sudo systemctl status zeuscar.service
```

---

## 9. 設計判断の背景

### Q1: なぜ階層化launchを採用したのか？

**答え:**
- 個別パッケージの独立性維持（他プロジェクトでの再利用）
- 段階的なテストが可能（デバッグ効率向上）
- 中間層launchを他の統合構成でも使い回せる

### Q2: なぜSLAMをデフォルトOFFにしたのか？

**答え:**
- 現時点でオドメトリソース未実装（SLAM動作不完全）
- センサー確認等、SLAM不要なユースケースが多い
- 明示的に有効化することでリソース消費を抑制

### Q3: なぜエラー時も起動継続するのか？

**答え:**
- センサー一部不具合でも他機能は動作させたい
- デバッグ時にログを複数ノードから取得したい
- ただし、TF（robot_state_publisher）失敗時は全体停止（依存度が高いため）

---

## 10. 次のステップ

### 実装完了後の確認事項

1. **全テストケース実行** （docs/operations/specs/STORY-014-015_bringup_design.md参照）
2. **ドキュメント更新** （README.md、setup_guide.md）
3. **実機動作確認** （ZeusCar実機での全機能テスト）
4. **トラブルシューティング追記** （発見した問題の対策記録）

### 将来拡張への準備

- Nav2統合（use_nav2フラグ追加）
- オドメトリソース実装（IMU + Wheel Odometry）
- マルチロボット対応（namespace管理）
- Docker化（環境依存の抽象化）

---

## 11. 参考資料

- **ROS 2公式ドキュメント**: https://docs.ros.org/en/jazzy/
- **Launch Tutorial**: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html
- **本プロジェクト要件定義書**: docs/zeus_car_ros_2_jazzy_rpi_requirements.md
- **設計仕様書**: docs/operations/specs/STORY-014-015_bringup_design.md
- **Pythonコーディング規約**: docs/python_coding_guidelines.md
- **コメント規約**: docs/COMMENT_STYLE_GUIDE.md

---

## 12. まとめ

統合bringupの実装により、ZeusCarシステムは以下を実現します。

**実現すること:**
1. 一つのコマンドでシステム全体起動
2. 柔軟な構成切り替え（センサー選択、SLAM ON/OFF等）
3. 保守性の高い階層化構成
4. 初心者にも理解しやすいドキュメント

**重要な原則:**
- **独立性**: 各パッケージは単体でも動作可能
- **階層性**: 下層から順に動作確認
- **柔軟性**: Launch Argumentsで環境差異を吸収
- **明確性**: 設計判断の理由を記録

このガイドを参考に、実装担当エージェントがコードを作成し、テスト担当エージェントが動作検証を行うことで、高品質な統合bringupシステムが完成します。

---

## 13. 実装フェーズの記録

### 13.1 実装で行った判断

#### URDF imu_link の取り付け位置

IMUの実測値が未計測のため、以下の仮値を使用した。

```xml
<xacro:property name="imu_x" value="0.0"/>   <!-- 本体中心 -->
<xacro:property name="imu_y" value="0.0"/>   <!-- 本体中心 -->
<xacro:property name="imu_z" value="0.05"/>  <!-- base_linkから50mm上 -->
```

実測値が得られたら更新すること。imu_linkは`<link name="imu_link"/>`としてvisual/collisionなしで定義した（TF接続のみが目的のため）。

#### IncludeLaunchDescription の使用

統合launchは各パッケージのlaunchファイルを`IncludeLaunchDescription`で呼び出す方式を採用した。直接`Node`を定義する方式と比較して、以下の利点がある。

- 各パッケージの独立性を維持（パラメータやconfigの管理はパッケージ側に委譲）
- パッケージ単体のlaunchとの整合性が保たれる
- パッケージ側の変更が統合launchに自動反映される

#### 条件付き起動の実装

`IfCondition`を`IncludeLaunchDescription`の`condition`引数に直接渡す形式を採用した。`GroupAction`でグループ化する方式も検討したが、個別のON/OFF制御を重視してシンプルな構成とした。

#### zeuscar.launch.pyでのセンサー系引数の透過

`zeuscar.launch.py`では`use_lidar`/`use_imu`を自身のLaunch Argumentsとして宣言し、`sensors.launch.py`に`launch_arguments`で透過する。`sensors.launch.py`側の`DeclareLaunchArgument`のデフォルト値は上書きされるが、`sensors.launch.py`を単体起動した場合はそちらのデフォルト値が使用される。

### 13.2 setup.py の変更

`data_files`に`config`ディレクトリを追加した。現時点では`zeuscar_params.yaml`（将来拡張用プレースホルダー）のみだが、今後のシステム共通パラメータ追加に備える。

### 13.3 flake8/pep257対応

ROS2のlaunchファイルでは`ament_index_python`と`launch`パッケージが両方サードパーティ扱いされるが、異なるパッケージグループとしてflake8のI201ルールに検出される。import文のグループ間に空行を挿入して対応した。

### 13.4 テスト結果

| テストファイル | テスト数 | 結果 |
|---|---|---|
| test_urdf_imu_link.py | 12 | 全パス |
| test_launch_files.py | 39 | 全パス |
| **合計** | **51** | **全パス** |

---

**作成日**: 2026-02-07
**作成者**: Design Agent（設計）、Implementer Agent（実装フェーズ追記）
**対象読者**: 実装担当エージェント、テスト担当エージェント、初心者開発者
