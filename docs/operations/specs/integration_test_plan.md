# 実機統合テスト計画書

## 改訂履歴

| 版数 | 日付 | 作成者 | 変更内容 |
|---|---|---|---|
| 1.0 | 2026-02-08 | Claude Code | 初版作成 |

---

## 1. 目的

ZeusCar の全コンポーネント（TF/URDF、LiDAR、IMU、Motor）を統合 launch で同時起動し、正常動作を確認する。

## 2. テスト環境

### 2.1 ハードウェア

| 機器 | 型番 | 接続方式 | デバイスパス |
|------|------|----------|------------|
| Raspberry Pi 4 | Model B | - | - |
| LiDAR | RPLIDAR A1M8 | USB (CP210x) | /dev/rplidar |
| Arduino | Uno R3 / 互換品 | USB (CH340等) | /dev/arduino or /dev/ttyACM0 |
| IMU | ICM-42688 | I2C (bus 1, addr 0x68) | /dev/i2c-1 |

### 2.2 ソフトウェア

- Ubuntu 24.04 + ROS 2 Jazzy
- 全パッケージビルド済み（colcon build）

### 2.3 前提条件

- [ ] udev ルール設定済み（/dev/rplidar, /dev/arduino）
- [ ] I2C 権限設定済み（ユーザーが i2c グループに所属）
- [ ] ROS 2 環境 source 済み（`. install/setup.bash`）

---

## 3. テスト手順

### ステージ 0: 事前確認

テスト実施前にハードウェア接続とソフトウェア環境を確認する。

#### S0-1: ハードウェア接続確認

```bash
# LiDAR デバイス確認
ls -l /dev/rplidar

# Arduino デバイス確認
ls -l /dev/ttyACM0
# または
ls -l /dev/arduino

# IMU (I2C) デバイス確認
i2cdetect -y 1
# 0x68 に検出されること
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| /dev/rplidar 存在 | シンボリックリンクが存在する | |
| Arduino デバイス存在 | /dev/ttyACM0 または /dev/arduino が存在する | |
| IMU I2C 検出 | アドレス 0x68 に応答がある | |

#### S0-2: ソフトウェア環境確認

```bash
# ROS 2 環境確認
cd ~/ros2_ws
source install/setup.bash
ros2 pkg list | grep zeuscar
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| zeuscar_bringup | パッケージ一覧に含まれる | |
| zeuscar_description | パッケージ一覧に含まれる | |
| zeuscar_lidar | パッケージ一覧に含まれる | |
| zeuscar_imu | パッケージ一覧に含まれる | |
| zeuscar_motor | パッケージ一覧に含まれる | |
| zeuscar_slam | パッケージ一覧に含まれる | |

#### S0-3: ビルド確認

```bash
cd ~/ros2_ws
colcon build
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| ビルド結果 | 全パッケージ成功（エラーなし） | |

---

### ステージ 1: 最小構成テスト（TF のみ）

TF 基盤が正常に起動することを確認する。

#### S1-1: TF 単独起動

```bash
# ターミナル1: 起動
ros2 launch zeuscar_bringup robot_base.launch.py use_motor:=false
```

```bash
# ターミナル2: 確認コマンド
ros2 node list
ros2 topic list
ros2 topic echo /tf_static --once
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| ノード起動 | /robot_state_publisher が表示される | |
| /tf_static トピック | トピック一覧に含まれる | |
| base_footprint → base_link | TF に含まれる | |
| base_link → laser_frame | TF に含まれる | |
| base_link → imu_link | TF に含まれる | |

**完了後:** Ctrl+C で停止

---

### ステージ 2: センサー追加テスト（TF + LiDAR + IMU）

センサーノードが TF と合わせて正常動作することを確認する。

#### S2-1: TF + LiDAR 起動

```bash
# ターミナル1: 起動（モーターOFF、IMU OFF）
ros2 launch zeuscar_bringup zeuscar.launch.py use_motor:=false use_imu:=false
```

```bash
# ターミナル2: 確認コマンド
ros2 node list
ros2 topic list
ros2 topic hz /scan
# 数秒待ち、周波数を確認後 Ctrl+C
ros2 topic echo /scan --once
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| rplidar_node 起動 | ノード一覧に含まれる | |
| /scan トピック | トピック一覧に含まれる | |
| /scan 周波数 | 約 5〜10 Hz（スキャン周波数） | |
| /scan frame_id | laser_frame | |
| /scan データ | ranges に有効な距離値が含まれる | |

**完了後:** Ctrl+C で停止

#### S2-2: TF + IMU 起動

```bash
# ターミナル1: 起動（モーターOFF、LiDAR OFF）
ros2 launch zeuscar_bringup zeuscar.launch.py use_motor:=false use_lidar:=false
```

```bash
# ターミナル2: 確認コマンド
ros2 node list
ros2 topic list
ros2 topic hz /imu/data_raw
# 数秒待ち、周波数を確認後 Ctrl+C
ros2 topic echo /imu/data_raw --once
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| imu_node 起動 | ノード一覧に含まれる | |
| /imu/data_raw トピック | トピック一覧に含まれる | |
| /imu/data_raw 周波数 | 約 50 Hz | |
| /imu/data_raw frame_id | imu_link | |
| linear_acceleration.z (静止) | 約 9.8 m/s² | |
| orientation_covariance[0] | -1.0（orientation 無効） | |

**完了後:** Ctrl+C で停止

#### S2-3: TF + LiDAR + IMU 同時起動

```bash
# ターミナル1: 起動（モーターOFF）
ros2 launch zeuscar_bringup zeuscar.launch.py use_motor:=false
```

```bash
# ターミナル2: 確認コマンド
ros2 node list
ros2 topic list
ros2 topic hz /scan &
ros2 topic hz /imu/data_raw &
# 5秒待ち
sleep 5
kill %1 %2
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| robot_state_publisher 起動 | ノード一覧に含まれる | |
| rplidar_node 起動 | ノード一覧に含まれる | |
| imu_node 起動 | ノード一覧に含まれる | |
| /scan 配信 | 周波数が安定している | |
| /imu/data_raw 配信 | 周波数が安定している | |
| エラーログなし | 起動ログにエラーが出ていない | |

**完了後:** Ctrl+C で停止

---

### ステージ 3: モーター追加テスト（TF + Motor）

モーターコントローラーが正常に起動し、コマンドを受け付けることを確認する。

#### S3-1: TF + Motor 起動

```bash
# ターミナル1: 起動（センサーOFF）
ros2 launch zeuscar_bringup robot_base.launch.py
```

```bash
# ターミナル2: 確認コマンド
ros2 node list
ros2 topic list

# cmd_vel トピックの購読確認
ros2 topic info /cmd_vel
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| motor_controller_node 起動 | ノード一覧に含まれる | |
| /cmd_vel トピック | トピック一覧に含まれる | |
| /cmd_vel 購読者数 | 1（motor_controller_node） | |
| /zeuscar/motor_cmd トピック | トピック一覧に含まれる | |
| シリアル接続 | エラーログなし | |

#### S3-2: モーター動作確認（任意）

**注意: ロボットが動き出すため、安全な場所で実施すること**

```bash
# ターミナル2: 前進コマンド送信（1秒後に停止）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 停止コマンド
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| 前進コマンド応答 | ロボットが前進する | |
| 停止コマンド応答 | ロボットが停止する | |

**完了後:** Ctrl+C で停止

---

### ステージ 4: 全機能統合テスト（TF + LiDAR + IMU + Motor）

全コンポーネントを同時起動し、システム全体の正常動作を確認する。

#### S4-1: 全機能起動（SLAM除く）

```bash
# ターミナル1: 全機能起動
ros2 launch zeuscar_bringup zeuscar.launch.py
```

```bash
# ターミナル2: ノード・トピック一覧確認
ros2 node list
ros2 topic list
```

**判定基準（ノード）:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| /robot_state_publisher | 起動している | |
| rplidar_node 系 | 起動している | |
| /imu_node | 起動している | |
| /motor_controller_node | 起動している | |

**判定基準（トピック）:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| /tf_static | 配信されている | |
| /scan | 配信されている | |
| /imu/data_raw | 配信されている | |
| /cmd_vel | 購読可能 | |
| /robot_description | 配信されている | |

#### S4-2: TF ツリー整合性確認

```bash
# ターミナル2: TFツリー確認
ros2 run tf2_tools view_frames
# frames_YYYY-MM-DD_HH.MM.SS.pdf が生成される
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| base_footprint → base_link | 接続されている | |
| base_link → laser_frame | 接続されている | |
| base_link → imu_link | 接続されている | |
| 孤立フレームなし | 全フレームがツリーに接続されている | |

#### S4-3: センサーデータ品質確認

```bash
# ターミナル2: 各センサーの周波数を同時確認
ros2 topic hz /scan &
ros2 topic hz /imu/data_raw &
sleep 10
kill %1 %2
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| /scan 周波数 | 安定（約 5〜10 Hz） | |
| /imu/data_raw 周波数 | 安定（約 50 Hz） | |
| 周波数の変動 | 大きな乱れがない（±10%以内） | |

#### S4-4: 長時間安定性確認（3分間）

```bash
# ターミナル1: 起動状態を維持（3分間）
# エラーログの出現を監視する
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| 3分間エラーなし | エラーログが出力されない | |
| ノード生存 | 全ノードが起動し続けている | |
| トピック配信継続 | /scan, /imu/data_raw が配信され続けている | |

**完了後:** Ctrl+C で停止

---

### ステージ 5: Launch Arguments 動作テスト

条件付き起動フラグが正しく機能することを確認する。

#### S5-1: LiDAR OFF テスト

```bash
# ターミナル1
ros2 launch zeuscar_bringup zeuscar.launch.py use_lidar:=false use_motor:=false
```

```bash
# ターミナル2
ros2 node list
ros2 topic list
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| rplidar_node | 起動していない | |
| /scan トピック | 存在しない | |
| imu_node | 正常に起動している | |

**完了後:** Ctrl+C で停止

#### S5-2: IMU OFF テスト

```bash
# ターミナル1
ros2 launch zeuscar_bringup zeuscar.launch.py use_imu:=false use_motor:=false
```

```bash
# ターミナル2
ros2 node list
ros2 topic list
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| imu_node | 起動していない | |
| /imu/data_raw トピック | 存在しない | |
| rplidar_node | 正常に起動している | |

**完了後:** Ctrl+C で停止

#### S5-3: Motor OFF テスト

```bash
# ターミナル1
ros2 launch zeuscar_bringup zeuscar.launch.py use_motor:=false
```

```bash
# ターミナル2
ros2 node list
```

**判定基準:**

| 確認項目 | 期待結果 | 結果 |
|---------|---------|------|
| motor_controller_node | 起動していない | |
| rplidar_node | 正常に起動している | |
| imu_node | 正常に起動している | |

**完了後:** Ctrl+C で停止

---

## 4. テスト結果サマリー

### 4.1 ステージ別結果

| ステージ | テスト内容 | 結果 | 備考 |
|---------|---------|------|------|
| S0 | 事前確認 | 合格 | 全デバイス検出、6パッケージビルド成功 |
| S1 | 最小構成（TF のみ） | 合格 | TFツリー3フレーム全確認 |
| S2 | センサー追加（LiDAR + IMU） | 合格 | /scan 5.9-6.6 Hz, /imu/data_raw 50 Hz |
| S3 | モーター追加 | 合格 | シリアル接続確認、cmd_vel送受信成功 |
| S4 | 全機能統合 | 条件付き合格 | 段階的起動で動作。同時起動は TSB-INT-003 |
| S5 | Launch Arguments 動作確認 | 合格 | 3フラグ全て正常動作 |

### 4.2 検出された問題

| No | ステージ | 問題内容 | 重要度 | 対応 |
|----|---------|---------|--------|------|
| TSB-INT-001 | S1 | /tf_static の QoS 不一致で echo 失敗 | 低 | 解決済（QoSオプション指定） |
| TSB-INT-002 | S2 | 残存プロセスによるノード重複 | 中 | 解決済（クリーンアップ手順確立） |
| TSB-INT-003 | S4 | 全機能同時起動時に rplidar 初期化失敗 | 高 | 段階的起動で回避。TimerAction 導入を推奨 |

### 4.3 テスト環境メモ

- **実施日**: 2026-02-08
- **実施者**: Claude Code (PM)
- **バッテリー状態**: 充電済み（Arduino電源ON）
- **その他**: 車輪浮かせ状態で実施。モーター動作は車輪回転で確認

---

## 5. 合否判定

### 5.1 合格基準

- ステージ 0〜4 の全判定項目が合格していること
- ステージ 5 の全判定項目が合格していること
- 3分間の安定性テスト（S4-4）でエラーが発生しないこと

### 5.2 条件付き合格

以下の場合は条件付き合格とし、備考に記録する:

- Arduino 未接続時（S3 スキップ可）: Motor 以外が正常動作すれば合格
- バッテリー残量不足（S3-2 スキップ可）: モーター動作確認を後日実施

### 5.3 不合格基準

- ステージ 1 (TF 基盤) が失敗した場合は以降のテストを中止
- センサーノードが起動後すぐにクラッシュする場合

---

## 6. 対応する仕様書・テスト項目の対応表

本テスト計画は `STORY-014-015_bringup_design.md` セクション 5 のテスト項目に基づく。

| 本計画 | 設計仕様テストID | 対応関係 |
|--------|---------------|---------|
| S0 | - | 前提条件確認（仕様書 5.3 に対応） |
| S1-1 | T1-1, T2-1 | 個別パッケージ + 統合 launch 起動 |
| S2-1 | T1-2, T4-3 | LiDAR 単体 + フレーム参照 |
| S2-2 | T1-3, T4-4 | IMU 単体 + フレーム参照 |
| S2-3 | T2-2, T2-4 | センサー統合 + 複合起動 |
| S3-1 | T1-4, T2-1 | Motor 単体 + ベースシステム起動 |
| S4-1 | T2-5 | 全機能起動 |
| S4-2 | T4-1, T4-2 | TF ツリー整合性 |
| S4-3 | - | データ品質（追加テスト） |
| S4-4 | - | 長時間安定性（追加テスト） |
| S5-1 | T3-1 | use_lidar:=false |
| S5-2 | T3-2 | use_imu:=false |
| S5-3 | T3-3 | use_motor:=false |

**対象外テスト（本計画ではスキップ）:**

| 設計仕様テストID | 理由 |
|---------|------|
| T1-5 (SLAM 単独) | オドメトリ未実装のため正常動作しない |
| T3-4 (use_slam:=true) | 同上 |
| T5-1〜T5-4 (エラーハンドリング) | デバイス切断テストは破壊的操作のため別途実施 |
| T6-1〜T6-4 (RViz) | ディスプレイ接続環境が必要なため別途実施 |
