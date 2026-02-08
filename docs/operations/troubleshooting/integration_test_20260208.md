# 実機統合テスト トラブルシューティング (2026-02-08)

## 概要

STORY-014/015 実機統合テストで発生した問題と解決策の記録。

---

## TSB-INT-001: /tf_static トピックの初回表示失敗

### 問題

`ros2 topic echo /tf_static --once` でデータが取得できない。
コマンドがタイムアウトし、TF の静的変換データを確認できない状況が発生した。

### 原因

`/tf_static` トピックは `transient_local` QoS durability を使用している。
通常の `ros2 topic echo` コマンドはデフォルトで `volatile` のサブスクライバーとして接続するため、遅延購読（パブリッシャーがデータを送信済みの後にサブスクライバーが接続する場合）ではデータを取得できないことがある。

`transient_local` は「遅延参加するサブスクライバーにも過去のデータを配信する」ための QoS 設定だが、サブスクライバー側も同じ `transient_local` を指定しなければこの機能が有効にならない。

### 解決策

`ros2 topic echo` コマンドに QoS オプションを明示的に指定して実行する。

```bash
ros2 topic echo /tf_static --qos-reliability reliable --qos-durability transient_local --once
```

補足として、QoS 設定の確認には以下のコマンドが有用である。

```bash
ros2 topic info /tf_static --verbose
```

このコマンドで Publisher 側の QoS ポリシーを確認し、Subscriber 側の設定を合わせることで確実にデータを受信できる。

### ステータス

解決済

---

## TSB-INT-002: 残存プロセスによるノード重複

### 問題

テストを複数回実行すると、前回の `/robot_state_publisher` や `/rplidar_node` プロセスが残存し、`ros2 node list` に重複ノードが表示される。
重複ノードが存在する状態でテストを実行すると、トピックのデータが期待通りに配信されないなどの予期しない動作が発生する。

### 原因

`ros2 launch` を Ctrl+C やバックグラウンド kill で終了した際に、子プロセス（特に `rplidar_composition`）が終了しないことがある。
ROS 2 の launch システムは SIGINT を受け取ると子プロセスにもシグナルを転送するが、一部のノード（特にハードウェアに直接アクセスするノード）ではシグナル処理中にブロックされ、正常終了しないケースがある。

### 解決策

テスト間で以下のクリーンアップコマンドを実行し、残存プロセスを強制終了する。

```bash
pkill -9 -f rplidar_composition
pkill -9 -f robot_state_publisher
pkill -9 -f imu_node
pkill -9 -f motor_controller
sleep 3
# 残存プロセスの確認
ps aux | grep -E "rplidar|robot_state|imu_node|motor_controller" | grep -v grep || echo "All clean"
```

**予防策**: テスト終了時は以下の手順で安全に終了する。

1. `kill -INT`（SIGINT）で launch プロセスを終了する
2. 3 秒間待機し、プロセスの正常終了を待つ
3. 残存プロセスがある場合は `kill -9` で強制終了する

```bash
# launch プロセスの PID を指定して終了する例
kill -INT <launch_pid>
sleep 3
# 残存プロセスがあれば強制終了
pkill -9 -f rplidar_composition 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f imu_node 2>/dev/null
pkill -9 -f motor_controller 2>/dev/null
```

### ステータス

解決済

---

## TSB-INT-003: 全機能同時起動時に /scan トピックが配信されない

### 問題

`zeuscar.launch.py`（TF + LiDAR + IMU + Motor 全機能）で起動すると、`rplidar_node` プロセスは存在するが `/scan` トピックにデータが配信されない。
`rplidar_composition` の CPU 使用率が 49% と異常に高くなる現象が確認された。

LiDAR 単独起動時（`ros2 launch zeuscar_lidar lidar.launch.py`）では約 6.2 Hz で正常にスキャンデータが配信される。

### 原因

以下の原因が考えられる（調査中）。

1. **リソース競合**: 4 ノード同時起動による Raspberry Pi 4 の CPU/メモリ負荷の増大。全機能同時起動時は CPU 使用率が全体的に上昇し、LiDAR のリアルタイムデータ処理が間に合わなくなる可能性がある
2. **通信タイミング競合**: LiDAR の USB シリアル通信と IMU の I2C 通信が同時に発生することで、カーネルレベルでの I/O スケジューリングに影響が出ている可能性がある
3. **プロセス再起動時のリソース未解放**: `rplidar_composition` が前回のプロセスのリソース（USB シリアルポートのロックなど）を完全に解放する前に再起動された可能性がある

### 暫定対応

- プロセス間のクリーンアップ待機時間を 5 秒以上に延長する
- 全機能起動後の待機時間を 30 秒に延長して再テストを実施予定
- ノードの起動順序を制御し、LiDAR ノードを最初に起動して安定した後に他のノードを起動する方法も検討中

### 今後の調査方針

- `top` / `htop` で全機能起動時の CPU 使用率の詳細を記録する
- ノードを 1 つずつ追加起動し、どの組み合わせで `/scan` が停止するか切り分ける
- launch ファイルに起動遅延（`TimerAction`）を導入し、段階的起動の効果を検証する

### 対策実施

`zeuscar.launch.py` に `TimerAction` を導入し、センサー系（LiDAR + IMU）の起動を robot_base の起動後に遅延させる修正を実施。

**変更内容:**
- `sensor_startup_delay` Launch Argument を追加（デフォルト: 3.0秒）
- `sensors_launch` を `TimerAction(period=sensor_startup_delay)` でラップ
- robot_base（TF + Motor）が先に起動し、3秒後にセンサー系が起動する

**起動遅延のカスタマイズ:**
```bash
# デフォルト（3秒遅延）
ros2 launch zeuscar_bringup zeuscar.launch.py

# 遅延時間を変更する場合
ros2 launch zeuscar_bringup zeuscar.launch.py sensor_startup_delay:=5.0
```

**テスト結果:** 55/55 テストパス（既存51 + 新規4）

### ステータス

解決済（TimerAction 導入）
