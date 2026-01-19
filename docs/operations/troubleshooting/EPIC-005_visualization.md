# EPIC-005: 可視化 トラブルシューティング

## 概要

EPIC-005（可視化）で発生した問題と対処法を記録する。

---

## 問題一覧

### TSB-005-001: RViz2が起動しない（OpenGLエラー）

**発生日**: -（予防的記録）

**症状**:
RViz2を起動すると、OpenGL関連のエラーが発生して起動に失敗する。

```
libGL error: MESA-LOADER: failed to open ...
libGL error: failed to load driver: ...
```

**原因**:
Raspberry PiのGPUドライバとRViz2（OpenGL）の互換性問題。

**解決策**:
ソフトウェアレンダリングを使用する:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```

永続化する場合:
```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
source ~/.bashrc
```

**備考**:
- ソフトウェアレンダリングはパフォーマンスが低下する
- 複雑なマップや大量のポイントクラウドでは遅延が発生する可能性がある

---

### TSB-005-002: LaserScanが表示されない

**発生日**: -（予防的記録）

**症状**:
RViz2を起動してもLaserScanデータが表示されない。

**原因候補**:

1. Fixed Frameが正しく設定されていない
2. QoS設定の不一致
3. /scanトピックがパブリッシュされていない

**解決策**:

1. Fixed Frameを確認・変更:
   - RViz2の「Global Options」→「Fixed Frame」
   - `map`、`odom`、または`base_footprint`に設定

2. QoS設定を変更:
   - LaserScan表示の「Topic」を展開
   - 「Reliability Policy」を「Best Effort」に変更

3. トピックの確認:
```bash
ros2 topic list | grep scan
ros2 topic echo /scan --once
```

---

### TSB-005-003: RobotModelが表示されない

**発生日**: -（予防的記録）

**症状**:
RViz2でRobotModel表示が赤くエラー表示される。

```
No transform from [base_link] to [map]
```

**原因候補**:

1. robot_state_publisherが起動していない
2. /robot_descriptionトピックがパブリッシュされていない
3. TFツリーが不完全

**解決策**:

1. robot_state_publisherを起動:
```bash
ros2 launch zeuscar_description description.launch.py
```

2. トピックの確認:
```bash
ros2 topic list | grep robot_description
ros2 topic echo /robot_description --once | head -20
```

3. TFツリーの確認:
```bash
ros2 run tf2_tools view_frames
```

---

### TSB-005-004: Mapが表示されない

**発生日**: -（予防的記録）

**症状**:
SLAMを起動してもRViz2でマップが表示されない。

**原因候補**:

1. /mapトピックがパブリッシュされていない
2. Fixed Frameが`map`になっていない
3. slam_toolboxがエラーで停止している

**解決策**:

1. マップトピックの確認:
```bash
ros2 topic list | grep map
ros2 topic info /map
```

2. Fixed Frameを`map`に設定

3. slam_toolboxのログを確認:
```bash
ros2 launch zeuscar_slam slam.launch.py
# エラーメッセージを確認
```

---

### TSB-005-005: リモートPCからRViz2で接続できない

**発生日**: -（予防的記録）

**症状**:
別PCでRViz2を起動しても、Raspberry Piのトピックが表示されない。

**原因候補**:

1. ROS_DOMAIN_IDが一致していない
2. ネットワークが分離されている
3. ファイアウォールがブロックしている

**解決策**:

1. ROS_DOMAIN_IDを合わせる:
```bash
# 両方のPCで同じ値を設定
export ROS_DOMAIN_ID=0
```

2. ネットワーク接続の確認:
```bash
# Raspberry PiのIPを確認
hostname -I

# 別PCからping
ping <raspberry_pi_ip>
```

3. トピックの確認:
```bash
# 別PCで
ros2 topic list
# Raspberry Piのトピックが表示されるか確認
```

---

## RViz2設定ファイルの場所

```bash
# ZeusCar用設定ファイル
~/ros2_ws/install/zeuscar_bringup/share/zeuscar_bringup/rviz/zeuscar.rviz
```

---

## 動作確認コマンド

```bash
# RViz2の起動
rviz2 -d ~/ros2_ws/install/zeuscar_bringup/share/zeuscar_bringup/rviz/zeuscar.rviz

# ソフトウェアレンダリングで起動
LIBGL_ALWAYS_SOFTWARE=1 rviz2

# トピック一覧
ros2 topic list

# TFツリーの可視化
ros2 run tf2_tools view_frames
```

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-19 | 初版作成（予防的トラブルシューティング） |
