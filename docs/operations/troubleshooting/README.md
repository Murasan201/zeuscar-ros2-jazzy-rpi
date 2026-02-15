# トラブルシューティングディレクトリ

## 概要

バックログID別のトラブルシューティングログを管理する。

## 命名規則

`<BacklogID>_<概要>.md`

例: `STORY-003_lidar_connection.md`

## 必須セクション

1. 問題の概要
2. 発生状況・再現手順
3. 調査ログ
4. 原因
5. 対処方法
6. 再発防止策

## トラブルシューティング一覧

| ID | ファイル名 | 概要 | ステータス |
|---|---|---|---|
| EPIC-001 | EPIC-001_ros2_environment.md | ROS 2環境構築時のエラー（リポジトリ設定、colcon、ros2コマンド） | 解決済 |
| EPIC-002 | EPIC-002_lidar.md | LiDAR統合時のエラー（udevルール、デバイス権限） | 解決済 |
| EPIC-003 | EPIC-003_tf_urdf.md | TF/URDF設計時のエラー（xacro未インストール、ParameterValue） | 解決済 |
| EPIC-004 | EPIC-004_slam.md | SLAM構築時のエラー（オドメトリ、マップ生成） | 準備中 |
| EPIC-005 | EPIC-005_visualization.md | 可視化時のエラー（RViz2、OpenGL、リモート接続） | 準備中 |
| GENERAL | GENERAL_ssh_connection.md | SSH接続エラー（ホストキー未登録、SSH鍵未設定） | 解決済 |
| IMU | IMU_test_report_20260203_211545.md | IMUテストレポート（加速度スケールファクター不一致） | 要修正 |
| STORY-014/015 | [integration_test_20260208.md](integration_test_20260208.md) | 実機統合テスト（TF静的トピック、プロセス残存、全機能同時起動） | 解決済 |
| STORY-011 | [STORY-011_odometry_ekf.md](STORY-011_odometry_ekf.md) | オドメトリ・EKF統合（DDS障害、slam_toolbox、LiDAR不安定化、マップ保存） | 解決済 |
| STORY-013 | [STORY-013_vnc_visualization.md](STORY-013_vnc_visualization.md) | VNC・RViz可視化（RealVNC構築、Wayland、ヘッドレスHDMI） | 解決済 |
| TSB-MOT | [motor_arduino_power.md](motor_arduino_power.md) | モーター駆動時Arduino電源断（バッテリー電圧降下）、シリアルポート競合 | 対応中 |
