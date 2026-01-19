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
