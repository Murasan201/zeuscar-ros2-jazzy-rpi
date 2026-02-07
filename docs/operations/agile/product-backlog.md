# プロダクトバックログ

## 概要

本ドキュメントはZeusCar ROS 2 Jazzy Raspberry Piプロジェクトのバックログを管理する。

## エピック一覧

| エピックID | タイトル | 説明 | ステータス |
|---|---|---|---|
| EPIC-001 | ROS 2環境構築 | Raspberry Pi上のROS 2 Jazzy環境を構築 | Done |
| EPIC-002 | LiDAR統合 | RPLIDAR A1M8のROS 2統合 | Done |
| EPIC-003 | TF/URDF設計 | ロボットのTFツリーとURDF定義 | Done |
| EPIC-004 | SLAM構築 | slam_toolboxによる自己位置推定 | In Progress |
| EPIC-005 | 可視化 | RVizによる可視化環境 | In Progress |
| EPIC-006 | Arduino駆動系統合 | Raspberry Pi-Arduino間の通信インタフェース | Done |
| EPIC-007 | IMU統合 | ICM-42688 6軸IMUのROS 2統合 | Done |

---

## バックログ詳細

### EPIC-001: ROS 2環境構築

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-001 | ROS 2ワークスペース作成 | High | - | Done | - |
| STORY-002 | 基本パッケージ構成作成 | High | - | Done | - |

### EPIC-002: LiDAR統合

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-003 | zeuscar_lidarパッケージ作成 | High | - | Done | - |
| STORY-004 | LiDAR launchファイル作成 | High | - | Done | - |
| STORY-005 | /scanトピック動作確認 | High | - | Done | - |

### EPIC-003: TF/URDF設計

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-006 | zeuscar_descriptionパッケージ作成 | High | - | Done | - |
| STORY-007 | URDF作成 | High | - | Done | - |
| STORY-008 | TFツリー設計・実装 | High | - | Done | - |

### EPIC-004: SLAM構築

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-009 | zeuscar_slamパッケージ作成 | Medium | - | Done | - |
| STORY-010 | slam_toolbox設定 | Medium | - | Done | - |
| STORY-011 | マップ生成動作確認 | Medium | - | ToDo | ※オドメトリ統合待ち |

### EPIC-005: 可視化

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-012 | RViz設定ファイル作成 | Medium | - | Done | - |
| STORY-013 | LaserScan/TF/RobotModel表示確認 | Medium | - | ToDo | ※ディスプレイ接続待ち |

### EPIC-006: Arduino駆動系統合

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-016 | zeuscar_motorパッケージ作成 | High | - | Done | [EPIC-006_arduino_interface.md](../specs/EPIC-006_arduino_interface.md) |
| STORY-017 | motor_controller_node実装 | High | - | Done | [EPIC-006_arduino_interface.md](../specs/EPIC-006_arduino_interface.md) |
| STORY-018 | cmd_vel対応 | Medium | - | Done | [EPIC-006_arduino_interface.md](../specs/EPIC-006_arduino_interface.md) |
| STORY-019 | udevルール設定（Arduino） | Medium | - | Done | - |
| STORY-020 | Arduino駆動系動作確認 | High | - | Done | - |

### EPIC-007: IMU統合

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-021 | zeuscar_imuパッケージ作成 | High | - | Done | [icm42688_imu_sensor.md](../../hardware/icm42688_imu_sensor.md) |
| STORY-022 | IMUドライバ実装（ICM-42688） | High | - | Done | [icm42688_imu_sensor.md](../../hardware/icm42688_imu_sensor.md) |
| STORY-023 | IMUテストノード実装 | High | - | Done | [icm42688_imu_sensor.md](../../hardware/icm42688_imu_sensor.md) |
| STORY-024 | IMU動作確認 | High | - | Done | - |
| STORY-025 | IMUデータパブリッシュノード実装 | Medium | - | Done | [STORY-025_imu_publish_node.md](../specs/STORY-025_imu_publish_node.md) |

### 統合・起動

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-014 | zeuscar_bringupパッケージ作成 | High | - | ToDo | - |
| STORY-015 | 統合launchファイル作成 | High | - | ToDo | - |

---

## 優先度定義

- **High**: 必須機能、他の機能の前提となる
- **Medium**: 主要機能、スコープ内で実装
- **Low**: 将来拡張、スコープ外

---

## 更新履歴

| 日付 | 更新者 | 内容 |
|---|---|---|
| 2026-01-12 | - | 初版作成 |
| 2026-01-12 | - | EPIC-001完了、STORY-001/002を Done に更新 |
| 2026-01-12 | - | EPIC-002進行中、STORY-003/004を Done に更新 |
| 2026-01-19 | - | EPIC-003完了、STORY-006/007/008を Done に更新 |
| 2026-01-19 | - | EPIC-002完了、STORY-005を Done に更新 |
| 2026-01-19 | - | EPIC-004進行中、STORY-009/010を Done に更新 |
| 2026-01-19 | - | EPIC-005進行中、STORY-012を Done に更新 |
| 2026-01-24 | - | EPIC-006追加、STORY-016〜020を登録 |
| 2026-01-24 | - | EPIC-006完了、STORY-020を Done に更新 |
| 2026-02-03 | - | EPIC-007追加、STORY-021〜025を登録 |
| 2026-02-03 | - | STORY-021/022を Done、STORY-023/024を In Progress に更新 |
| 2026-02-04 | - | STORY-023/024を Done に更新（IMUテスト2回目全項目合格） |
| 2026-02-06 | - | STORY-025を In Progress に更新、仕様書・テスト作成（Redフェーズ） |
| 2026-02-06 | - | STORY-025 Greenフェーズ完了（実機テスト待ち） |
