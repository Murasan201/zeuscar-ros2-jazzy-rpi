# プロダクトバックログ

## 概要

本ドキュメントはZeusCar ROS 2 Jazzy Raspberry Piプロジェクトのバックログを管理する。

## エピック一覧

| エピックID | タイトル | 説明 | ステータス |
|---|---|---|---|
| EPIC-001 | ROS 2環境構築 | Raspberry Pi上のROS 2 Jazzy環境を構築 | Done |
| EPIC-002 | LiDAR統合 | RPLIDAR A1M8のROS 2統合 | Done |
| EPIC-003 | TF/URDF設計 | ロボットのTFツリーとURDF定義 | Done |
| EPIC-004 | SLAM構築 | slam_toolboxによる自己位置推定 | ToDo |
| EPIC-005 | 可視化 | RVizによる可視化環境 | ToDo |

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
| STORY-009 | zeuscar_slamパッケージ作成 | Medium | - | ToDo | - |
| STORY-010 | slam_toolbox設定 | Medium | - | ToDo | - |
| STORY-011 | マップ生成動作確認 | Medium | - | ToDo | - |

### EPIC-005: 可視化

| ID | タイトル | 優先度 | 見積 | ステータス | 仕様書 |
|---|---|---|---|---|---|
| STORY-012 | RViz設定ファイル作成 | Medium | - | ToDo | - |
| STORY-013 | LaserScan/TF/RobotModel表示確認 | Medium | - | ToDo | - |

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
