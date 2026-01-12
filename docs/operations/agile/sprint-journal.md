# スプリントジャーナル

## 概要

本ドキュメントはスプリント計画・進行ログの要約と各バックログジャーナルへのリンク集を管理する。

---

## プロジェクト初期化フェーズ

### 2026-01-12: プロジェクト基盤整備

#### 実施内容

1. **CLAUDE.mdルールファイル作成**
   - 業務開始時の確認事項（要件定義書参照）
   - ドキュメント管理ルール（docs/集約、README.md索引更新）
   - 開発プロセス（アジャイル・TDD、マルチエージェント体制）
   - 実装ルール（Python、コーディングガイドライン、コメントスタイル）

2. **docs/ディレクトリ整備**
   - 既存ドキュメントをdocs/に集約
   - docs/README.md索引を作成

3. **operations/ディレクトリ構成作成**
   - agile/ - バックログ、スプリントジャーナル、アクションアイテム
   - specs/ - 仕様書
   - pm-briefs/ - PMエスカレーション
   - work-orders/ - 作業依頼書
   - troubleshooting/ - トラブルシューティング
   - templates/ - テンプレート

4. **プロダクトバックログ初版作成**
   - 要件定義書に基づきエピック・ストーリーを定義
   - EPIC-001〜005、STORY-001〜015を登録

#### 成果物

- `CLAUDE.md`
- `docs/README.md`
- `docs/operations/agile/product-backlog.md`
- `docs/operations/agile/sprint-journal.md`
- `docs/operations/agile/action-items.md`
- `docs/operations/information-requests.md`
- 各種READMEとテンプレート

#### 次のアクション

- スプリント1の計画
- 優先バックログの選定と着手

---

## 現在のスプリント

### スプリント 1

- **期間**: 2026-01-12〜
- **ゴール**: ROS 2環境構築完了、LiDAR統合開始
- **ステータス**: 進行中

#### 選定バックログ

| ID | タイトル | 担当 | ステータス | ジャーナル |
|---|---|---|---|---|
| STORY-001 | ROS 2ワークスペース作成 | - | Done | - |
| STORY-002 | 基本パッケージ構成作成 | - | Done | - |

#### 完了した作業

**2026-01-12: EPIC-001完了**

1. **ROS 2 Jazzyインストール**
   - ros-jazzy-desktopをインストール
   - ~/.bashrcにsource設定を追加

2. **colconビルドツールインストール**
   - python3-colcon-common-extensionsをインストール

3. **ワークスペース作成**
   - ~/ros2_ws/src/ ディレクトリ構造を作成
   - .colcon_defaults.yamlでビルド設定を定義

4. **ZeusCar用パッケージ作成**
   - zeuscar_bringup（起動用）
   - zeuscar_description（URDF/TF）
   - zeuscar_lidar（LiDARラッパー）
   - zeuscar_slam（SLAM設定）

5. **ビルド確認**
   - colcon buildで4パッケージのビルド成功

6. **セットアップガイド更新**
   - docs/setup_guide.mdにワークスペース作成手順を追記

#### PMブリーフ管理

| ID | 概要 | ステータス |
|---|---|---|
| - | - | - |

---

## 過去のスプリント

（スプリント完了後にここへ移動）

---

## 更新履歴

| 日付 | 更新者 | 内容 |
|---|---|---|
| 2026-01-12 | - | 初版作成、プロジェクト初期化フェーズ記録 |
| 2026-01-12 | - | スプリント1開始、EPIC-001完了記録 |
