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
| STORY-003 | zeuscar_lidarパッケージ作成 | - | Done | - |
| STORY-004 | LiDAR launchファイル作成 | - | Done | - |
| STORY-005 | /scanトピック動作確認 | - | Done | EPIC-002 |
| STORY-006 | zeuscar_descriptionパッケージ作成 | - | Done | EPIC-003 |
| STORY-007 | URDF作成 | - | Done | EPIC-003 |
| STORY-008 | TFツリー設計・実装 | - | Done | EPIC-003 |

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

**2026-01-12: EPIC-002 LiDAR統合**

1. **rplidar_rosパッケージインストール**
   - ros-jazzy-rplidar-rosをインストール

2. **zeuscar_lidarパッケージ更新**
   - lidar.launch.pyを作成（rplidar_rosをラップ）
   - rplidar_a1.yamlで設定パラメータを定義
   - 99-rplidar.rulesでudevルールを作成
   - setup.pyにlaunch/configのインストール設定を追加
   - package.xmlに依存関係を追加

3. **セットアップガイド更新**
   - Section 4（LiDARのセットアップ）を追記

**2026-01-19: EPIC-002 LiDAR統合完了**

1. **udevルール設定**
   - /etc/udev/rules.d/99-rplidar.rules を作成
   - /dev/rplidar シンボリックリンクを確認

2. **LiDAR動作確認**
   - ros2 launch zeuscar_lidar lidar.launch.py 起動成功
   - RPLIDAR SDK Version: 1.12.0
   - Firmware Ver: 1.29, Hardware Rev: 7
   - Scan mode: Sensitivity, max_distance: 12.0m

3. **/scanトピック確認**
   - Type: sensor_msgs/msg/LaserScan
   - frame_id: laser_frame（正しく設定）
   - Publisher count: 2

**2026-01-19: EPIC-003 TF/URDF設計完了**

1. **ロボット本体寸法の実測**
   - 全長（X軸方向）: 163mm
   - 全幅（Y軸方向）: 177mm

2. **LiDAR取り付け位置の実測**
   - X位置: +3.5mm（ロボット中心から前方）
   - Y位置: -4.5mm（ロボット中心から右方向）
   - Z位置: 235mm（地面からセンサー中央）
   - Yaw角: +90°（設計値、左方向に90度回転）

3. **TF変換パラメータ確定**
   - x: +0.0035m, y: -0.0045m, z: +0.235m
   - roll: 0, pitch: 0, yaw: +1.5708 rad

4. **仕様書更新**
   - IR-001_epic003_robot_dimensions.md にすべての実測値を記録

5. **zeuscar_descriptionパッケージ作成**
   - URDF（zeuscar.urdf.xacro）を作成
   - description.launch.pyを作成
   - xacroパッケージをインストール（ros-jazzy-xacro）
   - ParameterValue問題を修正（ROS 2 Jazzy固有の対応）

6. **ビルドと動作確認**
   - colcon buildでzeuscar_descriptionをビルド
   - robot_state_publisherが正常起動を確認
   - /tf_staticトピックで正しいTF変換を確認
     - base_footprint → base_link: z=0.05m
     - base_link → laser_frame: x=0.0035m, y=-0.0045m, z=0.185m, yaw=90°

7. **トラブルシューティング記録**
   - EPIC-003_tf_urdf.md を作成
   - TSB-003-001: xacroパッケージ未インストール
   - TSB-003-002: robot_descriptionパラメータのYAMLパースエラー

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
| 2026-01-12 | - | EPIC-002 LiDAR統合進捗を記録 |
| 2026-01-19 | - | EPIC-003 TF/URDF設計完了（ビルド・検証含む） |
| 2026-01-19 | - | EPIC-002 LiDAR統合完了（/scanトピック動作確認） |
