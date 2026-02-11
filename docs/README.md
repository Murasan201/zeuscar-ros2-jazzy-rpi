# docs/ ドキュメント索引

## 要件定義

- [zeus_car_ros_2_jazzy_rpi_requirements.md](zeus_car_ros_2_jazzy_rpi_requirements.md) - 本プロジェクトの要件定義書

## セットアップガイド

- [setup_guide.md](setup_guide.md) - 環境構築手順（初心者向け）

## 開発プロセス

- [agile_tdd_operating_rules.md](agile_tdd_operating_rules.md) - アジャイル・TDD運用ルール
- [multi_agent_development_flow.md](multi_agent_development_flow.md) - マルチエージェント開発フロー

## 実装ガイドライン

- [python_coding_guidelines.md](python_coding_guidelines.md) - Pythonコーディングガイドライン
- [COMMENT_STYLE_GUIDE.md](COMMENT_STYLE_GUIDE.md) - コメントスタイルガイド

## 教育用資料・実装ガイド (guides/)

- [guides/imu_setup_guide.md](guides/imu_setup_guide.md) - ICM-42688 6軸IMUセットアップガイド（配線からROS 2トピック配信まで）
- [guides/imu_publish_node_implementation_guide.md](guides/imu_publish_node_implementation_guide.md) - IMUパブリッシュノード実装ガイド（TDD教材）
- [guides/bringup_integration_guide.md](guides/bringup_integration_guide.md) - 統合Bringup実装ガイド（システム全体起動の設計と実装）
- [guides/odometry_ekf_implementation_guide.md](guides/odometry_ekf_implementation_guide.md) - オドメトリ（EKF）実装ガイド（IMU→オドメトリ→SLAM）
- [guides/agent_teams_guide.md](guides/agent_teams_guide.md) - Claude Code Agent Teams ガイド（並列協調開発）

## ハードウェア仕様

- [hardware/icm42688_imu_sensor.md](hardware/icm42688_imu_sensor.md) - ICM-42688 IMUセンサー仕様書（配線図・動作確認テスト手順含む）

## リソース

- `images/` - 画像ファイル格納ディレクトリ

## 運用ドキュメント (operations/)

### アジャイル管理 (operations/agile/)

- [product-backlog.md](operations/agile/product-backlog.md) - プロダクトバックログ
- [sprint-journal.md](operations/agile/sprint-journal.md) - スプリントジャーナル
- [action-items.md](operations/agile/action-items.md) - アクションアイテム
- `journals/` - バックログ別ジャーナル
- `assignments/` - 担当者指示書

### 仕様・設計 (operations/specs/)

- [specs/README.md](operations/specs/README.md) - 仕様書ガイド
- [specs/EPIC-006_arduino_interface.md](operations/specs/EPIC-006_arduino_interface.md) - Arduino通信インタフェース仕様書
- [specs/STORY-025_imu_publish_node.md](operations/specs/STORY-025_imu_publish_node.md) - IMUデータパブリッシュノード仕様書
- [specs/STORY-014-015_bringup_design.md](operations/specs/STORY-014-015_bringup_design.md) - 統合bringup設計仕様書（全パッケージ統合起動の設計）
- [specs/integration_test_plan.md](operations/specs/integration_test_plan.md) - 実機統合テスト計画書（全コンポーネント同時起動テスト）
- [specs/STORY-011_odometry_slam_verification.md](operations/specs/STORY-011_odometry_slam_verification.md) - オドメトリ統合・SLAM動作確認 実装計画書

### エスカレーション (operations/pm-briefs/)

- [pm-briefs/README.md](operations/pm-briefs/README.md) - PMブリーフガイド

### 作業依頼 (operations/work-orders/)

- [work-orders/README.md](operations/work-orders/README.md) - 作業依頼書ガイド

### トラブルシューティング (operations/troubleshooting/)

- [troubleshooting/README.md](operations/troubleshooting/README.md) - トラブルシューティングガイド

### その他

- [information-requests.md](operations/information-requests.md) - 情報依頼ログ

### テンプレート (operations/templates/)

- [backlog_journal.md](operations/templates/backlog_journal.md) - バックログジャーナルテンプレート
