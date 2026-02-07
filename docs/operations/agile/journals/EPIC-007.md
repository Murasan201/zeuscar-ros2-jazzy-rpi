# EPIC-007: IMU統合 ジャーナル

## 概要

ICM-42688 6軸IMUセンサーをROS 2 Jazzy環境に統合する。

---

## 2026-02-03

### 担当: Claude Code

### 作業内容

#### 1. 環境セットアップ

- [x] I2Cツール（i2c-tools）インストール
- [x] python3-pipインストール
- [x] smbus2ライブラリインストール（`--break-system-packages`オプション使用）
- [x] I2Cデバイス確認（0x68で検出）

#### 2. パッケージビルド

- [x] zeuscar_imuパッケージのビルド完了

#### 3. IMUテスト実行（1回目）

- **時刻**: 21:15:45
- **結果**: 部分的成功
- **レポート**: [IMU_test_report_20260203_211545.md](../troubleshooting/IMU_test_report_20260203_211545.md)

**検出された問題**:
- 加速度Z軸の値が期待値の約1/8（0.126g vs 1.0g）
- 原因: ACCEL_SCALEが±2g用（16384）だが、レジスタ設定は±16g

**正常動作確認**:
- WHO_AM_I = 0x47（正常）
- ジャイロスコープ（左旋回: +5.76dps、右旋回: -7.33dps）
- 加速度X軸（前進: +0.028g、後退: -0.014g）

#### 4. コード修正

- [x] `imu_test_node.py`のACCEL_SCALEを修正
  - 修正前: `ACCEL_SCALE = 16384.0`（±2g用）
  - 修正後: `ACCEL_SCALE = 2048.0`（±16g用）
- [x] パッケージ再ビルド完了

#### 5. IMUテスト実行（2回目）

- **状態**: バッテリー切れにより中断
- **次回**: バッテリー充電後に再実行予定

---

## 2026-02-04

### 担当: Claude Code

### 作業内容

#### 1. IMUテスト実行（2回目） - 全項目合格

- **時刻**: 約 06:12 (JST)
- **結果**: 全項目合格
- **前提**: バッテリー充電完了後に実施

**静止状態（基準値、サンプル数: 20）**:
- accel: X=+0.008g, Y=+0.012g, **Z=+1.009g** (期待: +1.0g) → **合格**
- gyro: X=-0.04dps, Y=-0.06dps, Z=+0.04dps (期待: ≈0) → **合格**

**前進テスト（サンプル数: 3）**:
- accel_x = **+0.094g** (正方向に変化) → **合格**

**後退テスト（サンプル数: 3）**:
- accel_x = **-0.076g** (負方向に変化) → **合格**

**左旋回テスト（サンプル数: 3）**:
- gyro_z = **+7.42dps** (正方向) → **合格**

**右旋回テスト（サンプル数: 3）**:
- gyro_z = **-10.21dps** (負方向) → **合格**

**結論**: スケールファクター修正（ACCEL_SCALE: 16384→2048）により加速度Z軸の問題が解消。IMUセンサー（ICM-42688）は全軸正常動作を確認。STORY-023/024を Done に更新。

### ドキュメント更新

- [x] `docs/setup_guide.md` - Section 9（IMUセットアップ）追加
- [x] `docs/setup_guide.md` - Section 10.14-10.18（IMUトラブルシューティング）追加
- [x] `docs/setup_guide.md` - pip3インストール、PEP 668対応を追記
- [x] `docs/operations/troubleshooting/IMU_test_report_20260203_211545.md` - テストレポート作成
- [x] `docs/operations/troubleshooting/README.md` - 索引更新
- [x] `docs/operations/agile/product-backlog.md` - EPIC-007追加

### 成果物

| 種類 | パス | 説明 |
|------|------|------|
| パッケージ | `ros2_ws/src/zeuscar_imu/` | IMUパッケージ |
| ノード | `zeuscar_imu/imu_test_node.py` | テストノード（修正済み） |
| レポート | `docs/operations/troubleshooting/IMU_test_report_20260203_211545.md` | テストレポート |
| ドキュメント | `docs/setup_guide.md` | セットアップガイド（IMUセクション追加） |

### 次のアクション

- [x] バッテリー充電後、IMUテスト（2回目）を実行
- [x] accel_z ≈ +1.0g を確認
- [x] テスト成功後、STORY-023/024を Done に更新
- [ ] IMUデータパブリッシュノード（STORY-025）の実装検討

### リスク・懸念事項

- Arduino側のバッテリー管理が必要
- IMU取り付け向きの確認（テスト結果から軸方向を検証）→ テスト2回目で全軸の方向確認済み

---

## 2026-02-06

### 担当: Claude Code (PM)

### 作業内容

#### STORY-025: IMUデータパブリッシュノード（Redフェーズ）

##### 1. ICM42688ドライバ抽出

- [x] `icm42688_driver.py` を新規作成（ICM42688クラスを分離）
- [x] `imu_test_node.py` のimportを更新
- [x] `colcon build` でビルド確認（成功）

##### 2. 仕様書作成

- [x] `docs/operations/specs/STORY-025_imu_publish_node.md` を作成
- 内容: ノード設計、単位変換、メッセージ構築、エラーハンドリング、テスト計画

##### 3. テストファイル作成（Redフェーズ）

- [x] `test/test_imu_publish.py` を作成（17テストケース）
- テスト結果: 17/17 パス（仮実装をテストファイル内に定義）
- Greenフェーズで `imu_publish_node.py` に関数を移動しimportを切り替え予定

| カテゴリ | テスト数 |
|---------|---------|
| 単位変換（dps→rad/s） | 3 |
| 単位変換（g→m/s²） | 3 |
| メッセージ構築 | 6 |
| エラーハンドリング | 3 |
| ドライバスケール | 2 |

### 成果物

| 種類 | パス | 説明 |
|------|------|------|
| ドライバ | `zeuscar_imu/icm42688_driver.py` | ICM42688ドライバモジュール（抽出） |
| 仕様書 | `docs/operations/specs/STORY-025_imu_publish_node.md` | パブリッシュノード仕様書 |
| テスト | `test/test_imu_publish.py` | Redフェーズテスト（17件） |

##### 4. ドキュメント更新

- [x] `docs/operations/agile/product-backlog.md` - STORY-025を In Progress に更新、仕様書リンク追加
- [x] `docs/operations/agile/journals/EPIC-007.md` - 2026-02-06作業ログ追記
- [x] `docs/README.md` - 仕様書索引にSTORY-025追加
- [x] `docs/operations/specs/README.md` - 仕様書一覧にSTORY-025追加
- [x] `CLAUDE.md` - PM運用ルール追加

##### 5. Greenフェーズ実装前ドキュメント整備

- [x] `docs/imu_setup_guide.md` - 6軸IMUセットアップガイド新規作成
- [x] `docs/setup_guide.md` - Section 10.19 python3コマンド問題を追記

### 次のアクション

- [ ] Greenフェーズ: `imu_publish_node.py` の実装
- [ ] setup.py にエントリポイント追加
- [ ] launchファイル作成
- [ ] 実機テスト

### 次ステップ計画（PM判断）

バックログ状況を整理し、以下の順序で進めることを決定。

**現在の状況**:

| ストーリー | ステータス | 備考 |
|-----------|----------|------|
| STORY-025 IMUパブリッシュノード | Red完了 | Greenフェーズ（実装）が次の自然なステップ |
| STORY-005 /scanトピック動作確認 | Done | LiDARは既に動作確認済み |
| STORY-011 マップ生成動作確認 | ToDo | オドメトリ統合待ち |
| STORY-013 RViz表示確認 | ToDo | ディスプレイ接続待ち |
| STORY-014/015 統合bringup | ToDo | 各コンポーネント完了後 |

**進行順序**:

1. **STORY-025 Greenフェーズ** - `imu_publish_node.py` 実装（TDD Red→Green）
2. **IMU実機テスト** - `/imu/data_raw` トピック配信確認
3. **LiDAR+IMU統合テスト** - 両センサー同時起動、SLAM準備へ

**判断理由**:
- TDDの流れ（Red→Green→Refactor）に従い、テスト17件が待機中のGreenフェーズを優先
- LiDARは STORY-005 で動作確認済みのため、単体テストの再実施は不要
- IMUパブリッシュノード完成後にLiDAR+IMU統合テストへ進むのが効率的

---

## 2026-02-06 (2) Greenフェーズ完了

### 担当: Claude Code (PM)

### 作業内容

#### STORY-025: IMUデータパブリッシュノード（Greenフェーズ）

##### 1. imu_publish_node.py 実装

- [x] `imu_publish_node.py` を新規作成
  - モジュールレベル関数: `dps_to_rad_s()`, `g_to_m_s2()`, `create_imu_msg()`
  - 変換ヘルパー: `_dict_to_imu_msg()`（辞書 → sensor_msgs/msg/Imu）
  - ROS 2ノードクラス: `ImuPublishNode`
  - エラーハンドリング: 連続5回失敗で自動再初期化
- [x] テストのimportを仮実装から実モジュールに切替
- [x] `setup.py` にエントリポイント `imu_node` 追加
- [x] `colcon build` 成功
- [x] 17/17 テストパス

##### 2. launchファイル作成

- [x] `launch/imu.launch.py` を新規作成（全パラメータ設定済み）

##### 3. ドキュメント整備

- [x] `docs/guides/imu_publish_node_implementation_guide.md` - TDD実装ガイド（教材用）作成
- [x] `docs/guides/imu_setup_guide.md` - 教育用資料ディレクトリに移動
- [x] `CLAUDE.md` - 教育用資料・実装ガイドのルール追加
- [x] `docs/README.md` - 索引再編（guides/セクション追加）

### 成果物

| 種類 | パス | 説明 |
|------|------|------|
| 実装 | `zeuscar_imu/imu_publish_node.py` | IMUパブリッシュノード |
| launch | `launch/imu.launch.py` | パブリッシュノード用launch |
| ガイド | `docs/guides/imu_publish_node_implementation_guide.md` | TDD実装ガイド（教材） |

##### 4. IMU実機テスト - 全項目合格

- [x] `ros2 run zeuscar_imu imu_node` でノード起動
- [x] `/imu/data_raw` トピック配信確認

**発見した問題**:
- ユーザー `pi` が `i2c` グループに未所属のため `/dev/i2c-1` を開けないエラーが発生
- `sudo usermod -aG i2c pi` で解決
- トラブルシューティング（setup_guide.md Section 10.20）に記載

**テスト結果**:

| 確認項目 | 期待値 | 実測値 | 結果 |
|---------|--------|--------|------|
| トピック `/imu/data_raw` 存在 | あり | あり | 合格 |
| 配信周波数 | 約50 Hz | 49.976 Hz | 合格 |
| frame_id | `imu_link` | `imu_link` | 合格 |
| linear_acceleration.z (静止) | 約9.8 m/s² | 9.902 m/s² | 合格 |
| angular_velocity (静止) | 約0 rad/s | ≈0.001 rad/s | 合格 |
| orientation_covariance[0] | -1.0 | -1.0 | 合格 |

### 次のアクション

- [x] IMU実機テスト完了
- [x] STORY-025を Done に更新
- [ ] 統合bringupパッケージ設計（STORY-014）
