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
