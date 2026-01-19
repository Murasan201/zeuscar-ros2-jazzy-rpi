# EPIC-002: LiDAR統合 トラブルシューティング

## 概要

EPIC-002（LiDAR統合）で発生した問題と対処法を記録する。

---

## 問題一覧

### TSB-002-001: udevルールファイル未作成

**発生日**: 2026-01-19

**症状**:
LiDARを接続しても `/dev/rplidar` シンボリックリンクが作成されない。
```bash
ls -la /dev/rplidar
ls: cannot access '/dev/rplidar': No such file or directory
```

`/dev/ttyUSB0` としては認識されている。

**原因**:
udevルールファイル（`/etc/udev/rules.d/99-rplidar.rules`）が作成されていない。

**解決策**:

1. udevルールファイルを作成:
```bash
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
# RPLIDAR A1M8 (Silicon Labs CP210x)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
EOF
```

2. udevルールを再読み込み:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**確認方法**:
```bash
ls -la /dev/rplidar
# 出力例: lrwxrwxrwx 1 root root 7 Jan 19 22:00 /dev/rplidar -> ttyUSB0
```

**備考**:
- セットアップガイドのSection 4.3に手順は記載されていたが、実行されていなかった
- udevルールはLiDAR接続前に設定しておくとよい

---

### TSB-002-002: udevトリガー後もシンボリックリンクが作成されない

**発生日**: 2026-01-19

**症状**:
udevルールを作成し `udevadm trigger` を実行しても、`/dev/rplidar` が作成されない。

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/rplidar
# ls: cannot access '/dev/rplidar': No such file or directory
```

**原因**:
`udevadm trigger` だけでは既存のデバイスに対してルールが適用されない場合がある。

**解決策**:
特定のデバイスに対して直接トリガーを実行する:

```bash
sudo udevadm trigger --action=add /dev/ttyUSB0
```

**確認方法**:
```bash
ls -la /dev/rplidar
# 出力例: lrwxrwxrwx 1 root root 7 Jan 19 22:00 /dev/rplidar -> ttyUSB0
```

**備考**:
- LiDARを一度抜いて再接続しても同じ効果がある
- デバイスのVendor ID/Product IDは `udevadm info -a -n /dev/ttyUSB0` で確認可能

---

### TSB-002-003: LiDARデバイスの権限エラー

**発生日**: -（予防的記録）

**症状**:
LiDARを起動しようとすると権限エラーが発生する。

```
[ERROR] [rplidar_node]: Error, cannot bind to the specified serial port /dev/rplidar
```

**原因**:
ユーザーがデバイスファイルへのアクセス権限を持っていない。

**解決策**:

方法1: udevルールでMODE="0666"を設定（推奨）
```bash
# /etc/udev/rules.d/99-rplidar.rules
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
```

方法2: ユーザーをdialoutグループに追加
```bash
sudo usermod -aG dialout $USER
# ログアウト・ログインが必要
```

**確認方法**:
```bash
# 権限の確認
ls -la /dev/ttyUSB0
# MODE="0666"の場合: crw-rw-rw- 1 root dialout ...

# グループの確認
groups
# dialout が含まれていればOK
```

---

## デバイス情報の確認方法

LiDARのUSBデバイス情報を確認する方法:

```bash
# Vendor ID / Product ID の確認
udevadm info -a -n /dev/ttyUSB0 | grep -E "(idVendor|idProduct)"
```

RPLIDAR A1M8 (Silicon Labs CP210x) の場合:
- idVendor: 10c4
- idProduct: ea60

---

## 動作確認手順

### 1. デバイス認識の確認

```bash
ls -la /dev/rplidar
# シンボリックリンクが存在することを確認
```

### 2. LiDARの起動

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch zeuscar_lidar lidar.launch.py
```

正常起動時のログ:
```
[rplidar_composition-1] [INFO] [xxx] [rplidar_node]: RPLIDAR running on ROS 2 package rplidar_ros. SDK Version: '1.12.0'
[rplidar_composition-1] [INFO] [xxx] [rplidar_node]: RPLIDAR S/N: xxxxxxxx
[rplidar_composition-1] [INFO] [xxx] [rplidar_node]: Firmware Ver: x.xx
[rplidar_composition-1] [INFO] [xxx] [rplidar_node]: Hardware Rev: x
[rplidar_composition-1] [INFO] [xxx] [rplidar_node]: RPLidar health status : '0'
[rplidar_composition-1] [INFO] [xxx] [rplidar_node]: Start
```

### 3. トピックの確認

```bash
# トピック一覧
ros2 topic list | grep scan

# トピック情報
ros2 topic info /scan

# データ確認
ros2 topic echo /scan --field header --once
```

---

### TSB-002-004: LiDAR回転による振動・共振問題

**発生日**: 2026-01-19

**症状**:
LiDARの回転（遠心力）によりセンサー自体が揺れる。
しばらく動作させていると振動が徐々に大きくなる（共振状態）。

**原因**:
マウントフレームの剛性不足により、LiDARの回転周波数（約5.5Hz）と
フレームの固有振動数が近く、共振が発生している。

**調査結果: 回転速度のソフトウェア調整について**

RPLIDAR A1シリーズは**ソフトウェアでの回転速度調整ができない**。

```cpp
// rplidar_ros のソースコード（rplidar_node.cpp）
// A-seriesは固定値(600 PWM)がハードコードされている
if(!scan_frequency_tunning_after_scan){ //for RPLIDAR A serials
    drv->setMotorSpeed(600);  // 固定値、パラメータで変更不可
}
```

| シリーズ | 回転速度調整 | 備考 |
|----------|--------------|------|
| A1/A2/A3 | ✗ 不可 | PWM値がハードコード（600） |
| S1/S2/S3 | ○ 可能 | `scan_frequency`パラメータで調整可 |
| T1 | ○ 可能 | `scan_frequency`パラメータで調整可 |

**解決策（ハードウェア対策）**:

1. **マウント強化（推奨）**
   - 支柱を太くする
   - 三角形の補強材（ガセット）を追加
   - L字金具で補強
   ```
   改善前:        改善後:
      [LiDAR]        [LiDAR]
         |             /|\
         |            / | \
      [細い支柱]   [三角補強]
         |             |
      [シャーシ]    [シャーシ]
   ```

2. **振動吸収材の追加**
   - LiDARとマウントの間にゴムワッシャーを挟む
   - 防振ゴム（ソルボセイン等）を使用
   - 共振周波数をずらす効果がある

3. **取り付け高さを下げる**
   - 支柱を短くすることで振動が減少
   - ただしスキャン範囲への影響を考慮

4. **支柱の材質変更**
   - アルミ→スチールなど剛性の高い材質に変更
   - または断面積を大きくする

**代替案（ソフトウェア改造、非推奨）**:

回転速度を下げたい場合は、rplidar_rosのソースを改造する必要がある：

1. rplidar_rosのソースをクローン
2. `src/rplidar_node.cpp`の`setMotorSpeed(600)`を小さい値に変更
3. 自分でビルド・インストール

注意: 回転速度を下げるとスキャン品質（点群密度）も低下する。

**備考**:
- RPLIDAR A1M8のデフォルト回転速度は約5.5Hz（毎秒5.5回転）
- 共振は特定の周波数で発生するため、フレームの固有振動数を変えることが効果的
- マウント強化が最も確実な解決策

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-19 | 初版作成、TSB-002-001/002を記録 |
| 2026-01-19 | TSB-002-003（権限エラー）を予防的に追加 |
| 2026-01-19 | TSB-002-004（振動・共振問題）を追加 |
