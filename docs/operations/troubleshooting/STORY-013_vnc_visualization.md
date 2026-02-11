# STORY-013 VNC・可視化 トラブルシューティング

## TSB-VIS-001: RealVNC Serverが未インストール（Ubuntu 24.04）

### 症状

VNCクライアントから接続できない。ポート5900がリスンしていない。

```bash
ss -tlnp | grep 5900
# → 出力なし
```

### 原因

Raspberry Pi OS にはRealVNC Serverがプリインストールされているが、Ubuntu 24.04では手動インストールが必要。

### 対策

```bash
# インストーラをクローン
git clone https://github.com/azalinux/realvnc-server-aarch64-ubuntu /tmp/realvnc-server-aarch64-ubuntu
cd /tmp/realvnc-server-aarch64-ubuntu

# インストール
sudo dpkg -i VNC-Server-7.12.0-Linux-ARM64.deb

# サービス有効化・起動
sudo systemctl enable --now vncserver-x11-serviced.service
```

### 確認

```bash
ss -tlnp | grep 5900
# → LISTEN 0 5 0.0.0.0:5900 ...
```

### ステータス

**解決済み**

---

## TSB-VIS-002: VNC接続後に画面が真っ暗（Wayland有効）

### 症状

VNCクライアントから接続・認証は成功するが、画面が真っ暗で何も表示されない。

VNCサーバーログ:
```
ConsoleDisplay: Cannot find a running X server on vt1
```

### 原因

Ubuntu 24.04のGDMはデフォルトでWaylandセッションを使用する。RealVNC ServerはX11のみ対応しており、Waylandセッションのデスクトップをキャプチャできない。

### 確認方法

```bash
cat /etc/gdm3/custom.conf | grep WaylandEnable
# → #WaylandEnable=false  ← コメントアウト = Wayland有効
```

### 対策

GDM設定でWaylandを無効化し、X11を強制する:

```bash
sudo sed -i 's/#WaylandEnable=false/WaylandEnable=false/' /etc/gdm3/custom.conf
sudo systemctl restart gdm3
```

### ステータス

**解決済み**

---

## TSB-VIS-003: VNC接続後に画面が真っ暗（GUIセッション未ログイン）

### 症状

TSB-VIS-002（Wayland無効化）を適用後もVNC接続で画面が真っ暗。VNCサーバーはXサーバーを検出しているが、デスクトップが表示されない。

VNCサーバーログ:
```
ConsoleDisplay: Found running X server (pid=XXXX, binary=/usr/lib/xorg/Xorg)
```

### 原因

GDMのログイン画面（X11）は動作しているが、ユーザー `pi` がGUIデスクトップセッションにログインしていない。VNC Service Modeは既存のデスクトップセッションをミラーリングする仕組みのため、ログイン済みセッションがないと黒画面になる。

### 確認方法

```bash
loginctl list-sessions --no-legend
# ユーザー pi のセッションに seat0 / tty2 がなければGUIセッション未ログイン
```

### 対策

GDMの自動ログインを有効化する:

```bash
sudo sed -i 's/#  AutomaticLoginEnable = true/AutomaticLoginEnable=true/' /etc/gdm3/custom.conf
sudo sed -i 's/#  AutomaticLogin = user1/AutomaticLogin=pi/' /etc/gdm3/custom.conf
sudo systemctl restart gdm3
```

設定結果:
```
[daemon]
WaylandEnable=false
AutomaticLoginEnable=true
AutomaticLogin=pi
```

### ステータス

**解決済み**

---

## TSB-VIS-004: VNC接続後に画面が真っ暗（ヘッドレス環境でHDMI未検出）【主原因】

### 症状

TSB-VIS-002（Wayland無効化）、TSB-VIS-003（自動ログイン）を全て適用済みでも画面が真っ暗。

Xサーバーの状態を確認すると、HDMI出力が `disconnected`:
```bash
sudo DISPLAY=:0 XAUTHORITY=$(find /run/user/1000 -name "Xauthority" 2>/dev/null | head -1) xrandr
# → HDMI-1 disconnected primary (normal left inverted right x axis y axis)
# → HDMI-2 disconnected (normal left inverted right x axis y axis)
```

`/boot/firmware/config.txt` に `hdmi_force_hotplug=1` を設定しても効果なし。

### 原因

Ubuntu 24.04のRaspberry Pi 4では `vc4-kms-v3d`（Full KMS）グラフィックスドライバが使用される。このドライバは従来の `hdmi_force_hotplug` 設定を**無視**する。

物理ディスプレイが接続されていない（ヘッドレス）環境では、KMSドライバがHDMIコネクタを `disconnected` と判定し、フレームバッファを作成しない。結果として、Xサーバーは起動するがレンダリング先がなく、VNCに黒画面が返される。

### 対策

カーネルコマンドラインでダミーHDMIディスプレイを強制する:

```bash
# /boot/firmware/cmdline.txt の先頭に以下を追加（1行で、スペース区切り）
video=HDMI-A-1:1920x1080@60D
```

`D` サフィックスは「ディスプレイが物理的に接続されていなくても、このモードを強制する」という意味。

具体的な手順:

```bash
sudo sed -i 's/^/video=HDMI-A-1:1920x1080@60D /' /boot/firmware/cmdline.txt
sudo reboot
```

### 確認方法

再起動後:
```bash
sudo DISPLAY=:0 XAUTHORITY=$(find /run/user/1000 -name "Xauthority" 2>/dev/null | head -1) xrandr
# → HDMI-1 connected primary 1920x1080+0+0 ...  ← connected になっていれば成功
```

### 補足

- `hdmi_force_hotplug=1`（`config.txt`）はレガシードライバ（`fkms` や非KMS）用の設定であり、Full KMS（`vc4-kms-v3d`）では機能しない
- `vc4-fkms-v3d`（Fake KMS）に変更する方法もあるが、Ubuntu 24.04では非推奨
- `video=HDMI-A-1:1920x1080@60D` はカーネルレベルの設定のため、KMSドライバでも確実に動作する

### 参考

- [Raspberry Pi Forums: VNC login fails if HDMI is unplugged](https://forums.raspberrypi.com/viewtopic.php?t=341820)
- [murasanブログ: Raspberry Pi VNC接続の解決法](https://murasan-net.com/2024/01/24/raspberry-pi-vnc-disconnected/)

### ステータス

**解決済み**

---

## 設定ファイルまとめ（最終状態）

### /etc/gdm3/custom.conf

```
[daemon]
WaylandEnable=false
AutomaticLoginEnable=true
AutomaticLogin=pi
```

### /boot/firmware/config.txt（[pi4]セクション）

```
[pi4]
hdmi_group=2
hdmi_mode=82
hdmi_force_hotplug=1
max_framebuffers=2
arm_boost=1
```

### /boot/firmware/cmdline.txt（先頭に追加）

```
video=HDMI-A-1:1920x1080@60D ...（既存のパラメータ）
```

---

## TSB-VIS-005: RViz2で前方の点群が正しい位置に表示されない（EKFドリフト）

### 症状

RViz2起動直後は全方向の点群（LaserScan）が正しい位置に表示されるが、数分後に前方の点群がずれて表示されるようになる。後方の壁は正しく表示され、SLAM Mapも正常。

### 環境

- RPi4上でRViz2 + SLAM + EKF + LiDAR + IMU を同時稼働
- LiDAR設置台がダンボールで不安定
- ホイールオドメトリ未統合（IMU角速度のみでEKF動作）

### 原因

複合要因:

1. **LiDARモーター振動による機体揺れ**: ダンボール台が不安定なためLiDAR回転の遠心力で機体が微振動
2. **IMUが振動を検出**: EKFが角速度を積分し、実際には動いていないのにodom→base_footprintが徐々にドリフト
3. **RPi4 CPU高負荷**: RViz2同時稼働でEKFがレート超過（`Failed to meet update rate`）、slam_toolboxがメッセージドロップ（`discarding message because the queue is full`）
4. **ホイールオドメトリ未統合**: 「静止している」という制約がないため、ドリフトを抑制できない

### ログ証跡

```
[ekf_node] [ERROR] ekf_filter_node: Failed to meet update rate! Took 0.08s
[slam_toolbox] [INFO] Message Filter dropping message: frame 'laser_frame' ... 'discarding message because the queue is full'
[rviz2] [ERROR] rviz/glsl120/indexed_8bit_image.vert ... GLSL link result : active samplers with a different type refer to the same texture image unit
```

### 対策

**短期（環境改善）:**
- 安定した設置面（木板や金属台）に固定する
- RViz2をリモートPC側で実行し、RPi4の負荷を軽減する

**中期（IMUフィルタ導入）:**
- `imu_filter_madgwick` を導入して加速度+ジャイロから姿勢を推定
- roll/pitchは重力基準で安定化 → 振動由来のドリフトを抑制
- EKFに姿勢（orientation）を入力する

**長期（ホイールオドメトリ統合）:**
- モーターエンコーダからオドメトリを計算
- IMU + ホイールのセンサーフュージョンで静止時ドリフトを大幅抑制

### 補足

- SLAM Map自体は正常（蓄積データから構築するため）
- 起動直後の表示は正常のため、STORY-013の受け入れ基準は合格
- 6軸IMU（磁力計なし）のため、yawドリフトはmadgwickでも完全には解消しない

### ステータス

**既知の制限事項**（中期対策で改善予定）

---

## ハードウェアなしで実行可能なユニットテスト

VNC・RViz2の実機確認にはハードウェア（LiDAR・IMU）とGUI環境が必要だが、設定ファイル・launchファイルの値検証はハードウェアなしで実行できる。

### 実行手順

```bash
cd ~/ros2_ws

# ビルド（初回または変更後）
colcon build --packages-select zeuscar_bringup
source install/setup.bash

# ユニットテスト実行
python3 -m pytest src/zeuscar_bringup/test/test_ekf_launch.py -v
```

### テスト結果（2026-02-11）

```
30 passed in 1.81s
```

### テスト内容

| クラス名 | テスト数 | 検証対象 |
|---|---|---|
| TestEkfParamsFile | 15 | ekf_params.yaml（フレーム名、IMUトピック、imu0_config各フラグ、orientation設定） |
| TestOdometryLaunchFile | 5 | odometry.launch.py（ファイル存在、import、Launch Arguments、EKFノード含有） |
| TestZeuscarEkfArgument | 2 | zeuscar.launch.py の use_ekf 引数 |
| TestImuFilterParamsFile | 5 | imu_filter_params.yaml（use_mag、publish_tf、world_frame、gain） |
| TestSensorsLaunchMadgwick | 1 | sensors.launch.py に madgwick ノードが含まれること |
| TestPackageXmlDependency | 2 | package.xml の依存（robot_localization、imu_filter_madgwick） |

### 補足

- LiDAR・IMU接続不要、GUI環境不要
- SSH経由のヘッドレス環境でも実行可能
- テストファイル: `ros2_ws/src/zeuscar_bringup/test/test_ekf_launch.py`
- 設定変更やlaunchファイル修正後の回帰テストとして活用できる

---

## 更新履歴

| 日付 | 内容 |
|---|---|
| 2026-02-10 | 初版作成（TSB-VIS-001〜004、RealVNCヘッドレス環境構築の全記録） |
| 2026-02-11 | TSB-VIS-005追加（EKFドリフトによる点群表示ずれ、原因分析と対策方針） |
| 2026-02-11 | ハードウェアなしで実行可能なユニットテスト手順を追記（30テスト全パス） |
