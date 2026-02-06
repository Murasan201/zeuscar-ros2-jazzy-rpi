# ZeusCar ROS 2 Jazzy セットアップガイド

このガイドでは、Raspberry Pi 4上にZeusCar用のROS 2 Jazzy環境を構築する手順を、初心者にもわかりやすく解説します。

---

## はじめに

### このガイドの対象者

- Raspberry Piを使ったロボット開発に興味がある方
- ROS 2を初めて触る方
- LiDARを使った自己位置推定（SLAM）を学びたい方

### 完成イメージ

このガイドを最後まで進めると、以下のことができるようになります：

- Raspberry Pi 4でROS 2 Jazzyが動作する
- LiDAR（RPLIDAR A1M8）からセンサデータを取得できる
- RVizでロボットとセンサデータを可視化できる
- SLAMで地図を作成し、自己位置を推定できる

### 必要なもの

| 項目 | 説明 |
|---|---|
| Raspberry Pi 4 | 4GB以上のRAM推奨 |
| microSDカード | 32GB以上推奨 |
| ZeusCar | メカナムホイールロボット |
| RPLIDAR A1M8 | SLAMTEC製LiDARセンサ |
| Ubuntu PC | 開発・操作用（オプション） |

---

## 目次

1. [Ubuntu 24.04のインストール](#1-ubuntu-2404のインストール)
2. [ROS 2 Jazzyのインストール](#2-ros-2-jazzyのインストール)
3. [ワークスペースの作成](#3-ワークスペースの作成)
4. [LiDARのセットアップ](#4-lidarのセットアップ)
5. [TF（座標変換）の設定](#5-tf座標変換の設定)
6. [SLAMの設定](#6-slamの設定)
7. [RVizでの可視化](#7-rvizでの可視化)
8. [Arduino駆動系のセットアップ](#8-arduino駆動系のセットアップ)
9. [IMU（慣性計測ユニット）のセットアップ](#9-imu慣性計測ユニットのセットアップ)
10. [トラブルシューティング](#10-トラブルシューティング)

---

## 1. Ubuntu 24.04のインストール

### 1.1 概要

Raspberry Pi 4にUbuntu 24.04 LTS（Noble Numbat）をインストールします。ROS 2 Jazzyは Ubuntu 24.04を公式にサポートしています。

### 1.2 Raspberry Pi Imagerでのインストール

1. **Raspberry Pi Imagerをダウンロード**
   - 公式サイト（https://www.raspberrypi.com/software/）からダウンロード

2. **OSを選択**
   - 「Other general-purpose OS」→「Ubuntu」→「Ubuntu Desktop 24.04.3 LTS (64-bit)」を選択
   - ※バージョン番号（24.04.x）は時期により異なる場合があります

3. **SDカードに書き込み**
   - microSDカードを選択し、「Write」をクリック

4. **初期設定**
   - Raspberry Piに SDカードを挿入し、電源を入れる
   - 画面の指示に従って初期設定を完了

### 1.3 環境の確認

インストール完了後、ターミナルを開いて以下のコマンドで環境を確認します：

```bash
# OSバージョンの確認
cat /etc/os-release | head -5
```

出力例：
```
PRETTY_NAME="Ubuntu 24.04 LTS"
NAME="Ubuntu"
VERSION_ID="24.04"
VERSION="24.04 LTS (Noble Numbat)"
VERSION_CODENAME=noble
```

```bash
# アーキテクチャの確認
dpkg --print-architecture
```

出力例：
```
arm64
```

---

## 2. ROS 2 Jazzyのインストール

### 2.1 ROS 2とは

ROS 2（Robot Operating System 2）は、ロボットアプリケーションを開発するためのオープンソースフレームワークです。「Jazzy Jalisco」は2024年にリリースされた長期サポート（LTS）版です。

### 2.2 ロケールの確認

ROS 2はUTF-8ロケールが必要です。以下のコマンドで確認します：

```bash
locale
```

`LANG=C.UTF-8` または `LANG=en_US.UTF-8` のようにUTF-8が設定されていればOKです。

### 2.3 必要なパッケージのインストール

```bash
sudo apt update
sudo apt install -y software-properties-common curl
```

### 2.4 ROS 2リポジトリの追加

GPGキーを追加します：

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

リポジトリを追加します：

```bash
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### 2.5 ROS 2 Jazzy Desktopのインストール

パッケージリストを更新し、ROS 2をインストールします：

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

> **注意**: Raspberry Pi 4では、このインストールに30分〜1時間程度かかることがあります。

### 2.6 環境設定

ROS 2を使うたびに環境をセットアップする必要があります。以下のコマンドを `~/.bashrc` に追加すると、ターミナルを開くたびに自動的にセットアップされます：

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.7 インストールの確認

以下のコマンドでROS 2が正しくインストールされたか確認します：

```bash
ros2 topic list
```

何も起動していない場合は空のリストが表示されますが、エラーが出なければOKです。

また、以下のコマンドでノードの動作確認ができます：

```bash
ros2 run demo_nodes_cpp talker
```

別のターミナルで：

```bash
ros2 run demo_nodes_cpp listener
```

「Hello World」のメッセージが送受信されれば成功です。

---

## 3. ワークスペースの作成

### 3.1 ワークスペースとは

ROS 2では、自分で開発するパッケージを「ワークスペース」と呼ばれるディレクトリで管理します。ワークスペースには以下の構造があります：

```
ros2_ws/                  # ワークスペースのルート
├── src/                  # ソースコード（パッケージ）を配置
├── build/               # ビルド時に自動生成
├── install/             # インストール先（自動生成）
└── log/                 # ビルドログ（自動生成）
```

### 3.2 colconのインストール

`colcon`はROS 2の標準ビルドツールです。以下のコマンドでインストールします：

```bash
sudo apt install -y python3-colcon-common-extensions
```

### 3.3 ワークスペースの作成

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 3.4 colconのデフォルト設定

ビルド時のオプションを毎回指定するのは面倒なので、デフォルト設定ファイルを作成します：

```bash
cat > .colcon_defaults.yaml << 'EOF'
build:
  symlink-install: true
  cmake-args:
    - -DCMAKE_BUILD_TYPE=Release
EOF
```

設定の説明：
- `symlink-install`: インストール時にシンボリックリンクを使用（開発中の変更がすぐ反映される）
- `CMAKE_BUILD_TYPE=Release`: 最適化されたバイナリを生成

### 3.5 ZeusCar用パッケージの作成

ZeusCar用に以下の4つのパッケージを作成します：

| パッケージ名 | 用途 |
|---|---|
| zeuscar_bringup | 起動用launchファイル |
| zeuscar_description | URDFモデル・TF設定 |
| zeuscar_lidar | LiDARドライバラッパー |
| zeuscar_slam | SLAM設定 |

```bash
cd ~/ros2_ws/src

# 各パッケージの作成
ros2 pkg create --build-type ament_python zeuscar_bringup
ros2 pkg create --build-type ament_python zeuscar_description
ros2 pkg create --build-type ament_python zeuscar_lidar
ros2 pkg create --build-type ament_python zeuscar_slam
```

### 3.6 ディレクトリ構造の準備

各パッケージに必要なディレクトリを作成します：

```bash
# launchディレクトリの作成
mkdir -p zeuscar_bringup/launch
mkdir -p zeuscar_lidar/launch
mkdir -p zeuscar_slam/launch

# 設定ファイル用ディレクトリ
mkdir -p zeuscar_lidar/config
mkdir -p zeuscar_slam/config

# URDFファイル用ディレクトリ
mkdir -p zeuscar_description/urdf
```

### 3.7 ワークスペースのビルド

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

成功すると以下のように表示されます：

```
Starting >>> zeuscar_bringup
Starting >>> zeuscar_description
Starting >>> zeuscar_lidar
Starting >>> zeuscar_slam
Finished <<< zeuscar_lidar [26.2s]
Finished <<< zeuscar_description [26.3s]
Finished <<< zeuscar_slam [26.3s]
Finished <<< zeuscar_bringup [26.4s]

Summary: 4 packages finished [27.7s]
```

### 3.8 環境設定の追加

ワークスペースの環境を自動的に読み込むように設定します：

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.9 パッケージの確認

以下のコマンドで作成したパッケージが認識されているか確認します：

```bash
ros2 pkg list | grep zeuscar
```

出力例：
```
zeuscar_bringup
zeuscar_description
zeuscar_lidar
zeuscar_slam
```

---

## 4. LiDARのセットアップ

### 4.1 RPLIDAR A1M8について

RPLIDAR A1M8は、SLAMTEC社製の低価格2D LiDARセンサです。360度スキャンが可能で、最大12mの測定距離を持ちます。

| 項目 | 仕様 |
|---|---|
| 測定範囲 | 0.15m〜12m |
| スキャン周波数 | 5.5Hz |
| 角度分解能 | 1度以下 |
| インターフェース | USB（シリアル） |

### 4.2 rplidar_rosパッケージのインストール

ROS 2用のRPLIDARドライバをインストールします：

```bash
sudo apt install -y ros-jazzy-rplidar-ros
```

### 4.3 udevルールの設定

LiDARを常に`/dev/rplidar`として認識させるため、udevルールを設定します。

```bash
# udevルールファイルを作成
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
# RPLIDAR A1M8 (Silicon Labs CP210x)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
EOF

# ルールを再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4.4 LiDARの接続確認

LiDARをUSBポートに接続し、認識されているか確認します：

```bash
# デバイスの確認
ls -la /dev/rplidar
```

出力例：
```
lrwxrwxrwx 1 root root 7 Jan 12 12:00 /dev/rplidar -> ttyUSB0
```

シンボリックリンクが作成されていれば成功です。

### 4.5 ZeusCar LiDARパッケージの確認

zeuscar_lidarパッケージには、RPLIDAR A1用のlaunchファイルが含まれています：

```bash
# パッケージの確認
ros2 pkg list | grep zeuscar_lidar

# launchファイルの確認
ls ~/ros2_ws/src/zeuscar_lidar/launch/
```

### 4.6 LiDARの起動

```bash
# ワークスペースの環境を読み込み
source ~/ros2_ws/install/setup.bash

# LiDARを起動
ros2 launch zeuscar_lidar lidar.launch.py
```

正常に起動すると、以下のようなログが表示されます：

```
[rplidar_node]: RPLIDAR running on ROS2 package rplidar_ros
[rplidar_node]: SDK Version: x.x.x
[rplidar_node]: Firmware Ver: x.xx
[rplidar_node]: Hardware Rev: x
```

### 4.7 /scanトピックの確認

別のターミナルで、/scanトピックが正しくパブリッシュされているか確認します：

```bash
source ~/ros2_ws/install/setup.bash

# トピック一覧
ros2 topic list | grep scan
```

出力例：
```
/scan
```

```bash
# トピックの型を確認
ros2 topic info /scan
```

出力例：
```
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0
```

```bash
# データの確認（Ctrl+Cで停止）
ros2 topic echo /scan --once
```

### 4.8 launchファイルのパラメータ

zeuscar_lidarのlaunchファイルは以下のパラメータをサポートしています：

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| serial_port | /dev/rplidar | シリアルポート |
| frame_id | laser_frame | TFフレームID |

例：別のポートを使用する場合：

```bash
ros2 launch zeuscar_lidar lidar.launch.py serial_port:=/dev/ttyUSB0
```

---

## 5. TF（座標変換）の設定

### 5.1 TFとは

TF（Transform）は、ROS 2における座標変換システムです。ロボットの各部品（ベース、センサなど）の相対位置を管理し、異なるフレーム間の座標変換を自動的に計算します。

### 5.2 ZeusCar TFツリー構成

本プロジェクトでは以下のTFツリーを構成します：

```
base_footprint（地面投影）
  └── base_link（ロボット本体中心）
        └── laser_frame（LiDARスキャン面）
```

各フレームの説明：

| フレーム | 説明 |
|----------|------|
| base_footprint | ロボット位置を地面に投影したフレーム（z=0） |
| base_link | ロボット本体の中心点 |
| laser_frame | LiDARのスキャン平面（/scanデータの基準） |

### 5.3 実機の計測方法

TFパラメータを正確に設定するため、実機の寸法を計測します。

#### 必要な道具

| 道具 | 用途 |
|------|------|
| メジャー（巻尺） | 大まかな寸法計測 |
| ノギス or 定規 | 細かいオフセット計測（mm単位） |
| 水平な床 | 高さ計測の基準面 |

#### ROS 2座標系の確認

計測前に、ROS 2の座標系（REP-103）を確認しておきます：

```
        前方 (+X)
           ↑
           |
     +-----|-----+
     |     |     |
左(+Y)←----+----→右(-Y)
     |     |     |
     +-----|-----+
           |
           ↓
        後方 (-X)
```

- **X軸**: 前方が正（+）、後方が負（-）
- **Y軸**: 左方が正（+）、右方が負（-）
- **Z軸**: 上方が正（+）

#### ロボット本体寸法の計測

1. **全長（Length）の計測**
   - ロボットを水平な床に置く
   - ロボット前面から後面までの最大距離を計測
   - これがX軸方向の寸法

2. **全幅（Width）の計測**
   - ロボット左端から右端までの最大距離を計測
   - これがY軸方向の寸法

3. **ロボット中心の特定**
   - 全長の半分、全幅の半分の位置がロボット中心（base_link原点）
   - シャーシの形状から対角線を引いて中心を特定する方法もある

#### LiDAR位置の計測

LiDARの位置は、ロボット中心（base_link原点）からの相対位置で計測します。

1. **X位置（前後方向）の計測**
   - ロボット中心からLiDAR中心までの前後距離を計測
   - LiDARがロボット中心より**前方**にある場合: **正の値（+）**
   - LiDARがロボット中心より**後方**にある場合: **負の値（-）**

2. **Y位置（左右方向）の計測**
   - ロボット中心線からLiDAR中心までの左右距離を計測
   - LiDARが中心線より**左**にある場合: **正の値（+）**
   - LiDARが中心線より**右**にある場合: **負の値（-）**

3. **Z位置（高さ）の計測**
   - ロボットを水平な床に置く
   - **地面からLiDARセンサ部分の中央**までの高さを計測
   - 常に正の値になる

4. **Yaw角（回転角度）の確認**
   - LiDARの前方向（テーパー端/モーター側）がロボット前方を向いているか確認
   - 向いている場合: yaw = 0
   - 左に90度回転している場合: yaw = +π/2（+1.5708 rad）
   - 右に90度回転している場合: yaw = -π/2（-1.5708 rad）
   - 後ろを向いている場合: yaw = π（3.1416 rad）

#### 計測例（ZeusCar）

```
計測結果:
- ロボット全長: 163mm
- ロボット全幅: 177mm
- ロボット中心: (81.5mm, 88.5mm) ※左前角を原点とした場合

LiDAR位置（ロボット中心からの相対位置）:
- X: ロボット中心より前方に 3.5mm → +3.5mm → +0.0035m
- Y: ロボット中心より右方向に 4.5mm → -4.5mm → -0.0045m
- Z: 地面から 235mm → +235mm → +0.235m
- Yaw: 左に90度回転 → +90° → +1.5708 rad
```

#### 計測時の注意点

- **単位変換**: 計測はmm単位で行い、URDFにはm単位で記載する（1mm = 0.001m）
- **符号に注意**: 座標系の向きを常に意識する
- **複数回計測**: 精度を上げるため、同じ箇所を2-3回計測して平均を取る
- **LiDAR中心**: センサの回転中心（円盤の中心）を基準にする

### 5.4 実測値に基づくTFパラメータ

LiDAR取り付け位置の実測値：

| パラメータ | 値 | 説明 |
|------------|-----|------|
| x | +0.0035m | ロボット中心から前方へ3.5mm |
| y | -0.0045m | ロボット中心から右方向へ4.5mm |
| z | +0.235m | 地面からLiDARセンサ中央まで235mm |
| yaw | +1.5708 rad | 左方向に90度回転（+π/2） |

### 5.5 xacroパッケージのインストール

URDFファイル（.xacro形式）を処理するために、xacroパッケージをインストールします：

```bash
sudo apt install -y ros-jazzy-xacro
```

インストール確認：

```bash
source /opt/ros/jazzy/setup.bash
xacro --version
```

> **注意**: ros-jazzy-desktopにはxacroが含まれていないため、別途インストールが必要です。

### 5.6 リポジトリの更新

最新のコードを取得します：

```bash
cd ~/zeuscar-ros2-jazzy-rpi
git pull
```

### 5.7 zeuscar_descriptionパッケージのビルド

URDFとTF設定を含むパッケージをビルドします：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_description
```

成功すると以下のように表示されます：

```
Starting >>> zeuscar_description
Finished <<< zeuscar_description [x.xs]

Summary: 1 package finished [x.xs]
```

### 5.8 TFのパブリッシュ

robot_state_publisherを起動してTFをパブリッシュします：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch zeuscar_description description.launch.py
```

正常に起動すると、以下のようなログが表示されます：

```
[robot_state_publisher-1] Parsing robot urdf xml string.
[robot_state_publisher-1] Link base_footprint had 1 children
[robot_state_publisher-1] Link base_link had 1 children
[robot_state_publisher-1] Link laser_frame had 0 children
```

### 5.9 TFの確認

#### TFツリーの確認

別のターミナルで以下を実行し、TFツリーが正しく構成されているか確認します：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_tools view_frames
```

`frames.pdf`というファイルが生成され、TFツリーを可視化できます。

#### 特定フレーム間の変換確認

base_linkからlaser_frameへの変換を確認します：

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

出力例：

```
At time 0.0
- Translation: [0.004, -0.005, 0.185]
- Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
- Rotation: in RPY (radian) [0.000, 0.000, 1.571]
- Rotation: in RPY (degree) [0.000, 0.000, 90.000]
```

### 5.10 LiDARと組み合わせた動作確認

LiDARとTFを同時に起動して、正しく連携するか確認します：

**ターミナル1: TFのパブリッシュ**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch zeuscar_description description.launch.py
```

**ターミナル2: LiDARの起動**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch zeuscar_lidar lidar.launch.py
```

**ターミナル3: トピックとTFの確認**
```bash
source ~/ros2_ws/install/setup.bash

# /scanトピックのフレームIDを確認
ros2 topic echo /scan --field header.frame_id --once

# TFが正しくパブリッシュされているか確認
ros2 topic echo /tf_static --once
```

### 5.11 URDFファイルの構成

`zeuscar_description/urdf/zeuscar.urdf.xacro`の主要部分：

```xml
<!-- ロボット本体寸法 -->
<xacro:property name="body_length" value="0.163"/>  <!-- 163mm -->
<xacro:property name="body_width" value="0.177"/>   <!-- 177mm -->

<!-- LiDAR取り付け位置 -->
<xacro:property name="lidar_x" value="0.0035"/>     <!-- +3.5mm -->
<xacro:property name="lidar_y" value="-0.0045"/>    <!-- -4.5mm -->
<xacro:property name="lidar_z" value="0.235"/>      <!-- 235mm -->
<xacro:property name="lidar_yaw" value="1.5708"/>   <!-- +90° -->
```

寸法を変更する場合は、このファイルのプロパティ値を編集してリビルドしてください。

### 5.12 トラブルシューティング

#### TFが見つからない

**症状**: `tf2_echo`で`Could not transform`エラーが発生する

**解決策**:
1. robot_state_publisherが起動しているか確認
2. URDFファイルに構文エラーがないか確認

```bash
# URDFの構文チェック
check_urdf <(xacro ~/ros2_ws/src/zeuscar_description/urdf/zeuscar.urdf.xacro)
```

#### LiDARのframe_idが一致しない

**症状**: RVizでLiDARデータが表示されない

**原因**: LiDARの`frame_id`とTFの`laser_frame`が一致していない

**解決策**:
```bash
# LiDARのframe_idを確認
ros2 topic echo /scan --field header.frame_id --once

# 必要に応じてlaunchファイルのframe_idパラメータを修正
```

---

## 6. SLAMの設定

### 6.1 SLAMとは

SLAM（Simultaneous Localization and Mapping）は、ロボットが移動しながら同時に地図を作成し、その地図上で自己位置を推定する技術です。本プロジェクトでは`slam_toolbox`を使用します。

### 6.2 前提条件

SLAMを動作させるには以下が必要です：

| 要件 | 説明 | ステータス |
|------|------|-----------|
| LiDAR | /scanトピックがパブリッシュされている | ✅ 完了 |
| TFツリー | base_footprint → base_link → laser_frame | ✅ 完了 |
| オドメトリ | /odomトピックまたはTF（odom → base_footprint） | ⚠️ 未実装 |

> **注意**: オドメトリソースがないと、slam_toolboxはスキャンマッチングのみで動作しますが、精度が低下します。IMU（ICM42688）とホイールオドメトリの統合後に本格的なテストを行ってください。

### 6.3 slam_toolboxのインストール

```bash
sudo apt install -y ros-jazzy-slam-toolbox
```

インストール確認：

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep slam
```

出力例：
```
slam_toolbox
```

### 6.4 zeuscar_slamパッケージのビルド

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_slam
```

### 6.5 SLAM設定ファイル

`zeuscar_slam/config/slam_params.yaml` に主要なパラメータが定義されています：

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| base_frame | base_footprint | ロボットのベースフレーム |
| scan_topic | /scan | LiDARトピック |
| resolution | 0.05 | マップ解像度（m/pixel） |
| max_laser_range | 12.0 | LiDAR最大距離（m） |
| mode | mapping | マッピングモード |

### 6.6 SLAMの起動（オドメトリ統合後）

```bash
source ~/ros2_ws/install/setup.bash

# ターミナル1: TFをパブリッシュ
ros2 launch zeuscar_description description.launch.py

# ターミナル2: LiDARを起動
ros2 launch zeuscar_lidar lidar.launch.py

# ターミナル3: SLAMを起動
ros2 launch zeuscar_slam slam.launch.py
```

### 6.7 マップの保存

マッピング完了後、マップを保存します：

```bash
# nav2_map_serverをインストール（未インストールの場合）
sudo apt install -y ros-jazzy-nav2-map-server

# マップを保存
ros2 run nav2_map_server map_saver_cli -f ~/maps/zeuscar_map
```

これにより、以下のファイルが生成されます：
- `zeuscar_map.pgm` - マップ画像
- `zeuscar_map.yaml` - マップメタデータ

---

## 7. RVizでの可視化

### 7.1 RViz2とは

RViz2は、ROS 2の3D可視化ツールです。ロボットモデル、センサデータ、TFツリー、マップなどを視覚的に確認できます。

### 7.2 前提条件

- ディスプレイが接続されている、またはリモートデスクトップ（VNC等）が設定されている
- ros-jazzy-desktopがインストールされている（RViz2含む）

### 7.3 zeuscar_bringupパッケージのビルド

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_bringup
```

### 7.4 RViz2の起動

```bash
source ~/ros2_ws/install/setup.bash

# ZeusCar用の設定ファイルでRViz2を起動
rviz2 -d ~/ros2_ws/install/zeuscar_bringup/share/zeuscar_bringup/rviz/zeuscar.rviz
```

### 7.5 表示項目

ZeusCar用のRViz設定には以下の表示が含まれています：

| 表示項目 | 説明 | トピック/フレーム |
|----------|------|------------------|
| Grid | グリッド線 | - |
| RobotModel | ロボットモデル | /robot_description |
| TF | 座標フレーム | TFツリー |
| LaserScan | LiDARデータ | /scan |
| Map | SLAMマップ | /map |

### 7.6 リモートでの可視化

Raspberry Piにディスプレイがない場合、以下の方法で可視化できます：

**方法1: 別PCでRViz2を起動**

```bash
# 別PCで（同一ネットワーク内）
export ROS_DOMAIN_ID=0  # Raspberry Piと同じドメインID
source /opt/ros/jazzy/setup.bash
rviz2
```

**方法2: VNCでリモートデスクトップ接続**

```bash
# Raspberry Piにx11vncをインストール
sudo apt install -y x11vnc

# VNCサーバーを起動
x11vnc -display :0 -auth guess -forever -loop -noxdamage -rfbauth ~/.vnc/passwd -rfbport 5900
```

### 7.7 トラブルシューティング

RViz2が起動しない場合：

```bash
# OpenGL関連のエラーの場合
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```

---

## 8. Arduino駆動系のセットアップ

### 8.1 概要

ZeusCarは、Arduino Uno R3でモーターを制御しています。Raspberry Piからシリアル通信でArduinoにコマンドを送信し、4輪のメカナムホイールを操作します。

#### システム構成

```
┌─────────────────┐      ROS 2 DDS      ┌─────────────────┐     Serial      ┌─────────────────┐
│    Host PC      │ ◄────────────────► │  Raspberry Pi   │ ◄─────────────► │   Arduino Uno   │
│ (Ubuntu 24.04)  │    /cmd_vel         │ (Ubuntu 24.04)  │  9600bps        │     R3          │
│  ROS 2 Jazzy    │    Topic            │  ROS 2 Jazzy    │  ASCII + '\n'   │  Motor Control  │
└─────────────────┘                     └─────────────────┘                 └─────────────────┘
```

#### メカナムホイールの動作

| コマンド | 動作 | 説明 |
|----------|------|------|
| FORWARD | 前進 | 4輪全て同じ方向に回転 |
| BACKWARD | 後退 | 前進の逆方向 |
| LEFT | 左横移動 | メカナムホイール特有の動き |
| RIGHT | 右横移動 | メカナムホイール特有の動き |
| LEFTFORWARD | 左斜め前 | 対角線方向の移動 |
| RIGHTFORWARD | 右斜め前 | 対角線方向の移動 |
| LEFTBACKWARD | 左斜め後 | 対角線方向の移動 |
| RIGHTBACKWARD | 右斜め後 | 対角線方向の移動 |
| TURNLEFT | 左旋回 | その場で左回転 |
| TURNRIGHT | 右旋回 | その場で右回転 |
| STOP | 停止 | 全モーター停止 |

### 8.2 前提条件

- Arduinoにファームウェア（raspi-ctrl-v_2_00.ino）が書き込まれている
- ArduinoとRaspberry PiがUSBケーブルで接続されている

### 8.3 pyserialのインストール確認

シリアル通信にはpyserialライブラリを使用します。通常はシステムにプリインストールされています。

```bash
python3 -c "import serial; print(f'pyserial version: {serial.__version__}')"
```

インストールされていない場合：

```bash
sudo apt install -y python3-serial
```

### 8.4 udevルールの設定

ArduinoをRaspberry Piに接続した際、常に `/dev/arduino` としてアクセスできるようにudevルールを設定します。

```bash
# udevルールファイルをコピー
sudo cp ~/ros2_ws/src/zeuscar_motor/udev/99-arduino.rules /etc/udev/rules.d/

# ルールを再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### Arduinoの接続確認

```bash
# デバイスの確認
ls -la /dev/arduino
```

出力例：
```
lrwxrwxrwx 1 root root 7 Jan 24 12:00 /dev/arduino -> ttyACM0
```

シンボリックリンクが作成されていれば成功です。

**リンクが作成されない場合**：
ArduinoのUSBケーブルを一度抜いて再接続してみてください。

### 8.5 zeuscar_motorパッケージのビルド

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_motor
```

成功すると以下のように表示されます：

```
Starting >>> zeuscar_motor
Finished <<< zeuscar_motor [x.xs]

Summary: 1 package finished [x.xs]
```

### 8.6 環境設定の更新

新しいパッケージを認識させるため、環境を再読み込みします：

```bash
source ~/ros2_ws/install/setup.bash
```

パッケージの確認：

```bash
ros2 pkg list | grep zeuscar_motor
```

### 8.7 モーターコントローラの起動

```bash
source ~/ros2_ws/install/setup.bash

# デフォルト設定で起動
ros2 launch zeuscar_motor motor.launch.py
```

正常に起動すると、以下のようなログが表示されます：

```
[motor_controller_node]: MotorControllerNode started. Serial: /dev/ttyACM0 @ 9600bps
[motor_controller_node]: Serial connection established: /dev/ttyACM0
```

#### シリアルポートを指定して起動

```bash
# 別のポートを使用する場合
ros2 launch zeuscar_motor motor.launch.py serial_port:=/dev/arduino

# ボーレートも指定する場合
ros2 launch zeuscar_motor motor.launch.py serial_port:=/dev/arduino baud_rate:=9600
```

### 8.8 動作確認

#### 方法1: 直接コマンドトピックを使用

別のターミナルで以下を実行して、直接コマンドを送信します：

```bash
source ~/ros2_ws/install/setup.bash

# 前進コマンドを送信
ros2 topic pub --once /zeuscar/motor_cmd std_msgs/msg/String "{data: 'FORWARD'}"

# 停止コマンドを送信
ros2 topic pub --once /zeuscar/motor_cmd std_msgs/msg/String "{data: 'STOP'}"
```

#### 方法2: cmd_velトピックを使用

ROS 2標準の速度指令トピックを使用することもできます：

```bash
source ~/ros2_ws/install/setup.bash

# 前進（linear.x = 0.5）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左旋回（angular.z = 0.5）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 方法3: キーボードで操作（teleop_twist_keyboard）

キーボードでロボットを操作できるパッケージを使用します：

```bash
# teleop_twist_keyboardのインストール（未インストールの場合）
sudo apt install -y ros-jazzy-teleop-twist-keyboard

# 起動
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

キー操作：
- `i`: 前進
- `,`: 後退
- `j`: 左旋回
- `l`: 右旋回
- `k`: 停止

### 8.9 トピックとパラメータ

#### サブスクライブトピック

| トピック名 | 型 | 説明 |
|-----------|-----|------|
| /cmd_vel | geometry_msgs/msg/Twist | ROS 2標準の速度指令 |
| /zeuscar/motor_cmd | std_msgs/msg/String | 直接コマンド（FORWARD等） |

#### パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| serial_port | /dev/ttyACM0 | Arduinoシリアルポート |
| baud_rate | 9600 | ボーレート |
| cmd_vel_timeout | 0.5 | タイムアウト秒（コマンド未受信時に自動停止） |
| linear_threshold | 0.1 | 速度閾値（これ以下は0として扱う） |
| angular_threshold | 0.1 | 角速度閾値（これ以下は0として扱う） |

### 8.10 cmd_velからコマンドへの変換

`/cmd_vel`トピック（geometry_msgs/msg/Twist）は以下のルールでArduinoコマンドに変換されます：

| linear.x | linear.y | angular.z | コマンド |
|----------|----------|-----------|----------|
| > 0 | 0 | 0 | FORWARD |
| < 0 | 0 | 0 | BACKWARD |
| 0 | > 0 | 0 | LEFT |
| 0 | < 0 | 0 | RIGHT |
| > 0 | > 0 | 0 | LEFTFORWARD |
| > 0 | < 0 | 0 | RIGHTFORWARD |
| < 0 | > 0 | 0 | LEFTBACKWARD |
| < 0 | < 0 | 0 | RIGHTBACKWARD |
| - | - | > 0 | TURNLEFT |
| - | - | < 0 | TURNRIGHT |
| 0 | 0 | 0 | STOP |

> **注意**: angular.z（旋回）が0でない場合、旋回コマンドが優先されます。

### 8.11 安全機能

#### タイムアウト自動停止

`cmd_vel_timeout`秒間コマンドを受信しない場合、自動的にSTOPコマンドが送信されます。これにより、通信断時にロボットが暴走することを防ぎます。

デフォルト: 0.5秒

#### シリアル接続断時の挙動

シリアル接続が切断された場合：
- 警告ログが出力されます
- 再接続は自動では行われません（ノードの再起動が必要）

---

## 9. IMU（慣性計測ユニット）のセットアップ

### 9.1 概要

ZeusCarには6軸IMU（ICM-42688）が搭載されています。IMUは加速度とジャイロスコープのデータを提供し、オドメトリの精度向上やSLAMの改善に役立ちます。

詳細なハードウェア仕様については、[ICM-42688 IMUセンサー仕様書](hardware/icm42688_imu_sensor.md)を参照してください。

### 9.2 ハードウェア仕様

| 項目 | 仕様 |
|------|------|
| センサーチップ | ICM-42688（TDK/InvenSense製） |
| センサータイプ | 6軸IMU（3軸ジャイロ + 3軸加速度計） |
| インターフェース | I2C（アドレス: 0x68） |
| 動作電圧 | 3.3V |

### 9.3 I2Cの有効化

Raspberry PiでI2Cを有効にする必要があります。

```bash
# raspi-configでI2Cを有効化
sudo raspi-config
```

メニューから以下を選択：
1. `Interface Options`
2. `I2C`
3. `Yes`（有効化）
4. `Finish`

変更を反映するため、再起動が必要な場合があります：

```bash
sudo reboot
```

### 9.4 I2Cツールのインストール

```bash
sudo apt install -y i2c-tools
```

### 9.5 IMUの接続確認

IMUがI2Cバスに正しく接続されているか確認します：

```bash
sudo i2cdetect -y 1
```

期待される出力（0x68にデバイスが表示される）：

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

`68`が表示されていればIMUが正しく認識されています。

### 9.6 smbus2ライブラリのインストール

PythonからI2Cデバイスにアクセスするためのライブラリをインストールします。

#### pip3のインストール

Ubuntu 24.04ではpipがデフォルトでインストールされていないため、まずpip3をインストールします：

```bash
sudo apt install -y python3-pip
```

#### smbus2のインストール

Ubuntu 24.04ではPEP 668により、システムワイドのpipインストールが制限されています。`--break-system-packages`オプションを使用します：

```bash
pip3 install --break-system-packages smbus2
```

> **注意**: `--break-system-packages`オプションを使用すると、システムのPython環境に直接インストールされます。ROS 2環境ではこの方法が一般的に使用されます。

インストール確認：

```bash
python3 -c "import smbus2; print('smbus2 OK')"
```

### 9.7 zeuscar_imuパッケージのビルド

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zeuscar_imu
```

成功すると以下のように表示されます：

```
Starting >>> zeuscar_imu
Finished <<< zeuscar_imu [x.xs]

Summary: 1 package finished [x.xs]
```

### 9.8 環境設定の更新

```bash
source ~/ros2_ws/install/setup.bash
```

パッケージの確認：

```bash
ros2 pkg list | grep zeuscar_imu
```

### 9.9 IMUテストの実行

IMUが正常に動作するか確認するため、テストノードを実行します。

> **警告**: このテストではロボットが動きます。周囲の安全を確認してください。

**ターミナル1: モーターコントローラを起動**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_motor motor_controller_node
```

**ターミナル2: IMUテストを実行**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run zeuscar_imu imu_test_node
```

または、launchファイルを使用：

```bash
ros2 launch zeuscar_imu imu_test.launch.py
```

### 9.10 テストシーケンス

テストノードは以下のシーケンスを自動実行します：

| 順序 | 動作 | 時間 | 確認項目 |
|------|------|------|----------|
| 1 | 静止 | 2.0秒 | 基準値取得（accel_z ≈ +1.0g） |
| 2 | 前進 | 0.3秒 | accel_x の変化 |
| 3 | 後退 | 0.3秒 | accel_x の変化（逆方向） |
| 4 | 左旋回 | 0.3秒 | gyro_z > 0 |
| 5 | 右旋回 | 0.3秒 | gyro_z < 0 |

### 9.11 出力例

```
[INFO] [imu_test_node]: === IMUセンサー動作確認テスト ===
[INFO] [imu_test_node]: I2Cバス: 1, アドレス: 0x68
[INFO] [imu_test_node]: WHO_AM_I: 0x47 (期待値: 0x47)
[INFO] [imu_test_node]: ICM-42688を検出しました
[INFO] [imu_test_node]: IMUを初期化しました

[INFO] [imu_test_node]: === テスト開始 ===
[INFO] [imu_test_node]: ※ロボットが動きます。周囲に注意してください。

[INFO] [imu_test_node]: --- 静止状態（基準値取得） (STOP, 2.0秒) ---
[INFO] [imu_test_node]: 静止状態（基準値取得） (サンプル数: 20):
  加速度平均 [g]: X=+0.012, Y=-0.008, Z=+0.998
  角速度平均 [dps]: X=+0.15, Y=-0.22, Z=+0.08
...
[INFO] [imu_test_node]: === テスト完了 ===
```

### 9.12 判定基準

| 項目 | 正常値 | 異常時の確認事項 |
|------|--------|------------------|
| WHO_AM_I | 0x47 | 配線確認、I2Cアドレス確認 |
| 静止時 accel_z | +0.95〜+1.05 g | センサー取り付け向き確認 |
| 静止時 gyro | ±5 dps 以内 | センサー初期化確認 |
| 前進時 accel_x | 正の変化 | モーター動作確認 |
| 左旋回時 gyro_z | 正の値 | センサー軸方向確認 |
| 右旋回時 gyro_z | 負の値 | センサー軸方向確認 |

### 9.13 パラメータ

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `i2c_bus` | 1 | I2Cバス番号 |
| `i2c_address` | 0x68 | I2Cアドレス |
| `sample_rate_hz` | 10.0 | サンプリングレート (Hz) |

---

## 10. トラブルシューティング

### 10.1 ROS 2リポジトリの追加に失敗する

**症状**: `sudo apt update` でROS 2のパッケージが取得できない

**原因**: リポジトリ設定ファイルが正しく作成されていない

**解決策**:
```bash
# ファイルの内容を確認
cat /etc/apt/sources.list.d/ros2.list

# 空の場合は再作成
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### 10.2 colconコマンドが見つからない

**症状**: `colcon build`を実行すると`colcon: command not found`エラーが表示される

**原因**: `ros-jazzy-desktop`にはcolconが含まれていない

**解決策**:
```bash
sudo apt install -y python3-colcon-common-extensions
```

### 10.3 ros2 --versionが動作しない

**症状**: `ros2 --version`を実行してもバージョンが表示されない

**原因**: ROS 2 CLIには`--version`オプションが存在しない

**解決策**:
インストール確認は以下のコマンドで行います：
```bash
# トピック一覧を表示（空でもエラーが出なければOK）
ros2 topic list

# またはデモノードで確認
ros2 run demo_nodes_cpp talker
```

### 10.4 xacroコマンドが見つからない

**症状**: URDFファイルの構文チェック時に`xacro: command not found`エラーが発生する

**原因**: ros-jazzy-desktopにはxacroパッケージが含まれていない

**解決策**:
```bash
sudo apt install -y ros-jazzy-xacro
```

**確認方法**:
```bash
source /opt/ros/jazzy/setup.bash
xacro --version
```

### 10.5 robot_descriptionパラメータのYAMLパースエラー

**症状**: launchファイル実行時に以下のエラーが発生する
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Unable to parse the value of parameter robot_description as yaml. If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```

**原因**: ROS 2 Jazzy（Iron以降）では、`Command`サブスティチューションで生成した文字列をそのままパラメータに渡すと、YAMLとしてパースしようとしてエラーになる

**解決策**:
`ParameterValue`でラップし、`value_type=str`を指定する

**修正前**:
```python
from launch.substitutions import Command

robot_state_publisher = Node(
    # ...
    parameters=[{
        'robot_description': Command(['xacro ', urdf_file]),
    }],
)
```

**修正後**:
```python
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

robot_description = ParameterValue(
    Command(['xacro ', urdf_file]),
    value_type=str
)

robot_state_publisher = Node(
    # ...
    parameters=[{
        'robot_description': robot_description,
    }],
)
```

**備考**:
- ROS 2 Humble以前では不要だった対応
- ROS 2 Jazzy（Iron以降）で必要になった変更
- 公式ドキュメントでも推奨されている方法

### 10.6 /dev/rplidarシンボリックリンクが作成されない

**症状**: LiDARを接続しても`/dev/rplidar`が作成されない

```bash
ls -la /dev/rplidar
# ls: cannot access '/dev/rplidar': No such file or directory
```

**原因1**: udevルールファイルが作成されていない

**解決策1**:
```bash
# udevルールファイルを作成
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
# RPLIDAR A1M8 (Silicon Labs CP210x)
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
EOF

# ルールを再読み込み
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**原因2**: udevトリガー後も作成されない場合

**解決策2**:
特定のデバイスに対して直接トリガーを実行する:
```bash
sudo udevadm trigger --action=add /dev/ttyUSB0
```

または、LiDARのUSBケーブルを一度抜いて再接続する。

### 10.7 LiDARデバイスの権限エラー

**症状**: LiDAR起動時に権限エラーが発生する

```
[ERROR] [rplidar_node]: Error, cannot bind to the specified serial port /dev/rplidar
```

**原因**: ユーザーがデバイスファイルへのアクセス権限を持っていない

**解決策1**: udevルールでMODE="0666"を設定（推奨）
```bash
# /etc/udev/rules.d/99-rplidar.rules にMODE="0666"が含まれていることを確認
cat /etc/udev/rules.d/99-rplidar.rules
```

**解決策2**: ユーザーをdialoutグループに追加
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

### 10.8 slam_toolboxが起動しない

**症状**: SLAMを起動すると「TF変換が見つからない」エラーが発生する

```
[slam_toolbox]: Could not get transform from base_footprint to odom
```

**原因**: オドメトリソース（/odomトピックまたはodom→base_footprintのTF）がない

**解決策**:
- IMU（ICM42688等）とホイールオドメトリを統合する
- または、robot_localizationパッケージでオドメトリを生成する

**一時的な回避策**:
slam_toolboxはスキャンマッチングのみでも動作しますが、精度が低下します。

### 10.9 RViz2が起動しない（OpenGLエラー）

**症状**: RViz2起動時にOpenGL関連のエラーが発生する

```
libGL error: failed to load driver: ...
```

**原因**: Raspberry PiのGPUドライバとRViz2の互換性問題

**解決策**:
ソフトウェアレンダリングを使用する：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```

永続化する場合は`~/.bashrc`に追加：

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
```

### 10.10 RViz2でLaserScanが表示されない

**症状**: RViz2を起動してもLiDARデータが表示されない

**原因1**: Fixed Frameが正しく設定されていない

**解決策1**:
RViz2の「Global Options」→「Fixed Frame」を`map`または`base_footprint`に設定

**原因2**: QoS設定の不一致

**解決策2**:
LaserScan表示の「Topic」→「Reliability Policy」を「Best Effort」に変更

### 10.11 Arduinoシリアルポートが見つからない

**症状**: motor_controller_node起動時にシリアルポートエラーが発生する

```
[motor_controller_node]: Failed to open serial port: [Errno 2] No such file or directory: '/dev/ttyACM0'
```

**原因1**: Arduinoが接続されていない

**解決策1**:
ArduinoがUSBケーブルでRaspberry Piに接続されていることを確認

**原因2**: シリアルポートのパスが異なる

**解決策2**:
```bash
# 接続されているシリアルデバイスを確認
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# 正しいポートを指定して起動
ros2 launch zeuscar_motor motor.launch.py serial_port:=/dev/ttyACM0
```

**原因3**: udevルールが設定されていない

**解決策3**:
```bash
# udevルールをインストール
sudo cp ~/ros2_ws/src/zeuscar_motor/udev/99-arduino.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Arduinoを再接続後、シンボリックリンクを確認
ls -la /dev/arduino
```

### 10.12 Arduinoシリアルポートの権限エラー

**症状**: シリアルポートへのアクセスが拒否される

```
[motor_controller_node]: Failed to open serial port: [Errno 13] Permission denied: '/dev/ttyACM0'
```

**原因**: ユーザーがシリアルポートへのアクセス権限を持っていない

**解決策1**: udevルールでMODE="0666"を設定（推奨）
```bash
# 99-arduino.rulesにMODE="0666"が含まれていることを確認
cat /etc/udev/rules.d/99-arduino.rules
```

**解決策2**: ユーザーをdialoutグループに追加
```bash
sudo usermod -aG dialout $USER
# ログアウト・ログインが必要
```

### 10.13 Arduinoがコマンドに反応しない

**症状**: コマンドを送信してもモーターが動かない

**確認手順**:

1. **シリアル接続の確認**
```bash
# motor_controller_nodeのログを確認
# "Serial connection established" が表示されているか確認
```

2. **コマンド送信の確認**
```bash
# トピックにメッセージが届いているか確認
ros2 topic echo /zeuscar/motor_cmd
# 別ターミナルでコマンド送信
ros2 topic pub --once /zeuscar/motor_cmd std_msgs/msg/String "{data: 'FORWARD'}"
```

3. **Arduinoのシリアルモニタで確認**
   - Arduino IDEのシリアルモニタを9600bpsで開く
   - 手動で「FORWARD」と入力し、Arduinoが反応するか確認

**原因1**: Arduinoにファームウェアが書き込まれていない

**解決策1**:
Arduino IDEで`raspi-ctrl-v_2_00.ino`を開いて書き込み

**原因2**: ボーレートが一致していない

**解決策2**:
```bash
# Arduinoのファームウェアは9600bpsで設定されている
ros2 launch zeuscar_motor motor.launch.py baud_rate:=9600
```

**原因3**: 電源不足

**解決策3**:
- Arduinoに外部電源を供給
- USBハブを使用している場合は、セルフパワーハブを使用

### 10.14 IMU（ICM-42688）が検出されない

**症状**: `i2cdetect -y 1`で0x68にデバイスが表示されない

**原因1**: I2Cが有効化されていない

**解決策1**:
```bash
sudo raspi-config
# Interface Options → I2C → Yes を選択
sudo reboot
```

**原因2**: 配線が正しくない

**解決策2**:
以下の配線を確認してください：

| IMUピン | Raspberry Pi |
|---------|--------------|
| VCC | 3.3V (Pin 1) |
| GND | GND (Pin 9) |
| SDA | GPIO2/SDA1 (Pin 3) |
| SCL | GPIO3/SCL1 (Pin 5) |
| SAO | GND (Pin 9) |
| CS | 3.3V (Pin 1) |

**原因3**: I2Cアドレスが異なる

**解決策3**:
SAOピンの接続先を確認してください：
- SAO → GND: アドレス 0x68
- SAO → 3.3V: アドレス 0x69

### 10.15 smbus2ライブラリがインストールできない

**症状1**: `pip`または`pip3`コマンドが見つからない

```
pip: コマンドが見つかりません
```

**原因**: Ubuntu 24.04ではpipがデフォルトでインストールされていない

**解決策**:
```bash
sudo apt install -y python3-pip
```

---

**症状2**: `externally-managed-environment`エラーが発生する

```
error: externally-managed-environment

× This environment is externally managed
```

**原因**: Ubuntu 24.04ではPEP 668により、システムPython環境への直接インストールが制限されている

**解決策（推奨）**:
```bash
# --break-system-packagesオプションを使用
pip3 install --break-system-packages smbus2
```

**代替解決策（仮想環境を使用）**:
```bash
# 仮想環境を作成
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install smbus2

# 注意: 仮想環境を使う場合、ROS 2のノード実行前に
# 毎回 source ~/ros2_venv/bin/activate が必要
```

---

**症状3**: 権限エラーが発生する

**解決策**:
```bash
# ユーザーインストールを試す
pip3 install --break-system-packages --user smbus2
```

### 10.16 IMU WHO_AM_I値が0x47でない

**症状**: WHO_AM_Iが0xFFや0x00など予期しない値を返す

**原因1**: 通信エラー

**解決策1**:
```bash
# I2C通信速度を下げる（/boot/firmware/config.txtに追加）
dtparam=i2c_baudrate=50000
# 再起動後に確認
sudo reboot
```

**原因2**: 電源供給の問題

**解決策2**:
- VCCとGNDの配線を確認
- 3.3V電源が正しく供給されているか確認

### 10.17 IMUテストノードでモーターが動かない

**症状**: IMUテスト中にロボットが動かない

**原因**: モーターコントローラノードが起動していない

**解決策**:
```bash
# 別ターミナルでモーターコントローラを起動
ros2 run zeuscar_motor motor_controller_node

# その後、IMUテストを実行
ros2 run zeuscar_imu imu_test_node
```

**確認方法**:
```bash
# トピックが存在するか確認
ros2 topic list | grep motor_cmd
```

### 10.18 IMU加速度Z軸の値が期待値の約1/8になる

**症状**: 静止状態でaccel_zが約0.12g（期待値: +1.0g）

**原因**: センサー初期化時のフルスケール設定とスケールファクターの不一致

現在のコード（`imu_test_node.py`）:
```python
# レジスタ設定: ±16gフルスケール
self._bus.write_byte_data(self.i2c_address, self.REG_ACCEL_CONFIG0, 0x06)

# スケールファクター: ±2g用（誤り）
ACCEL_SCALE = 16384.0
```

**問題の詳細**:
- `ACCEL_CONFIG0 = 0x06` → Bits 7:5 = 000 → ±16g設定
- ±16gの正しいスケールファクター: 2048 LSB/g
- ±2gのスケールファクター: 16384 LSB/g（現在の設定）
- 比率: 16384 / 2048 = 8（値が1/8になる原因）

**解決策1（推奨）**: スケールファクターを修正
```python
# ±16g設定に合わせて変更
ACCEL_SCALE = 2048.0  # ±16g設定時: 2048 LSB/g
```

**解決策2**: レジスタ設定を±2gに変更
```python
# ACCEL_CONFIG0を±2g, 1kHzに設定
# 0x66 = 0b01100110 (FS_SEL=011 → ±2g, ODR=0110 → 1kHz)
self._bus.write_byte_data(self.i2c_address, self.REG_ACCEL_CONFIG0, 0x66)
```

**ICM-42688 ACCEL_FS_SEL設定値**:

| FS_SEL (Bits 7:5) | フルスケール | スケールファクター |
|-------------------|--------------|-------------------|
| 000 | ±16g | 2048 LSB/g |
| 001 | ±8g | 4096 LSB/g |
| 010 | ±4g | 8192 LSB/g |
| 011 | ±2g | 16384 LSB/g |

**関連テストレポート**: [IMU_test_report_20260203_211545.md](troubleshooting/IMU_test_report_20260203_211545.md)

### 10.19 `python`コマンドが見つからない

**症状**: `python`コマンドを実行すると「コマンドが見つかりません」エラーが表示される

```
python: コマンドが見つかりません
```

**原因**: Ubuntu 24.04ではデフォルトで`python`コマンドが存在しない。`python3`のみが提供されている。

**解決策1（推奨）**: `python3`コマンドを使用する
```bash
# pythonの代わりにpython3を使用
python3 -m pytest tests/ -v
python3 script.py
```

**解決策2**: `python-is-python3`パッケージをインストール
```bash
sudo apt install -y python-is-python3
```

これにより`python`コマンドが`python3`へのシンボリックリンクとして作成される。

**確認方法**:
```bash
which python3
# /usr/bin/python3

python3 --version
# Python 3.12.x
```

---

## 更新履歴

| 日付 | 内容 |
|---|---|
| 2026-01-12 | 初版作成（目次構成） |
| 2026-01-12 | Ubuntu 24.04インストール手順を追加 |
| 2026-01-12 | ROS 2 Jazzyインストール手順を追加 |
| 2026-01-12 | ワークスペース作成手順を追加 |
| 2026-01-12 | トラブルシューティング追加（colcon、ros2 --version） |
| 2026-01-12 | LiDARセットアップ手順を追加 |
| 2026-01-19 | TF（座標変換）の設定手順を追加 |
| 2026-01-19 | 実機の計測方法（Section 5.3）を追加 |
| 2026-01-19 | xacroパッケージインストール手順を追加（Section 5.5） |
| 2026-01-19 | トラブルシューティング追加（8.4 xacro、8.5 ParameterValue） |
| 2026-01-19 | トラブルシューティング追加（8.6 udevルール、8.7 LiDAR権限） |
| 2026-01-19 | SLAM設定手順を追加（Section 6） |
| 2026-01-19 | RViz可視化手順を追加（Section 7） |
| 2026-01-19 | トラブルシューティング追加（8.8 slam_toolbox、8.9-8.10 RViz） |
| 2026-01-24 | Arduino駆動系セットアップ手順を追加（Section 8） |
| 2026-01-24 | トラブルシューティング追加（10.11-10.13 Arduino関連） |
| 2026-02-03 | IMU（ICM-42688）セットアップ手順を追加（Section 9） |
| 2026-02-03 | トラブルシューティング追加（10.14-10.17 IMU関連） |
| 2026-02-03 | Section 9.6 pip3インストール手順とPEP 668対応を追記 |
| 2026-02-03 | 10.15 PEP 668エラー・pipコマンド未インストール対応を追記 |
| 2026-02-03 | 10.18 IMU加速度スケールファクター不一致の問題と解決策を追記 |
| 2026-02-06 | 10.19 pythonコマンド未インストール問題を追記 |
