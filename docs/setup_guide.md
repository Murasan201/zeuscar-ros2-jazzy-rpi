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
8. [トラブルシューティング](#8-トラブルシューティング)

---

## 1. Ubuntu 24.04のインストール

### 1.1 概要

Raspberry Pi 4にUbuntu 24.04 LTS（Noble Numbat）をインストールします。ROS 2 Jazzyは Ubuntu 24.04を公式にサポートしています。

### 1.2 Raspberry Pi Imagerでのインストール

1. **Raspberry Pi Imagerをダウンロード**
   - 公式サイト（https://www.raspberrypi.com/software/）からダウンロード

2. **OSを選択**
   - 「Other general-purpose OS」→「Ubuntu」→「Ubuntu Desktop 24.04 LTS (64-bit)」を選択

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

（作業実施後に手順を記載）

---

## 7. RVizでの可視化

（作業実施後に手順を記載）

---

## 8. トラブルシューティング

### 8.1 ROS 2リポジトリの追加に失敗する

**症状**: `sudo apt update` でROS 2のパッケージが取得できない

**原因**: リポジトリ設定ファイルが正しく作成されていない

**解決策**:
```bash
# ファイルの内容を確認
cat /etc/apt/sources.list.d/ros2.list

# 空の場合は再作成
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### 8.2 colconコマンドが見つからない

**症状**: `colcon build`を実行すると`colcon: command not found`エラーが表示される

**原因**: `ros-jazzy-desktop`にはcolconが含まれていない

**解決策**:
```bash
sudo apt install -y python3-colcon-common-extensions
```

### 8.3 ros2 --versionが動作しない

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

### 8.4 xacroコマンドが見つからない

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

### 8.5 robot_descriptionパラメータのYAMLパースエラー

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

### 8.6 /dev/rplidarシンボリックリンクが作成されない

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

### 8.7 LiDARデバイスの権限エラー

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
