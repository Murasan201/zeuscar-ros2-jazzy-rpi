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

```bash
ros2 --version
```

出力例：
```
ros2 0.11.x
```

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

（作業実施後に手順を記載）

---

## 5. TF（座標変換）の設定

（作業実施後に手順を記載）

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

---

## 更新履歴

| 日付 | 内容 |
|---|---|
| 2026-01-12 | 初版作成（目次構成） |
| 2026-01-12 | Ubuntu 24.04インストール手順を追加 |
| 2026-01-12 | ROS 2 Jazzyインストール手順を追加 |
| 2026-01-12 | ワークスペース作成手順を追加 |
