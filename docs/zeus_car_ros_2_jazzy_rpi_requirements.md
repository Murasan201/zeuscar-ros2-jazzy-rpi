# ZeusCar ROS 2 Jazzy Raspberry Pi システム 要件定義書

## 1. 目的（Purpose）
本プロジェクトは、ZeusCar（メカナムホイールロボット）に対し、Raspberry Pi 4 + Ubuntu 24.04 Desktop + ROS 2 Jazzy Jalisco を用いたロボット制御・センサ統合・自己位置推定（SLAM）システムを構築することを目的とする。

本リポジトリは、Raspberry Pi 側の ROS 2 システムに特化し、RViz を用いた可視化および LiDAR による自己位置推定を実現するための基盤を提供する。

---

## 2. プロジェクトの位置づけ
- Raspberry Pi 側の ROS 2 Jazzy 環境に特化したサブシステム
- Arduino 側ファームウェアおよび PC 側制御ノードは既存構成を維持
- 過去の ZeusCar プロジェクトは参照専用（fork しない）

参照用リポジトリ:
- https://github.com/Murasan201/zeuscar-project

---

## 3. 対象リポジトリ
### 新規作成リポジトリ
- Repository Name: **zeuscar-ros2-jazzy-rpi**

### 参照専用リポジトリ（旧構成）
- Ubuntu 22.04 + ROS 2 Humble
- 設計思想・通信構成・Arduino/PC 側の実装参照用

---

## 4. システム全体構成

### 4.1 ハードウェア構成
| 区分 | 機器 |
|---|---|
| メイン制御 | Raspberry Pi 4 |
| モータ制御 | Arduino |
| ロボット | ZeusCar（メカナムホイール） |
| センサ | SLAMTEC RPLIDAR A1M8 |
| 操作PC | Ubuntu PC（既存環境） |

### 4.2 ソフトウェア構成
| 項目 | 採用 |
|---|---|
| OS | Ubuntu 24.04 LTS Desktop |
| ROS 2 | Jazzy Jalisco (LTS) |
| 可視化 | RViz2 |
| SLAM | slam_toolbox |
| LiDAR Driver | sllidar_ros2 |
| ビルド | colcon |

---

## 5. 本リポジトリのスコープ

### 5.1 In Scope
- Raspberry Pi 上の ROS 2 Jazzy 環境構築
- LiDAR（A1M8）の ROS 2 統合
- TF 設計（base_link / base_footprint / laser）
- RViz による可視化
- SLAM による自己位置推定
- ZeusCar 向け bringup / launch 構成
- 再現性のあるセットアップドキュメント

### 5.2 Out of Scope
- Arduino ファームウェアの設計変更
- PC 側制御ノードの刷新
- Navigation2 の本格運用

---

## 6. リポジトリ構成（設計方針）

本リポジトリ構成は初期設計案であり、**開発の進行、検証結果、運用上の知見に応じて適宜変更・再編されることを前提**とする。

### 6.1 リポジトリ構成（初期案）
```
zeuscar-ros2-jazzy-rpi/
├─ README.md
├─ docs/
│  ├─ 01_setup_ubuntu24_04_desktop.md
│  ├─ 02_install_ros2_jazzy.md
│  ├─ 03_network_dds.md
│  ├─ 04_lidar_a1m8.md
│  ├─ 05_tf_design.md
│  ├─ 06_slam.md
│  └─ 07_rviz.md
├─ ros2_ws/
│  ├─ src/
│  │  ├─ zeuscar_bringup/
│  │  ├─ zeuscar_description/
│  │  ├─ zeuscar_lidar/
│  │  └─ zeuscar_slam/
│  └─ .colcon_defaults.yaml
├─ tools/
│  ├─ install_deps.sh
│  ├─ udev_rules/
│  └─ env/
└─ .github/workflows/
```

### 6.2 ファイル・ディレクトリ役割一覧
| パス | 種別 | 役割 |
|---|---|---|
| README.md | doc | リポジトリ全体の概要とクイックスタート |
| docs/ | doc | 手順・設計思想・解説を体系的に管理 |
| docs/01_setup_ubuntu24_04_desktop.md | doc | Ubuntu 導入・初期設定 |
| docs/02_install_ros2_jazzy.md | doc | ROS 2 Jazzy インストール |
| docs/03_network_dds.md | doc | PC-Pi 間 ROS2 通信設定 |
| docs/04_lidar_a1m8.md | doc | A1M8 導入・動作確認 |
| docs/05_tf_design.md | doc | TF 設計方針 |
| docs/06_slam.md | doc | SLAM 構築手順 |
| docs/07_rviz.md | doc | RViz 設定と確認 |
| ros2_ws/ | code | ROS 2 ワークスペース |
| zeuscar_bringup | pkg | 起動統合パッケージ |
| zeuscar_description | pkg | URDF/TF 定義 |
| zeuscar_lidar | pkg | LiDAR 起動ラッパ |
| zeuscar_slam | pkg | SLAM 起動構成 |
| tools/ | tool | 運用補助 |

---

## 7. 機能要件
- LiDAR データの取得（/scan）
- RViz での LaserScan / TF / RobotModel 表示
- slam_toolbox による自己位置推定

---

## 8. 非機能要件
| 項目 | 要件 |
|---|---|
| 再現性 | 手順通りで再構築可能 |
| 可読性 | 学習用途に適した構成 |
| 拡張性 | Nav2 等への拡張余地 |

---

## 9. 設計・運用ルール
- 構成変更時は README / docs を更新
- 起動エントリは zeuscar_bringup に集約
- TF 変更時は tf 設計ドキュメントを更新

---

## 10. 将来拡張（参考）
- Navigation2
- IMU / Camera 統合
- Docker 化

---

## 11. 関連資料
- ZeusCar ROS2 制御（旧構成）
  - https://murasan-net.com/2025/01/31/zeuscar-raspberry-pi-ros2-control/
- RPLIDAR A1M8 + ROS2
  - https://murasan-net.com/2024/05/16/a1m8-lidar-ros2-build/
- 旧リポジトリ
  - https://github.com/Murasan201/zeuscar-project

