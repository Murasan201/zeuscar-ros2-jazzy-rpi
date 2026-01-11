
# ZeusCar ROS 2 Jazzy Raspberry Pi システム  
## 要件定義書

## 1. 目的（Purpose）
本プロジェクトは、ZeusCar（メカナムホイールロボット）に対し、Raspberry Pi 4 + Ubuntu 24.04 Desktop + ROS 2 Jazzy を用いたロボット制御・センサ統合・自己位置推定（SLAM）システムを構築することを目的とする。

## 2. プロジェクトの位置づけ
- Raspberry Pi 側の ROS 2 システムに特化
- 既存 Arduino / PC 側構成は維持
- 旧リポジトリは参照専用

## 3. 対象リポジトリ
- 新規: zeuscar-ros2-jazzy-rpi
- 旧: https://github.com/Murasan201/zeuscar-project

## 4. システム構成
### ハードウェア
- Raspberry Pi 4
- Arduino
- ZeusCar
- RPLIDAR A1M8

### ソフトウェア
- Ubuntu 24.04 LTS Desktop
- ROS 2 Jazzy
- RViz2
- slam_toolbox

## 5. スコープ
### In Scope
- Pi 側 ROS 2 環境構築
- LiDAR / SLAM / RViz

### Out of Scope
- Arduino FW刷新
- Nav2 本格運用

## 6. リポジトリ構成（設計方針）
本構成は初期案であり、開発の進行に応じて適宜変更されることを前提とする。

(詳細構成と役割一覧は省略せず正式版に記載)

## 7. 機能要件
- /scan publish
- RViz 表示
- SLAM 実行

## 8. 非機能要件
- 再現性
- 拡張性
- 可読性

## 9. 将来拡張
- Nav2
- センサ追加

