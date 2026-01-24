# EPIC-006: Arduino通信インタフェース仕様書

## 1. 概要

### 1.1 目的

本仕様書は、Raspberry Pi（ROS 2 Jazzy）とArduino Uno R3間の通信インタフェースを定義し、ZeusCarの駆動系制御を実現するための設計指針を提供する。

### 1.2 スコープ

- Raspberry Pi側のROS 2ノード（motor_controllerノード）の設計
- Arduino側ファームウェアとの通信プロトコル
- ROS 2トピックインタフェース

### 1.3 スコープ外

- Arduino側ファームウェアの変更（既存実装を使用）
- PC側制御ノードの実装（別リポジトリ/環境で対応）

---

## 2. システム構成

### 2.1 全体アーキテクチャ

```
┌─────────────────┐      ROS 2 DDS      ┌─────────────────┐     Serial      ┌─────────────────┐
│    Host PC      │ ◄────────────────► │  Raspberry Pi   │ ◄─────────────► │   Arduino Uno   │
│ (Ubuntu 24.04)  │    /cmd_vel         │ (Ubuntu 24.04)  │  9600bps        │     R3          │
│  ROS 2 Jazzy    │    /zeuscar/cmd     │  ROS 2 Jazzy    │  ASCII + '\n'   │  Motor Control  │
└─────────────────┘                     └─────────────────┘                 └─────────────────┘
```

### 2.2 ハードウェア接続

| 接続元 | 接続先 | 接続方式 |
|--------|--------|----------|
| Host PC | Raspberry Pi | LAN (WiFi/Ethernet) |
| Raspberry Pi | Arduino Uno | USB Serial (/dev/ttyACM0) |
| Arduino Uno | モーター | PWM (SoftPWM) |

### 2.3 モーター配置

```
  [0]--|||--[1]    ← 前方
   |         |
   |  Robot  |
   |  Body   |
   |         |
  [3]-------[2]    ← 後方

  |||: メカナムホイール
  モーター0: 左前
  モーター1: 右前
  モーター2: 右後
  モーター3: 左後
```

---

## 3. 通信プロトコル

### 3.1 シリアル通信仕様（Raspberry Pi ↔ Arduino）

| 項目 | 値 |
|------|-----|
| シリアルポート | `/dev/ttyACM0`（デフォルト） |
| ボーレート | 9600 bps |
| データビット | 8 |
| パリティ | なし |
| ストップビット | 1 |
| フロー制御 | なし |
| 文字コード | ASCII |
| 区切り文字 | 改行 (`\n`) |

### 3.2 コマンド一覧

| 番号 | コマンド文字列 | 動作説明 | モーター制御パターン |
|------|----------------|----------|---------------------|
| 0 | `FORWARD` | 前進 | 4輪正転 |
| 1 | `BACKWARD` | 後退 | 4輪逆転 |
| 2 | `LEFT` | 左横移動 | 0,2逆転 / 1,3正転 |
| 3 | `RIGHT` | 右横移動 | 0,2正転 / 1,3逆転 |
| 4 | `LEFTFORWARD` | 左斜め前進 | 1,3正転 / 0,2停止 |
| 5 | `RIGHTFORWARD` | 右斜め前進 | 0,2正転 / 1,3停止 |
| 6 | `LEFTBACKWARD` | 左斜め後退 | 0,2逆転 / 1,3停止 |
| 7 | `RIGHTBACKWARD` | 右斜め後退 | 1,3逆転 / 0,2停止 |
| 8 | `TURNLEFT` | 左旋回（その場回転） | 0,3逆転 / 1,2正転 |
| 9 | `TURNRIGHT` | 右旋回（その場回転） | 0,3正転 / 1,2逆転 |
| 10 | `STOP` | 停止 | 全輪停止 |

### 3.3 モーターパワー設定

| パラメータ | 値 | 説明 |
|------------|-----|------|
| デフォルトパワー | 80 | 0-100の範囲 |
| 最小パワー | 28/255 | PWM最小値 |
| 最大パワー | 255/255 | PWM最大値 |

---

## 4. ROS 2インタフェース設計

### 4.1 ノード構成

```
zeuscar_motor/
├── motor_controller_node    ← 新規作成するノード
│   ├── Subscribe: /cmd_vel (geometry_msgs/Twist)
│   ├── Subscribe: /zeuscar/motor_cmd (std_msgs/String)
│   └── Serial → Arduino
```

### 4.2 サブスクライブトピック

#### 4.2.1 /cmd_vel（推奨）

ROS 2標準の速度指令トピック。

| 項目 | 値 |
|------|-----|
| トピック名 | `/cmd_vel` |
| メッセージ型 | `geometry_msgs/msg/Twist` |
| QoS | Reliable, Volatile, depth=10 |

**Twistメッセージからコマンドへの変換ロジック:**

```python
linear.x > 0 and linear.y == 0 and angular.z == 0  → FORWARD
linear.x < 0 and linear.y == 0 and angular.z == 0  → BACKWARD
linear.x == 0 and linear.y > 0 and angular.z == 0  → LEFT
linear.x == 0 and linear.y < 0 and angular.z == 0  → RIGHT
linear.x > 0 and linear.y > 0                       → LEFTFORWARD
linear.x > 0 and linear.y < 0                       → RIGHTFORWARD
linear.x < 0 and linear.y > 0                       → LEFTBACKWARD
linear.x < 0 and linear.y < 0                       → RIGHTBACKWARD
angular.z > 0                                       → TURNLEFT
angular.z < 0                                       → TURNRIGHT
すべて0                                             → STOP
```

#### 4.2.2 /zeuscar/motor_cmd（直接コマンド）

デバッグ・テスト用の直接コマンドトピック。

| 項目 | 値 |
|------|-----|
| トピック名 | `/zeuscar/motor_cmd` |
| メッセージ型 | `std_msgs/msg/String` |
| QoS | Reliable, Volatile, depth=10 |

**使用可能な文字列:**

- `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`
- `LEFTFORWARD`, `RIGHTFORWARD`, `LEFTBACKWARD`, `RIGHTBACKWARD`
- `TURNLEFT`, `TURNRIGHT`, `STOP`

### 4.3 パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|-------------|-----|-------------|------|
| `serial_port` | string | `/dev/ttyACM0` | Arduinoシリアルポート |
| `baud_rate` | int | `9600` | ボーレート |
| `cmd_vel_timeout` | double | `0.5` | cmd_velタイムアウト秒（この時間受信がないとSTOP） |
| `linear_threshold` | double | `0.1` | 速度閾値（これ以下は無視） |
| `angular_threshold` | double | `0.1` | 角速度閾値（これ以下は無視） |

---

## 5. 参照実装との差分

### 5.1 参照実装（Humble版）の課題

| 課題 | 参照実装 | Jazzy版での改善 |
|------|----------|----------------|
| トピック名 | `topic`（汎用的すぎる） | `/cmd_vel`, `/zeuscar/motor_cmd` |
| メッセージ型 | `std_msgs/String`のみ | `geometry_msgs/Twist`対応追加 |
| パラメータ化 | ハードコード | ROS 2パラメータで設定可能 |
| タイムアウト | なし | cmd_velタイムアウトで自動停止 |
| udevルール | なし | `/dev/arduino`シンボリックリンク |

### 5.2 後方互換性

- `/zeuscar/motor_cmd`トピックで参照実装と同じ文字列コマンドをサポート
- 参照実装のPC側Publisherノードからも接続可能（トピック名の変更が必要）

---

## 6. 実装計画

### 6.1 パッケージ構成

```
ros2_ws/src/
└── zeuscar_motor/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── zeuscar_motor/
    │   ├── __init__.py
    │   └── motor_controller_node.py
    ├── config/
    │   └── motor_params.yaml
    ├── launch/
    │   └── motor.launch.py
    └── udev/
        └── 99-arduino.rules
```

### 6.2 ストーリー分割

| ID | タイトル | 説明 | 依存 |
|-----|---------|------|------|
| STORY-016 | zeuscar_motorパッケージ作成 | パッケージ雛形作成 | - |
| STORY-017 | motor_controller_node実装 | シリアル通信・トピック購読 | STORY-016 |
| STORY-018 | cmd_vel対応 | Twist→コマンド変換ロジック | STORY-017 |
| STORY-019 | udevルール設定 | /dev/arduino作成 | - |
| STORY-020 | 動作確認 | 実機テスト | STORY-017, STORY-019 |

---

## 7. テスト計画

### 7.1 単体テスト

- [ ] Twist→コマンド変換ロジックのテスト
- [ ] シリアル通信モック使用テスト

### 7.2 統合テスト

- [ ] シリアルポート接続確認
- [ ] /zeuscar/motor_cmdトピックからの動作確認
- [ ] /cmd_velトピックからの動作確認
- [ ] タイムアウト機能の確認

### 7.3 実機テスト

- [ ] 全11コマンドの動作確認
- [ ] Host PCからのリモート制御確認

---

## 8. 安全対策

### 8.1 タイムアウト機能

- `cmd_vel_timeout`パラメータで設定した時間、コマンドを受信しない場合は自動的にSTOPコマンドを送信
- デフォルト: 0.5秒

### 8.2 シリアル接続断時の挙動

- 接続断を検知した場合、ログに警告を出力
- 再接続を試行（設定可能な間隔で）

---

## 改訂履歴

| 日付 | 担当者 | 内容 |
|------|--------|------|
| 2026-01-24 | - | 初版作成（参照実装から仕様抽出） |
