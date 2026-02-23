# ホイールオドメトリ統合（センサーフュージョン）仕様書

## 改訂履歴

| 版数 | 日付 | 作成者 | 変更内容 |
|---|---|---|---|
| 1.0 | 2026-02-23 | PM | 初版作成 |

---

## 1. 目的・背景

### 1.1 目的

コマンドベースのホイールオドメトリを実装し、EKF（拡張カルマンフィルタ）に速度情報（vx, vy）を提供する。
これによりIMUのみのEKFでは不可能だった線速度・位置の推定を可能にし、`slam_toolbox`依存度を下げる。

### 1.2 背景

スプリント1の全ストーリー（STORY-001〜025）が完了し、以下のセンサーフュージョン構成が動作中:

- **IMU（ICM-42688）**: madgwickフィルタ経由で姿勢（roll, pitch）と角速度（wx, wy, wz）をEKFに入力
- **EKF**: `odom → base_footprint` TF変換を生成
- **slam_toolbox**: LiDARスキャンマッチングでマップ生成・位置補正

**現状の限界:**

| 項目 | 現状（IMUのみ） | ホイールオドメトリ追加後 |
|---|---|---|
| 線速度（vx, vy） | 推定不可（常にゼロ） | コマンドベースで推定可能 |
| 位置推定 | SLAMに完全依存 | EKFで粗い位置推定が可能 |
| 停止検知 | 不可 | ゼロ速度拘束で高精度 |
| SLAM未使用時の走行 | 位置が動かない | 大まかな走行軌跡が得られる |

### 1.3 技術的前提

**エンコーダ未搭載（重要な制約）:**
- モーターに物理エンコーダが搭載されていない
- RPi→Arduino 通信は一方向（コマンド送信のみ、フィードバックなし）
- Arduino側の変更はユーザーが許可済みだが、本フェーズでは不要

**設計方針: コマンドベース速度推定（Open-Loop Odometry）:**

`/cmd_vel`で送信したコマンドとキャリブレーション済み速度定数から速度を推定する。
エンコーダベースの精密なオドメトリとは異なり、「どの方向にどの程度の速度で動いているはず」
という推定値をEKFに提供する。EKFはこの情報をIMUデータと融合し、より良い状態推定を行う。

---

## 2. スコープと非対象

### 2.1 スコープ（Phase 1: コマンドベース速度推定）

- `twist_converter.py` 共有モジュールの作成（Twist→コマンド変換ロジック抽出）
- `wheel_odom_node` の新規作成（`zeuscar_motor`パッケージ内）
- `/cmd_vel` 購読 → コマンド判定 → 速度推定 → `/wheel_odom` 配信
- EKFに `odom0` として速度（vx, vy）のみを融合
- キャリブレーション手順の策定と実施
- ユニットテスト・実機テスト

### 2.2 非対象（Phase 2: 将来のエンコーダ対応）

- Arduino側のエンコーダ読み取り実装
- エンコーダフィードバックによるクローズドループオドメトリ
- Nav2ナビゲーション統合
- 自律走行

---

## 3. アーキテクチャ

### 3.1 システム構成図（目標）

```
[teleop / nav2]
    └→ /cmd_vel (geometry_msgs/Twist)
        ├→ [motor_controller_node]  ← 既存（Arduinoへ送信）
        │     └→ Serial → Arduino → モーター駆動
        └→ [wheel_odom_node]        ← 新規
              └→ /wheel_odom (nav_msgs/Odometry)
                    └→ [ekf_filter_node]  ← odom0として融合
                          ├→ imu0: /imu/data（既存）
                          ├→ odom0: /wheel_odom（新規）
                          ├→ /odometry/filtered
                          └→ /tf (odom → base_footprint)
```

### 3.2 データフロー

```
/cmd_vel ──→ wheel_odom_node ──→ /wheel_odom ──→ EKF（odom0: vx, vy）
                                                      ↓
/imu/data ────────────────────────────────────→ EKF（imu0: roll, pitch, wx, wy, wz）
                                                      ↓
                                              /odometry/filtered
                                              /tf (odom → base_footprint)
```

### 3.3 新規ノード: `wheel_odom_node`

| 項目 | 値 |
|---|---|
| パッケージ | `zeuscar_motor` |
| ノード名 | `wheel_odom_node` |
| ソースファイル | `zeuscar_motor/wheel_odom_node.py` |
| エントリポイント | `wheel_odom_node` |

**配置を `zeuscar_motor` パッケージにする理由:**
- モーターコマンドと密接に関連（同じTwist→コマンド変換ロジックを使用）
- `twist_converter.py` を共有モジュールとして抽出できる
- 将来エンコーダを追加する際も、モーター関連パッケージ内で完結

### 3.4 共有モジュール: `twist_converter.py`

現在 `motor_controller_node.py` 内にある `_twist_to_command()` ロジックを共有モジュールに抽出する。

| 項目 | 値 |
|---|---|
| パッケージ | `zeuscar_motor` |
| ファイル | `zeuscar_motor/twist_converter.py` |
| 関数 | `twist_to_command(linear_x, linear_y, angular_z, linear_threshold, angular_threshold) -> str` |

**抽出の理由:**
- `motor_controller_node` と `wheel_odom_node` で同一のTwist→コマンド変換ロジックが必要
- ロジックの重複を避け、コマンド判定の一貫性を保証する
- 単体テストが容易になる

---

## 4. メカナムキネマティクス

### 4.1 ホイール配置

ZeusCarのメカナムホイール配置（Arduino `raspi-ctrl-v_2_00.ino` より）:

```
  前方 (FORWARD: +x)
  ←──────────→
  [0]--|||--[1]     ||| = キャスター（前方マーカー）
   |         |      ↑ +y（LEFT方向）
   |         |
   |         |
  [3]-------[2]
```

### 4.2 コマンド → モーター出力マッピング

Arduinoの `carSetMotors(power0, power1, power2, power3)` 呼び出し:

| コマンド | Motor 0 | Motor 1 | Motor 2 | Motor 3 | 動作 |
|---|---|---|---|---|---|
| FORWARD | +P | +P | +P | +P | 全輪前進 |
| BACKWARD | -P | -P | -P | -P | 全輪後退 |
| LEFT | -P | +P | -P | +P | 左横移動（ストレーフ） |
| RIGHT | +P | -P | +P | -P | 右横移動（ストレーフ） |
| LEFTFORWARD | 0 | +P | 0 | +P | 左前斜め移動 |
| RIGHTFORWARD | +P | 0 | +P | 0 | 右前斜め移動 |
| LEFTBACKWARD | -P | 0 | -P | 0 | 左後斜め移動 |
| RIGHTBACKWARD | 0 | -P | 0 | -P | 右後斜め移動 |
| TURNLEFT | -P | +P | +P | -P | 左旋回（反時計回り） |
| TURNRIGHT | +P | -P | -P | +P | 右旋回（時計回り） |
| STOP | 0 | 0 | 0 | 0 | 停止 |

P = power（デフォルト80/100）

### 4.3 コマンド → 推定速度マッピング

各コマンドに対する推定速度 `(vx, vy, wz)`:

| コマンド | vx | vy | wz | 備考 |
|---|---|---|---|---|
| FORWARD | +V_linear | 0 | 0 | 前進 |
| BACKWARD | -V_linear | 0 | 0 | 後退 |
| LEFT | 0 | +V_strafe | 0 | 左ストレーフ（+y方向） |
| RIGHT | 0 | -V_strafe | 0 | 右ストレーフ（-y方向） |
| LEFTFORWARD | +V_diag | +V_diag | 0 | 45度斜め（2輪駆動） |
| RIGHTFORWARD | +V_diag | -V_diag | 0 | 45度斜め（2輪駆動） |
| LEFTBACKWARD | -V_diag | +V_diag | 0 | 45度斜め（2輪駆動） |
| RIGHTBACKWARD | -V_diag | -V_diag | 0 | 45度斜め（2輪駆動） |
| TURNLEFT | 0 | 0 | +W_angular | 左旋回（反時計回り） |
| TURNRIGHT | 0 | 0 | -W_angular | 右旋回（時計回り） |
| STOP | 0 | 0 | 0 | 停止 |

**速度定数の関係:**
- `V_linear`: 直進時の線速度 [m/s]（キャリブレーションで実測）
- `V_strafe`: 横移動時の線速度 [m/s]（キャリブレーションで実測、通常 V_linear と異なる）
- `V_diag`: 斜め移動時の各軸速度 [m/s]（2輪駆動のため V_linear より小さい）
  - 理論値: `V_diag = V_linear * cos(45°) ≈ V_linear * 0.707`
  - ただしキャリブレーションで補正する
- `W_angular`: 旋回時の角速度 [rad/s]（キャリブレーションで実測）

---

## 5. パラメータ設計

### 5.1 wheel_odom_node パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `v_linear` | double | 0.15 | 直進時の推定線速度 [m/s] |
| `v_strafe` | double | 0.12 | 横移動時の推定線速度 [m/s] |
| `v_diag` | double | 0.10 | 斜め移動時の各軸推定速度 [m/s] |
| `w_angular` | double | 0.5 | 旋回時の推定角速度 [rad/s] |
| `linear_threshold` | double | 0.1 | Twist線速度の閾値 |
| `angular_threshold` | double | 0.1 | Twist角速度の閾値 |
| `publish_rate` | double | 30.0 | オドメトリ配信周波数 [Hz] |
| `cmd_vel_timeout` | double | 0.5 | コマンドタイムアウト [s] |
| `covariance_moving` | double | 0.1 | 移動時の速度共分散 |
| `covariance_stopped` | double | 0.001 | 停止時の速度共分散 |

**デフォルト値の根拠:**
- `v_linear = 0.15`: ZeusCarのサイズ（約20cm四方）と power=80/100 から推定する初期値。キャリブレーションで更新する
- `v_strafe`: メカナムホイールの横移動は直進より遅いため V_linear の80%を初期値とする
- `v_diag`: 2輪駆動のため V_linear * 0.707 ≈ 0.10 を初期値とする
- `w_angular = 0.5`: 小型ロボットの一般的な旋回速度。キャリブレーションで更新する

### 5.2 共分散設計

コマンドベースオドメトリの共分散はロボットの状態に応じて動的に変化させる:

| 状態 | 速度共分散 | 理由 |
|---|---|---|
| 停止中（STOP） | 0.001（非常に低い） | 停止状態は高確信。ゼロ速度拘束として機能 |
| 移動中（その他） | 0.1（高め） | コマンドベース推定のため実際の速度との乖離が大きい |

**ゼロ速度拘束の効果:**
停止時に低共分散でゼロ速度を配信することで、EKFが「ロボットは止まっている」と高い確信を持つ。
これはIMUの加速度バイアスによるドリフトを抑制する効果がある。

---

## 6. 入出力インタフェース

### 6.1 wheel_odom_node

#### 入力

| トピック | 型 | QoS | ソース |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | RELIABLE, depth=10 | teleop / nav2 |

#### 出力

| トピック | 型 | QoS | 説明 |
|---|---|---|---|
| `/wheel_odom` | `nav_msgs/Odometry` | RELIABLE, depth=10 | 推定速度オドメトリ |

#### Odometryメッセージの構成

```
nav_msgs/Odometry:
  header:
    frame_id: "odom"
  child_frame_id: "base_footprint"
  pose:                           # 使用しない（位置はEKFに任せる）
    covariance: [大きな値]         # 位置は信頼しないことを示す
  twist:
    twist:
      linear:
        x: vx                    # 推定前進速度
        y: vy                    # 推定横移動速度
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: wz                    # 推定角速度
    covariance: [動的に変化]      # 停止/移動で変化
```

**TF配信は行わない:**
`wheel_odom_node` は TF を配信しない。TF（`odom → base_footprint`）の配信は EKF が一元的に行う。

---

## 7. EKF統合設計

### 7.1 ekf_params.yaml への追加設定

現在のEKF設定（imu0のみ）に `odom0` を追加する:

```yaml
ekf_filter_node:
  ros__parameters:
    # --- 既存: IMUセンサー設定 ---
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  false,
                  false, false, false,
                  true,  true,  true,
                  false, false, false]
    imu0_remove_gravitational_acceleration: true
    imu0_queue_size: 10
    imu0_differential: false
    imu0_relative: false

    # --- 新規: ホイールオドメトリ設定 ---
    odom0: /wheel_odom

    # odom0_config: [x,  y,  z,        位置（コマンドベースでは信頼できない）
    #                r,  p,  yaw,      姿勢（IMUが優れる）
    #                vx, vy, vz,       速度（★ここを使用）
    #                wx, wy, wz,       角速度（IMUが優れる）
    #                ax, ay, az]       加速度（不要）
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, false,
                   false, false, false]

    odom0_queue_size: 10
    odom0_differential: false
    odom0_relative: false
```

### 7.2 odom0_config の設計判断

| インデックス | 状態変数 | 使用 | 理由 |
|---|---|---|---|
| 0-2 | x, y, z（位置） | false | コマンドベースの積分位置はドリフトが大きい |
| 3-5 | roll, pitch, yaw（姿勢） | false | 姿勢はIMU（madgwick）の方が精度が高い |
| 6-7 | vx, vy（線速度） | **true** | コマンドから推定した速度をEKFに提供 |
| 8 | vz（Z軸速度） | false | 平面ロボットのためZ方向速度は不要 |
| 9-11 | wx, wy, wz（角速度） | false | 角速度はIMUジャイロの方が精度が高い |
| 12-14 | ax, ay, az（加速度） | false | 不要 |

> **設計判断: wz をホイールオドメトリから取らない理由**
>
> コマンドベースの角速度推定よりも、IMUのジャイロセンサーの方が遥かに精度が高い。
> コマンドベースでは「TURNLEFTコマンドを送った → W_angular [rad/s] で回転しているはず」
> という粗い推定しかできないが、IMUジャイロは実際の角速度を直接計測できる。

### 7.3 センサーフュージョンの役割分担

| 状態変数 | データソース | 理由 |
|---|---|---|
| roll, pitch | IMU（madgwick） | 加速度計＋ジャイロ融合で安定 |
| yaw（変化量） | IMU（ジャイロwz積分） | 磁力計なしのため絶対yawは不可 |
| vx, vy | ホイールオドメトリ | IMUでは速度推定不可 |
| 位置（x, y） | EKFが速度を積分 | SLAMが補正 |

---

## 8. キャリブレーション手順

### 8.1 概要

コマンドベースオドメトリの精度はキャリブレーション定数の精度に依存する。
以下の3つの定数を実測で決定する。

### 8.2 V_linear（直進速度）の計測

1. 床に1mの直線を計測してマーキングする
2. ロボットをスタート位置にセットする
3. FORWARDコマンドを送信し、ストップウォッチで計測する
4. 1m走行した時点で停止、経過時間を記録する
5. `V_linear = 1.0 / 経過時間 [s]` を計算する
6. 3回以上計測して平均を取る

```bash
# コマンド例
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --once
# 計測後に停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### 8.3 V_strafe（横移動速度）の計測

1. 床に1mの直線を横方向にマーキングする
2. LEFTコマンドで横移動させ、1m到達までの時間を計測する
3. `V_strafe = 1.0 / 経過時間 [s]` を計算する
4. 3回以上計測して平均を取る

### 8.4 W_angular（旋回角速度）の計測

1. ロボットの向きを壁に対して正面にセットする
2. TURNLEFTコマンドを送信する
3. 360度（1回転）するまでの時間を計測する
4. `W_angular = 2π / 経過時間 [s]` を計算する
5. 3回以上計測して平均を取る

### 8.5 V_diag（斜め移動速度）の計測

1. V_linear のキャリブレーション後、理論値 `V_linear * 0.707` を初期値とする
2. LEFTFORWARDコマンドで1m（対角距離）走行させ、時間を計測する
3. 必要に応じて補正する

### 8.6 キャリブレーション結果の反映

計測した値を `wheel_odom_node` のパラメータとして設定する:

```yaml
# launch引数またはパラメータファイルで設定
wheel_odom_node:
  ros__parameters:
    v_linear: 0.XX    # 計測値
    v_strafe: 0.XX    # 計測値
    v_diag: 0.XX      # 計測値
    w_angular: 0.XX   # 計測値
```

---

## 9. 実装フェーズ

### Phase 1: twist_converter 共有モジュール作成（STORY-026）

#### 1-1. テスト作成（Red フェーズ）

テスト担当エージェントが `test_twist_converter.py` を作成:

- 全11コマンドに対する変換テスト
- 閾値処理テスト
- 旋回優先ロジックテスト
- エッジケーステスト（全ゼロ入力）

#### 1-2. 実装（Green フェーズ）

実装担当エージェントが以下を実装:

1. **`zeuscar_motor/twist_converter.py`** — `twist_to_command()` 関数を抽出
2. **`motor_controller_node.py` の修正** — `twist_converter` を使用するようリファクタ

### Phase 2: wheel_odom_node 実装（STORY-027）

#### 2-1. テスト作成（Red フェーズ）

テスト担当エージェントが `test_wheel_odom_node.py` を作成:

- 各コマンドに対する速度出力テスト
- 共分散の動的変化テスト（停止時 vs 移動時）
- タイムアウトテスト
- Odometryメッセージのフレーム名テスト
- パラメータ設定テスト

#### 2-2. 実装（Green フェーズ）

実装担当エージェントが以下を実装:

1. **`zeuscar_motor/wheel_odom_node.py`** — ノード本体
2. **`setup.py` 更新** — エントリポイント追加
3. **`package.xml` 更新** — 必要な依存追加

#### 2-3. 処理フロー

```
[初期化]
  ├→ パラメータ読み込み（速度定数、共分散、タイムアウト）
  ├→ /cmd_vel サブスクライバ作成
  ├→ /wheel_odom パブリッシャ作成
  └→ タイマー作成（publish_rate Hz）

[/cmd_vel コールバック]
  ├→ タイムスタンプ記録
  └→ twist_to_command() でコマンド判定 → 保持

[タイマーコールバック（30Hz）]
  ├→ タイムアウト判定
  │    └→ タイムアウト時: コマンド = STOP
  ├→ コマンド → (vx, vy, wz) 変換
  ├→ 共分散計算（停止/移動）
  └→ Odometryメッセージ作成・配信
```

### Phase 3: EKF設定更新（STORY-028）

#### 3-1. テスト作成（Red フェーズ）

テスト担当エージェントが既存 `test_ekf_launch.py` を拡張:

- `odom0` 設定の存在確認
- `odom0_config` の値検証（vx, vy のみ true）
- `odom0_queue_size` の存在確認

#### 3-2. 実装（Green フェーズ）

実装担当エージェントが以下を更新:

1. **`ekf_params.yaml`** — odom0 設定を追加
2. **`odometry.launch.py`** — 必要に応じて `wheel_odom_node` の起動を追加

### Phase 4: キャリブレーション・統合テスト（STORY-029）

#### 4-1. キャリブレーション実施

セクション8の手順に従い、速度定数を実測で決定する。

#### 4-2. 統合テスト

全ノードを起動し、以下を確認:
- EKFが `/wheel_odom` を購読し、速度情報を融合していること
- 停止時にオドメトリ位置がドリフトしないこと
- 走行時にオドメトリ位置が概ね移動方向に追従すること

---

## 10. ファイル構成（変更計画）

### 10.1 新規作成ファイル

```
ros2_ws/src/zeuscar_motor/
├── zeuscar_motor/
│   ├── twist_converter.py           # [新規] Twist→コマンド変換共有モジュール
│   └── wheel_odom_node.py           # [新規] ホイールオドメトリノード
└── test/
    ├── test_twist_converter.py      # [新規] twist_converter単体テスト
    └── test_wheel_odom_node.py      # [新規] wheel_odom_node単体テスト
```

### 10.2 変更ファイル

```
ros2_ws/src/zeuscar_motor/
├── zeuscar_motor/
│   └── motor_controller_node.py     # [変更] twist_converter使用にリファクタ
├── setup.py                         # [変更] wheel_odom_nodeエントリポイント追加
└── package.xml                      # [変更] nav_msgs依存追加

ros2_ws/src/zeuscar_bringup/
├── config/
│   └── ekf_params.yaml              # [変更] odom0設定追加
└── launch/
    └── odometry.launch.py           # [変更] wheel_odom_node起動追加（検討）
```

### 10.3 変更しないファイル

- `slam_params.yaml` — SLAM設定は変更不要
- `zeuscar.urdf.xacro` — 物理的な変更なし
- `sensors.launch.py` — センサーノードは変更不要
- `raspi-ctrl-v_2_00.ino` — Arduino側変更なし

---

## 11. 受け入れ基準

### AC-1: twist_converter 共有モジュール

- [ ] `twist_to_command()` が全11コマンドを正しく変換する
- [ ] `motor_controller_node` が `twist_converter` を使用する形にリファクタされている
- [ ] 既存テストが全てパスする（リグレッションなし）

### AC-2: wheel_odom_node 動作

- [ ] `/wheel_odom` トピックが `nav_msgs/Odometry` 型で配信される
- [ ] `frame_id` が `odom`、`child_frame_id` が `base_footprint` である
- [ ] FORWARD 受信時に `twist.linear.x > 0` が配信される
- [ ] STOP 受信時に全速度がゼロで配信される
- [ ] 停止時の共分散が移動時より低い値である
- [ ] `/cmd_vel` タイムアウト時に自動でSTOP状態に遷移する

### AC-3: EKF統合

- [ ] EKFが `/wheel_odom` を購読してエラーなく動作する
- [ ] 停止状態でオドメトリ位置がドリフトしない（ゼロ速度拘束の効果）
- [ ] 走行時にオドメトリ位置が移動方向に概ね追従する
- [ ] IMU姿勢推定に悪影響がない（roll/pitch の精度維持）

### AC-4: キャリブレーション

- [ ] V_linear, V_strafe, W_angular の計測値が記録されている
- [ ] キャリブレーション後の速度推定誤差が30%以内である

---

## 12. テスト・検証計画

### 12.1 ユニットテスト（自動）

| テストID | テスト内容 | ファイル |
|---|---|---|
| T-TC-01 | FORWARD→'FORWARD'変換 | `test_twist_converter.py` |
| T-TC-02 | 全11コマンドの変換検証 | `test_twist_converter.py` |
| T-TC-03 | 閾値処理（微小入力→STOP） | `test_twist_converter.py` |
| T-TC-04 | 旋回優先ロジック | `test_twist_converter.py` |
| T-WO-01 | FORWARD受信→vx>0のOdometry配信 | `test_wheel_odom_node.py` |
| T-WO-02 | STOP時の共分散が低い値 | `test_wheel_odom_node.py` |
| T-WO-03 | 移動時の共分散が高い値 | `test_wheel_odom_node.py` |
| T-WO-04 | タイムアウト→STOP遷移 | `test_wheel_odom_node.py` |
| T-WO-05 | frame_id/child_frame_id検証 | `test_wheel_odom_node.py` |
| T-WO-06 | パラメータ設定値の反映 | `test_wheel_odom_node.py` |
| T-EKF-10 | odom0設定の存在確認 | `test_ekf_launch.py`（拡張） |
| T-EKF-11 | odom0_configの値検証 | `test_ekf_launch.py`（拡張） |

### 12.2 実機テスト（手動）

| テストID | テスト内容 | 期待結果 |
|---|---|---|
| T-INT-10 | wheel_odom_node単体起動 | `/wheel_odom` トピックが配信される |
| T-INT-11 | FORWARD走行中のvx確認 | `vx ≈ V_linear`（キャリブレーション値） |
| T-INT-12 | 停止時のゼロ速度拘束 | EKFオドメトリが静止（位置変化なし） |
| T-INT-13 | EKF統合後のSLAM動作 | 既存SLAM機能に影響なし |
| T-INT-14 | 30秒直進後の位置精度 | SLAM補正なしで誤差30%以内 |
| T-INT-15 | キャリブレーション計測 | V_linear, V_strafe, W_angular の記録 |

---

## 13. 将来のエンコーダ対応パス（Phase 2）

### 13.1 設計方針

Phase 2 でエンコーダを追加する際、同一トピック `/wheel_odom` を維持する設計とする。
これにより EKF 側の設定変更を最小限に抑えられる。

### 13.2 移行パス

```
Phase 1（現在）:
  /cmd_vel → [wheel_odom_node（コマンドベース）] → /wheel_odom → EKF

Phase 2（将来）:
  Arduino → エンコーダ値 → Serial → [wheel_odom_node（エンコーダベース）] → /wheel_odom → EKF
```

**変更点:**
- `wheel_odom_node` の入力ソースを `/cmd_vel` からエンコーダデータに変更
- 速度推定ロジックをエンコーダカウントベースに置き換え
- 共分散を大幅に低下（エンコーダはコマンドベースより精度が高い）
- EKF設定（`odom0_config`）は変更不要

### 13.3 Arduino側の変更（Phase 2 実施時）

- エンコーダ読み取り機能の追加
- シリアル通信の双方向化（エンコーダ値のフィードバック）
- RPi側にエンコーダデータ受信ノードの追加

---

## 14. リスクと対策

| リスク | 影響度 | 対策 |
|---|---|---|
| コマンドベース速度推定の精度が低い | 中 | 共分散を高めに設定し、EKFがIMUと適切に融合する。SLAMが最終的に補正 |
| 床面の摩擦変動でスリップ発生 | 中 | コマンドベースのため検知不可。共分散で不確実性を表現 |
| motor_controller_nodeリファクタによるリグレッション | 高 | 既存テストを全てパスさせる。twist_converter の単体テストを充実 |
| EKF odom0追加で既存IMU融合に悪影響 | 中 | vx/vyのみ使用。IMU由来の姿勢・角速度は独立して動作 |
| cmd_velのタイミングとモーター実動作のラグ | 低 | タイムアウト機構で緩和。共分散で不確実性を表現 |

---

## 15. 設計判断記録

### Q1: 新規ノード vs 既存ノード拡張

**判断:** `wheel_odom_node` を新規ノードとして作成

**理由:**
- 単一責任原則: `motor_controller_node` はモーター制御に専念
- テスタビリティ: オドメトリ推定ロジックを独立してテスト可能
- 将来の差し替え: エンコーダベース実装への移行が容易
- 障害分離: オドメトリノードが落ちてもモーター制御に影響しない

### Q2: 配置パッケージの選択

**判断:** `zeuscar_motor` パッケージ内に配置

**理由:**
- モーターコマンドと密接に関連（同じTwist→コマンド変換を使用）
- `twist_converter.py` を共有モジュールとして活用
- 新規パッケージ作成の必要性がない

### Q3: EKFで使用する値（vx, vy のみ）

**判断:** 位置・姿勢・角速度は使用しない

**理由:**
- 位置: コマンドベース積分はドリフトが大きく、EKFの位置推定を劣化させる
- 姿勢: IMU（madgwick）の方が遥かに精度が高い
- 角速度: IMUジャイロが直接計測値を提供。コマンドベースの粗い推定は不要
- vx, vy: **唯一IMUから得られない情報**であり、コマンドベースでも有用

### Q4: 停止時の共分散戦略

**判断:** 停止時は非常に低い共分散（0.001）でゼロ速度を配信

**理由:**
- 「モーターに電力を供給していない → 速度はゼロ」は非常に高い確信度
- EKFがこのゼロ速度拘束を信頼し、IMU加速度バイアスによるドリフトを抑制
- エンコーダがなくても、停止検知は確実に行える

### Q5: twist_converter の共有方法

**判断:** `twist_converter.py` として独立モジュールに抽出

**理由:**
- `motor_controller_node` と `wheel_odom_node` で同一ロジックが必要
- コードの重複を避け、コマンド判定の一貫性を保証
- 単体テストが容易（Nodeに依存しない純粋関数）

---

## 16. 依存関係

### 16.1 パッケージ依存

| パッケージ | 用途 | 状態 |
|---|---|---|
| `nav_msgs` | Odometryメッセージ型 | ROS 2標準（利用可能） |
| `geometry_msgs` | Twistメッセージ型 | ROS 2標準（利用可能） |
| `robot_localization` | EKF | インストール済み |

### 16.2 ストーリー依存

| 依存先 | 状態 | 必要なもの |
|---|---|---|
| STORY-017/018（motor_controller + cmd_vel） | Done | `_twist_to_command()` ロジック |
| STORY-011（EKF設定） | Done | `ekf_params.yaml` |
| STORY-014/015（bringup統合） | Done | `odometry.launch.py` |

---

## 17. 参考資料

- [robot_localization wiki](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- 本プロジェクト仕様書:
  - `docs/operations/specs/STORY-011_odometry_slam_verification.md` — EKF設定の設計判断
  - `docs/operations/specs/EPIC-006_arduino_interface.md` — Arduino通信仕様
  - `docs/operations/specs/STORY-014-015_bringup_design.md` — bringup統合設計
- Arduino実装: `reference/zeuscar-project/arduino/raspi-ctrl-v_2_00.ino` — モーター出力マッピング

---

**以上**
