# コメントスタイルガイド

本プロジェクトにおけるコード内コメントの記述規約を定めます。

---

## 1. 基本方針

- コメントは**日本語**で記述する
- **何をするか**ではなく**なぜそうするか**を説明する
- 自明なコードにはコメントを付けない
- コメントは常にコードと同期させる

---

## 2. コメントの種類

### 2.1 ファイルヘッダー

ファイルの先頭に目的を記述する。

```python
"""
LiDARドライバーラッパー

RPLIDAR A1M8を制御し、/scanトピックをパブリッシュする。
zeuscar_lidarパッケージのメインモジュール。
"""
```

### 2.2 モジュールDocstring

```python
"""zeuscar_lidar.lidar_node モジュール.

LiDARノードの実装を提供する。

Example:
    ros2 run zeuscar_lidar lidar_node
"""
```

### 2.3 関数・メソッドDocstring

Google スタイルを使用する。

```python
def calculate_offset(measured: float, reference: float) -> float:
    """オフセット値を計算する.

    測定値と基準値からオフセットを算出する。
    結果はメートル単位で返される。

    Args:
        measured: 測定値 (メートル)
        reference: 基準値 (メートル)

    Returns:
        オフセット値 (メートル)

    Raises:
        ValueError: 測定値が負の場合

    Note:
        精度は±1mmを想定している。
    """
    if measured < 0:
        raise ValueError('測定値は正の値である必要があります')
    return measured - reference
```

### 2.4 クラスDocstring

```python
class LidarNode(Node):
    """LiDARノード.

    RPLIDAR A1M8からスキャンデータを取得し、
    /scanトピックとしてパブリッシュする。

    Attributes:
        serial_port: シリアルポートのパス
        frame_id: TFフレームID
        scan_frequency: スキャン周波数 (Hz)

    Example:
        node = LidarNode()
        rclpy.spin(node)
    """
```

---

## 3. インラインコメント

### 3.1 基本ルール

- コードの右側に記述する場合、スペース2つ以上空ける
- 行が長くなる場合は上の行に記述する

```python
# Good: 上の行にコメント
# LiDARの向きが90度回転しているため、yaw角を補正
yaw_offset = math.pi / 2

# Good: 右側にコメント（短い説明）
max_range = 12.0  # RPLIDAR A1M8の最大測定距離

# Bad: コメントが長すぎる
yaw_offset = math.pi / 2  # LiDARはロボット正面から見て左に90度回転して取り付けられているため、スキャンデータの座標変換時にこのオフセットを適用する必要がある
```

### 3.2 説明が必要なケース

```python
# 定数の意味
SCAN_TIMEOUT = 2.0  # スキャン待機タイムアウト (秒)

# 複雑なロジック
# base_linkからlaser_frameへの変換
# Z値はbase_linkからの相対高さなので、地面からの高さから引く
laser_z_from_base = lidar_z - base_link_height

# 一時的な対処（TODO付き）
# TODO: rplidar_rosの次期バージョンで修正予定
self._workaround_scan_direction()
```

### 3.3 説明不要なケース

```python
# Bad: 自明なコード
x = x + 1  # xを1増やす
items = []  # 空のリストを作成

# Good: コメントなしでも意図が明確
x += 1
items = []
```

---

## 4. 特殊コメント

### 4.1 TODO

未実装や将来の改善点を示す。

```python
# TODO: パラメータをYAMLファイルから読み込むように変更
# TODO(username): 期限 2026-02-01 までに対応
```

### 4.2 FIXME

既知のバグや問題点を示す。

```python
# FIXME: 高速回転時にデータが欠落する問題
# FIXME: メモリリークの可能性あり
```

### 4.3 NOTE

重要な注意点を示す。

```python
# NOTE: この関数はスレッドセーフではない
# NOTE: 座標系はREP-103に準拠
```

### 4.4 HACK

一時的な回避策を示す。

```python
# HACK: ライブラリのバグを回避するための一時的な対処
```

### 4.5 WARNING

危険な操作や注意が必要な箇所を示す。

```python
# WARNING: この操作はデバイスをリセットする
# WARNING: 本番環境では使用しないこと
```

---

## 5. セクションコメント

長いファイルをセクションに分割する場合に使用する。

```python
# ==============================================================================
# 定数定義
# ==============================================================================

DEFAULT_PORT = '/dev/rplidar'
DEFAULT_BAUDRATE = 115200

# ==============================================================================
# ユーティリティ関数
# ==============================================================================

def validate_port(port: str) -> bool:
    """ポートの存在を確認する."""
    pass

# ==============================================================================
# メインクラス
# ==============================================================================

class LidarNode(Node):
    """LiDARノード."""
    pass
```

---

## 6. URDF/XMLコメント

```xml
<?xml version="1.0"?>
<!--
  ZeusCar URDF定義

  フレーム構成:
    base_footprint → base_link → laser_frame

  座標系: REP-103準拠
-->
<robot name="zeuscar">

  <!-- ========== パラメータ定義 ========== -->

  <!-- ロボット本体寸法 (実測値) -->
  <xacro:property name="body_length" value="0.163"/>  <!-- 163mm -->

  <!-- LiDAR取り付け位置
       X: +3.5mm (前方)
       Y: -4.5mm (右方向)
       Z: 235mm (地面から)
  -->
  <xacro:property name="lidar_x" value="0.0035"/>

</robot>
```

---

## 7. Launchファイルコメント

```python
"""
ZeusCar description launchファイル

robot_state_publisherを起動し、URDFからTFをパブリッシュする。

起動方法:
    ros2 launch zeuscar_description description.launch.py

パブリッシュされるTF:
    - base_footprint → base_link
    - base_link → laser_frame
"""

from launch import LaunchDescription

def generate_launch_description():
    """Launchファイルのエントリポイント."""

    # URDFファイルのパスを取得
    urdf_file = os.path.join(pkg_share, 'urdf', 'zeuscar.urdf.xacro')

    # robot_state_publisher: URDFをパースしてTFをパブリッシュ
    robot_state_publisher = Node(
        package='robot_state_publisher',
        # ...
    )
```

---

## 8. 避けるべきパターン

### 8.1 コメントアウトしたコード

```python
# Bad: コメントアウトしたコードを残す
# old_value = calculate_old_way(x)
# result = old_value * 2
new_value = calculate_new_way(x)
result = new_value * 2

# Good: 不要なコードは削除する（Gitで履歴が残る）
new_value = calculate_new_way(x)
result = new_value * 2
```

### 8.2 嘘のコメント

```python
# Bad: コメントとコードが一致していない
# 距離を計算する
angle = math.atan2(y, x)  # 実際は角度を計算している
```

### 8.3 過剰なコメント

```python
# Bad: すべての行にコメント
# xを取得する
x = get_x()
# yを取得する
y = get_y()
# 距離を計算する
distance = math.sqrt(x**2 + y**2)
# 距離を返す
return distance

# Good: 必要な箇所のみコメント
x = get_x()
y = get_y()
return math.sqrt(x**2 + y**2)
```

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-19 | 初版作成 |
