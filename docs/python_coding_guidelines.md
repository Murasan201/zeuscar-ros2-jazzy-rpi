# Pythonコーディングガイドライン

本プロジェクトにおけるPythonコードの記述規約を定めます。

---

## 1. 基本方針

- **PEP 8**に準拠する
- **ROS 2のコーディング規約**に従う
- 可読性と保守性を重視する

---

## 2. フォーマット

### 2.1 インデント

- スペース4つを使用（タブは使用しない）

```python
# Good
def calculate_distance(x, y):
    return math.sqrt(x**2 + y**2)

# Bad
def calculate_distance(x, y):
	return math.sqrt(x**2 + y**2)  # タブを使用
```

### 2.2 行の長さ

- 最大**100文字**（PEP 8の79文字より緩和）
- 長い行は適切に改行する

```python
# Good
result = some_function(
    argument_one,
    argument_two,
    argument_three
)

# Bad
result = some_function(argument_one, argument_two, argument_three, argument_four, argument_five)
```

### 2.3 インポート

- 標準ライブラリ、サードパーティ、ローカルの順にグループ化
- 各グループ間は空行で区切る
- アルファベット順に並べる

```python
# Good
import os
import sys

import rclpy
from rclpy.node import Node

from zeuscar_lidar.utils import calculate_offset
```

---

## 3. 命名規則

### 3.1 一般的な規則

| 対象 | 規則 | 例 |
|------|------|-----|
| モジュール | snake_case | `lidar_driver.py` |
| クラス | PascalCase | `LidarNode` |
| 関数・メソッド | snake_case | `get_scan_data()` |
| 変数 | snake_case | `scan_data` |
| 定数 | UPPER_SNAKE_CASE | `MAX_RANGE` |
| プライベート | 先頭にアンダースコア | `_internal_state` |

### 3.2 ROS 2固有の命名

| 対象 | 規則 | 例 |
|------|------|-----|
| ノード名 | snake_case | `lidar_node` |
| トピック名 | snake_case | `/scan`, `/cmd_vel` |
| サービス名 | snake_case | `/get_map` |
| パラメータ名 | snake_case | `serial_port` |

---

## 4. 型ヒント

Python 3.10+の型ヒントを使用する。

```python
from typing import Optional

def process_scan(data: list[float], threshold: float = 0.5) -> Optional[dict]:
    """スキャンデータを処理する."""
    if not data:
        return None
    return {'min': min(data), 'max': max(data)}
```

---

## 5. ドキュメンテーション

### 5.1 Docstring

Google スタイルのdocstringを使用する。

```python
def calculate_transform(x: float, y: float, theta: float) -> tuple[float, float]:
    """座標変換を計算する.

    Args:
        x: X座標 (メートル)
        y: Y座標 (メートル)
        theta: 回転角度 (ラジアン)

    Returns:
        変換後の(x, y)座標のタプル

    Raises:
        ValueError: thetaが不正な値の場合
    """
    pass
```

### 5.2 クラスのDocstring

```python
class LidarProcessor:
    """LiDARデータを処理するクラス.

    Attributes:
        frame_id: TFフレームID
        min_range: 最小測定距離 (メートル)
        max_range: 最大測定距離 (メートル)
    """

    def __init__(self, frame_id: str = 'laser_frame'):
        """初期化.

        Args:
            frame_id: TFフレームID
        """
        self.frame_id = frame_id
```

---

## 6. ROS 2固有のガイドライン

### 6.1 ノードの実装

```python
import rclpy
from rclpy.node import Node


class MyNode(Node):
    """サンプルノード."""

    def __init__(self):
        """初期化."""
        super().__init__('my_node')

        # パラメータの宣言
        self.declare_parameter('param_name', 'default_value')

        # パブリッシャー/サブスクライバーの作成
        self._publisher = self.create_publisher(String, 'topic', 10)
        self._subscription = self.create_subscription(
            String,
            'topic',
            self._callback,
            10
        )

    def _callback(self, msg):
        """コールバック関数."""
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    """エントリポイント."""
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 6.2 ロギング

```python
# Good: ROS 2のロガーを使用
self.get_logger().debug('デバッグ情報')
self.get_logger().info('情報メッセージ')
self.get_logger().warn('警告メッセージ')
self.get_logger().error('エラーメッセージ')

# Bad: printを使用
print('メッセージ')  # 使用しない
```

---

## 7. エラーハンドリング

### 7.1 例外処理

```python
# Good: 具体的な例外をキャッチ
try:
    value = int(input_string)
except ValueError as e:
    self.get_logger().error(f'Invalid input: {e}')
    return None

# Bad: 全ての例外をキャッチ
try:
    value = int(input_string)
except:  # 使用しない
    pass
```

### 7.2 早期リターン

```python
# Good: 早期リターンで可読性向上
def process_data(data):
    if data is None:
        return None

    if len(data) == 0:
        return []

    # メイン処理
    return [x * 2 for x in data]
```

---

## 8. テスト

### 8.1 テストファイルの命名

- テストファイル: `test_<module_name>.py`
- テスト関数: `test_<function_name>_<scenario>()`

```python
# test_lidar_processor.py

def test_calculate_distance_positive_values():
    """正の値での距離計算をテスト."""
    result = calculate_distance(3, 4)
    assert result == 5.0

def test_calculate_distance_zero():
    """ゼロでの距離計算をテスト."""
    result = calculate_distance(0, 0)
    assert result == 0.0
```

### 8.2 pytestの使用

```bash
# テスト実行
colcon test --packages-select zeuscar_lidar
colcon test-result --verbose
```

---

## 9. ツール

### 9.1 リンター

- **flake8**: PEP 8準拠チェック
- **mypy**: 型チェック

```bash
# flake8
flake8 --max-line-length=100 src/

# mypy
mypy src/
```

### 9.2 フォーマッター

- **black**: 自動フォーマット（推奨）

```bash
black --line-length=100 src/
```

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-19 | 初版作成 |
