# EPIC-003: TF/URDF設計 トラブルシューティング

## 概要

EPIC-003（TF/URDF設計）で発生した問題と対処法を記録する。

---

## 問題一覧

### TSB-003-001: xacroパッケージ未インストール

**発生日**: 2026-01-19

**症状**:
```
xacro: command not found
```

URDFファイル（.xacro形式）の構文チェック時にxacroコマンドが見つからない。

**原因**:
ros-jazzy-desktopにはxacroパッケージが含まれていない。

**解決策**:
```bash
sudo apt install -y ros-jazzy-xacro
```

**確認方法**:
```bash
source /opt/ros/jazzy/setup.bash
xacro --version
```

**備考**:
- robot_state_publisherは別途インストール済みだった
- xacroはURDF/xacroファイルを使用する際に必要
- package.xmlにexec_dependとしてxacroを追加済み

---

### TSB-003-002: robot_descriptionパラメータのYAMLパースエラー

**発生日**: 2026-01-19

**症状**:
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Unable to parse the value of parameter robot_description as yaml. If the parameter is meant to be a string, try wrapping it in launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```

`ros2 launch zeuscar_description description.launch.py` 実行時にエラーが発生。

**原因**:
ROS 2 Jazzyでは、`Command`サブスティチューションで生成した文字列をそのままパラメータに渡すと、YAMLとしてパースしようとしてエラーになる。

**解決策**:
`ParameterValue`でラップし、`value_type=str`を指定する。

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
- ROS 2 Jazzy (Iron以降) で必要になった変更
- 公式ドキュメントでも推奨されている方法

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-19 | 初版作成、TSB-003-001を記録 |
| 2026-01-19 | TSB-003-002を追加（ParameterValueラップ） |
