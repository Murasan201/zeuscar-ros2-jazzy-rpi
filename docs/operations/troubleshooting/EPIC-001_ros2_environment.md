# EPIC-001: ROS 2環境構築 トラブルシューティング

## 概要

EPIC-001（ROS 2環境構築）で発生したエラーと解決策をまとめる。

---

## TS-001: ROS 2リポジトリ設定ファイルが空になる

### 問題の概要

`sudo tee`コマンドでROS 2リポジトリを追加しようとしたが、ファイルが空のまま作成された。

### 発生状況・再現手順

```bash
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

上記コマンド実行後、`/etc/apt/sources.list.d/ros2.list`が空ファイルになる。

### 調査ログ

```bash
cat /etc/apt/sources.list.d/ros2.list
# （空の出力）
```

### 原因

特定の環境で`> /dev/null`のリダイレクトが`tee`の出力だけでなく入力にも影響を与えた可能性がある。

### 対処方法

リダイレクトなしで再実行：

```bash
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### 再発防止策

- `tee`コマンド使用時は、出力リダイレクトを省略するか、実行後にファイル内容を確認する
- セットアップガイドのコマンドはリダイレクトなしの形式で記載

---

## TS-002: colconコマンドが見つからない

### 問題の概要

`colcon build`を実行しようとしたが、`colcon: command not found`エラーが発生した。

### 発生状況・再現手順

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

エラー出力：
```
/bin/bash: line 1: colcon: command not found
```

### 調査ログ

```bash
which colcon
# （出力なし）

dpkg -l | grep colcon
# （出力なし）
```

### 原因

`ros-jazzy-desktop`パッケージにはcolconが含まれていない。colconは別途インストールが必要。

### 対処方法

```bash
sudo apt install -y python3-colcon-common-extensions
```

### 再発防止策

- セットアップガイドにcolconインストール手順を明記（Section 3.2に追加済み）

---

## TS-003: ros2 --versionが認識されない

### 問題の概要

ROS 2のインストール確認のため`ros2 --version`を実行したが、認識されなかった。

### 発生状況・再現手順

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

### 調査ログ

```bash
ros2 --help
# --versionオプションがリストに存在しない
```

### 原因

`ros2`コマンドには`--version`オプションが存在しない。これはROS 2 CLIの仕様。

### 対処方法

以下のコマンドでインストールを確認：

```bash
# トピック一覧（何も起動していなければ空のリスト）
ros2 topic list

# またはデモノードで確認
ros2 run demo_nodes_cpp talker
```

### 再発防止策

- セットアップガイドでは`ros2 topic list`またはデモノードでの確認方法を推奨

---

## 更新履歴

| 日付 | 内容 |
|---|---|
| 2026-01-12 | 初版作成（TS-001〜003） |
