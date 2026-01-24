# CLAUDE.md

## 業務開始時の確認事項

- 本プロジェクトの要件定義書は `docs/zeus_car_ros_2_jazzy_rpi_requirements.md` である
- 業務開始時にこの内容を確認してから開始すること

## ドキュメント管理ルール

- すべてのドキュメントは `docs/` ディレクトリに集約する
- 新規作成するドキュメントも `docs/` に配置すること
- ドキュメントを追加した際は `docs/README.md` の索引も更新すること

## 参照ファイル管理

- `reference/` ディレクトリは参照専用のファイルを配置する場所である
- このディレクトリ内のファイルは **絶対に変更してはならない**
- 参照目的でのみ使用し、編集・削除・移動は禁止する
- このディレクトリは git の管理対象外である

### reference/ の内容

- 過去に同じハードウェア（ZeusCar）で ROS 2 Humble を使用して作成したプロジェクト
- 別の管理用 Linux マシンからリモートでこのロボットを操作する構成
- 本プロジェクト（Jazzy）の設計・実装時に参照として活用する

## 開発プロセス

- 本プロジェクトの開発は `docs/agile_tdd_operating_rules.md` に従って進める
- マルチエージェント体制については `docs/multi_agent_development_flow.md` に従って進める

## 実装ルール

- 本プロジェクトは基本的にPythonで実装する
- コードの実装方法は `docs/python_coding_guidelines.md` に従う
- コードのコメントは `docs/COMMENT_STYLE_GUIDE.md` に従う

## セットアップガイド

- 環境構築作業は `docs/setup_guide.md` に記録する
- 初心者にもわかりやすい説明を心がけること
- 実施した全ての手順を漏れなく記載すること

## Git操作

- GitHubへのプッシュは `GITHUB_TOKEN` 環境変数を使用する
- プッシュコマンド例:
  ```bash
  git push https://Murasan201:${GITHUB_TOKEN}@github.com/Murasan201/zeuscar-ros2-jazzy-rpi.git main
  ```
- `gh` CLI が利用可能な場合は `gh auth status` で認証状態を確認できる
