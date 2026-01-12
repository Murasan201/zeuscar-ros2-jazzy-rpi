# マルチエージェント開発運用ルール・仕様書

このドキュメントは、**AI エージェントのサブエージェント機能を活用し、複数エージェントで開発プロジェクトをアジャイル的に進めるための仕様書**です。

本書では、メインエージェント（PM）/ サブエージェント（機能担当）の構成と、Git 運用・自動化スクリプト・PR 運用までの一連の流れを整理します。

1. 背景・考え方（なぜこの構成にするのか）
2. 具体的な運用ルール
3. 実際に動かすための設定ファイル・スクリプト・プロンプト

---

# 1. なぜ「サブエージェントによるマルチエージェント開発」なのか

通常の LLM を使って開発を進める場合、**1つのチャットスレッドで設計・実装・レビュー・テストを同一モデルに任せる**ことになります。

しかし、実際のソフトウェア開発は、

- タスク分割（Backlog 生成）
- PM によるタスク管理
- 担当者ごとの実装
- PR 提出とレビュー
- CI による検証

という **複数の役割が並行して進む構造**です。

そこで、使用するエージェント基盤の**サブエージェント起動機能**を活用し、**バックログIDごとに専任のサブエージェントを立てる**ことで、以下を実現します。

- メインエージェント（PM）が全体のタスク管理を行う
- Backlog ID ごとにサブエージェントが独立して実装を進める
- サブエージェントは**コンテキストが完全に分離**され、専用ディレクトリと専用ブランチで安全に作業する
- PM は複数のサブエージェントを並列実行し、開発速度と品質を両立する

---

# 2. 全体アーキテクチャ

## 2.1 エージェントの役割

### メインエージェント（PM）

人間のプロジェクトマネージャーに相当するエージェントです。役割は次の通りです。

- バックログ ID ごとにタスクを配分する
- サブエージェントを起動し、進捗を集約・可視化する
- 設計 → 実装 → テスト のフェーズをコントロールする
- PR の作成・レビュー・マージを管理する

### サブエージェント（機能担当 / Assignee）

1つのバックログ ID（例: `PROJ-101`）を担当するエージェントです。サブエージェントは**独立したコンテキスト**で動作し、以下を実行します。

- 自分に割り当てられた Backlog ID の要件を理解し、設計案を作成する
- 専用ディレクトリ内でコードやテストを実装する
- 作業の結果と次アクションを所定形式で PM に報告する

## 2.2 サブエージェント起動機能の特徴

```
サブエージェント起動機能呼び出し（例）:
  subagent_type: "general-purpose" または専用タイプ
  prompt: バックログID、作業指示、制約条件を含む詳細なプロンプト
  run_in_background: true（並列実行時）
```

- **コンテキスト分離**: サブエージェントはメインエージェントの会話履歴を引き継がない
- **並列実行**: 複数のサブエージェントを同時起動可能（`run_in_background: true`）
- **結果取得**: 利用環境の結果取得機能で最終レポートを取得

> パラメータ名や呼び出し方法は利用するエージェント基盤の仕様に合わせて読み替える。

## 2.3 Git／リポジトリの考え方

- 1つのリポジトリに全てのコードを集約（fork ベースにはしない）
- Backlog ID ごとに 1 ブランチ（例: `feature/PROJ-101`）を割り当てる
- `git worktree` を用いて Backlog ID ごとの作業ディレクトリを分離する

これにより、

- CI/CD の設定や Secrets を一元管理できる
- Backlog ID 単位で差分や履歴を追いやすい
- 各サブエージェントが担当範囲を越えてファイルを変更しにくくなる

## 2.4 データフロー（1チケットのライフサイクル）

```
┌─────────────────────────────────────────────────────────────────────┐
│  1. Backlog 作成  →  2. worktree 準備  →  3. サブエージェント起動   │
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  4. 実装作業  →  5. PR 作成  →  6. レビュー（人間 + AI エージェント）│
└─────────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│  7. 承認・マージ  →  8. バックログ更新  →  完了                     │
└─────────────────────────────────────────────────────────────────────┘
```

### レビュープロセスの役割分担

| レビュー観点 | 人間 | メインエージェント | 別モデル（任意） |
|-------------|------|--------------------|------------------|
| 設計意図・要件適合 | ✅ | - | - |
| コード品質・可読性 | ✅ | ✅ | ✅ |
| 規約・スタイル準拠 | - | ✅ | ✅ |
| ロジック・アルゴリズム検証 | ✅ | ✅ | ✅ |
| セキュリティ | ✅ | ✅ | ✅ |
| CI 結果確認 | ✅ | ✅ | - |
| 最終承認 | ✅ | - | - |

---

# 3. Git 運用規約（単一リポ＋ブランチ＋worktree）

## 3.1 ブランチ命名規則

Backlog ID とブランチ名は 1 対 1 に対応させる。

```text
feature/PROJ-101-core-module
fix/PROJ-201-hotfix
refactor/PROJ-340-cleanup
```

- 先頭の種別（feature/fix/refactor）は任意だが、Backlog ID を必ず含める
- Backlog ID を Git の共通キーにすることで追跡・自動化が容易になる

## 3.2 worktree 構造

```text
repo/
 ├─ .git
 ├─ PROJ-101/   ← feature/PROJ-101 の worktree（サブエージェントの作業ディレクトリ）
 ├─ PROJ-102/
 ├─ PROJ-103/
 └─ ...
```

作成コマンド例：

```bash
git fetch origin

git worktree add ./PROJ-101 feature/PROJ-101
git worktree add ./PROJ-102 feature/PROJ-102
git worktree add ./PROJ-103 feature/PROJ-103
```

## 3.3 PR の運用ルール

- **1 Backlog ID = 1 PR** を基本とする
- PR タイトルには必ず Backlog ID を含める（例: `PROJ-101: core-module 実装`）
- 原則として Squash Merge（1 PR = 1 コミット）で履歴を整理

### PR テンプレート（例: `.github/pull_request_template.md`）

```markdown
## 背景（Backlog ID）

## 変更内容

## 影響範囲

## 動作確認手順

## 残課題・リスク
```

## 3.4 保護ブランチ設定

- `main` に直接 push することは禁止
- `main` にマージするためには PR が必須
- CI（lint / unit test / build）を通過した PR のみマージ可能にする

## 3.5 CODEOWNERS

```text
*         @project-manager
src/api/* @backend-lead
src/ui/*  @frontend-lead
```

## 3.6 ホスティングサービス認証と権限管理

- エージェントがホスティングサービスへ push / fetch する際は、最小権限の PAT / Deploy Token を利用する
- トークンは環境変数（例: `VCS_TOKEN`）または credential helper 経由で供給する
- HTTPS 利用時は `https://<user>:${VCS_TOKEN}@vcs.example.com/<org>/<repo>.git` 形式を用いる
- トークンは共有しない。不要になったトークンは速やかに失効させる
- Backlog 単位のトークン命名は `VCS_TOKEN_<BacklogID>` のように統一し、担当と対応付ける

---

# 4. サブエージェント起動と管理

## 4.1 サブエージェント起動パラメータ（例）

| パラメータ | 必須 | 説明 |
|-----------|------|------|
| `description` | ✅ | 3-5語のタスク概要（例: `PROJ-101 設計フェーズ実行`） |
| `prompt` | ✅ | サブエージェントへの詳細な指示（役割、制約、期待する出力形式を含む） |
| `subagent_type` | ✅ | エージェントタイプ（例: `general-purpose`） |
| `run_in_background` | - | `true` で並列実行、`false` で同期実行 |
| `model` | - | 利用環境でサポートされるモデル識別子 |

> パラメータ名は利用環境に合わせて読み替える。

## 4.2 サブエージェント起動例

```
サブエージェント起動機能呼び出し:
  description: "PROJ-101 core-module 実装"
  subagent_type: "general-purpose"
  run_in_background: false
  prompt: |
    あなたは PROJ-101（core-module 実装）の担当エージェントです。

    【作業ディレクトリ】
    /home/user/project/PROJ-101/

    【Git ブランチ】
    feature/PROJ-101

    【タスク】
    1. core-module の雛形を作成
    2. 設定ファイルを追加
    3. 動作確認用のテストを追加

    【制約】
    - 作業ディレクトリ外のファイルを編集しない
    - main ブランチを直接操作しない

    【成果物レポート】
    作業完了後、以下の形式で報告してください：
    ## 進捗状況
    ## 成果物一覧
    ## 次のアクション
    ## リスク・懸念事項
```

## 4.3 並列実行と結果取得

1. **起動**: 複数のサブエージェントを `run_in_background: true` で同時呼び出し
2. **監視**: 結果取得機能で各サブエージェントの状態を確認
3. **結果取得**: 完了したサブエージェントから順に結果を取得

---

# 5. worktree 自動生成スクリプト

```bash
#!/usr/bin/env bash
set -euo pipefail

# 対象の Backlog ID をここに列挙
BACKLOG_IDS=("PROJ-101" "PROJ-102" "PROJ-103")

REPO_ROOT="$(git rev-parse --show-toplevel)"

echo "=== Worktree セットアップ開始 ==="

for id in "${BACKLOG_IDS[@]}"; do
  BRANCH="feature/${id}"
  DIR="${REPO_ROOT}/${id}"

  echo "Processing ${id}..."

  if git ls-remote --heads origin "${BRANCH}" | grep -q "${BRANCH}"; then
    echo "  Remote branch exists: ${BRANCH}"
    git fetch origin "${BRANCH}"
  else
    echo "  Creating new branch: ${BRANCH}"
    git branch "${BRANCH}" 2>/dev/null || true
  fi

  if [ -d "$DIR" ]; then
    echo "  Worktree already exists: ${DIR}"
  else
    git worktree add "$DIR" "$BRANCH"
    echo "  Created worktree: ${DIR}"
  fi
done

echo ""
echo "=== セットアップ完了 ==="
git worktree list
```

---

# 6. サブエージェント用プロンプトテンプレート

## 6.0 【必須】サブエージェント起動前チェックリスト（PM 用）

```bash
# 1. worktree が存在するか確認
git worktree list | grep "PROJ-101"

# 2. feature ブランチが存在するか確認
git branch -a | grep "feature/PROJ-101"

# 3. worktree が正しいブランチを指しているか確認
git -C ./PROJ-101 branch --show-current
# 期待出力: feature/PROJ-101
```

### 未準備の場合の対応

```bash
git branch feature/PROJ-101 2>/dev/null || true
git worktree add ./PROJ-101 feature/PROJ-101
```

## 6.1 基本プロンプト構造

```markdown
あなたは {{BACKLOG_ID}}（{{TASK_TITLE}}）の担当エージェントです。

## 作業環境
- **作業ディレクトリ**: {{WORK_DIR}}
- **Git ブランチ**: {{GIT_BRANCH}}
- **参照ドキュメント**: docs/operations/agile/assignments/{{BACKLOG_ID}}.md

## タスク内容
{{TASK_DESCRIPTION}}

## 受け入れ条件（AC）
{{ACCEPTANCE_CRITERIA}}

## 制約事項
- 作業ディレクトリ外のファイルを編集しない
- main ブランチを直接操作しない
- Secrets や認証情報を生成・外部送信しない
- 外部検証が必要な項目は PM へエスカレーション
- **ジャーナル・トラブルシューティングは追記のみ（過去記載の削除・上書き禁止）**

## 成果物レポート形式
作業完了後、以下の Markdown 形式で報告してください：

### 進捗状況
- フェーズ: design / impl / test / done / blocked
- 完了率: XX%

### 成果物一覧
| 種類 | パス | 説明 |
|------|------|------|
| file | path/to/file | 説明 |

### 次のアクション
- [ ] アクション1
- [ ] アクション2

### リスク・懸念事項
- リスク1
- リスク2

### PM へのエスカレーション事項
- （外部検証が必要な場合など）
```

## 6.2 プロンプト例

```markdown
あなたは PROJ-101（core-module 実装）の担当エージェントです。

## 作業環境
- **作業ディレクトリ**: /home/user/project/PROJ-101/
- **Git ブランチ**: feature/PROJ-101
- **参照ドキュメント**: docs/operations/agile/assignments/PROJ-101.md

## タスク内容
1. src/modules/core/ の雛形を作成
2. config/core.yml を追加
3. tests/core/ に最低限のテストを追加

## 受け入れ条件（AC）
- `scripts/build.sh` が成功する
- `scripts/test.sh` が成功する

## 制約事項
- 作業ディレクトリ外のファイルを編集しない
- main ブランチを直接操作しない
- 外部検証が必要な場合は PM へエスカレーション
```

## 6.3 作業完了フロー

1. **設計→実装→テスト**の各フェーズで `docs/operations/agile/journals/{{BACKLOG_ID}}.md` へログを追記
2. DoD を満たしたら `git commit`（Conventional Commit 形式）を実行
3. 必要に応じて `git push origin feature/{{BACKLOG_ID}}`
4. **最終レポート**を Markdown 形式で返してセッション終了
5. `main` へのマージは**PM のみ**が実行（サブエージェントは所感を伝えるのみ）

## 6.4 ジャーナル・トラブルシューティング記録ルール（必須）

**⚠️ 重要**: 以下のドキュメントは**追記のみ**とし、過去の記載を**絶対に削除・上書きしない**こと。

| ドキュメント種別 | パス | 用途 |
|------------------|------|------|
| スプリントジャーナル | `docs/operations/agile/sprint-journal.md` | 全体の進捗・決定事項・計画の経過記録 |
| 機能別ジャーナル | `docs/operations/agile/journals/<BacklogID>.md` | 各バックログの作業ログ |
| トラブルシューティング | `docs/operations/troubleshooting/troubleshooting_<BacklogID>_*.md` | 障害・課題の対応記録 |

---

# 7. PM からの並列実行パターン

## 7.1 並列起動の実行例

```
サブエージェント起動機能呼び出し（並列）:
  description: "PROJ-101 API 実装"
  prompt: "（PROJ-101 用プロンプト）"
  run_in_background: true

  description: "PROJ-102 CLI 改善"
  prompt: "（PROJ-102 用プロンプト）"
  run_in_background: true

  description: "PROJ-103 ドキュメント更新"
  prompt: "（PROJ-103 用プロンプト）"
  run_in_background: true
```

## 7.2 PM の統合レポート例

```markdown
# スプリント進捗サマリー（YYYY-MM-DD）

| ID | タイトル | フェーズ | 完了率 | 次アクション |
|----|----------|----------|--------|--------------|
| PROJ-101 | API 実装 | impl | 60% | 統合テスト追加 |
| PROJ-102 | CLI 改善 | design | 30% | 設計レビュー |
| PROJ-103 | ドキュメント更新 | blocked | 20% | 仕様確認待ち |

## エスカレーション事項
- PROJ-101: 外部環境の検証が必要
- PROJ-103: 仕様決定が必要
```

---

# 8. CI/CD 仕様

## 8.1 推奨 CI 構成

| ワークフロー | トリガー | 内容 |
|-------------|----------|------|
| `lint.yml` | PR 作成時 | スタイル・静的解析 |
| `build.yml` | PR 作成時 | ビルド検証 |
| `test.yml` | PR 作成時 | ユニットテスト |
| `deploy.yml` | main マージ時 | パッケージ生成・デプロイ |

## 8.2 CI 設定例（汎用）

```yaml
stages:
  - lint
  - build
  - test

lint:
  stage: lint
  script:
    - ./scripts/lint.sh

build:
  stage: build
  script:
    - ./scripts/build.sh

test:
  stage: test
  script:
    - ./scripts/test.sh
```

---

# 9. 成果物レポート仕様

## 9.1 サブエージェント最終レポート形式

```markdown
# 成果物レポート: {{BACKLOG_ID}}

## 進捗状況
- **フェーズ**: impl
- **完了率**: 70%
- **ステータス**: in_progress

## 成果物一覧
| 種類 | パス | 説明 |
|------|------|------|
| file | src/modules/core/index.ts | コアロジック |
| config | config/core.yml | 設定ファイル |
| test | tests/core/core.test.ts | テスト |
| spec | docs/operations/specs/{{BACKLOG_ID}}_spec.md | 仕様書 |

## 実行したコマンド
```bash
git add src/modules/core config/core.yml tests/core
git commit -m "feat(core): 初期実装 ({{BACKLOG_ID}})"
```

## 次のアクション
- [ ] 統合テスト追加
- [ ] レビュー対応

## リスク・懸念事項
- 外部依存の検証が未完了

## PM へのエスカレーション
- 外部検証環境の予約が必要
```

---

# 10. 本仕様のゴール

- **サブエージェント**が Backlog ID ごとに"担当者"として独立して動作する
- **メインエージェント（PM）**が全体のタスクを統括し、進捗を可視化できる
- **コンテキスト分離**により、各サブエージェントが安全に作業できる
- **Git worktree** によって作業範囲が物理的に分離される
- **CI/CD** によって品質を一定以上に保てる
- **Markdown レポート**で、進捗や成果物を人間にも読みやすい形式で記録できる
