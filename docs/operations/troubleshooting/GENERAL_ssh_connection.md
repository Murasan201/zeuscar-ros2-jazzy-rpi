# SSH接続トラブルシューティング

## 問題の概要

Windows PC (WSL2) から Raspberry Pi (192.168.11.12) への SSH 接続が失敗する。

## 発生状況・再現手順

1. Windows PC の WSL2 環境から `ssh zeus@192.168.11.12` を実行
2. 接続エラーが発生

## 調査ログ

### 1. ネットワーク疎通確認

```bash
ping -c 3 192.168.11.12
```

結果: **正常** - パケットロスなし

### 2. SSH接続の詳細ログ確認

```bash
ssh -v zeus@192.168.11.12 exit 2>&1
```

結果: 以下の2つのエラーを確認

#### エラー1: Host key verification failed

```
debug1: Server host key: ssh-ed25519 SHA256:wvKz+gEuGRbMBmU9XmCfYDxmrWAWJX+x9z3mNWMlCdo
debug1: hostkeys_find_by_key_hostfile: hostkeys file /home/win/.ssh/known_hosts2 does not exist
Host key verification failed.
```

#### エラー2: Permission denied

```
zeus@192.168.11.12: Permission denied (publickey,password).
```

### 3. known_hosts確認

```bash
grep "192.168.11.12" ~/.ssh/known_hosts
```

結果: エントリなし

### 4. SSH鍵確認

```bash
ls -la ~/.ssh/*.pub
```

結果: 公開鍵なし

## 原因

### 原因1: ホストキー未登録

`~/.ssh/known_hosts` に Raspberry Pi のホストキーが登録されていなかった。

### 原因2: SSH鍵未設定

Windows PC 側に SSH 鍵ペアが存在せず、公開鍵認証ができなかった。

## 対処方法

### 対処1: ホストキーの登録

```bash
ssh-keyscan -H 192.168.11.12 >> ~/.ssh/known_hosts
```

### 対処2: SSH鍵の設定（2つの方法）

**方法A: TeraTerm等のSSHクライアントで鍵生成・転送**

1. TeraTermで Raspberry Pi に接続（パスワード認証）
2. TeraTerm の機能で SSH 鍵ペアを生成
3. 公開鍵を Raspberry Pi の `~/.ssh/authorized_keys` に登録

**方法B: コマンドラインで設定**

```bash
# SSH鍵ペアを生成
ssh-keygen -t ed25519

# 公開鍵をRaspberry Piに転送
ssh-copy-id zeus@192.168.11.12
```

## 再発防止策

1. 新しい Raspberry Pi や OS 再インストール時は、ホストキーの更新を忘れずに行う
2. SSH 鍵ペアは開発環境構築時に作成しておく
3. `docs/setup_guide.md` に SSH 接続設定手順を追記することを検討

## 関連情報

- 発生日: 2026-02-03
- 環境: Windows 11 + WSL2 (Ubuntu) → Raspberry Pi 4 (Ubuntu 24.04)
- 解決方法: TeraTerm で SSH 鍵を設定
