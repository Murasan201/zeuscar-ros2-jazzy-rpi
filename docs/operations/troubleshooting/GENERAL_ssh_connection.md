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

---

# SSH接続トラブルシューティング（2回目: IPアドレス競合）

## 問題の概要

Windows PC (WSL2 / TeraTerm) から Raspberry Pi (192.168.11.12) への SSH 接続が失敗する。TeraTerm では「ホストに接続できませんでした」のダイアログで接続不可。

## 発生状況・再現手順

1. Windows PC の WSL2 環境から `ssh zeus@192.168.11.12` を実行 → Connection timed out
2. TeraTerm から 192.168.11.12 に接続 → 「ホストに接続できませんでした」
3. Raspberry Pi 側では正常に起動しており、SSH サービス（ポート22）も稼働中

## 調査ログ

### 1. ネットワーク疎通確認

```bash
ping -c 3 192.168.11.12
```

結果: **応答あり**だが **TTL=128**（Linux の標準は TTL=64、TTL=128 は Windows デバイスの値）

### 2. SSH接続の詳細ログ確認

```bash
ssh -v -o ConnectTimeout=5 zeus@192.168.11.12 exit 2>&1
```

結果: `connect to address 192.168.11.12 port 22: Connection timed out`

### 3. SSHポート確認

```bash
nc -zv -w5 192.168.11.12 22
```

結果: タイムアウト（ポート22に到達不可）

### 4. ARPテーブル確認（Windows側）

```
192.168.11.12    e8-4e-06-a5-cd-f5    動的
```

### 5. Raspberry Pi側のMACアドレス確認

```
d8:3a:dd:20:a9:35（Raspberry Pi Trading Ltd）
```

### 6. ネットワーク全体スキャン

192.168.11.1〜254 をスキャンしたが、SSHポート22が開いているホストは 0 台。Raspberry Pi の MAC アドレス `d8:3a:dd:*` を持つデバイスが見つからなかった。

## 原因

### IPアドレス競合

`192.168.11.12` を2台のデバイスが同時に使用していた。

| デバイス | MACアドレス | TTL |
|---------|------------|-----|
| Raspberry Pi（本物） | `d8:3a:dd:20:a9:35` | 64 |
| 別のデバイス（Windows機） | `e8:4e:06:a5:cd:f5` | 128 |

Windows PC からの通信が別のデバイス（Windows 機）に到達してしまい、Raspberry Pi には届かなかった。

### 判別ポイント

- **TTL値**: ping の TTL=128 は Windows デバイスの応答。Linux（Raspberry Pi）なら TTL=64
- **MACアドレス不一致**: ARP テーブルの MAC が Raspberry Pi のものと異なる

## 対処方法

Raspberry Pi の IP アドレスを競合しない固定 IP に変更した。

- **変更前**: `192.168.11.12`（DHCP / 競合あり）
- **変更後**: `192.168.11.20`（固定IP）

変更後、Windows PC (WSL2 / TeraTerm) から `192.168.11.20` への SSH 接続が正常に成功。

## 再発防止策

1. Raspberry Pi には**固定IPアドレスを設定**し、DHCP による IP 変動・競合を防ぐ
2. ルーター（Buffalo）の DHCP 範囲と固定 IP が重複しないように管理する
3. SSH 接続不可時は **TTL値** と **MACアドレス** を確認し、正しいデバイスに到達しているか検証する

## 現在の接続情報

- Raspberry Pi IP: **192.168.11.20**
- SSH 接続コマンド: `ssh zeus@192.168.11.20`

## 関連情報

- 発生日: 2026-02-04
- 環境: Windows 11 + WSL2 (Ubuntu) → Raspberry Pi 4 (Ubuntu 24.04)
- 解決方法: Raspberry Pi の固定 IP を `192.168.11.20` に変更
