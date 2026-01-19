# EPIC-004: SLAM構築 トラブルシューティング

## 概要

EPIC-004（SLAM構築）で発生した問題と対処法を記録する。

---

## 問題一覧

### TSB-004-001: オドメトリソースがない

**発生日**: 2026-01-19（予防的記録）

**症状**:
slam_toolboxを起動すると、以下のようなTF変換エラーが発生する。

```
[slam_toolbox]: Could not get transform from base_footprint to odom
[slam_toolbox]: Lookup would require extrapolation into the past
```

**原因**:
オドメトリソース（/odomトピックまたはodom→base_footprintのTF）が提供されていない。

**解決策**:

方法1: IMU + ホイールオドメトリを統合
- ICM42688 IMUセンサーを接続
- ホイールエンコーダからオドメトリを計算
- robot_localizationパッケージでセンサーフュージョン

方法2: ホイールオドメトリのみ
- モーターエンコーダからオドメトリを計算
- /odomトピックをパブリッシュ

方法3: スキャンマッチングのみ（非推奨）
- オドメトリなしでもslam_toolboxは動作するが、精度が大幅に低下
- 高速移動時にマップが破綻しやすい

**備考**:
- ICM42688 IMUセンサーは発注済み（到着待ち）
- IMU統合後に本格的なSLAMテストを実施予定

---

### TSB-004-002: マップが更新されない

**発生日**: -（予防的記録）

**症状**:
ロボットを移動させてもマップが拡張されない。

**原因候補**:

1. minimum_travel_distance / minimum_travel_heading の値が大きすぎる
2. スキャンマッチングの失敗
3. TFツリーの不整合

**解決策**:

1. slam_params.yaml のパラメータを調整:
```yaml
minimum_travel_distance: 0.1  # 0.3から0.1に減少
minimum_travel_heading: 0.1   # 0.3から0.1に減少
```

2. debug_logging を有効にして原因を調査:
```yaml
debug_logging: true
```

3. TFツリーを確認:
```bash
ros2 run tf2_tools view_frames
```

---

### TSB-004-003: マップにノイズが多い

**発生日**: -（予防的記録）

**症状**:
生成されるマップにノイズや不正確な壁が含まれる。

**原因候補**:

1. LiDARのノイズ
2. 動的物体（人など）の干渉
3. ループクロージャの誤検出

**解決策**:

1. 解像度を下げる:
```yaml
resolution: 0.1  # 0.05から0.1に変更
```

2. 相関パラメータを調整:
```yaml
link_match_minimum_response_fine: 0.2  # 0.1から0.2に上げる
```

3. 動的物体がない環境でマッピングする

---

## slam_toolboxの主要パラメータ

| パラメータ | 推奨値 | 説明 |
|-----------|--------|------|
| resolution | 0.05 | マップ解像度（m/pixel） |
| max_laser_range | 12.0 | LiDAR最大距離（RPLIDAR A1M8） |
| minimum_travel_distance | 0.3 | 更新に必要な最小移動距離 |
| minimum_travel_heading | 0.3 | 更新に必要な最小回転角 |
| do_loop_closing | true | ループクロージャを有効化 |

---

## 動作確認コマンド

```bash
# トピックの確認
ros2 topic list | grep -E "(scan|map|odom)"

# TFツリーの確認
ros2 run tf2_tools view_frames

# マップの確認
ros2 topic echo /map --once | head -20
```

---

## 更新履歴

| 日付 | 内容 |
|------|------|
| 2026-01-19 | 初版作成（予防的トラブルシューティング） |
