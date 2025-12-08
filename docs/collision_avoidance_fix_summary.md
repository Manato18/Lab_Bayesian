# 衝突回避システムの修正サマリー

## 作成日時
2025-12-08

## 概要
このドキュメントは、コウモリロボットの衝突回避システムにおいて発見された3つの重大な問題とその修正内容をまとめたものです。

---

## 1. ロボットの物理構造と座標系の理解

### ロボットの構造
```
     ┌─────────────┐
    ╱               ╲
   │                 │  半径約20cmの円形
   │    ● body      │  ← 中心、回転軸
   │    (x, y)      │
   │                 │
   │         ◎ head │  ← 円周上のパルス放射位置
    ╲   (head_x, head_y)
     └─────────────┘
```

### 重要な概念
- **body (x, y)**: ロボット本体位置、**回転の軸**となる
- **head (head_x, head_y)**: パルス放射位置、円周上に位置
- **fd (flight direction)**: 頭部方向、進行方向を示す
- **pd (pulse direction)**: パルス放射方向

### 設計思想
- **衝突判定**: head位置から前方を確認（パルスを放射する位置から見る）
- **移動計算**: body位置を基準に計算（回転軸として扱う）
- **この2つの基準の違いは物理的に正しい設計**

---

## 2. 発見された問題と修正内容

### 問題1: パルス放射方向の計算ロジック ⚠️ **重要**

#### 問題の詳細
**修正前のコード** (bayes_code/agent.py:312-318):
```python
# 新しい方向を計算
new_fd = current_position['fd'] - avoid_angle  # 回避後の方向

# パルス放射方向の計算
if step < 8:
    # 最初の8ステップは進行方向より30度左
    new_pd = current_position['fd'] - 30.0  # ← 回避前のfdを使用
else:
    # ステップ8以降は回避角度の1.5倍で調整
    new_pd = current_position['fd'] + (avoid_angle * 1.5)  # ← 回避前のfdを使用
```

**問題点**:
- `current_position['fd']`（回避**前**の方向）を使用
- コメント「進行方向より30度左」と実際の動作が一致しない
- 回避角度によってパルス方向が意図せず変化する

**具体例** (右に10度回避、`avoid_angle = 10`):
- step < 8: `new_pd = 90 - 30 = 60°` → 実際は `new_fd - 20°`（回避後から20度左）
- step >= 8: `new_pd = 90 + 15 = 105°` → 実際は `new_fd + 25°`（回避後から25度右）

**修正後のコード**:
```python
# パルス放射方向の計算（回避後の方向 new_fd を基準にする）
if step < 8:
    # 最初の8ステップは回避後の進行方向より30度左（固定）
    new_pd = new_fd - 30.0
else:
    # ステップ8以降は回避後の進行方向を基準に、回避角度の半分だけ放射方向をずらす
    new_pd = new_fd + (avoid_angle * 0.5)
```

**修正の効果**:
- 常に回避**後**の方向を基準にするため動作が予測可能
- コメントと実装が一致
- step < 8: 常に`new_fd - 30°`（固定）
- step >= 8: `new_fd + avoid_angle * 0.5`（回避角度の半分）

---

### 問題2: head情報の欠落 ⚠️ **クリティカル**

#### データフローの問題
```
Step N:
1. marker_serverから取得
   → current_position = {'x', 'y', 'head_x', 'head_y', ...}  ✓ head含む

2. agent.calculate_avoidance_command(current_position)
   → new_position = {'x', 'y', 'fd', 'pd'}  ✗ headなし！

3. control_pc.update_current_position(new_position)
   → self.current_position = {'x', 'y', 'fd', 'pd'}  ✗ head情報が失われる！

Step N+1:
1. marker_server取得失敗の場合:
   → current_position = self.current_position  ✗ head情報がない！
   → agent.calculate_avoidance_command()でエラー
```

**修正前のコード** (bayes_code/agent.py:333-349):
```python
# 新しい位置を計算
move_distance_m = move_distance / 1000.0
new_x = current_position['x'] + move_distance_m * np.cos(np.deg2rad(new_fd))
new_y = current_position['y'] + move_distance_m * np.sin(np.deg2rad(new_fd))

# 新しい位置
new_position = {
    'x': float(new_x),
    'y': float(new_y),
    'fd': float(new_fd),
    'pd': float(new_pd)
}  # head情報が含まれていない！
```

**修正後のコード**:
```python
# 新しい位置を計算
move_distance_m = move_distance / 1000.0

# body位置の計算（回転軸として扱う）
new_x = current_position['x'] + move_distance_m * np.cos(np.deg2rad(new_fd))
new_y = current_position['y'] + move_distance_m * np.sin(np.deg2rad(new_fd))

# head位置も同じ移動ベクトルで計算（bodyとheadは一緒に移動する）
new_head_x = current_position['head_x'] + move_distance_m * np.cos(np.deg2rad(new_fd))
new_head_y = current_position['head_y'] + move_distance_m * np.sin(np.deg2rad(new_fd))

# 新しい位置（head情報を含む）
new_position = {
    'x': float(new_x),
    'y': float(new_y),
    'head_x': float(new_head_x),      # 追加
    'head_y': float(new_head_y),      # 追加
    'head_z': float(current_position.get('head_z', 0.0)),  # 追加
    'fd': float(new_fd),
    'pd': float(new_pd)
}
```

**修正の効果**:
- marker server取得失敗時もhead情報が維持される
- 次のステップの衝突判定が正常に動作
- データの一貫性が保たれる

---

### 問題3: 前回位置の保存不足

**修正前のコード** (bayes_code/agent.py:360-364):
```python
self.prev_x = current_position['x']
self.prev_y = current_position['y']
self.prev_fd = current_position['fd']
self.prev_pd = current_position['pd']
```

**修正後のコード**:
```python
self.prev_x = current_position['x']
self.prev_y = current_position['y']
self.prev_head_x = current_position.get('head_x', current_position['x'])  # 追加
self.prev_head_y = current_position.get('head_y', current_position['y'])  # 追加
self.prev_fd = current_position['fd']
self.prev_pd = current_position['pd']
```

**初期化も追加** (bayes_code/agent.py:45-51):
```python
# 前回の位置を保存（可視化用）
self.prev_x = None
self.prev_y = None
self.prev_head_x = None  # 追加
self.prev_head_y = None  # 追加
self.prev_fd = None
self.prev_pd = None
```

---

## 3. 可視化の改善

### posterior画像の変更点

#### 修正内容
1. **ロボット半径の動的計算**:
   ```python
   robot_radius = np.sqrt((current_pos['x'] - current_pos['head_x'])**2 +
                         (current_pos['y'] - current_pos['head_y'])**2)
   ```

2. **bodyを中心とした円の追加**:
   - 前回位置: 水色の円
   - 現在位置: 赤色の円
   - 次の位置: 緑色の円

3. **不要なプロットの削除**:
   - body位置の点（'o'マーカー）→ 円の中心で十分
   - 候補角度の円（オレンジ色）→ デバッグ用なので削除

4. **残したプロット**:
   - ✓ 衝突チェック範囲の円（青色、0/-10/30度）
   - ✓ 頭部方向（fd）の矢印
   - ✓ 放射方向（pd）の線
   - ✓ 移動経路の線

#### 可視化で使用される座標

| 画像種類 | 使用座標 | ファイル | 説明 |
|---------|---------|---------|------|
| **posterior** | body位置<br>`(x, y)` | bayes_code/agent.py<br>`_plot_posterior_global()` | bodyを中心に円を描画<br>半径はbody-head距離 |
| **frame** | head位置<br>`(head_x, head_y)` | control_pc.py<br>`plot_current_state()` | パルス放射位置を表示 |

---

## 4. 重要な理解事項

### なぜ衝突判定と移動計算で異なる基準を使うのか

**誤った考え方**:
「起点が異なるのは混乱を招く問題」→ 統一すべき

**正しい理解**:
- **衝突判定**: head位置から前方を見る必要がある（パルス放射位置から見る）
- **移動計算**: body位置を基準にする必要がある（回転軸として扱う）
- **この違いは物理的に正しい設計**

### bodyとheadの関係

```
移動時の計算:
- body移動量 = move_distance * cos(new_fd), move_distance * sin(new_fd)
- head移動量 = move_distance * cos(new_fd), move_distance * sin(new_fd)  # 同じ
- bodyとheadは同じ移動ベクトルで動く（剛体として一緒に移動）
```

---

## 5. 修正ファイル一覧

### bayes_code/agent.py

#### 修正箇所サマリー
1. **Line 45-51**: 初期化にhead位置を追加
2. **Line 313-321**: パルス放射方向の計算を修正
3. **Line 337-361**: head情報をnew_positionに追加
4. **Line 372-378**: 前回位置の保存にhead情報を追加
5. **Line 366-368**: print文を更新してhead情報も表示
6. **Line 585-588**: robot_radiusを動的に計算
7. **Line 621-629**: 衝突チェック範囲の円をrobot_radiusに変更
8. **Line 654-675**: 前回位置にbody中心の円を追加、点を削除
9. **Line 680-701**: 現在位置にbody中心の円を追加、点を削除
10. **Line 706-729**: 次の位置にbody中心の円を追加、点を削除
11. **削除**: 候補角度の円（オレンジ色）を削除

---

## 6. テスト時の確認ポイント

### 動作確認
1. **パルス放射方向**:
   - step < 8: 常に回避後の方向から30度左になっているか
   - step >= 8: 回避角度の半分だけずれているか

2. **head情報の維持**:
   - marker server取得失敗時もhead情報が残っているか
   - 次のステップで衝突判定が正常に動作するか

3. **可視化**:
   - posterior画像にbody中心の円が表示されているか
   - 円の半径がbody-head距離になっているか

### ログ出力例
```
[Visualization] ロボット半径（body-head距離）: 0.200m
[移動指令計算] ステップ3: パルス放射方向を回避後の方向から左30度固定 (new_fd=90.0° → pd=60.0°)
[移動指令計算] 新body位置: (1.234, 2.345), 新head位置: (1.434, 2.345)
[移動指令計算] 新方向: fd=90.0度, pd=60.0度
```

---

## 7. 今後の拡張性

### 可視化の拡張
- head位置の軌跡を表示（現在は履歴を保存済み）
- bodyとheadを線で結ぶ（ロボットの向きをより明確に）

### データの拡張
- z座標の活用（現在は維持のみ）
- 速度情報の追加

---

## 付録: 角度の符号規則

```
符号規則（統一済み）:
- 負の角度 = 左回転
- 正の角度 = 右回転

例:
- avoid_angle = -10  → 左に10度回避
- avoid_angle = 10   → 右に10度回避
- new_pd = new_fd - 30  → 進行方向から30度左
```

---

## まとめ

今回の修正により、以下が改善されました：

1. **パルス放射方向**: 回避後の方向を基準にするため、動作が予測可能に
2. **データの一貫性**: head情報が維持され、marker server失敗時も動作
3. **可視化**: bodyを中心とした円でロボットの大きさと位置が明確に
4. **コードの可読性**: コメントと実装が一致し、理解しやすく

これらの修正により、システムの信頼性と保守性が大幅に向上しました。
