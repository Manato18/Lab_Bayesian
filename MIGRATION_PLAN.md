# シミュレーション環境から実機環境への移行計画

## 1. 現在のシステム構成（シミュレーション環境）

### 1.1 システム概要
3つのプロセスで構成されるシミュレーションシステム：

```
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│  env_server.py  │◄────►│ control_pc.py   │◄────►│robot_simulator.py│
│  (Port: 6000)   │      │  (Port: 6001)   │      │                 │
└─────────────────┘      └─────────────────┘      └─────────────────┘
  環境情報管理              ベイズ推論・回避計算        ロボットシミュレータ
```

### 1.2 各コンポーネントの役割

#### env_server.py（環境情報サーバー）
- **ポート**: 6000
- **役割**:
  - ロボットの位置情報（x, y, fd, pd）を管理
  - 障害物の位置情報を提供
  - CSVファイル（`chain_position_y.csv`）から障害物データを読み込み
- **提供API**:
  - `get_robot_position`: ロボット位置取得
  - `get_obstacles`: 障害物情報取得
  - `update_position`: ロボット位置更新

#### control_pc.py（制御PC）
- **ポート**: 6001
- **役割**:
  - ベイズ推論による事後分布計算
  - 回避方向・移動距離の計算
  - 物体定位結果から環境認識
  - 可視化（BatVisualizer）
- **処理フロー**:
  1. robot_simulatorからエコーデータ（_data_Multi.dat）を受信
  2. Localizerで物体定位を実行
  3. ベイズ推論で事後分布を更新
  4. 回避指令を計算
  5. env_serverに新しい位置を更新
  6. 移動指令をrobot_simulatorに返す

#### robot_simulator.py（ロボットシミュレータ）
- **役割**:
  - 固定データ（`robot_data/wall_000/goldorak/`）から_data_Multi.datファイルを読み込み
  - control_pcに送信して移動指令を受信
  - 位置情報は持たない（env_serverが管理）
- **通信プロトコル**:
  - 大容量データ送信（Base64エンコード、サイズプリフィックス付き）

---

## 2. 実機環境の構成要素

### 2.1 OptiTrackマーカートラッキングシステム

#### marker_tracker.py（マーカートラッカー）
- **ポート**: 6000（HTTP）
- **動作モード**:
  - `server`: HTTPサーバーで最新データを提供
  - `print`: コンソールに定期出力
  - `test`: ダミーデータ提供（開発・テスト用）
- **提供API**:
  - `GET /latest`: 最新スナップショット全体
  - `GET /marker_set?name=<name>`: 特定マーカーセットのみ
- **取得可能データ**:
  - `robot_head`: ロボットのマーカーセット（3個のマーカー）
  - その他のマーカーセット（障害物など）
  - 剛体（Rigid Body）の位置・姿勢

#### marker_test.py（テストクライアント）
- marker_tracker.pyからデータを取得して表示するテストツール
- 実装例を確認できる

### 2.2 実機ロボット（Golang実装）

#### 通信プロトコル
- **サーバーアドレス**: `192.168.11.156:60000`
- **送信データ**（ロボット→サーバー）:
  ```json
  [
    [crosscor_l],  // 左耳の相互相関データ
    [crosscor_r]   // 右耳の相互相関データ
  ]
  ```
- **受信データ**（サーバー→ロボット）:
  ```json
  {
    "Time": "2025-01-16T10:30:00",
    "NextMove": 0.15,      // 移動距離[m]
    "NextAngle": 0.3       // 回避角度[rad]
  }
  ```

#### ロボットの動作フロー
1. エコーセンシング実行（`echolyzer.DoSensing()`）
2. 相互相関データをサーバーに送信
3. サーバーから移動指令を受信
4. 移動実行（`mover.DoSingleMove()`）
5. 音声パルス発生（`caller.CreateCall()`）

### 2.3 現在のダミーサーバー（Python）
- **ポート**: 60000
- **現状**: ランダムな移動指令を返すだけ
- **今後**: 実際のベイズ推論を実装する必要あり

---

## 3. 移行の目標と変更点

### 3.1 移行の目標
1. **位置情報の取得をOptiTrackに置き換え**
   - env_server.pyの位置管理機能 → marker_tracker.pyから取得
   - CSVファイルの障害物情報 → マーカーセットから取得

2. **ロボットシミュレータを実機に置き換え**
   - robot_simulator.py → 実機ロボット（Golang）
   - 固定データファイル → 実際のエコーデータ（相互相関）

3. **ダミーサーバーを実際のベイズ推論サーバーに置き換え**
   - 現在のダミーサーバー → control_pc.pyの機能を統合

### 3.2 主要な変更点

#### 変更1: 位置情報取得の変更
**Before（シミュレーション）**:
```python
# env_serverから取得
response = env_server.get_robot_position()
# {'x': 1.5, 'y': 2.0, 'fd': 90, 'pd': 0}
```

**After（実機）**:
```python
# marker_trackerから取得
response = requests.get('http://localhost:6000/marker_set?name=robot_head')
markers = response.json()['markers']  # [[x1,y1,z1], [x2,y2,z2], [x3,y3,z3]]
# 3点の重心からロボット位置を計算
robot_x = sum(m[0] for m in markers) / 3
robot_y = sum(m[1] for m in markers) / 3
```

#### 変更2: 障害物情報取得の変更
**Before（シミュレーション）**:
```python
# CSVファイルから読み込み
chain_loc = pd.read_csv('chain_position_y.csv')
obs_x = chain_loc["X"].values
obs_y = chain_loc["Y"].values
```

**After（実機）**:
```python
# marker_trackerから取得（例: 'obstacles'マーカーセット）
response = requests.get('http://localhost:6000/marker_set?name=obstacles')
obstacles = response.json()['markers']
obs_x = [m[0] for m in obstacles]
obs_y = [m[1] for m in obstacles]
```

#### 変更3: エコーデータ入力の変更
**Before（シミュレーション）**:
```python
# _data_Multi.dat ファイルから読み込み
detections = localizer.localize(save_path)
# [{'distance': mm, 'angle': deg, 'intensity': float}, ...]
```

**After（実機）**:
```python
# 実機から相互相関データを受信
data = receive_from_robot()  # [[crosscor_l], [crosscor_r]]
# Localizerで物体定位
detections = localizer.localize_from_crosscor(data[0], data[1])
```

#### 変更4: 移動指令の形式変更
**Before（シミュレーション）**:
```python
# control_pcが返す形式
{
  'avoidance_direction': 30.0,  # 度
  'move_distance': 150.0,       # mm
  'pulse_direction': 0.0        # 度
}
```

**After（実機）**:
```python
# 実機ロボットが期待する形式
{
  'Time': '2025-01-16T10:30:00',
  'NextMove': 0.15,      # m（150mm → 0.15m）
  'NextAngle': 0.524     # rad（30度 → 0.524rad）
}
```

---

## 4. 新しいアーキテクチャの提案

### 4.1 実機環境のシステム構成

```
┌─────────────────────┐
│ marker_tracker.py   │  OptiTrackから位置情報を取得
│ (HTTP: 6000)        │  - robot_head マーカーセット
│ [test mode可能]     │  - obstacles マーカーセット
└──────────┬──────────┘
           │ HTTP GET
           ▼
┌─────────────────────┐      ┌─────────────────────┐
│ bayes_server.py     │◄────►│  実機ロボット       │
│ (TCP: 60000)        │      │  (Golang)           │
│                     │      │  IP: 192.168.11.x   │
└─────────────────────┘      └─────────────────────┘
  ベイズ推論サーバー            エコーセンシング
  - 位置情報取得               - 相互相関計算
  - 物体定位計算               - 移動実行
  - 事後分布更新               - パルス発生
  - 回避指令計算
```

### 4.2 新しいbayes_server.pyの設計

#### 統合する機能
1. **control_pc.pyの機能**:
   - ベイズ推論（Bayesian）
   - 物体定位（Localizer）
   - 回避計算（Agent）
   - 可視化（BatVisualizer）

2. **marker_trackerからの位置情報取得**:
   - ロボット位置（robot_head）
   - 障害物位置（obstacles等）

3. **実機ロボットとの通信**:
   - TCPソケット（ポート60000）
   - 相互相関データ受信
   - 移動指令送信

#### 処理フロー
```
1. marker_trackerから位置情報を取得
   ├─ robot_head マーカーセット → ロボット位置
   └─ obstacles マーカーセット → 障害物位置

2. 実機ロボットから接続を待つ（TCPリスン）

3. 相互相関データを受信
   └─ [[crosscor_l], [crosscor_r]]

4. 物体定位を実行
   └─ Localizer.localize_from_crosscor()

5. ベイズ推論で事後分布を更新
   └─ Bayesian.update_belief()

6. 回避指令を計算
   └─ Agent.calculate_avoidance_command()

7. 移動指令を送信
   └─ {'Time': ..., 'NextMove': ..., 'NextAngle': ...}

8. 可視化画像を保存
   └─ BatVisualizer.plot_single_step()
```

---

## 5. 実装ステップ

### Phase 1: marker_trackerの統合（テストモード）
- [ ] marker_tracker.pyをtestモードで起動できることを確認
- [ ] HTTPクライアントで/latestと/marker_setからデータ取得
- [ ] robot_headマーカーセットから重心位置を計算
- [ ] テストスクリプトを作成

### Phase 2: Localizerの拡張
- [ ] Localizer.localize_from_crosscor()メソッドを実装
- [ ] 相互相関データから物体定位を行う処理を追加
- [ ] 既存のlocalize()メソッドとの整合性を確認

### Phase 3: bayes_server.pyの実装
- [ ] control_pc.pyをベースに新規作成
- [ ] marker_trackerからの位置情報取得機能を追加
- [ ] 実機ロボット通信プロトコルに対応
- [ ] 単位変換（mm↔m、degree↔radian）を実装
- [ ] エラーハンドリングとログ出力を強化

### Phase 4: 統合テスト
- [ ] marker_tracker.py（testモード）起動
- [ ] bayes_server.py起動
- [ ] ダミークライアントでエコーデータを送信してテスト
- [ ] 移動指令の正しさを確認

### Phase 5: 実機連携テスト
- [ ] OptiTrackシステムでmarker_tracker.pyを起動
- [ ] robot_headマーカーを配置して位置取得を確認
- [ ] 実機ロボットとbayes_server.pyを接続
- [ ] エンドツーエンドでの動作確認

---

## 6. 注意点とリスク

### 6.1 座標系の統一
- OptiTrackの座標系（x, y, z）とシミュレーションの座標系（x, y）
- z座標の扱い（高さ方向）をどう処理するか
- 原点位置の調整が必要

### 6.2 マーカーセットの命名規則
- `robot_head`: ロボットの位置
- `obstacles`: 障害物（複数のマーカー）
- Motive側で適切に設定する必要がある

### 6.3 ネットワーク遅延
- marker_tracker（HTTP）とbayes_server（TCP）の通信遅延
- リアルタイム性の確保

### 6.4 エラーハンドリング
- マーカーが見えない場合の処理
- 通信エラー時のフォールバック
- ロボットの安全停止機構

### 6.5 データ形式の違い
- ファイルベース（.dat）→ ストリームベース（相互相関配列）
- Localizerの入力形式の変更が必要

---

## 7. 開発環境とテスト戦略

### 7.1 テストモードの活用
marker_tracker.pyのtestモードを活用することで、OptiTrackなしで開発可能：
```bash
# marker_trackerをtestモードで起動
python marker_tracker.py --mode test --port 6000

# bayes_serverを起動（marker_trackerから位置取得）
python bayes_server.py --marker-tracker localhost:6000
```

### 7.2 段階的な移行
1. **ステージ1**: marker_tracker（test）+ bayes_server + ダミークライアント
2. **ステージ2**: marker_tracker（real）+ bayes_server + ダミークライアント
3. **ステージ3**: marker_tracker（real）+ bayes_server + 実機ロボット

### 7.3 ログとデバッグ
- 各ステップで詳細なログ出力
- 可視化画像の保存
- 位置情報・エコーデータ・指令のCSV記録

---

## 8. まとめ

### 現在の状況
- シミュレーション環境（env_server + control_pc + robot_simulator）が動作中
- OptiTrackマーカートラッキングシステムが利用可能
- 実機ロボット（Golang）の通信プロトコルが定義済み

### 目標
- シミュレーション環境を実機環境に移行
- OptiTrackから位置情報を取得
- 実機ロボットとベイズ推論サーバーを接続

### 次のアクション
1. marker_tracker.pyのtestモードで動作確認
2. Localizer.localize_from_crosscor()の実装
3. bayes_server.pyの新規作成
4. 統合テストの実施
