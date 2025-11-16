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

### Phase 1: marker_trackerの統合（テストモード）✅ **完了**
- [x] marker_tracker.pyをtestモードで起動できることを確認
  - `marker_tracker.py:89-104, 340-468, 470-487, 652-674` で実装
  - `--mode test` オプションでNatNet接続なしでダミーデータを提供
- [x] HTTPクライアントで/latestと/marker_setからデータ取得
  - `marker_tracker_client.py:56-144` でHTTPクライアント実装
  - `/latest` と `/marker_set?name=<name>` エンドポイントに対応
- [x] robot_headマーカーセットから重心位置を計算
  - `marker_tracker_client.py:146-240` でrobot_bodyとrobot_headから位置・方向を計算
  - robot_bodyのz座標最大点をロボット位置として使用
  - robot_headの3点から真ん中の点を選択し、fd/pdを計算
- [x] テストスクリプトを作成
  - `marker_tracker_client.py:402-441` にテストコード実装

**実装ファイル:**
- `marker_tracker.py`: testモード追加、ダミーデータ生成機能
- `marker_tracker_client.py`: HTTPクライアント（新規作成）
- `control_pc.py:28, 55-100, 112-128, 206-237, 684-698`: MarkerTrackerClient統合
- `bayes_code/world.py:183-189`: CSVからの障害物読み込みを無効化

### Phase 2: Localizerの拡張 ✅ **完了**
- [x] Localizer.localize_from_crosscor()メソッドを実装
  - `bayes_code/localization.py:89-113` で実装完了
  - 相互相関データ（[[crosscor_l], [crosscor_r]]）を直接処理
- [x] 相互相関データから物体定位を行う処理を追加
  - `_localize_from_arrays()` 内部メソッドで処理を共通化（115-163行目）
- [x] 既存のlocalize()メソッドとの整合性を確認
  - 既存の`localize(file_path)`は内部で`_localize_from_arrays()`を呼び出す設計に変更
  - コードの重複を避け、保守性を向上

### Phase 3: control_pc.pyの拡張（実機ロボット対応） ✅ **完了**
**設計変更**: bayes_server.pyを新規作成せず、control_pc.pyを拡張する方針に変更
- [x] marker_trackerからの位置情報取得機能を追加 ✅
  - `control_pc.py:55-100, 112-128, 206-237` で実装完了（Phase 1で完了）
- [x] 実機ロボット通信プロトコルに対応 ✅
  - `control_pc.py:669-755` に`handle_real_robot_request()`メソッドを実装
  - 相互相関データ受信: `[[crosscor_l], [crosscor_r]]`
  - 実機形式で応答: `{"Time": ISO8601, "NextMove": m, "NextAngle": rad}`
  - ポート6001で通信（既存のポートを使用）
- [x] 単位変換（mm↔m、degree↔radian）を実装 ✅
  - `control_pc.py:649-667` に単位変換メソッドを実装
    - `_mm_to_m()`, `_m_to_mm()` - 距離変換
    - `_deg_to_rad()`, `_rad_to_deg()` - 角度変換
  - `handle_real_robot_request()`内で自動変換
- [x] run()メソッドを新形式用に変更 ✅
  - `control_pc.py:840-921` で改行終端のJSON受信に対応
  - 新形式を自動処理
- [x] エラーハンドリングとログ出力を強化 ✅
  - marker_tracker接続失敗時のフォールバック処理
  - 通信エラー時も適切な形式で応答

**実装ファイル:**
- `control_pc.py:202-203`: ステップカウンター追加
- `control_pc.py:649-667`: 単位変換メソッド
- `control_pc.py:669-755`: `handle_real_robot_request()` メソッド
- `control_pc.py:840-921`: `run()` メソッド（新形式対応）
- 既存の`handle_request()`は保持（後方互換性）

### Phase 4: 統合テスト ✅ **完了**
- [x] marker_tracker.py（testモード）起動 ✅
  - `python marker_tracker.py --mode test --port 6000`
- [x] control_pc.py起動 ✅
  - `python control_pc.py`
  - ポート6001で待機、新形式（相互相関配列）に対応
- [x] robot_simulator.pyをGolang風に書き換え ✅
  - `robot_simulator.py` を実機ロボット（Golang）の動作に似せて実装
  - DoSensing → 通信 → DoSingleMove → CreateCall の流れを再現
  - 相互相関データ送信: `[[crosscor_l], [crosscor_r]]`
  - 実機形式の応答受信: `{"Time": str, "NextMove": m, "NextAngle": rad}`
- [x] 統合テスト実施 ✅（基本動作確認完了）
  - `python robot_simulator.py 20`
  - 実行確認済み、エンドツーエンドで動作

**実装ファイル:**
- `robot_simulator.py`: 全面的に書き換え
  - `do_sensing()`: _data_Multi.datファイルから相互相関データを読み込み
  - `send_data_to_server()`: JSON形式で送信、改行終端
  - `do_single_move()`: 移動シミュレート
  - `create_call()`: パルス発生シミュレート

### Phase 5: 実機連携テスト ⚠️ **未完了**
- [ ] OptiTrackシステムでmarker_tracker.pyを起動
  - testモードは完了、実機モード（--mode server）での動作確認が必要
- [ ] robot_headマーカーを配置して位置取得を確認
  - Motive側でマーカーセットの設定が必要
- [ ] 実機ロボットとbayes_server.pyを接続
  - bayes_server.pyの実装が先に必要
- [ ] エンドツーエンドでの動作確認

---

## 6. 実装状況サマリー

### ✅ 完了した項目（2025-01-16更新）

1. **marker_trackerのtestモード実装** (Phase 1) ✅
   - ダミーデータ生成（robot_body, robot_head, obstacles）
   - HTTPサーバー機能（/latest, /marker_set）
   - 120 FPS相当のリアルタイムデータ更新

2. **marker_tracker_clientの実装** (Phase 1) ✅
   - HTTPクライアント機能
   - ロボット位置・方向の計算ロジック
   - 障害物データ取得

3. **control_pc.pyへのmarker_tracker統合** (Phase 1) ✅
   - MarkerTrackerClientの統合
   - 初期化時の位置・障害物データ取得
   - 定期的な位置更新

4. **Localizer.localize_from_crosscor()の実装** (Phase 2) ✅
   - 実機ロボットから送られる相互相関データを直接処理
   - `_localize_from_arrays()` 内部メソッドで処理を共通化
   - 既存の`localize(file_path)`との互換性を維持

5. **control_pc.pyの拡張（実機ロボット対応）** (Phase 3) ✅
   - `handle_real_robot_request()` メソッド実装
   - 相互相関データ受信: `[[crosscor_l], [crosscor_r]]`
   - 単位変換（mm↔m、度↔rad）実装
   - 実機形式で応答: `{"Time": ISO8601, "NextMove": m, "NextAngle": rad}`
   - 既存の`handle_request()`は保持（後方互換性）

6. **robot_simulator.pyのGolang風書き換え** (Phase 4) ✅
   - DoSensing → 通信 → DoSingleMove → CreateCall の流れを再現
   - 相互相関データ送信プロトコル実装
   - 実機形式の応答受信

7. **統合テスト** (Phase 4) ✅（基本動作確認完了）
   - marker_tracker (test mode) + control_pc + robot_simulator
   - エンドツーエンドで動作確認済み

### ⚠️ 未完了・残課題

#### 高優先度（実機連携に必須）
1. **実機モードでのmarker_tracker動作確認** (Phase 5)
   - OptiTrackシステムとの接続
   - Motive側のマーカーセット設定（robot_body, robot_head, obstacles）

2. **実機ロボット（Golang）との接続テスト** (Phase 5)
   - 現在はrobot_simulator.py（Python）でのテストのみ
   - 実際のGolangロボットとの通信確認が必要

#### 中優先度（動作確認・改善）
3. **エラーハンドリングの強化**
   - マーカーが見えない場合の処理（一部実装済み）
   - 通信エラー時のフォールバック（一部実装済み）
   - ロボットの安全停止機構

4. **パフォーマンス最適化**
   - 大容量データ（30000サンプル）の効率的な処理
   - 可視化の高速化

---

## 7. 注意点とリスク

### 7.1 座標系の統一
- OptiTrackの座標系（x, y, z）とシミュレーションの座標系（x, y）
- z座標の扱い（高さ方向）をどう処理するか
- 原点位置の調整が必要

### 7.2 マーカーセットの命名規則
- `robot_body`: ロボット本体（z座標最大の点をロボット位置とする）
- `robot_head`: ロボット頭部（3個のマーカー、真ん中の点をヘッド位置とする）
- `obstacles`: 障害物（複数のマーカー）
- Motive側で適切に設定する必要がある

### 7.3 ネットワーク遅延
- marker_tracker（HTTP）とbayes_server（TCP）の通信遅延
- リアルタイム性の確保

### 7.4 エラーハンドリング
- マーカーが見えない場合の処理
- 通信エラー時のフォールバック
- ロボットの安全停止機構

### 7.5 データ形式の違い
- ファイルベース（.dat）→ ストリームベース（相互相関配列）
- Localizerの入力形式の変更が必要

---

## 8. 開発環境とテスト戦略

### 8.1 テストモードの活用
marker_tracker.pyのtestモードを活用することで、OptiTrackなしで開発可能：
```bash
# marker_trackerをtestモードで起動
python marker_tracker.py --mode test --port 6000

# bayes_serverを起動（marker_trackerから位置取得）
python bayes_server.py --marker-tracker localhost:6000
```

### 8.2 段階的な移行
1. **ステージ1**: marker_tracker（test）+ control_pc + robot_simulator ✅ **完了**
   - testモードで動作確認済み
2. **ステージ2**: marker_tracker（real）+ bayes_server + ダミークライアント ⚠️ **未完了**
   - bayes_server.pyの実装が必要
3. **ステージ3**: marker_tracker（real）+ bayes_server + 実機ロボット ⚠️ **未完了**
   - Phase 5の実機連携テスト

### 8.3 ログとデバッグ
- 各ステップで詳細なログ出力
- 可視化画像の保存
- 位置情報・エコーデータ・指令のCSV記録

---

## 9. まとめ

### 現在の状況（2025-01-16更新）
✅ **Phase 1-4完了:**
- marker_tracker.pyのtestモード実装完了
- marker_tracker_client.pyの作成完了
- control_pc.pyへのmarker_tracker統合完了
- Localizer.localize_from_crosscor()の実装完了
- control_pc.pyの実機ロボット対応完了（単位変換含む）
- robot_simulator.pyのGolang風書き換え完了
- 統合テスト完了（基本動作確認）

⚠️ **Phase 5未完了:**
- 実機モードでのmarker_tracker動作確認が必要
- 実機ロボット（Golang）との接続テストが必要

### 目標達成状況
- ✅ OptiTrackから位置情報を取得（testモードで達成）
- ✅ シミュレーション環境を実機環境に移行（**完了**）
- ⚠️ 実機ロボットとベイズ推論サーバーを接続（**テスト準備完了、実機テスト未実施**）

### システム構成（現在）
```
┌─────────────────────────┐
│ marker_tracker.py       │ OptiTrack位置情報
│ (test mode: port 6000)  │ - robot_body, robot_head, obstacles
└───────────┬─────────────┘
            │ HTTP GET
            ▼
┌─────────────────────────┐      ┌─────────────────────────┐
│ control_pc.py           │◄────►│ robot_simulator.py      │
│ (TCP: port 6001)        │      │ (Golang風)              │
│                         │      │                         │
│ - 位置取得              │      │ - DoSensing             │
│ - 物体定位（新実装）    │      │ - [[crosscor_l],        │
│ - ベイズ推論            │      │   [crosscor_r]] 送信    │
│ - 回避計算              │      │ - DoSingleMove          │
│ - 単位変換（新実装）    │      │ - CreateCall            │
│ - 可視化                │      │                         │
└─────────────────────────┘      └─────────────────────────┘
```

### 次のアクション（優先順位順）
1. **実機モードでのmarker_tracker動作確認** (Phase 5)
   - OptiTrackシステムとの接続
   - Motive側のマーカーセット設定（robot_body, robot_head, obstacles）
   - `python marker_tracker.py --mode server --server-ip <Motive_IP> --client-ip <THIS_PC_IP> --port 6000`

2. **実機ロボット（Golang）との接続テスト** (Phase 5)
   - Golangロボットをcontrol_pc.pyに接続
   - エンドツーエンドでの動作確認
   - データ形式・単位変換の確認

3. **エラーハンドリングの強化**
   - マーカーが見えない場合の安全処理
   - 通信断時のフォールバック
   - ロボットの安全停止機構

4. **パフォーマンス最適化**
   - 大容量データ処理の効率化
   - 可視化の高速化

### 起動手順（現在）
```bash
# 1. marker_trackerをtestモードで起動
python marker_tracker.py --mode test --port 6000

# 2. control_pcを起動
python control_pc.py

# 3. robot_simulatorを起動
python robot_simulator.py 20  # 20ステップ実行
```

### 実機連携時の起動手順（Phase 5）
```bash
# 1. marker_trackerを実機モードで起動
python marker_tracker.py --mode server --server-ip <Motive_IP> --client-ip <THIS_PC_IP> --port 6000

# 2. control_pcを起動
python control_pc.py

# 3. 実機ロボット（Golang）を起動
# （Golangロボット側で実行）
```
