# コウモリのエコロケーションロボットシステム - プロジェクトドキュメント

## 目次
1. [プロジェクト概要](#プロジェクト概要)
2. [システムアーキテクチャ](#システムアーキテクチャ)
3. [主要コンポーネント](#主要コンポーネント)
4. [ファイル構成](#ファイル構成)
5. [セットアップと使用方法](#セットアップと使用方法)
6. [開発状況](#開発状況)
7. [関連ドキュメント](#関連ドキュメント)

---

## プロジェクト概要

このプロジェクトは、コウモリの反響定位（エコロケーション）を模倣したロボットシステムの研究開発プロジェクトです。ベイズ推論を用いた環境認識と障害物回避機能を実装しています。

### 研究目的
- コウモリのエコロケーション能力をロボットで再現
- ベイズ推論による確率的環境マッピング
- 超音波センシングを用いた自律的な障害物回避

### プロジェクトの2つのフェーズ

#### **Phase 1: シミュレーション環境**
- Python実装による完全なシミュレーション
- 固定データファイルを使用したエコー処理
- CSV ベースの環境情報管理

#### **Phase 2: 実機環境（現在進行中）**
- Golangで実装された実機ロボットとの統合
- OptiTrackモーションキャプチャシステムによる位置トラッキング
- リアルタイムエコーデータ処理

---

## システムアーキテクチャ

### シミュレーション環境のアーキテクチャ

```
┌─────────────────┐
│   env_server    │  環境情報管理（非推奨）
│  (Port: 6000)   │  - CSV ベースの障害物管理
└─────────────────┘  - ロボット位置管理
        ▲
        │
        ▼
┌─────────────────┐      ┌─────────────────────┐
│  control_pc.py  │◄────►│ robot_simulator.py  │
│  (Port: 6001)   │      │                     │
└─────────────────┘      └─────────────────────┘
 ベイズ推論・制御            ロボットシミュレータ
 - 物体定位                 - エコーデータ送信
 - 事後分布計算             - 移動指令実行
 - 回避計算
 - 可視化
```

### 実機環境のアーキテクチャ（Phase 2）✅ **Phase 1-4 完了**

```
┌─────────────────────────┐
│ marker_tracker.py       │  OptiTrack位置トラッキング
│ (HTTP: Port 6000)       │  - robot_body マーカーセット
│                         │  - robot_head マーカーセット
│ [test mode 実装済み]    │  - obstacles マーカーセット
└───────────┬─────────────┘
            │ HTTP GET
            ▼
┌─────────────────────────┐      ┌─────────────────────────┐
│ control_pc.py           │◄────►│ 実機ロボット (Golang)   │
│ (TCP: Port 6001)        │      │ または                  │
│                         │      │ robot_simulator.py      │
│ - 位置取得（新実装）    │      │                         │
│ - 物体定位（拡張）      │      │ - DoSensing             │
│ - ベイズ推論            │      │ - 相互相関データ送信    │
│ - 回避計算              │      │   [[crosscor_l],        │
│ - 単位変換（新実装）    │      │    [crosscor_r]]        │
│ - 可視化                │      │ - DoSingleMove          │
└─────────────────────────┘      │ - CreateCall            │
                                  └─────────────────────────┘
```

---

## 主要コンポーネント

### 1. **control_pc.py** - ベイズ推論制御サーバー

**役割**: システムの中核となるベイズ推論エンジンと制御ロジック

**主要クラス**: `ControlPC`

**主要機能**:
- **物体定位** (`Localizer`): エコーデータから物体の位置を推定
- **ベイズ推論** (`Bayesian`): 環境の事後確率分布を更新
- **回避計算** (`Agent`): 最適な移動方向と距離を計算
- **可視化** (`BatVisualizer`): リアルタイムの環境マップ生成
- **位置トラッキング**: OptiTrackから位置情報を取得（Phase 2）
- **実機通信**: Golangロボットとの通信プロトコル実装（Phase 2）

**主要メソッド**:
| メソッド名 | 機能 |
|-----------|------|
| `__init__()` | システム初期化、各コンポーネントのセットアップ |
| `get_robot_position_from_marker_tracker()` | OptiTrackから位置取得（Phase 2） |
| `calculate_posterior_from_detections()` | 物体定位とベイズ更新 |
| `calculate_movement_command()` | 移動指令計算 |
| `handle_real_robot_request()` | 実機ロボット対応（Phase 2） |
| `handle_request()` | シミュレータ対応（後方互換性） |
| `run()` | サーバーメインループ |

**通信プロトコル**:
- **旧形式（シミュレータ）**: サイズプリフィックス付きBase64エンコード
- **新形式（実機）**: 改行終端JSON形式
  - 受信: `[[crosscor_l], [crosscor_r]]`
  - 送信: `{"Time": ISO8601, "NextMove": m, "NextAngle": rad}`

**ファイルパス**: `control_pc.py:30-925`

---

### 2. **marker_tracker.py** - OptiTrackマーカートラッキングシステム

**役割**: OptiTrackモーションキャプチャからロボットと障害物の位置を取得

**主要クラス**: `SimpleMarkerTracker`

**動作モード**:
1. **server**: OptiTrackシステムと接続してリアルタイムデータを取得
2. **print**: コンソールにデータを定期出力
3. **test**: ダミーデータを生成（開発・テスト用）✅ **実装済み**

**HTTP API エンドポイント**:
| エンドポイント | 機能 |
|---------------|------|
| `GET /latest` | 最新スナップショット全体を取得 |
| `GET /marker_set?name=<name>` | 特定のマーカーセットのみ取得 |

**マーカーセット命名規則**:
- `robot_body`: ロボット本体（z座標最大の点を位置とする）
- `robot_head`: ロボット頭部（3個のマーカー、fd/pd計算用）
- `obstacles`: 障害物（複数のマーカー）

**起動例**:
```bash
# testモード（OptiTrack不要）
python marker_tracker.py --mode test --port 6000

# 実機モード
python marker_tracker.py --mode server --server-ip <Motive_IP> --client-ip <THIS_PC_IP> --port 6000
```

**ファイルパス**: `marker_tracker.py`

---

### 3. **marker_tracker_client.py** - OptiTrackクライアント

**役割**: marker_tracker.pyからHTTPでデータを取得し、ロボット位置を計算

**主要機能**:
- HTTPクライアント実装
- robot_bodyから重心位置を計算
- robot_headから方向（fd, pd）を計算
- 障害物位置の取得

**使用例**:
```python
from marker_tracker_client import MarkerTrackerClient

client = MarkerTrackerClient(url="http://localhost:6000")
position = client.get_robot_position()
# {'x': 1.5, 'y': 2.0, 'fd': 90, 'pd': 0}

obstacles = client.get_obstacles()
# [(x1, y1), (x2, y2), ...]
```

**ファイルパス**: `marker_tracker_client.py`

---

### 4. **robot_simulator.py** - ロボットシミュレータ

**役割**: 実機ロボット（Golang）の動作を模倣したシミュレータ

**主要クラス**: `RobotSimulator`

**実装フロー**（実機に準拠）:
```
1. DoSensing      : エコーセンシング（_data_Multi.dat読み込み）
2. 通信           : control_pcに相互相関データを送信
3. DoSingleMove   : 受信した移動指令を実行
4. CreateCall     : 音声パルス発生
```

**データ形式**:
- **送信**: `[[crosscor_l], [crosscor_r]]` - 左右耳の相互相関データ
- **受信**: `{"Time": str, "NextMove": m, "NextAngle": rad}` - 移動指令

**起動例**:
```bash
python robot_simulator.py 20  # 20ステップ実行
```

**ファイルパス**: `robot_simulator.py`

**Phase 4で全面改修済み**: Golang実装の動作を再現 ✅

---

### 5. **bayes_code/** - ベイズ推論コアモジュール

#### **bayesian.py** - ベイズ推論エンジン

**主要クラス**: `Bayesian`

**機能**:
- 事前確率分布の初期化
- 尤度計算（2次元空間）
- ベイズ更新による事後確率分布計算
- デシベルスケール変換

**主要メソッド**:
| メソッド名 | 機能 |
|-----------|------|
| `Init()` | 事前確率分布の初期化 |
| `new_likelyhood_2D()` | 2次元尤度計算 |
| `update_belief()` | ベイズ更新 |
| `dB_trans()` | デシベル変換 |

---

#### **localization.py** - 物体定位モジュール

**主要クラス**: `Localizer`

**機能**:
- エコーデータから物体の距離・角度・強度を推定
- ファイルベース処理（シミュレーション用）
- 配列ベース処理（実機用）✅ **Phase 2で拡張**

**主要メソッド**:
| メソッド名 | 機能 |
|-----------|------|
| `localize(file_path)` | ファイルから物体定位（シミュレーション） |
| `localize_from_crosscor(crosscor_l, crosscor_r)` | 相互相関配列から物体定位（実機）✅ **新実装** |
| `_localize_from_arrays()` | 内部共通処理メソッド |

**ファイルパス**: `bayes_code/localization.py:89-163`

---

#### **agent.py** - エージェント（回避行動）

**主要クラス**: `Agent`

**機能**:
- ロボットの状態管理（位置、方向）
- 事後分布から最適な移動方向を計算
- 移動距離の決定

**主要メソッド**:
| メソッド名 | 機能 |
|-----------|------|
| `calculate_avoidance_command()` | 回避指令計算 |
| `update_position()` | 位置更新 |

---

#### **world.py** - 環境管理

**主要クラス**: `World`

**機能**:
- 環境の境界設定
- 壁の座標管理
- 障害物の座標管理（CSV読み込み - シミュレーション用）

**Phase 2での変更**: CSVからの障害物読み込みを無効化し、marker_trackerから取得するように変更

---

#### **calc.py** - 計算ユーティリティ

**主要機能**:
- 座標変換（デカルト座標 ⇔ 極座標）
- 音波減衰計算（距離・方向）
- エコー検出判定
- 超音波往復距離計算

**主要関数**:
| 関数名 | 機能 |
|-------|------|
| `round_angle()` | 角度の正規化 |
| `XY_to_r_theta_calc()` | デカルト→極座標変換 |
| `dist_attenuation()` | 距離減衰計算 |
| `direc_attenuation()` | 方向減衰計算 |
| `real_dist_goback()` | 超音波往復距離 |

---

#### **robot_visualize.py** - 可視化モジュール

**主要クラス**: `BatVisualizer`

**機能**:
- ロボット位置のプロット
- 事後確率分布のヒートマップ
- 障害物表示
- フレーム画像の保存
- GIFアニメーション生成

**主要メソッド**:
| メソッド名 | 機能 |
|-----------|------|
| `plot_frame()` | 単一フレームのプロット |
| `plot_single_step()` | 1ステップの可視化 |
| `create_gif_from_frames()` | GIFアニメーション作成 |

---

#### **config.py** - 設定管理

**機能**:
- システムパラメータの一元管理
- 環境設定
- センサーパラメータ

---

## ファイル構成

```
Lab_Bayesian/
├── README.md                    # シミュレーション環境の説明
├── MIGRATION_PLAN.md            # 実機環境への移行計画
├── PROJECT_DOCUMENTATION.md     # 本ドキュメント（プロジェクト全体）
├── requirements.txt             # Python依存パッケージ
├── .gitignore
│
├── control_pc.py                # ベイズ推論制御サーバー（メイン）
├── marker_tracker.py            # OptiTrackマーカートラッキング
├── marker_tracker_client.py     # OptiTrackクライアント
├── marker_test.py               # マーカートラッカーテストツール
├── robot_simulator.py           # ロボットシミュレータ（Golang風）
│
├── bayes_code/                  # ベイズ推論コアモジュール
│   ├── __init__.py
│   ├── config.py               # 設定管理
│   ├── bayesian.py             # ベイズ推論エンジン
│   ├── localization.py         # 物体定位モジュール
│   ├── agent.py                # エージェント（回避行動）
│   ├── world.py                # 環境管理
│   ├── calc.py                 # 計算ユーティリティ
│   └── robot_visualize.py      # 可視化モジュール
│
├── robot_data/                  # エコーデータ（シミュレーション用）
│   └── wall_000/
│       └── goldorak/
│           └── _data_Multi.dat  # 固定エコーデータファイル
│
├── bayse_olddata2/              # 過去のシミュレーションデータ
│
└── .serena/                     # Serenaメモリ（プロジェクト情報）
```

---

## セットアップと使用方法

### 環境構築

#### 1. 依存パッケージのインストール

```bash
# 仮想環境の作成（推奨）
python3 -m venv mac_venv
source mac_venv/bin/activate  # macOS/Linux
# または
mac_venv\Scripts\activate  # Windows

# 依存パッケージをインストール
pip install -r requirements.txt
```

#### 2. 必要なライブラリ
- `numpy` - 数値計算
- `scipy` - 科学計算
- `matplotlib` - 可視化
- `pandas` - データ処理
- `flask` - HTTPサーバー（marker_tracker）
- その他（`requirements.txt` 参照）

---

### シミュレーション環境での実行

#### ステップ1: marker_trackerをtestモードで起動

```bash
python marker_tracker.py --mode test --port 6000
```

出力例:
```
🔧 Running in TEST mode (no OptiTrack connection)
🌐 Starting HTTP server on localhost:6000
Press Ctrl+C to stop.
```

#### ステップ2: control_pcを起動

```bash
python control_pc.py
```

出力例:
```
=== Control PC Initialization ===
✅ MarkerTrackerClient initialized
✅ Initial robot position retrieved
✅ Localizer initialized
✅ Visualizer initialized
TCP server listening on 0.0.0.0:6001
```

#### ステップ3: robot_simulatorを起動

```bash
python robot_simulator.py 20  # 20ステップ実行
```

出力例:
```
=== Robot Simulator ===
Target server: localhost:6001
Max steps: 20

Step 1/20
  [DoSensing] Reading data from: robot_data/wall_000/goldorak/_data_Multi.dat
  [Communication] Sending crosscor data (30000 samples)...
  [Communication] Received response: NextMove=0.15m, NextAngle=0.3rad
  [DoSingleMove] Moving...
  [CreateCall] Pulse generated

...
```

---

### 実機環境での実行（Phase 5 - 未完了）

#### ステップ1: marker_trackerを実機モードで起動

```bash
python marker_tracker.py --mode server \
  --server-ip <Motive_IP> \
  --client-ip <THIS_PC_IP> \
  --port 6000
```

**注意**: Motive側でマーカーセット（`robot_body`, `robot_head`, `obstacles`）を設定しておく必要があります。

#### ステップ2: control_pcを起動

```bash
python control_pc.py
```

#### ステップ3: 実機ロボット（Golang）を起動

```bash
# Golangロボット側で実行
# （実機ロボットの起動コマンド）
```

---

## 開発状況

### ✅ Phase 1: marker_trackerの統合（テストモード） - **完了**

- [x] marker_tracker.pyのtestモード実装
- [x] HTTPクライアントでデータ取得
- [x] robot_headマーカーセットから重心位置計算
- [x] テストスクリプト作成（marker_test.py, marker_tracker_client.py）

**実装ファイル**:
- `marker_tracker.py:89-104, 340-468, 470-487, 652-674`
- `marker_tracker_client.py`
- `control_pc.py:28, 55-100, 112-128, 206-237, 684-698`

---

### ✅ Phase 2: Localizerの拡張 - **完了**

- [x] `Localizer.localize_from_crosscor()` メソッド実装
- [x] 相互相関データから物体定位を行う処理追加
- [x] 既存のlocalize()メソッドとの整合性確認

**実装ファイル**:
- `bayes_code/localization.py:89-163`

---

### ✅ Phase 3: control_pc.pyの拡張（実機ロボット対応） - **完了**

- [x] marker_trackerからの位置情報取得機能追加
- [x] 実機ロボット通信プロトコル対応
- [x] 単位変換（mm↔m、degree↔radian）実装
- [x] `handle_real_robot_request()` メソッド実装
- [x] run()メソッドの新形式対応
- [x] エラーハンドリング強化

**実装ファイル**:
- `control_pc.py:649-667` - 単位変換メソッド
- `control_pc.py:669-755` - `handle_real_robot_request()`
- `control_pc.py:840-921` - `run()` メソッド

---

### ✅ Phase 4: 統合テスト - **完了**

- [x] marker_tracker（testモード）起動確認
- [x] control_pc起動確認
- [x] robot_simulator.pyをGolang風に書き換え
- [x] エンドツーエンド動作確認

**実装ファイル**:
- `robot_simulator.py` - 全面改修

---

### ⚠️ Phase 5: 実機連携テスト - **未完了**

- [ ] OptiTrackシステムでmarker_tracker.pyを起動
- [ ] robot_headマーカーを配置して位置取得確認
- [ ] 実機ロボット（Golang）との接続
- [ ] エンドツーエンド動作確認

---

## 技術詳細

### データフロー

#### シミュレーション環境
```
1. robot_simulator.py
   └─ _data_Multi.datファイル読み込み
   └─ 相互相関データ抽出: [[crosscor_l], [crosscor_r]]
   └─ control_pc.pyに送信（TCP、JSON形式）

2. control_pc.py
   └─ データ受信
   └─ Localizer.localize_from_crosscor() で物体定位
      └─ [{'distance': mm, 'angle': deg, 'intensity': float}, ...]
   └─ Bayesian.update_belief() でベイズ更新
      └─ 事後確率分布を計算
   └─ Agent.calculate_avoidance_command() で回避計算
      └─ {'avoidance_direction': deg, 'move_distance': mm}
   └─ 単位変換（mm→m、deg→rad）
   └─ 移動指令を送信: {"Time": ISO8601, "NextMove": m, "NextAngle": rad}
   └─ BatVisualizer.plot_single_step() で可視化

3. robot_simulator.py
   └─ 移動指令受信
   └─ DoSingleMove() で移動シミュレート
   └─ CreateCall() でパルス発生
```

#### 実機環境（Phase 5）
```
1. marker_tracker.py
   └─ OptiTrackからマーカーデータ取得
   └─ HTTPサーバーで提供（ポート6000）

2. control_pc.py
   └─ marker_trackerから位置情報取得（HTTP GET）
   └─ ロボットからエコーデータ受信（TCP）
   └─ 以降はシミュレーションと同じフロー

3. 実機ロボット（Golang）
   └─ DoSensing() でエコーセンシング
   └─ 相互相関データ送信
   └─ 移動指令受信
   └─ DoSingleMove() で実際に移動
   └─ CreateCall() で音声パルス発生
```

---

### 通信プロトコル

#### 旧形式（シミュレータ用、非推奨）
- **エンコード**: Base64
- **サイズプリフィックス**: あり
- **形式**: バイナリ

#### 新形式（実機用、推奨）✅
- **エンコード**: JSON
- **終端**: 改行 (`\n`)
- **送信**: `[[crosscor_l], [crosscor_r]]`
- **受信**: `{"Time": "2025-01-16T10:30:00", "NextMove": 0.15, "NextAngle": 0.3}`

---

### 単位系

| データ | シミュレーション内部 | 実機ロボット | 変換メソッド |
|--------|---------------------|-------------|-------------|
| 距離 | mm | m | `_mm_to_m()`, `_m_to_mm()` |
| 角度 | degree | radian | `_deg_to_rad()`, `_rad_to_deg()` |
| 時刻 | - | ISO8601文字列 | `datetime.now().isoformat()` |

**実装箇所**: `control_pc.py:649-667`

---

### 座標系

#### OptiTrackの座標系
- **x軸**: 水平方向
- **y軸**: 水平方向（x軸に直交）
- **z軸**: 鉛直方向（高さ）

#### シミュレーションの座標系
- **x軸**: 水平方向
- **y軸**: 水平方向
- **z軸**: 使用しない（2次元平面）

**変換**:
- OptiTrackのz座標はロボット高さの判定に使用
- robot_bodyの最も高いマーカー（z座標最大）をロボット位置とする

---

## トラブルシューティング

### よくある問題

#### 1. marker_trackerに接続できない

**症状**:
```
[WARNING] marker_tracker connection failed: [Errno 61] Connection refused
Using fallback position: x=0.0, y=0.0
```

**原因**: marker_tracker.pyが起動していない

**解決策**:
```bash
# 別のターミナルでmarker_trackerを起動
python marker_tracker.py --mode test --port 6000
```

---

#### 2. ポートが使用中

**症状**:
```
OSError: [Errno 48] Address already in use
```

**解決策**:
```bash
# 使用中のプロセスを確認
lsof -i :6001

# プロセスを終了
kill -9 <PID>
```

---

#### 3. 相互相関データのサイズ不一致

**症状**:
```
ValueError: crosscor_l and crosscor_r must have the same length
```

**原因**: _data_Multi.datファイルが破損または形式が不正

**解決策**:
- 正しいフォーマットの_data_Multi.datファイルを使用
- `robot_data/wall_000/goldorak/_data_Multi.dat` を確認

---

#### 4. マーカーが見つからない

**症状**:
```
[WARNING] Marker set 'robot_head' not found
```

**原因**: Motive側でマーカーセットが設定されていない

**解決策**:
- Motiveでマーカーセット `robot_body`, `robot_head`, `obstacles` を作成
- 各セットにマーカーを割り当て

---

## 今後の開発予定

### 高優先度
1. **実機モードでのmarker_tracker動作確認** (Phase 5)
   - OptiTrackシステムとの接続
   - Motive側のマーカーセット設定

2. **実機ロボット（Golang）との接続テスト** (Phase 5)
   - 実際のGolangロボットとの通信確認
   - データ形式・単位変換の検証

### 中優先度
3. **エラーハンドリングの強化**
   - マーカーが見えない場合の安全処理
   - 通信断時のフォールバック
   - ロボットの安全停止機構

4. **パフォーマンス最適化**
   - 大容量データ（30000サンプル）の効率的な処理
   - 可視化の高速化

### 低優先度
5. **ログとデバッグ機能の拡充**
   - 位置情報・エコーデータ・指令のCSV記録
   - リプレイ機能

6. **テストカバレッジの向上**
   - ユニットテスト追加
   - 統合テスト自動化

---

## 関連ドキュメント

- **[README.md](README.md)**: シミュレーション環境の詳細説明
- **[MIGRATION_PLAN.md](MIGRATION_PLAN.md)**: シミュレーションから実機への移行計画
- **Serenaメモリ**: `migration_plan_simulation_to_real` - 移行計画の詳細情報

---

## プロジェクトメタデータ

- **プログラミング言語**: Python 3
- **開発環境**: macOS (Darwin 25.1.0)
- **最終更新**: 2025-01-16（Phase 1-4完了、Phase 5は未完了）
- **Gitリポジトリ**: あり（main branch）
- **最新コミット**: "Complete Phase 2-4: Real robot migration implementation"

---

## ライセンスと著作権

（プロジェクト固有のライセンス情報をここに記載）

---

## お問い合わせ

（連絡先情報をここに記載）

---

**Document Version**: 1.0
**Generated**: 2025-11-23
**Author**: Claude Code (assisted documentation)
