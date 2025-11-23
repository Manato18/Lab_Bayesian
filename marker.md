# OptiTrackマーカートラッキングシステム - 詳細ドキュメント

## 目次

1. [概要](#概要)
   - [3つのコンポーネント](#3つのコンポーネント)
   - [システムアーキテクチャ](#システムアーキテクチャ)
   - [クイックスタート](#クイックスタート)
2. [marker_server.py - データ提供サーバー](#marker_serverpy---データ提供サーバー)
   - [役割と機能](#役割と機能)
   - [動作モード](#動作モード)
   - [コマンドライン引数](#コマンドライン引数)
   - [HTTP API仕様](#http-api仕様)
   - [内部実装](#内部実装)
   - [使用例](#使用例-marker_serverpy)
3. [marker_client.py - クライアントライブラリ](#marker_clientpy---クライアントライブラリ)
   - [役割と機能](#役割と機能-1)
   - [クラス仕様](#クラス仕様)
   - [主要メソッド](#主要メソッド)
   - [ロボット位置計算ロジック](#ロボット位置計算ロジック)
   - [使用例](#使用例-marker_clientpy)
4. [marker_test.py - テスト・デバッグツール](#marker_testpy---テストデバッグツール)
   - [役割と機能](#役割と機能-2)
   - [コマンドライン引数](#コマンドライン引数-1)
   - [出力フォーマット](#出力フォーマット)
   - [使用例](#使用例-marker_testpy)
5. [データ構造仕様](#データ構造仕様)
   - [スナップショット構造](#スナップショット構造)
   - [マーカーセット構造](#マーカーセット構造)
   - [座標系と単位](#座標系と単位)
6. [実践的な使用ワークフロー](#実践的な使用ワークフロー)
   - [開発・テスト環境](#開発テスト環境)
   - [実機運用環境](#実機運用環境)
   - [デバッグワークフロー](#デバッグワークフロー)
7. [トラブルシューティング](#トラブルシューティング)
8. [開発者向け情報](#開発者向け情報)
   - [拡張方法](#拡張方法)
   - [パフォーマンス最適化](#パフォーマンス最適化)
   - [テストモードのダミーデータ仕様](#テストモードのダミーデータ仕様)

---

## 概要

このドキュメントでは、OptiTrackモーションキャプチャシステムからマーカーデータを取得し、ロボット制御システムに提供するための3つのPythonコンポーネントについて説明します。

### 3つのコンポーネント

| コンポーネント | タイプ | 役割 | 実行方法 |
|---------------|--------|------|---------|
| **marker_server.py** | サーバー | OptiTrackからデータ取得→HTTP配信 | スタンドアロン実行（常駐） |
| **marker_client.py** | ライブラリ | HTTPからデータ取得→Python API | インポートして使用 |
| **marker_test.py** | ツール | データ取得→コンソール表示 | スタンドアロン実行（デバッグ用） |

### システムアーキテクチャ

```
┌──────────────────────────┐
│  OptiTrack (Motive)      │  モーションキャプチャシステム
│  - カメラ群              │  - マーカー3D位置をリアルタイム追跡
│  - NatNetストリーミング  │  - 120 FPS でデータ送信
└────────────┬─────────────┘
             │ NatNet Protocol (UDP 1510/1511)
             ▼
┌────────────────────────────────────────────────────────────┐
│  marker_server.py                                         │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ SimpleMarkerTracker クラス                           │ │
│  │  - NatNetClient でデータ受信                         │ │
│  │  - スナップショット生成・管理                        │ │
│  │  - HTTPサーバー（Flask不使用、標準ライブラリ）       │ │
│  └──────────────────────────────────────────────────────┘ │
│                                                            │
│  動作モード:                                               │
│  ┌─────────┬─────────┬──────────────────────────────┐   │
│  │ server  │ print   │ test (ダミーデータ生成)     │   │
│  └─────────┴─────────┴──────────────────────────────┘   │
└────────────┬───────────────────────────────────────────────┘
             │ HTTP API (ポート6000)
             │ - GET /latest
             │ - GET /marker_set?name=<name>
             │
       ┌─────┴──────────────────────┐
       ▼                            ▼
┌────────────────────┐    ┌──────────────────────────────┐
│ marker_test.py     │    │ marker_client.py     │
│                    │    │ (MarkerTrackerClient)        │
│ - デバッグツール   │    │                              │
│ - データ可視化     │    │ - HTTPクライアント           │
│ - スタンドアロン   │    │ - 位置計算ロジック           │
│   実行             │    │ - エラーハンドリング         │
└────────────────────┘    └──────────┬───────────────────┘
                                     │ Python API
                                     ▼
                          ┌──────────────────────────────┐
                          │ control_pc.py                │
                          │ - ベイズ推論制御サーバー     │
                          │ - ロボット位置管理           │
                          │ - 障害物情報管理             │
                          └──────────────────────────────┘
```

### クイックスタート

#### 開発・テスト環境（OptiTrack不要）

```bash
# ステップ1: marker_serverをtestモードで起動
python marker_server.py --mode test --port 6000

# ステップ2: 動作確認（別ターミナル）
python marker_test.py --host localhost --port 6000

# ステップ3: 実際のシステムで使用
python control_pc.py  # 内部でMarkerTrackerClientを使用
```

#### 実機環境（OptiTrack使用）

```bash
# ステップ1: Motiveでストリーミングを有効化
# - Edit > Preferences > Streaming
# - Local Interface: <このPCのIP>
# - Broadcast Frame Data: ON

# ステップ2: marker_serverを実機モードで起動
python marker_server.py --mode server \
  --server-ip 192.168.1.100 \
  --client-ip 192.168.1.50 \
  --port 6000

# ステップ3: データ確認
python marker_test.py --host 192.168.1.50 --port 6000
```

---

## marker_server.py - データ提供サーバー

### 役割と機能

**marker_server.py** は、OptiTrackモーションキャプチャシステム（Motive）からNatNetプロトコルでマーカーデータを受信し、HTTP APIとして他のプログラムに提供するサーバープログラムです。

**主要機能**:
1. **NatNet受信**: OptiTrackから120 FPS でマーカー3D座標を受信
2. **スナップショット管理**: 最新データをスレッドセーフに保持
3. **HTTP配信**: `/latest` と `/marker_set` エンドポイントでデータ提供
4. **テストモード**: OptiTrack不要でダミーデータを生成（開発用）
5. **プリントモード**: コンソールにデータを定期出力（デバッグ用）

**ファイルパス**: `marker_server.py` (744行)

**主要クラス**: `SimpleMarkerTracker`

---

### 動作モード

marker_server.pyは3つの動作モードを持ちます。

#### 1. serverモード（デフォルト）

**用途**: 実機運用時、OptiTrackと接続してHTTPでデータ配信

**特徴**:
- NatNetでOptiTrack（Motive）からデータ受信
- バックグラウンドでHTTPサーバーを起動
- `/latest` と `/marker_set` エンドポイントでデータ提供
- スレッドセーフなスナップショット管理

**起動コマンド**:
```bash
python marker_server.py --mode server \
  --server-ip <Motive_IP> \
  --client-ip <THIS_PC_IP> \
  --port 6000
```

**実装箇所**: `marker_server.py:630-650`

---

#### 2. printモード

**用途**: デバッグ・確認用、OptiTrackと接続してコンソールに出力

**特徴**:
- NatNetでOptiTrack（Motive）からデータ受信
- 指定間隔（デフォルト1秒）でコンソールに出力
- HTTPサーバーは起動しない
- フレーム受信カウントを表示

**起動コマンド**:
```bash
python marker_server.py --mode print \
  --server-ip <Motive_IP> \
  --client-ip <THIS_PC_IP> \
  --interval 1.0
```

**出力例**:
```
=== シンプルマーカートラッカー ===
クライアントIP: 192.168.1.50
サーバーIP: 192.168.1.100
接続方式: ユニキャスト
出力間隔: 1.0秒
========================================
データ受信開始...
フレーム受信カウント: 120

[18:10:12] フレーム: 871510
--------------------------------------------------
ラベル付きマーカー数: 6
  マーカー1 (ID:65537): X=1.617, Y=0.197, Z=0.903
  マーカー2 (ID:65538): X=1.638, Y=0.192, Z=0.905
  ...
マーカーセット数: 2, 総マーカー数: 6
  セット1 'robot_head': 3個
    マーカー1: X=1.617, Y=0.197, Z=0.903
    ...
```

**実装箇所**: `marker_server.py:612-629`

---

#### 3. testモード ✅ **開発で頻繁に使用**

**用途**: 開発・テスト用、OptiTrack不要でダミーデータ生成

**特徴**:
- **OptiTrackへの接続不要**（NatNet不使用）
- ダミーデータを120 FPS相当でリアルタイム生成
- HTTPサーバーで `/latest` と `/marker_set` を提供
- 周期的な動き（10秒周期）をシミュレート

**起動コマンド**:
```bash
python marker_server.py --mode test --port 6000
```

**出力例**:
```
🔧 Running in TEST mode (no OptiTrack connection)
テストモードで起動しています...
HTTPサーバを起動しています...
テストデータ生成開始（120 FPS相当）...
テストモード起動完了: http://0.0.0.0:6000/latest
ダミーデータをHTTPで提供します（/latest, /marker_set）
停止するには Ctrl+C を押してください
```

**ダミーデータ内容**:
- `robot_body`: 5個のマーカー（z座標が異なる）
- `robot_head`: 3個のマーカー（直線上に配置）
- `obstacles`: 5個のマーカー（障害物）
- 周期的な動き: sin/cos関数で位置が変化

**実装箇所**:
- モード処理: `marker_server.py:652-674`
- ダミーデータ生成: `marker_server.py:340-468`
- データ更新スレッド: `marker_server.py:470-487`

---

### コマンドライン引数

| 引数 | 型 | デフォルト | 説明 |
|------|-------|-----------|------|
| `--mode` | str | `server` | 動作モード: `server`, `print`, `test` |
| `--server-ip` | str | `localhost` | Motive（OptiTrack）サーバーのIPアドレス |
| `--client-ip` | str | `localhost` | このPC（クライアント）のIPアドレス |
| `--port` | int | `6000` | HTTPサーバーのポート番号 |
| `--multicast` | flag | `False` | マルチキャスト接続を使用（デフォルトはユニキャスト） |
| `--interval` | float | `1.0` | printモード時の出力間隔（秒） |

**IP アドレスの解決順序**:
1. コマンドライン引数（`--server-ip`, `--client-ip`）
2. 環境変数（`NATNET_SERVER_IP`, `NATNET_CLIENT_IP`）
3. デフォルト値（`localhost`）

**例**:
```bash
# 環境変数で設定
export NATNET_SERVER_IP=192.168.1.100
export NATNET_CLIENT_IP=192.168.1.50
python marker_server.py --mode server

# コマンドライン引数で設定（環境変数を上書き）
python marker_server.py --mode server \
  --server-ip 192.168.1.100 \
  --client-ip 192.168.1.50
```

**実装箇所**: `marker_server.py:721-741`

---

### HTTP API仕様

#### エンドポイント1: `GET /latest`

**説明**: 最新のスナップショット全体を取得

**URL**: `http://<host>:<port>/latest`

**レスポンス**:
```json
{
  "ok": true,
  "snapshot": {
    "timestamp": 1705384212.345,
    "frame": 871510,
    "labeled_markers": [
      {"id": 65537, "pos": [1.617, 0.197, 0.903]},
      {"id": 65538, "pos": [1.638, 0.192, 0.905]},
      ...
    ],
    "marker_sets": [
      {
        "name": "robot_head",
        "markers": [
          [1.617, 0.197, 0.903],
          [1.638, 0.192, 0.905],
          [1.598, 0.192, 0.898]
        ]
      },
      {
        "name": "robot_body",
        "markers": [
          [1.600, 0.200, 0.850],
          [1.590, 0.210, 0.840],
          ...
        ]
      },
      ...
    ],
    "unlabeled_markers": [],
    "legacy_other_markers": [
      [1.636, 0.273, 0.825],
      ...
    ],
    "rigid_bodies": [
      {
        "id": 1,
        "pos": [1.618, 0.194, 0.902],
        "tracking_valid": true
      }
    ]
  }
}
```

**使用例**:
```bash
curl http://localhost:6000/latest | jq .
```

**実装箇所**: `marker_server.py:500-513`

---

#### エンドポイント2: `GET /marker_set?name=<name>`

**説明**: 特定のマーカーセットのみを取得

**URL**: `http://<host>:<port>/marker_set?name=<name>`

**パラメータ**:
- `name` (required): マーカーセット名（例: `robot_head`, `robot_body`, `obstacles`）

**レスポンス（成功時）**:
```json
{
  "ok": true,
  "name": "robot_head",
  "timestamp": 1705384212.345,
  "frame": 871510,
  "count": 3,
  "markers": [
    [1.617, 0.197, 0.903],
    [1.638, 0.192, 0.905],
    [1.598, 0.192, 0.898]
  ],
  "found": true
}
```

**レスポンス（マーカーセットが見つからない場合）**:
```json
{
  "ok": true,
  "name": "unknown_set",
  "timestamp": 1705384212.345,
  "frame": 871510,
  "count": 0,
  "markers": [],
  "found": false
}
```

**レスポンス（nameパラメータ未指定時）**:
```json
{
  "ok": false,
  "error": "missing 'name' parameter"
}
```

**使用例**:
```bash
curl "http://localhost:6000/marker_set?name=robot_head" | jq .
```

**実装箇所**: `marker_server.py:515-563`

---

### 内部実装

#### クラス構造

```python
class SimpleMarkerTracker:
    def __init__(self, client_ip, server_ip, use_multicast,
                 print_interval, mode, server_port):
        """
        初期化
        - NatNetClient作成
        - 接続設定
        - スナップショット管理用のロックとバッファ
        """

    def setup_client(self):
        """NatNetクライアントの設定"""

    def receive_new_frame(self, data_dict):
        """
        NatNetコールバック（フレーム受信時）
        - スナップショット生成
        - プリントモード時は定期出力
        """

    def build_snapshot(self, data_dict):
        """
        MoCapDataオブジェクトからJSON化可能なスナップショットを生成
        - labeled_markers, marker_sets, rigid_bodiesなどを抽出
        - bytes文字列のデコード処理
        """

    def get_latest_snapshot(self):
        """最新スナップショットを取得（スレッドセーフ）"""

    def generate_test_snapshot(self, frame_number):
        """testモード用のダミースナップショットを生成"""

    def update_test_data(self):
        """testモード用：120 FPSでダミーデータを更新"""

    def serve_latest_http(self):
        """HTTPサーバーを起動（/latest, /marker_set）"""

    def start_tracking(self):
        """
        トラッキング開始（メインループ）
        - modeに応じた処理を実行
        - Ctrl+Cで安全停止
        """

    def safe_shutdown(self):
        """安全なシャットダウン処理"""
```

**ファイルパス**: `marker_server.py:88-719`

---

#### スレッド構成

marker_server.pyは複数のスレッドで動作します。

**serverモード / testモード**:
```
[メインスレッド]
  └─ start_tracking() でイベントループ（無限ループ）

[HTTPサーバースレッド]（daemon=True）
  └─ serve_latest_http()
      └─ ThreadingHTTPServer で /latest, /marker_set を提供

[NatNet受信スレッド]（serverモードのみ、NatNetClient内部）
  └─ receive_new_frame() コールバック
      └─ スナップショット更新（_snapshot_lock で排他制御）

[テストデータ更新スレッド]（testモードのみ、daemon=True）
  └─ update_test_data()
      └─ 120 FPS でダミーデータ生成・更新
```

**スレッドセーフ実装**:
- `_snapshot_lock` (threading.Lock) でスナップショットへのアクセスを排他制御
- HTTPサーバーは `ThreadingHTTPServer` で複数リクエストを並行処理
- daemon=True で親プロセス終了時に自動終了

**実装箇所**:
- ロック: `marker_server.py:116`
- HTTPスレッド起動: `marker_server.py:633-634, 658-659`
- テストスレッド起動: `marker_server.py:662-663`

---

#### エラーハンドリング

**JSON化エラー対策**:
- bytes文字列の自動デコード（UTF-8 → CP932フォールバック）
- シリアライズ失敗時もサーバー継続

```python
# bytes文字列のデコード（marker_server.py:280-291）
if isinstance(name, bytes):
    try:
        name = name.decode('utf-8')
    except Exception:
        try:
            name = name.decode('cp932', errors='replace')
        except Exception:
            name = str(name)
```

**HTTP例外処理**:
- 500エラー時もスレッドを落とさず継続
- エラー詳細をJSONで返却

```python
# marker_server.py:572-586
except Exception as e:
    err = json.dumps({"ok": False, "error": str(e)}).encode('utf-8')
    self.send_response(500)
    self.send_header("Content-Type", "application/json; charset=utf-8")
    self.end_headers()
    self.wfile.write(err)
```

**シャットダウン処理**:
- ソケット未初期化時の安全停止
- Ctrl+C での確実な終了

```python
# marker_server.py:697-719
def safe_shutdown(self):
    try:
        if (hasattr(self.natnet_client, 'command_socket') and
            self.natnet_client.command_socket is not None):
            self.natnet_client.shutdown()
        else:
            self.natnet_client.stop_threads = True
    except Exception as e:
        print(f"シャットダウン中にエラー: {e}")
```

---

### 使用例 (marker_server.py)

#### 例1: テストモードで起動（開発・テスト用）✅

```bash
python marker_server.py --mode test --port 6000
```

**出力**:
```
🔧 Running in TEST mode (no OptiTrack connection)
テストモードで起動しています...
HTTPサーバを起動しています...
HTTPサーバ起動: http://0.0.0.0:6000/latest
テストデータ生成開始（120 FPS相当）...
テストモード起動完了: http://0.0.0.0:6000/latest
ダミーデータをHTTPで提供します（/latest, /marker_set）
停止するには Ctrl+C を押してください
```

**データ確認**:
```bash
# 別ターミナルで
curl http://localhost:6000/latest | jq '.snapshot.marker_sets[] | select(.name == "robot_head")'
```

---

#### 例2: 実機モードで起動（OptiTrack接続）

```bash
python marker_server.py --mode server \
  --server-ip 192.168.1.100 \
  --client-ip 192.168.1.50 \
  --port 6000
```

**出力**:
```
HTTPサーバを起動しています...
HTTPサーバ起動: http://0.0.0.0:6000/latest
NatNetに接続中...
接続成功！最新データをHTTPで提供します（/latest, /marker_set）
停止するには Ctrl+C を押してください
```

**Motive側の設定**:
1. Edit > Preferences > Streaming
2. Local Interface: 192.168.1.100
3. Broadcast Frame Data: ON
4. Type: Unicast（または Multicast）

---

#### 例3: プリントモードで起動（デバッグ用）

```bash
python marker_server.py --mode print \
  --server-ip 192.168.1.100 \
  --client-ip 192.168.1.50 \
  --interval 0.5
```

**出力**:
```
=== シンプルマーカートラッカー ===
クライアントIP: 192.168.1.50
サーバーIP: 192.168.1.100
接続方式: ユニキャスト
出力間隔: 0.5秒
========================================
接続中...
接続成功！マーカーデータを受信中...
停止するには Ctrl+C を押してください
データ受信開始...
フレーム受信カウント: 60

[18:10:12] フレーム: 871510
--------------------------------------------------
ラベル付きマーカー数: 6
  マーカー1 (ID:65537): X=1.617, Y=0.197, Z=0.903
  ...
```

---

## marker_client.py - クライアントライブラリ

### 役割と機能

**marker_client.py** は、marker_server.pyが提供するHTTP APIにアクセスし、ロボット位置や障害物情報を取得するPythonライブラリです。`control_pc.py`で使用されます。

**主要機能**:
1. **HTTPクライアント**: marker_server.pyからデータ取得
2. **ロボット位置計算**: robot_bodyとrobot_headから位置・方向を計算
3. **障害物取得**: obstaclesマーカーセットから障害物座標を取得
4. **エラーハンドリング**: 接続エラー、タイムアウト、JSONデコードエラーに対応
5. **接続テスト**: marker_serverへの接続確認

**ファイルパス**: `marker_client.py` (441行)

**主要クラス**: `MarkerTrackerClient`

---

### クラス仕様

#### クラス定義

```python
class MarkerTrackerClient:
    """
    marker_server.py (HTTPサーバー) からマーカーデータを取得するクライアント
    """

    def __init__(self, host='localhost', port=6000, timeout=5.0):
        """
        Args:
            host (str): marker_serverのホスト
            port (int): marker_serverのポート
            timeout (float): HTTPリクエストのタイムアウト（秒）
        """
        self.host = host
        self.port = port
        self.base_url = f"http://{host}:{port}"
        self.timeout = timeout
```

**実装箇所**: `marker_client.py:35-54`

---

### 主要メソッド

#### 1. `get_latest()` - 最新スナップショット全体取得

**説明**: `/latest` エンドポイントから最新のスナップショット全体を取得

**シグネチャ**:
```python
def get_latest(self) -> Optional[Dict]
```

**戻り値**:
```python
{
    'ok': True,
    'snapshot': {
        'timestamp': float,
        'frame': int,
        'labeled_markers': [...],
        'marker_sets': [...],
        'unlabeled_markers': [...],
        'legacy_other_markers': [...],
        'rigid_bodies': [...]
    }
}
```

**使用例**:
```python
client = MarkerTrackerClient(host='localhost', port=6000)
result = client.get_latest()

if result and result['ok']:
    snapshot = result['snapshot']
    print(f"フレーム: {snapshot['frame']}")
    print(f"マーカーセット数: {len(snapshot['marker_sets'])}")
```

**エラーハンドリング**:
- 接続エラー → `None` を返す
- タイムアウト → `None` を返す
- JSONデコードエラー → `None` を返す

**実装箇所**: `marker_client.py:56-96`

---

#### 2. `get_marker_set(name)` - 特定マーカーセット取得

**説明**: `/marker_set` エンドポイントから特定のマーカーセットのみを取得

**シグネチャ**:
```python
def get_marker_set(self, name: str) -> Optional[Dict]
```

**引数**:
- `name` (str): マーカーセット名（例: `'robot_head'`, `'obstacles'`）

**戻り値**:
```python
{
    'ok': True,
    'name': 'robot_head',
    'timestamp': 1705384212.345,
    'frame': 871510,
    'count': 3,
    'markers': [
        [1.617, 0.197, 0.903],
        [1.638, 0.192, 0.905],
        [1.598, 0.192, 0.898]
    ],
    'found': True
}
```

**使用例**:
```python
result = client.get_marker_set('robot_head')

if result and result['ok'] and result['found']:
    markers = result['markers']
    print(f"マーカー数: {result['count']}")
    for i, pos in enumerate(markers):
        print(f"  マーカー{i+1}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
```

**実装箇所**: `marker_client.py:98-144`

---

#### 3. `get_robot_position()` - ロボット位置・方向取得 ⭐ **重要**

**説明**: robot_bodyとrobot_headマーカーセットからロボットの位置と方向を計算

**シグネチャ**:
```python
def get_robot_position(self,
                      body_marker_set='robot_body',
                      head_marker_set='robot_head') -> Optional[Dict[str, float]]
```

**引数**:
- `body_marker_set` (str): ロボット本体のマーカーセット名（デフォルト: `'robot_body'`）
- `head_marker_set` (str): ロボット頭部のマーカーセット名（デフォルト: `'robot_head'`）

**戻り値**:
```python
{
    'x': 1.600,      # ロボット位置のx座標 [m]
    'y': 0.200,      # ロボット位置のy座標 [m]
    'z': 0.850,      # ロボット位置のz座標 [m]
    'fd': 45.0,      # 放射方向（head 3点の並び方向）[度、0-360]
    'pd': 30.0       # 頭部方向（bodyからheadへの方向）[度、0-360]
}
```

**処理フロー**:
```
1. robot_bodyマーカーセットを取得
   └─ z座標が最大のマーカーをロボット位置とする

2. robot_headマーカーセットを取得（3個のマーカー必須）
   └─ 3点の真ん中の点をヘッド位置とする（PCAで計算）

3. pd（頭部方向）を計算
   └─ bodyからheadへのベクトルの角度 [度]

4. fd（放射方向）を計算
   └─ head 3点の主軸方向（PCAで計算）[度]
```

**使用例**:
```python
position = client.get_robot_position()

if position:
    print(f"位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
    print(f"fd（放射方向）: {position['fd']:.1f}度")
    print(f"pd（頭部方向）: {position['pd']:.1f}度")
```

**実装箇所**: `marker_client.py:146-240`

---

#### 4. `get_obstacles(marker_set_name)` - 障害物座標取得

**説明**: 障害物位置をマーカーセットから取得

**シグネチャ**:
```python
def get_obstacles(self, marker_set_name='obstacles') -> Tuple[np.ndarray, np.ndarray]
```

**引数**:
- `marker_set_name` (str): 障害物のマーカーセット名（デフォルト: `'obstacles'`）

**戻り値**:
```python
(obs_x, obs_y)  # NumPy配列

# obs_x: [2.5, 1.0, 3.0, 0.5, 3.5]
# obs_y: [1.5, 2.0, 3.0, 0.5, 1.0]
```

**使用例**:
```python
obs_x, obs_y = client.get_obstacles('obstacles')

print(f"障害物数: {len(obs_x)}")
for i in range(len(obs_x)):
    print(f"  障害物{i+1}: ({obs_x[i]:.3f}, {obs_y[i]:.3f})")
```

**注意**:
- マーカーセットが見つからない場合は空配列 `(np.array([]), np.array([]))` を返す
- z座標は無視（2次元平面のみ）

**実装箇所**: `marker_client.py:321-360`

---

#### 5. `test_connection()` - 接続テスト

**説明**: marker_serverへの接続テストを実行

**シグネチャ**:
```python
def test_connection(self) -> bool
```

**戻り値**:
- `True`: 接続成功
- `False`: 接続失敗

**使用例**:
```python
client = MarkerTrackerClient(host='localhost', port=6000)

if client.test_connection():
    print("接続成功！")
else:
    print("接続失敗...")
```

**出力例**:
```
marker_server接続テスト: http://localhost:6000
✓ 接続成功: フレーム=871510, タイムスタンプ=1705384212.345
  マーカーセット数: 3
  利用可能なマーカーセット: ['robot_body', 'robot_head', 'obstacles']
  剛体数: 2
```

**実装箇所**: `marker_client.py:362-398`

---

### ロボット位置計算ロジック

#### 計算アルゴリズム

**1. robot_bodyからロボット位置を決定**

```python
# z座標が最大のマーカーをロボット位置とする
body_markers_array = np.array(body_markers)
max_z_idx = np.argmax(body_markers_array[:, 2])
body_pos = body_markers_array[max_z_idx]

# body_pos = [x, y, z]
```

**実装箇所**: `marker_client.py:186-188`

**理由**: ロボット本体の最も高い点（頭部に最も近い点）をロボット位置とするため

---

**2. robot_headから頭部位置を決定**

```python
# 3点の真ん中の点を見つける（PCAで主軸方向を求め、射影値でソート）
middle_idx = self._find_middle_point(head_markers_array)
head_pos = head_markers_array[middle_idx]

# head_pos = [x, y, z]
```

**実装箇所**: `marker_client.py:212-213`

**`_find_middle_point()` の内部処理**:
```python
def _find_middle_point(self, points: np.ndarray) -> int:
    # 重心を計算
    centroid = np.mean(points, axis=0)

    # 重心を原点とする座標系に変換
    centered = points - centroid

    # 共分散行列を計算（3D）
    cov = np.cov(centered.T)

    # 固有値と固有ベクトルを計算
    eigenvalues, eigenvectors = np.linalg.eig(cov)

    # 最大固有値に対応する固有ベクトル（主軸方向）
    principal_axis = eigenvectors[:, np.argmax(eigenvalues)]

    # 各点を主軸に射影
    projections = centered @ principal_axis

    # 射影値でソートして中央のインデックスを返す
    sorted_indices = np.argsort(projections)
    middle_idx = sorted_indices[1]  # 中央の点

    return middle_idx
```

**実装箇所**: `marker_client.py:242-276`

**理由**: 3点が直線上に並んでいると仮定し、主成分分析（PCA）で主軸方向を求め、その方向に射影した値でソートして中央の点を選択

---

**3. pd（頭部方向）を計算**

```python
# bodyからheadへの方向角度
dx = head_pos[0] - body_pos[0]
dy = head_pos[1] - body_pos[1]
pd = np.degrees(np.arctan2(dy, dx))

# 0-360度の範囲に正規化
if pd < 0:
    pd += 360
```

**実装箇所**: `marker_client.py:219-224`

**意味**: ロボット本体から頭部への方向を表す角度（ロボットが向いている方向）

---

**4. fd（放射方向）を計算**

```python
def _calculate_head_direction(self, head_markers: np.ndarray) -> float:
    # 重心を計算
    centroid = np.mean(head_markers, axis=0)

    # 重心を原点とする座標系に変換
    centered = head_markers - centroid

    # xy平面での主軸を求める（z方向は無視）
    centered_2d = centered[:, :2]

    # 共分散行列を計算（2D）
    cov_2d = np.cov(centered_2d.T)

    # 固有値と固有ベクトルを計算
    eigenvalues, eigenvectors = np.linalg.eig(cov_2d)

    # 最大固有値に対応する固有ベクトル（主軸方向）
    principal_axis_2d = eigenvectors[:, np.argmax(eigenvalues)]

    # 方向を角度に変換
    fd = np.degrees(np.arctan2(principal_axis_2d[1], principal_axis_2d[0]))

    # 0-360度の範囲に正規化
    if fd < 0:
        fd += 360

    return float(fd)
```

**実装箇所**: `marker_client.py:278-319`

**意味**: ロボット頭部の3点が並ぶ方向（パルス放射方向）

---

#### 座標系とマーカー配置

```
z軸（高さ）
↑
│
│      robot_head（3個のマーカー、直線上）
│      ●─────●─────●
│       ↑     ↑     ↑
│     先端  真ん中  後端
│      ｜     ｜
│      ｜   head_pos（真ん中の点）
│      ｜
│   robot_body（5個のマーカー、z座標が異なる）
│      ●  ← z最大（最も高い）= body_pos（ロボット位置）
│      ●
│      ●
│      ●
│      ●  ← z最小
│
└─────────────────────→ x軸, y軸（水平面）

pd（頭部方向）: body_pos → head_pos のベクトルの角度
fd（放射方向）: head 3点の主軸方向（直線の向き）
```

---

### 使用例 (marker_client.py)

#### 例1: ロボット位置と障害物を取得

```python
from marker_client import MarkerTrackerClient

# クライアント初期化
client = MarkerTrackerClient(host='localhost', port=6000)

# 接続テスト
if not client.test_connection():
    print("marker_serverに接続できません")
    exit(1)

# ロボット位置を取得
position = client.get_robot_position(
    body_marker_set='robot_body',
    head_marker_set='robot_head'
)

if position:
    print(f"ロボット位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
    print(f"放射方向: {position['fd']:.1f}度")
    print(f"頭部方向: {position['pd']:.1f}度")
else:
    print("ロボット位置を取得できませんでした")

# 障害物を取得
obs_x, obs_y = client.get_obstacles('obstacles')
print(f"障害物数: {len(obs_x)}")
for i in range(len(obs_x)):
    print(f"  障害物{i+1}: ({obs_x[i]:.3f}, {obs_y[i]:.3f})")
```

**出力例**:
```
MarkerTrackerClient初期化: http://localhost:6000
marker_server接続テスト: http://localhost:6000
✓ 接続成功: フレーム=871510, タイムスタンプ=1705384212.345
  マーカーセット数: 3
  利用可能なマーカーセット: ['robot_body', 'robot_head', 'obstacles']
  剛体数: 2
  robot_body: 5個のマーカーからz最大点を選択
    選択された点（インデックス0）: (1.600, 0.200, 0.850)
  robot_head: 3個のマーカーから真ん中の点を選択
    選択された点（インデックス1）: (1.638, 0.192, 0.905)
✓ ロボット位置取得: (1.600, 0.200, 0.850)
  fd（放射方向）=45.0度, pd（頭部方向）=30.0度
ロボット位置: (1.600, 0.200, 0.850)
放射方向: 45.0度
頭部方向: 30.0度
✓ 障害物データ取得: 5個
  障害物1: 位置=(2.500, 1.500, 0.100)
  障害物2: 位置=(1.000, 2.000, 0.100)
  障害物3: 位置=(3.000, 3.000, 0.100)
  障害物4: 位置=(0.500, 0.500, 0.100)
  障害物5: 位置=(3.500, 1.000, 0.100)
障害物数: 5
  障害物1: (2.500, 1.500)
  障害物2: (1.000, 2.000)
  障害物3: (3.000, 3.000)
  障害物4: (0.500, 0.500)
  障害物5: (3.500, 1.000)
```

---

#### 例2: control_pc.pyでの使用

```python
# control_pc.py の一部（簡略版）
from marker_client import MarkerTrackerClient

class ControlPC:
    def __init__(self, ...):
        # MarkerTrackerClientの初期化
        self.marker_server = MarkerTrackerClient(
            host='localhost',
            port=6000,
            timeout=5.0
        )

        # 初期位置を取得
        initial_position = self.get_robot_position_from_marker_server()

        # 障害物を取得
        obs_x, obs_y = self.marker_server.get_obstacles('obstacles')

        # Worldオブジェクトに設定
        self.world = World(...)
        self.world.obs_x = obs_x
        self.world.obs_y = obs_y

    def get_robot_position_from_marker_server(self):
        """marker_serverからロボット位置を取得"""
        try:
            position = self.marker_server.get_robot_position(
                body_marker_set='robot_body',
                head_marker_set='robot_head'
            )

            if position:
                return {
                    'x': position['x'],
                    'y': position['y'],
                    'fd': position['fd'],
                    'pd': position['pd']
                }
            else:
                # フォールバック: デフォルト位置
                return {'x': 0.0, 'y': 0.0, 'fd': 90.0, 'pd': 0.0}

        except Exception as e:
            print(f"[WARNING] marker_server接続失敗: {e}")
            return {'x': 0.0, 'y': 0.0, 'fd': 90.0, 'pd': 0.0}
```

**実装箇所**: `control_pc.py:55-100, 209-240`

---

#### 例3: スタンドアロンテスト

marker_client.pyはスタンドアロンで実行可能です。

```bash
# 事前にmarker_serverをtestモードで起動
python marker_server.py --mode test --port 6000

# 別ターミナルでテスト実行
python marker_client.py
```

**出力例**:
```
╔══════════════════════════════════════════════════════════╗
║        MarkerTrackerClient テスト                        ║
║                                                          ║
║  事前に marker_server.py をtestモードで起動:            ║
║  python marker_server.py --mode test --port 6000        ║
╚══════════════════════════════════════════════════════════╝

MarkerTrackerClient初期化: http://localhost:6000

============================================================
1. 接続テスト
============================================================
marker_server接続テスト: http://localhost:6000
✓ 接続成功: フレーム=871510, タイムスタンプ=1705384212.345
  マーカーセット数: 3
  利用可能なマーカーセット: ['robot_body', 'robot_head', 'obstacles']
  剛体数: 2

============================================================
2. ロボット位置取得テスト
============================================================
  robot_body: 5個のマーカーからz最大点を選択
    選択された点（インデックス0）: (1.600, 0.200, 0.850)
  robot_head: 3個のマーカーから真ん中の点を選択
    選択された点（インデックス1）: (1.638, 0.192, 0.905)
✓ ロボット位置取得: (1.600, 0.200, 0.850)
  fd（放射方向）=45.0度, pd（頭部方向）=30.0度
位置: x=1.600m, y=0.200m, z=0.850m
方向: fd=45.0度, pd=30.0度

============================================================
3. 障害物取得テスト
============================================================
✓ 障害物データ取得: 5個
  障害物1: 位置=(2.500, 1.500, 0.100)
  障害物2: 位置=(1.000, 2.000, 0.100)
  障害物3: 位置=(3.000, 3.000, 0.100)
  障害物4: 位置=(0.500, 0.500, 0.100)
  障害物5: 位置=(3.500, 1.000, 0.100)
障害物数: 5
最初の障害物: x=2.500m, y=1.500m
```

**実装箇所**: `marker_client.py:402-441`

---

## marker_test.py - テスト・デバッグツール

### 役割と機能

**marker_test.py** は、marker_server.pyの動作確認とデバッグを行うためのスタンドアロンツールです。HTTPでデータを取得し、見やすい形式でコンソールに表示します。

**主要機能**:
1. **全データ表示**: `/latest` エンドポイントから全データを取得して整形表示
2. **特定マーカーセット表示**: `/marker_set` エンドポイントで特定セットのみ表示
3. **定期ポーリング**: 指定間隔（デフォルト1秒）でデータを更新
4. **生JSON出力**: `--raw` オプションで機械可読なJSON形式で出力

**ファイルパス**: `marker_test.py` (274行)

**用途**: 開発・デバッグ時のみ使用（control_pc.pyからは使用しない）

---

### コマンドライン引数

| 引数 | 型 | デフォルト | 説明 |
|------|-------|-----------|------|
| `--host` | str | `localhost` | marker_serverのホスト |
| `--port` | int | `6000` | marker_serverのポート |
| `--interval` | float | `1.0` | ポーリング間隔（秒） |
| `--raw` | flag | `False` | 生のJSONで出力（整形なし） |
| `--model` | str | `None` | 特定マーカーセットのみ表示（例: `robot_head`） |

**実装箇所**: `marker_test.py:218-225`

---

### 出力フォーマット

#### 整形表示モード（デフォルト）

```
[時刻 18:10:12] フレーム番号: 871510
ラベル付きマーカー数: 6 | マーカーセット数: 2 | ラベルなしマーカー数: 0 | レガシーマーカー数: 3 | 剛体数: 1

- ラベル付きマーカー一覧:
  [1] ID=65537 位置=(1.617, 0.197, 0.903)
  [2] ID=65538 位置=(1.638, 0.192, 0.905)
  [3] ID=65539 位置=(1.598, 0.192, 0.898)
  [4] ID=16027 位置=(1.636, 0.273, 0.825)
  [5] ID=16029 位置=(2.016, -0.032, 0.888)
  [6] ID=16454 位置=(1.985, -0.009, 0.760)

- マーカーセット一覧:
  [1] セット名='robot_head' マーカー数=3
      マーカー1: X=1.617, Y=0.197, Z=0.903
      マーカー2: X=1.638, Y=0.192, Z=0.905
      マーカー3: X=1.598, Y=0.192, Z=0.898
  [2] セット名='robot_body' マーカー数=5
      マーカー1: X=1.600, Y=0.200, Z=0.850
      マーカー2: X=1.590, Y=0.210, Z=0.840
      マーカー3: X=1.610, Y=0.190, Z=0.835
      マーカー4: X=1.595, Y=0.205, Z=0.830
      マーカー5: X=1.605, Y=0.195, Z=0.825

- レガシーその他マーカー一覧:
  [1] X=1.636, Y=0.273, Z=0.825
  [2] X=2.016, Y=-0.032, Z=0.888
  [3] X=1.985, Y=-0.009, Z=0.760

- 剛体一覧:
  [1] ID=1 位置=(1.618, 0.194, 0.902) 追跡有効=True
```

**実装箇所**: `marker_test.py:125-215`

---

#### 生JSONモード（`--raw`）

```json
{"timestamp": 1705384212.345, "frame": 871510, "labeled_markers": [{"id": 65537, "pos": [1.617, 0.197, 0.903]}, {"id": 65538, "pos": [1.638, 0.192, 0.905]}, ...], "marker_sets": [{"name": "robot_head", "markers": [[1.617, 0.197, 0.903], [1.638, 0.192, 0.905], [1.598, 0.192, 0.898]]}, ...], "unlabeled_markers": [], "legacy_other_markers": [[1.636, 0.273, 0.825], ...], "rigid_bodies": [{"id": 1, "pos": [1.618, 0.194, 0.902], "tracking_valid": true}]}
```

**用途**: パイプやリダイレクトでログファイルに保存、別プログラムで解析

---

#### 特定マーカーセット表示モード（`--model`）

```
[時刻 18:10:35] マーカーセット='robot_head' マーカー数=3
  マーカー1: X=1.617, Y=0.197, Z=0.903
  マーカー2: X=1.638, Y=0.192, Z=0.905
  マーカー3: X=1.598, Y=0.192, Z=0.897
```

**実装箇所**: `marker_test.py:237-252`

---

### 使用例 (marker_test.py)

#### 例1: 全データを整形表示（最も一般的）

```bash
python marker_test.py --host localhost --port 6000
```

**出力**:
```
マーカーサーバへポーリング: http://localhost:6000/latest  （1.0秒ごと, Ctrl+Cで停止）
[時刻 18:10:12] フレーム番号: 871510
ラベル付きマーカー数: 6 | マーカーセット数: 2 | ...
...
[時刻 18:10:13] フレーム番号: 871630
...
```

**用途**: marker_server.pyが正常に動作しているか確認

---

#### 例2: 生JSONで出力してファイルに保存

```bash
python marker_test.py --host localhost --port 6000 --raw > output.jsonl
```

**出力（output.jsonl）**:
```json
{"timestamp": 1705384212.345, "frame": 871510, ...}
{"timestamp": 1705384213.345, "frame": 871630, ...}
{"timestamp": 1705384214.345, "frame": 871750, ...}
...
```

**用途**: データログ収集、後処理・解析

---

#### 例3: 特定マーカーセットのみ表示

```bash
python marker_test.py --host localhost --port 6000 --model robot_head
```

**出力**:
```
マーカーサーバへポーリング: http://localhost:6000/marker_set?name=robot_head  （1.0秒ごと, Ctrl+Cで停止）
[時刻 18:10:35] マーカーセット='robot_head' マーカー数=3
  マーカー1: X=1.617, Y=0.197, Z=0.903
  マーカー2: X=1.638, Y=0.192, Z=0.905
  マーカー3: X=1.598, Y=0.192, Z=0.897
[時刻 18:10:36] マーカーセット='robot_head' マーカー数=3
...
```

**用途**: 特定のマーカーセットのみ監視

---

#### 例4: ポーリング間隔を0.5秒に変更

```bash
python marker_test.py --host localhost --port 6000 --interval 0.5
```

**用途**: より高頻度でデータを確認（デフォルトは1秒）

---

#### 例5: リモートホストに接続

```bash
python marker_test.py --host 192.168.1.50 --port 6000
```

**用途**: 別PCで動作しているmarker_server.pyを監視

---

## データ構造仕様

### スナップショット構造

marker_server.pyが提供するスナップショットの完全な構造を以下に示します。

```python
{
    # タイムスタンプ
    "timestamp": float,  # UNIXタイムスタンプ（秒、小数点以下も含む）

    # フレーム番号
    "frame": int,  # OptiTrackのフレーム番号

    # ラベル付きマーカー（個別にIDが割り当てられたマーカー）
    "labeled_markers": [
        {
            "id": int,           # マーカーID（例: 65537）
            "pos": [float, float, float]  # [x, y, z] 座標 [m]
        },
        ...
    ],

    # マーカーセット（名前付きマーカーグループ）
    "marker_sets": [
        {
            "name": str,  # セット名（例: "robot_head"）
            "markers": [
                [float, float, float],  # [x, y, z] 座標 [m]
                ...
            ]
        },
        ...
    ],

    # ラベルなしマーカー（マーカーセット内で認識されているがIDがないもの）
    "unlabeled_markers": [
        [float, float, float],  # [x, y, z] 座標 [m]
        ...
    ],

    # レガシーその他マーカー（古いバージョンのNatNetで使用）
    "legacy_other_markers": [
        [float, float, float],  # [x, y, z] 座標 [m]
        ...
    ],

    # 剛体（Motive側で定義された剛体オブジェクト）
    "rigid_bodies": [
        {
            "id": int,           # 剛体ID
            "pos": [float, float, float],  # [x, y, z] 座標 [m]
            "tracking_valid": bool  # 追跡が有効かどうか
        },
        ...
    ]
}
```

**実装箇所**: `marker_server.py:245-333`

---

### マーカーセット構造

**マーカーセット**は、複数のマーカーを1つのグループとして扱うための仕組みです。Motive側で定義します。

#### このプロジェクトで使用するマーカーセット

| マーカーセット名 | 用途 | マーカー数 | 説明 |
|-----------------|------|-----------|------|
| `robot_body` | ロボット本体 | 5個 | z座標が異なる5個のマーカー、最も高い点をロボット位置とする |
| `robot_head` | ロボット頭部 | 3個 | 直線上に配置された3個のマーカー、真ん中の点をヘッド位置とする |
| `obstacles` | 障害物 | 可変 | 各マーカーが個別の障害物を表す |

#### robot_bodyの構造

```
マーカー配置（z座標順）:
  マーカー1: [x, y, z_max]  ← 最も高い（ロボット位置として使用）
  マーカー2: [x, y, z]
  マーカー3: [x, y, z]
  マーカー4: [x, y, z]
  マーカー5: [x, y, z_min]  ← 最も低い
```

**MarkerTrackerClientでの使用**:
```python
# z座標が最大のマーカーを選択
max_z_idx = np.argmax(body_markers_array[:, 2])
body_pos = body_markers_array[max_z_idx]
```

---

#### robot_headの構造

```
マーカー配置（直線上）:
  マーカー1: [x1, y1, z1]  先端
  マーカー2: [x2, y2, z2]  真ん中（ヘッド位置として使用）
  マーカー3: [x3, y3, z3]  後端

主軸方向（PCAで計算）→ fd（放射方向）
```

**MarkerTrackerClientでの使用**:
```python
# PCAで主軸方向を求め、射影値でソートして中央の点を選択
middle_idx = self._find_middle_point(head_markers_array)
head_pos = head_markers_array[middle_idx]

# 主軸方向をfdとする
fd = self._calculate_head_direction(head_markers_array)
```

---

#### obstaclesの構造

```
マーカー配置（個別の障害物）:
  マーカー1: [x1, y1, z1]  障害物1
  マーカー2: [x2, y2, z2]  障害物2
  マーカー3: [x3, y3, z3]  障害物3
  ...
```

**MarkerTrackerClientでの使用**:
```python
# 各マーカーを個別の障害物として扱う
obs_x = markers_array[:, 0]
obs_y = markers_array[:, 1]
```

---

### 座標系と単位

#### 座標系

**OptiTrack（Motive）の座標系**:
```
       z軸（↑ 上方向、鉛直）
       │
       │
       │
       └─────── x軸（→ 水平方向）
      /
     /
    y軸（↙ 水平方向、x軸に直交）
```

**右手座標系**:
- x軸: 水平方向
- y軸: 水平方向（x軸に垂直）
- z軸: 鉛直方向（上向きが正）

---

#### 単位

| データ | 単位 | 説明 |
|--------|------|------|
| 位置座標 (x, y, z) | メートル [m] | OptiTrackから取得した生データ |
| 角度 (fd, pd) | 度 [deg] | 0-360度、`np.degrees()` で変換 |
| タイムスタンプ | 秒 [s] | UNIXタイムスタンプ、小数点以下も含む |
| フレーム番号 | なし | OptiTrackの内部カウンター |

---

#### 角度の定義

**pd（頭部方向、Pulse Direction）**:
```
       y軸
       ↑
       │
       │   head_pos
       │      ●
       │     /
       │    / pd（角度）
       │   /
       │  ● body_pos
       │
       └─────────→ x軸

pd = arctan2(head_y - body_y, head_x - body_x)
```

- 範囲: 0-360度
- x軸正方向（右）= 0度
- y軸正方向（上）= 90度
- 反時計回りに増加

---

**fd（放射方向、Forward Direction）**:
```
       y軸
       ↑
       │
       │   robot_head（3点）
       │   ●───●───●
       │    ＼  ｜  ／
       │     ＼ ｜ ／
       │      主軸方向（PCAで計算）
       │         ↓ fd（角度）
       │
       └─────────→ x軸

fd = arctan2(principal_axis_y, principal_axis_x)
```

- 範囲: 0-360度
- x軸正方向（右）= 0度
- y軸正方向（上）= 90度
- PCAで求めた主軸方向の角度

---

## 実践的な使用ワークフロー

### 開発・テスト環境

**前提**: OptiTrack不要、marker_serverのtestモードを使用

#### ワークフロー1: システム全体のテスト

```bash
# ターミナル1: marker_serverをtestモードで起動
python marker_server.py --mode test --port 6000

# ターミナル2: marker_testでデータ確認（オプション）
python marker_test.py --host localhost --port 6000

# ターミナル3: control_pcを起動
python control_pc.py

# ターミナル4: robot_simulatorを起動
python robot_simulator.py 20
```

**期待される出力**:
- marker_server: HTTPサーバー起動、120 FPSでダミーデータ生成
- marker_test: 1秒ごとにマーカーデータを表示
- control_pc: ロボット位置・障害物取得、TCP待機
- robot_simulator: 20ステップ実行、エコーデータ送信→移動指令受信

---

#### ワークフロー2: marker_clientのテスト

```bash
# ターミナル1: marker_serverをtestモードで起動
python marker_server.py --mode test --port 6000

# ターミナル2: marker_clientのテスト実行
python marker_client.py
```

**期待される出力**:
```
╔══════════════════════════════════════════════════════════╗
║        MarkerTrackerClient テスト                        ║
╚══════════════════════════════════════════════════════════╝

MarkerTrackerClient初期化: http://localhost:6000

============================================================
1. 接続テスト
============================================================
✓ 接続成功: フレーム=871510, タイムスタンプ=1705384212.345
  マーカーセット数: 3
  利用可能なマーカーセット: ['robot_body', 'robot_head', 'obstacles']

============================================================
2. ロボット位置取得テスト
============================================================
✓ ロボット位置取得: (1.600, 0.200, 0.850)
  fd（放射方向）=45.0度, pd（頭部方向）=30.0度

============================================================
3. 障害物取得テスト
============================================================
✓ 障害物データ取得: 5個
障害物数: 5
```

---

### 実機運用環境

**前提**: OptiTrackシステムと接続

#### ワークフロー3: 実機環境でのシステム起動

**事前準備（Motive側）**:
1. Edit > Preferences > Streaming
2. Local Interface: `<このPCのIP>`（例: 192.168.1.50）
3. Broadcast Frame Data: ON
4. Type: Unicast（または Multicast）
5. マーカーセットを作成:
   - `robot_body`: ロボット本体の5個のマーカー
   - `robot_head`: ロボット頭部の3個のマーカー
   - `obstacles`: 障害物のマーカー（可変）

**起動手順**:
```bash
# ターミナル1: marker_serverを実機モードで起動
python marker_server.py --mode server \
  --server-ip 192.168.1.100 \
  --client-ip 192.168.1.50 \
  --port 6000

# ターミナル2: marker_testでデータ確認（オプション）
python marker_test.py --host 192.168.1.50 --port 6000 --model robot_head

# ターミナル3: control_pcを起動
python control_pc.py

# 実機ロボット（Golang）を起動
```

**期待される出力**:
- marker_server: NatNet接続成功、HTTPサーバー起動
- marker_test: robot_headのマーカー3個を1秒ごとに表示
- control_pc: 実機からロボット位置取得、TCP待機
- 実機ロボット: エコーセンシング→control_pcに送信→移動指令受信→移動実行

---

### デバッグワークフロー

#### デバッグシナリオ1: マーカーが見つからない

**症状**:
```
[WARNING] Marker set 'robot_head' not found
Using fallback position: x=0.0, y=0.0
```

**デバッグ手順**:
```bash
# 1. marker_testで利用可能なマーカーセットを確認
python marker_test.py --host localhost --port 6000

# 出力例:
#   利用可能なマーカーセット: ['RobotHead', 'RobotBody', 'Obstacles']
#                             ↑ 名前が違う！

# 2. Motive側でマーカーセット名を確認・修正
# 3. または、control_pc.pyでマーカーセット名を変更
```

---

#### デバッグシナリオ2: marker_serverに接続できない

**症状**:
```
✗ marker_server接続エラー: [Errno 61] Connection refused
```

**デバッグ手順**:
```bash
# 1. marker_serverが起動しているか確認
ps aux | grep marker_server

# 2. ポートが正しいか確認
netstat -an | grep 6000

# 3. marker_serverを起動
python marker_server.py --mode test --port 6000

# 4. 再度接続テスト
python marker_test.py --host localhost --port 6000
```

---

#### デバッグシナリオ3: ロボット位置の計算が不正確

**症状**:
```
✓ ロボット位置取得: (1.600, 0.200, 0.850)
  fd（放射方向）=180.0度, pd（頭部方向）=270.0度
  ↑ 期待値と異なる
```

**デバッグ手順**:
```bash
# 1. 生のマーカーデータを確認
python marker_test.py --host localhost --port 6000 --model robot_body
python marker_test.py --host localhost --port 6000 --model robot_head

# 2. マーカーの配置を確認
#    - robot_body: z座標が異なる5個のマーカー
#    - robot_head: 直線上に配置された3個のマーカー

# 3. Motive側でマーカーの配置を調整

# 4. 再度確認
python marker_client.py
```

---

## トラブルシューティング

### 問題1: marker_server.pyが起動しない

#### エラー: NatNetClient が見つかりません

**症状**:
```
ModuleNotFoundError: NatNetClient が見つかりません。NatNetSDK の Python クライアントパスが通っているか確認してください。
試したパス: /Users/manato/Documents/学校系/NatNetSDK/Samples/PythonClient
```

**原因**: NatNetSDKが正しくインストールされていない、またはパスが間違っている

**解決策**:
1. NatNetSDKをダウンロード・展開
2. `NatNetSDK/Samples/PythonClient/` ディレクトリが存在することを確認
3. marker_server.pyの以下の行を環境に合わせて修正:

```python
# marker_server.py:72-76
_this_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.abspath(os.path.join(_this_dir, os.pardir))
_natnet_py_dir = os.path.join(_repo_root, 'NatNetSDK', 'Samples', 'PythonClient')
if _natnet_py_dir not in sys.path:
    sys.path.insert(0, _natnet_py_dir)
```

または、環境変数PYTHONPATHを設定:
```bash
export PYTHONPATH="/path/to/NatNetSDK/Samples/PythonClient:$PYTHONPATH"
python marker_server.py --mode test --port 6000
```

---

#### エラー: ポートが使用中

**症状**:
```
OSError: [Errno 48] Address already in use
```

**原因**: ポート6000が既に別のプロセスで使用されている

**解決策**:
```bash
# 使用中のプロセスを確認
lsof -i :6000

# プロセスを終了
kill -9 <PID>

# または、別のポートを使用
python marker_server.py --mode test --port 6001
```

---

### 問題2: OptiTrackに接続できない（実機モード）

#### エラー: 接続できませんでした

**症状**:
```
エラー: 正常に接続できませんでした。Motiveのストリーミングが有効になっているか確認してください。
```

**原因**:
- Motive側でストリーミングが無効
- IPアドレスが間違っている
- ファイアウォールでブロックされている

**解決策**:
1. **Motive側の設定を確認**:
   - Edit > Preferences > Streaming
   - Broadcast Frame Data: ON
   - Local Interface: `<MotiveのIP>`
   - Type: Unicast（または Multicast）

2. **IPアドレスを確認**:
   ```bash
   # MacOS/Linux
   ifconfig

   # Windows
   ipconfig
   ```

3. **ファイアウォールを確認**:
   - ポート1510（コマンド）と1511（データ）を許可
   - Unicastの場合、特定のIPアドレスからの接続を許可

4. **接続テスト**:
   ```bash
   # Unicast（推奨）
   python marker_server.py --mode print \
     --server-ip 192.168.1.100 \
     --client-ip 192.168.1.50

   # Multicast（ネットワーク設定が複雑な場合）
   python marker_server.py --mode print \
     --server-ip 192.168.1.100 \
     --client-ip 192.168.1.50 \
     --multicast
   ```

---

### 問題3: マーカーセットが見つからない

#### エラー: Marker set 'robot_head' not found

**症状**:
```
✗ マーカーセット 'robot_head' が見つかりません
```

**原因**:
- Motive側でマーカーセットが作成されていない
- マーカーセット名が間違っている（大文字小文字の違いなど）

**解決策**:
1. **利用可能なマーカーセットを確認**:
   ```bash
   python marker_test.py --host localhost --port 6000
   ```

   出力例:
   ```
   利用可能なマーカーセット: ['RobotHead', 'RobotBody', 'Obstacles']
   ```

2. **Motive側でマーカーセットを作成**:
   - マーカーを選択
   - 右クリック > Rigid Body > Create From Selected Markers
   - 名前を `robot_head` に設定（小文字）

3. **コード側でマーカーセット名を変更**:
   ```python
   # control_pc.py
   position = self.marker_server.get_robot_position(
       body_marker_set='RobotBody',   # 実際の名前に合わせる
       head_marker_set='RobotHead'
   )
   ```

---

### 問題4: ロボット位置の計算が不安定

#### 症状: 位置や角度が異常な値になる

**症状**:
```
✓ ロボット位置取得: (1.600, 0.200, 0.850)
  fd（放射方向）=350.0度, pd（頭部方向）=10.0度
  ↓ 次のフレーム
✓ ロボット位置取得: (1.600, 0.200, 0.850)
  fd（放射方向）=170.0度, pd（頭部方向）=190.0度
  ↑ 180度反転している！
```

**原因**:
- PCAの主軸方向に180度の曖昧性がある
- マーカーの配置が直線上にない

**解決策**:
1. **マーカーの配置を確認**:
   - robot_headの3個のマーカーが直線上に配置されているか確認
   - マーカー間の距離が均等か確認

2. **180度反転の補正**:
   ```python
   # marker_client.py の _calculate_head_direction() に追加
   # pdとfdの整合性をチェック
   if abs(fd - pd) > 90 and abs(fd - pd) < 270:
       fd = (fd + 180) % 360  # 180度反転
   ```

3. **別のアプローチ**:
   - robot_headの代わりに、方向マーカーを2個使用（前後）
   - 前後のマーカーから直接方向を計算

---

### 問題5: データ取得のタイムアウト

#### エラー: タイムアウトが発生

**症状**:
```
✗ marker_server接続エラー: timed out
```

**原因**:
- marker_serverの処理が遅い
- ネットワーク遅延
- データ量が大きすぎる

**解決策**:
1. **タイムアウト時間を延長**:
   ```python
   # marker_client.py
   client = MarkerTrackerClient(host='localhost', port=6000, timeout=10.0)  # 5秒→10秒
   ```

2. **marker_serverのパフォーマンスを確認**:
   ```bash
   # HTTPサーバーの応答時間を測定
   time curl http://localhost:6000/latest
   ```

3. **ネットワークを確認**:
   ```bash
   ping 192.168.1.50
   ```

---

## 開発者向け情報

### 拡張方法

#### 新しいマーカーセットを追加

**シナリオ**: 新しいマーカーセット `obstacles_dynamic` を追加して、動的な障害物を追跡したい

**手順**:
1. **Motive側で設定**:
   - 動的障害物のマーカーを選択
   - 右クリック > Rigid Body > Create From Selected Markers
   - 名前を `obstacles_dynamic` に設定

2. **marker_client.pyに新しいメソッドを追加**:
   ```python
   # marker_client.py に追加
   def get_dynamic_obstacles(self, marker_set_name='obstacles_dynamic'):
       """
       動的障害物をマーカーセットから取得
       """
       result = self.get_marker_set(marker_set_name)

       if result is None or not result.get('ok') or not result.get('found'):
           print(f"警告: マーカーセット '{marker_set_name}' が見つかりませんでした")
           return np.array([]), np.array([])

       markers = result.get('markers', [])
       if len(markers) == 0:
           return np.array([]), np.array([])

       markers_array = np.array(markers)
       obs_x = markers_array[:, 0]
       obs_y = markers_array[:, 1]

       return obs_x, obs_y
   ```

3. **control_pc.pyで使用**:
   ```python
   # control_pc.py の __init__() に追加
   dynamic_obs_x, dynamic_obs_y = self.marker_server.get_dynamic_obstacles()
   print(f"動的障害物数: {len(dynamic_obs_x)}")
   ```

---

#### 新しいエンドポイントを追加

**シナリオ**: `/health` エンドポイントを追加して、marker_serverの健全性をチェックしたい

**手順**:
1. **marker_server.pyに新しいハンドラーを追加**:
   ```python
   # marker_server.py の Handler.do_GET() に追加
   if path == "/health":
       # 健全性チェック
       snapshot = tracker.get_latest_snapshot()
       is_healthy = snapshot is not None

       payload = {
           "ok": True,
           "healthy": is_healthy,
           "timestamp": time.time()
       }

       body = json.dumps(payload).encode('utf-8')
       self.send_response(200 if is_healthy else 503)
       self.send_header("Content-Type", "application/json; charset=utf-8")
       self.send_header("Content-Length", str(len(body)))
       self.end_headers()
       self.wfile.write(body)
       return
   ```

2. **marker_client.pyに新しいメソッドを追加**:
   ```python
   # marker_client.py に追加
   def check_health(self) -> bool:
       """marker_serverの健全性をチェック"""
       url = f"{self.base_url}/health"

       try:
           req = urllib.request.Request(url, method='GET')
           with urllib.request.urlopen(req, timeout=self.timeout) as response:
               if response.status != 200:
                   return False

               data = response.read()
               text = data.decode('utf-8', errors='replace')
               result = json.loads(text)

               return result.get('healthy', False)

       except Exception:
           return False
   ```

3. **使用例**:
   ```python
   client = MarkerTrackerClient(host='localhost', port=6000)

   if client.check_health():
       print("marker_server is healthy")
   else:
       print("marker_server is unhealthy")
   ```

---

### パフォーマンス最適化

#### 最適化1: スナップショット更新頻度の削減

**問題**: 120 FPSでデータ更新しているが、HTTPクライアントは1 FPSしか使わない

**解決策**: testモードでデータ更新頻度を削減
```python
# marker_server.py の update_test_data() を変更
def update_test_data(self):
    frame_number = 0
    update_interval = 1.0 / 30.0  # 120 FPS → 30 FPS

    while True:
        frame_number += 1
        snapshot = self.generate_test_snapshot(frame_number)

        with self._snapshot_lock:
            self._latest_snapshot = snapshot

        time.sleep(update_interval)
```

---

#### 最適化2: HTTPレスポンスのキャッシュ

**問題**: 複数のクライアントが同時にアクセスすると、同じスナップショットを複数回JSON化する

**解決策**: JSONレスポンスをキャッシュ
```python
# marker_server.py の SimpleMarkerTracker に追加
def __init__(self, ...):
    ...
    self._cached_json = None
    self._cached_timestamp = None

def serve_latest_http(self):
    tracker = self

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if path == "/latest":
                snapshot = tracker.get_latest_snapshot()

                # キャッシュチェック
                ts = snapshot.get('timestamp') if snapshot else None
                if ts == tracker._cached_timestamp and tracker._cached_json:
                    body = tracker._cached_json
                else:
                    payload = {"ok": True, "snapshot": snapshot}
                    body = json.dumps(payload).encode('utf-8')
                    tracker._cached_json = body
                    tracker._cached_timestamp = ts

                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
```

---

### テストモードのダミーデータ仕様

testモードで生成されるダミーデータの詳細仕様を以下に示します。

#### ダミーデータの生成ロジック

```python
# marker_server.py:340-468
def generate_test_snapshot(self, frame_number):
    ts = time.time()

    # 周期的な動きをシミュレート（10秒周期）
    t = ts % 10.0
    offset_x = 0.02 * math.sin(2 * math.pi * t / 10.0)
    offset_y = 0.01 * math.cos(2 * math.pi * t / 10.0)
    offset_z = 0.015 * math.sin(2 * math.pi * t / 5.0)

    # robot_body マーカーセット（5個）
    robot_body_markers = [
        [1.600 + offset_x, 0.200 + offset_y, 0.850 + offset_z],  # z最大
        [1.590 + offset_x, 0.210 + offset_y, 0.840 + offset_z],
        [1.610 + offset_x, 0.190 + offset_y, 0.835 + offset_z],
        [1.595 + offset_x, 0.205 + offset_y, 0.830 + offset_z],
        [1.605 + offset_x, 0.195 + offset_y, 0.825 + offset_z],  # z最小
    ]

    # robot_head マーカーセット（3個、直線上）
    robot_head_markers = [
        [1.617 + offset_x, 0.197 + offset_y, 0.903 + offset_z],
        [1.638 + offset_x, 0.192 + offset_y, 0.905 + offset_z],  # 真ん中
        [1.598 + offset_x, 0.192 + offset_y, 0.898 + offset_z],
    ]

    # obstacles マーカーセット（5個、固定位置）
    obstacles_markers = [
        [2.5, 1.5, 0.1],
        [1.0, 2.0, 0.1],
        [3.0, 3.0, 0.1],
        [0.5, 0.5, 0.1],
        [3.5, 1.0, 0.1],
    ]

    ...
```

#### ダミーデータの特徴

| 項目 | 値 | 説明 |
|------|-----|------|
| 更新頻度 | 120 FPS | リアルタイム感を再現 |
| 周期 | 10秒 | offset_x, offset_y が10秒で1周期 |
| 周期 | 5秒 | offset_z が5秒で1周期 |
| offset_x | ±0.02 m | x方向の振幅 |
| offset_y | ±0.01 m | y方向の振幅 |
| offset_z | ±0.015 m | z方向の振幅 |

#### ダミーマーカーID

| マーカーセット | マーカー | ID範囲 |
|---------------|---------|--------|
| robot_body | 5個 | 60001-60005 |
| robot_head | 3個 | 65537-65539 |
| legacy_other_markers | 3個 | 16027, 16029, 16454 |
| obstacles | 5個 | 20001-20005 |

---

## まとめ

このドキュメントでは、OptiTrackマーカートラッキングシステムの3つのコンポーネント（marker_server.py、marker_client.py、marker_test.py）について詳細に説明しました。

### 主要なポイント

1. **marker_server.py**: OptiTrackからデータを取得しHTTPで配信するサーバー
   - 3つの動作モード（server, print, test）
   - testモードでOptiTrack不要で開発可能
   - `/latest` と `/marker_set` エンドポイント

2. **marker_client.py**: HTTPからデータを取得するPythonライブラリ
   - control_pc.pyで使用
   - ロボット位置・方向の計算ロジック（PCA使用）
   - 障害物座標の取得

3. **marker_test.py**: デバッグ・確認用ツール
   - データを見やすく表示
   - 定期ポーリング
   - 特定マーカーセットのみ表示可能

### 推奨される使用方法

**開発・テスト時**:
```bash
# marker_serverをtestモードで起動
python marker_server.py --mode test --port 6000

# control_pcを起動（内部でMarkerTrackerClientを使用）
python control_pc.py
```

**実機運用時**:
```bash
# marker_serverを実機モードで起動
python marker_server.py --mode server \
  --server-ip <Motive_IP> \
  --client-ip <THIS_PC_IP> \
  --port 6000

# control_pcを起動
python control_pc.py
```

---

**Document Version**: 1.0
**Generated**: 2025-11-23
**Author**: Claude Code (assisted documentation)
