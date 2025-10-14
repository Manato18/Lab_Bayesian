# パラメータの流れと依存関係の調査レポート

## 概要

このドキュメントは、main.pyから各クラスへのパラメータの受け渡しと、その設計思想をまとめたものです。

## 全体のアーキテクチャ

```
config.py
    ↓ (全パラメータをインポート)
main.py
    ├─→ World (空間・時間の基礎を構築)
    │     ├─ 入力: config.pyの基礎パラメータ
    │     └─ 出力: X, Y, wall_x, wall_y, x_max, y_max, t_ax など計算済みデータ
    │
    ├─→ Bayesian (ベイズ推論エンジン)
    │     ├─ 入力: config.pyの推論パラメータ
    │     └─ 後で Init(world, agent) で初期化
    │
    ├─→ Agent (コウモリエージェント)
    │     ├─ 入力: Bayesian, World からの計算済みデータ, config.py の一部
    │     └─ コウモリの行動と認知を管理
    │
    └─→ BatVisualizer (可視化)
          ├─ 入力: World からの計算済みデータ, config.py の可視化パラメータ
          └─ 結果の可視化を担当
```

---

## 1. World クラス

### 受け取るパラメータ（main.py: 73-82行）

| パラメータ | 出典 | 目的 |
|----------|------|------|
| `x_max` | config.py | シミュレーション空間のx方向の最大値 |
| `y_max` | config.py | シミュレーション空間のy方向の最大値 |
| `margin_space` | config.py | 壁と空間の境界マージン |
| `h` | config.py | 格子点間隔（空間解像度） |
| `t_max` | config.py | エコー時間の最大レンジ |
| `dt` | config.py | 時間刻み幅 |
| `c` | config.py | 音速 |
| `folder_name` | config.py | データファイルの保存先 |

### なぜこれらを渡すのか

**World は「空間と時間の基礎」を構築するクラス**です。
- **空間**: x_max, y_max, h を使って、シミュレーション空間のメッシュグリッド（X, Y）を生成
- **時間**: t_max, dt を使って、エコー時間軸（t_ax）を生成
- **壁**: margin_space を使って、壁の座標（wall_x, wall_y）を計算
- **障害物**: folder_name を使って、CSVファイルから障害物位置を読み込み

### Worldが生成する計算済みデータ

これらは他のクラスで使用されます：
- `X, Y`: 2次元メッシュグリッド（Agent, Visualizerで使用）
- `wall_x, wall_y`: 壁の座標（Visualizerで使用）
- `x_max, y_max`: 計算に使った空間サイズ（Visualizerで使用）
- `Mx, My`: 空間格子点数（Bayesianで使用）
- `t_ax`: 時間軸配列（Visualizerで使用）
- `pole_x, pole_y`: 障害物座標（Visualizerで使用）

---

## 2. Bayesian クラス

### 受け取るパラメータ（main.py: 84-88行）

| パラメータ | 出典 | 目的 |
|----------|------|------|
| `sigma2` | config.py | ベイズ更新の分散パラメータ（観測ノイズの強さ） |
| `min_p` | config.py | 最小確率値（対数スケール、計算安定性のため） |
| `c` | config.py | 音速（距離と時間の変換に使用） |

### Init()メソッドで受け取るもの（main.py: 103行）

| パラメータ | 出典 | 目的 |
|----------|------|------|
| `world` | Worldオブジェクト | world.Mx, world.My を使って確率分布配列のサイズを決定 |
| `agent` | Agentオブジェクト | （現在は使用されていない様子） |

### なぜこれらを渡すのか

**Bayesian は「ベイズ推論エンジン」**です。
- **sigma2**: 尤度関数の計算に使用（観測データのノイズモデル）
- **min_p**: 確率が極端に小さくなった時の下限値（数値計算の安定性）
- **c**: エコー時間から距離への変換に使用
- **world (Init時)**: 空間格子のサイズ（Mx, My）を知る必要があるため

### 設計上の注意点

- `__init__` ではパラメータのみ受け取り、状態変数は None で初期化
- `Init(world, agent)` で実際の配列を初期化（Worldが先に作られている必要がある）
- config.py から直接インポート: `h, y_max, margin_space, world_wall_pos`（calculate_convergenceなどで使用）

---

## 3. Agent クラス

### 受け取るパラメータ（main.py: 90-101行）

| パラメータ | 出典 | 目的 |
|----------|------|------|
| `bayesian` | Bayesianオブジェクト | ベイズ更新を実行するために必要 |
| `margin_space` | config.py | 壁のマージンを知るため |
| `folder_name` | config.py | 位置データのCSV保存先 |
| `X` | world.X | 空間のメッシュグリッド（事後分布の解析に使用） |
| `Y` | world.Y | 空間のメッシュグリッド（事後分布の解析に使用） |
| `sim` (辞書) | config.py | `trials`: 試行回数, `init_pos`: 初期位置 |
| `world` | Worldオブジェクト | do_sensing時に使用 |

### なぜこれらを渡すのか

**Agent は「コウモリの行動と認知」を管理するクラス**です。
- **bayesian**: エコー信号から環境を推定するためのベイズ推論エンジン
- **X, Y**: 事後確率分布を解析して、最適な移動方向を決定するため
- **sim (trials, init_pos)**: シミュレーション開始時の設定（初期位置と試行回数）
- **margin_space, folder_name**: 環境情報の保存と管理
- **world**: センシング時に障害物情報（world.pole_x, world.pole_y）を取得

---

## 4. BatVisualizer クラス

### 受け取るパラメータ（main.py: 105-115行）

| パラメータ | 出典 | 目的 |
|----------|------|------|
| `output_dir` | config.output_dir_movie | 動画の保存先ディレクトリ |
| `X` | world.X | プロット用のメッシュグリッド |
| `Y` | world.Y | プロット用のメッシュグリッド |
| `c_percentile` | config.py | カラーマップの上限をパーセンタイルで設定 |
| `min_p` | config.py | カラーマップの下限値 |
| `x_max` | world.x_max | プロット範囲のx方向最大値 |
| `y_max` | world.y_max | プロット範囲のy方向最大値 |
| `wall_x` | world.wall_x | 壁の座標（プロット用） |
| `wall_y` | world.wall_y | 壁の座標（プロット用） |

### なぜこれらを渡すのか

**BatVisualizer は「結果の可視化」を担当するクラス**です。
- **X, Y**: pcolormeshでヒートマップを描画するために必要
- **x_max, y_max, wall_x, wall_y**: プロット範囲と壁の描画
- **c_percentile, min_p**: カラーマップの範囲調整（見やすい可視化のため）
- **output_dir**: 画像とGIFの保存先

---

## 設計パターンの分析

### パターン1: 直接config.pyからインポート
**使用ファイル**: calc.py, bayesian.py (一部), world.py (一部)

```python
from config import h, freq, c, a, ear_dist, ...
```

**理由**:
- 固定的な物理定数や計算で頻繁に使用するパラメータ
- クラスではなく関数ベースのコード（calc.py）

### パターン2: main.pyで受け取り、引数として渡す
**使用ファイル**: main.py → World, Bayesian, Agent, BatVisualizer

```python
world = World(x_max=config.x_max, y_max=config.y_max, ...)
```

**理由**:
- クラスの初期化時に設定を注入（依存性注入パターン）
- テストや将来的な変更が容易
- 各クラスが必要とするパラメータが明確

### パターン3: 計算済みデータの受け渡し
**使用**: World → Agent, BatVisualizer

```python
agent = Agent(..., X=world.X, Y=world.Y)
visualizer = BatVisualizer(..., X=world.X, Y=world.Y, wall_x=world.wall_x, ...)
```

**理由**:
- Worldで一度計算したメッシュグリッドや壁座標を再利用
- 計算の重複を避ける
- データの一貫性を保証

### パターン4: オブジェクト全体の受け渡し
**使用**: main.py → Agent (bayesian, world)

```python
agent = Agent(bayesian=bayesian, world=world, ...)
```

**理由**:
- AgentはBayesianのメソッドを呼び出す必要がある（update_belief など）
- Agentはdo_sensing時にWorldの障害物情報を参照する必要がある
- 密結合だが、機能的に必要な関係

---

## 現在の設計の問題点と改善案

### 問題1: 混在する設計パターン
- calc.pyは直接config.pyからインポート
- agent.py, visualize.pyは引数で受け取る
- 一貫性がない

**改善案**:
- agent.pyとvisualize.pyも必要なパラメータをconfig.pyから直接インポートする
- main.pyはオブジェクトの組み立てのみに集中

### 問題2: 重複計算
- world.py: `self.Mx, self.My = int(np.round(x_max / h)), int(np.round(y_max / h))`
- config.py: `Mx = My = int(np.round(y_max / h))`

**改善案**:
- config.pyの`Mx, My`をworldが使用する
- または、worldの計算結果をconfig.pyで定義しない

### 問題3: Init()の遅延初期化
- Bayesian.Init(world, agent)が後から呼ばれる
- 初期化の順序依存がある

**改善案**:
- Bayesian.__init__でworldを受け取る設計に変更
- または、factory関数で適切な順序で初期化

---

## 推奨される今後のリファクタリング方針

### ステップ1: agent.pyとvisualize.pyをconfigから直接読み込むように変更
```python
# agent.py
from config import margin_space, folder_name, trials, init_pos

class Agent:
    def __init__(self, bayesian, X, Y, world):
        self.margin_space = margin_space  # configから
        self.folder_name = folder_name    # configから
        self.trials = trials              # configから
        # ...
```

### ステップ2: main.pyをシンプルにする
```python
# main.py
world = World()  # configから全て読み込む
bayesian = Bayesian()  # configから全て読み込む
agent = Agent(bayesian=bayesian, world=world)
visualizer = BatVisualizer(world=world)
```

### ステップ3: 重複を排除
- config.pyに計算ロジックは入れない（Mx, My, t_axなど）
- 基礎パラメータのみconfig.pyに記載
- 計算済みデータはWorldが提供

---

## まとめ

現在の設計は**「データフロー型」**の構造になっています：

1. **config.py**: 全ての基礎パラメータを定義
2. **main.py**: パラメータを収集し、各クラスに注入（ハブの役割）
3. **World**: 基礎パラメータから計算済みデータを生成
4. **Bayesian, Agent, Visualizer**: 計算済みデータを使用

この設計は**明示的で追跡しやすい**という利点がありますが、**main.pyが肥大化しやすい**という欠点もあります。

今後は、各クラスが必要なパラメータをconfig.pyから直接読み込む設計に統一することで、よりシンプルで保守しやすいコードになると考えられます。
