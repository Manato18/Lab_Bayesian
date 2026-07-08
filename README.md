# コウモリ型ロボットのベイズ推論制御システム

コウモリのエコロケーション（超音波反響定位）を模した自律ロボット制御の実機システム。
制御PC がベイズ推論で障害物の存在確率マップ（信念分布）を更新しながら、
実機ロボット（Golang / Raspberry Pi）へ回避方向・移動距離の指令を送る。

姉妹リポジトリ [Lab_Bayesian_Simulation](https://github.com/Manato18/Lab_Bayesian_Simulation)
（純シミュレーション版）を実機向けに発展させたもの。

## システム構成（3プロセス）

```
┌─────────────────┐  HTTP(6000)  ┌──────────────────┐  TCP(6001)  ┌────────────────────┐
│ marker_server.py │ ──────────→ │  control_pc.py    │ ←─────────→ │ 実機ロボット(Go)    │
│ (Motive MoCap    │  位置・障害物 │  (ベイズ推論・     │  相互相関/   │ または             │
│  データ配信)      │              │   回避計算)       │  移動指令    │ robot_simulator.py │
└─────────────────┘              └──────────────────┘             └────────────────────┘
```

| ファイル | 役割 |
|---|---|
| `marker_server.py` | モーションキャプチャ(Motive)のデータ配信サーバー。`--mode test` で実機なしダミー配信 |
| `control_pc.py` | 本番オーケストレータ。ロボット位置取得 → Localizer で物体定位 → ベイズ更新 → 移動指令 |
| `robot_simulator.py` | 実機ロボット(Golang)の模擬クライアント。実機なしで end-to-end 検証する用 |
| `marker_test.py` | marker_server の配信データ確認ツール |

実機での起動手順は [ロボットベイズ実行時メモ.md](./ロボットベイズ実行時メモ.md)、
マーカー仕様は [marker.md](./marker.md) を参照。

## bayes_code/ パッケージ

| ファイル | 役割 |
|---|---|
| `config.py` | 全パラメータの一元管理（物理定数・空間設定・検出閾値・フラグ） |
| `world.py` | 環境（壁・障害物座標）の管理。障害物は marker_server から取得して上書きされる |
| `bayesian.py` | ベイズ更新。`update_belief` は `BeliefSnapshot`（dataclass）を返す。冒頭に**記号対応表**（`Px2L_log` 等の読み方）あり |
| `calc.py` | エコー生成・減衰・座標変換などの計算関数群。`calc()` は `SensingResult`（dataclass）を返す |
| `agent.py` | 回避ロジック。`calculate_avoidance_command`（本番経路・control_pc から呼ばれる）と `_sim_flight2`（シミュレーション経路）が共通の分析 `_analyze_posterior_for_avoidance` を使う。回避チューニング定数はファイル冒頭に集約 |
| `localization.py` | `Localizer`: 左右マイクの相互相関データからピーク検出し距離・角度を定位（実機観測の入口） |
| `robot_visualize.py` | 単一ステップの可視化（`plot_single_step`。緊急回避時はロボット色が変わる） |

## 実行方法（実機なしの end-to-end 検証）

依存パッケージは `requirements.txt`。`uv` でプロジェクト隔離の仮想環境を作る。

```bash
uv venv --python 3.12
uv pip install -r requirements.txt

# ターミナル1: テストモードの marker_server（Motive 不要）
uv run python marker_server.py --mode test --port 6000
# ターミナル2: 制御PC
uv run python control_pc.py
# ターミナル3: ロボット模擬クライアント（20ステップ実行）
uv run python robot_simulator.py 20
```

## テスト（回帰チェック）

リファクタリング中に挙動が変わっていないことを保証するため、実機・ソケットなしで
bayes_code のコア（calc → update_belief → 回避計算）をシード固定で実行し、
数値状態をベースラインと比較する回帰チェックを用意している。

```bash
# 基準となる出力を保存（変更前に一度だけ実行）
uv run python tests/test_regression.py --save-baseline
# 変更後にベースラインと一致するか確認
uv run python tests/test_regression.py
```

シナリオA（`agent.one_step` 経路）とシナリオC（本番ラッパー
`calculate_avoidance_command` 経路）の両方で、緊急回避・通常回避の両分岐をカバーする。

## リファクタリング記録

可読性向上リファクタリングの方針・各フェーズの記録・既知の疑義（`[!question]`）は
[REFACTORING_PLAN.md](./REFACTORING_PLAN.md) にまとめている。
