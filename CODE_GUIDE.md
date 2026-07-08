# コード理解ガイド — Lab_Bayesian

このドキュメントは、初めてこのリポジトリを読む人（将来の自分を含む）が
**最短でコード全体を理解するためのロードマップ**です。
「何を実行すると・どういう処理を経て・何が出力されるか」を1枚で把握できるようにしています。

---

## 1. これは何か（30秒）

コウモリのエコロケーションを模した**実機ロボット制御システム**。
Lab_Bayesian_Simulation のベイズ推論モデルを実世界に持ち出し、
モーションキャプチャ（Motive）でロボット位置を取得しながら、
実機の超音波センサ（左右マイクの相互相関）から障害物を定位し、
ベイズ更新した信念分布に基づいて回避指令をロボットへ送る。

- **3プロセス構成**（ソケット通信）が最大の特徴
- 実機がなくても `robot_simulator.py` + `marker_server.py --mode test` で end-to-end 検証可能

## 2. システム構成と実行方法

```
┌─────────────────┐  HTTP(6000)  ┌──────────────────┐  TCP(6001)  ┌────────────────────┐
│ marker_server.py │ ──────────→ │  control_pc.py    │ ←─────────→ │ 実機ロボット(Go)    │
│ Motive MoCap配信  │  位置・障害物 │  ベイズ推論・回避   │  相互相関/   │ または             │
│ (--mode testで    │              │  ★本システムの心臓  │  移動指令    │ robot_simulator.py │
│  ダミー配信可)     │              │                   │             │ (疑似ロボット)      │
└─────────────────┘              └──────────────────┘             └────────────────────┘
```

**起動順序が重要**（実機なし検証の場合）:

```bash
uv venv --python 3.12 && uv pip install -r requirements.txt
# ターミナル1: マーカーサーバ（テストモード = Motive不要のダミーデータ）
uv run python marker_server.py --mode test --port 6000
# ターミナル2: 制御PC（マーカーサーバに接続してから待ち受け開始）
uv run python control_pc.py
# ターミナル3: 疑似ロボット（20ステップ実行）
uv run python robot_simulator.py 20
```

実機での本番手順（Raspi・モーター調整・Motive設定を含む）は
[ロボットベイズ実行時メモ.md](./ロボットベイズ実行時メモ.md) を参照。

## 3. 1ステップの処理フロー（本番経路: どのファイルが何をするか）

```
ロボット（Go / robot_simulator.py）
│  超音波センシング → 左右マイクの相互相関データ [[crosscor_l],[crosscor_r]] をTCP送信
▼
control_pc.py（1ステップの処理）
├─ ① marker_client.py 経由で marker_server から現在位置を取得
│     （robot_body=位置, robot_head=頭方向。障害物 obstacles は起動時に取得済み）
├─ ② localization.py: Localizer.localize_from_crosscor(crosscor_l, crosscor_r)
│     └─ 相互相関のピーク検出 → 左右到達時間差から障害物の距離・角度を定位
├─ ③ 検出結果から観測エコー時間 y_el / y_er を構築し、
│     bayes_code/calc.py の個別関数（r_theta_matrix, real_dist_goback_matrix,
│     dist_attenuation, direc_attenuation, sigmoid）で空間行列を計算
├─ ④ bayes_code/bayesian.py: update_belief(...)
│     └─ 尤度→事後（基本モデル+記憶保持モデル）→壁マスク → BeliefSnapshot を返す
├─ ⑤ bayes_code/agent.py: calculate_avoidance_command(現在位置, step)
│     ├─ step<10: 直進（パルスは左50°固定）
│     └─ step>=10: _analyze_posterior_for_avoidance で回避角を決定
│         （危険≥-50が近距離0.4m内→±60°緊急回避+移動50mm / なければ最安全角+150mm）
├─ ⑥ 移動指令 {NextMove[mm], NextAngle, PulseDirection} をロボットへTCP返信
└─ ⑦ bayes_code/robot_visualize.py: plot_single_step でこのステップの図を保存
▼
ロボットが移動・パルス放射 → 次のステップへ
```

**想定される出力**: `bayse_olddata2/output/` 配下
- `visualization/initial_state.png`・`frame_NNNN.png`（各ステップの6パネル図。緊急回避時はロボット色が変わる）
- `position_data.csv`（agent が記録する位置ログ）
- `movie/事後分布/`（前方視点の事後分布図）

## 4. コードリーディングのロードマップ（読む順番）

| 順 | ファイル | 注目ポイント |
|---|---|---|
| 1 | `bayes_code/config.py` | 全パラメータ一元管理。**Simulationとの違い**: 空間4.5m・margin0.5・ear_dist=0.116（実機値）・freq/threshold/grad も実機調整値 |
| 2 | `bayes_code/world.py` | 障害物はCSVでなく **marker_server から control_pc が上書き**する設計（_real_obs は無効化済み） |
| 3 | `bayes_code/bayesian.py` | **冒頭の記号対応表を先に読む**。Simulationとほぼ同一（CSV収束ログなし版） |
| 4 | `bayes_code/localization.py` | 実機ならでは。相互相関→ピーク→左右時間差→距離・角度。実機観測の入口 |
| 5 | `bayes_code/agent.py` | **2つの経路がある**: `calculate_avoidance_command`（本番・control_pcから呼ばれる）と `_sim_flight2`（シミュレーション経路）。共通ロジックは `_analyze_posterior_for_avoidance` |
| 6 | `control_pc.py` | オーケストレータ（1067行）。__init__ の接続シーケンス → メインループ → 上記フロー③④⑤の該当箇所 |
| 7 | `marker_server.py` / `marker_client.py` | MoCap配信。仕様の詳細は [marker.md](./marker.md) |
| 8 | `robot_simulator.py` | 実機（Go）のプロトコル互換の疑似クライアント。通信仕様の生きた資料 |

## 5. 大事なポイント（理解の勘所）

- **`calc()` メガ関数と `agent.do_sensing` は本番では使われない**:
  これらは agent 経由のシミュレーション経路専用。本番（control_pc）は
  Localizer で実観測を作り、calc の個別関数だけを使って update_belief を直接呼ぶ。
  「どちらの経路を読んでいるか」を常に意識すること
- **回避ロジックは2箇所にほぼ重複**（`_sim_flight2` と `calculate_avoidance_command`）。
  片方だけ変更してはいけない。回帰ハーネスが両経路（シナリオA/C）を守っている
- **単位の混在に注意**: シミュレーション経路は m（0.15/0.05）、本番の移動指令は
  mm（150/50、ロボット通信仕様）。agent.py 冒頭の定数コメント参照
- **回避の二段階構造**（Simulationと同じ）: 初回分析（step10）は前方未照射のため必ず緊急回避。
  旋回後に通常回避へ移行する
- **障害物の出どころ**: Motive 上で `obstacles` マーカーセットを組むと control_pc が
  起動時に取得して world.pole_x/pole_y に注入する

## 6. 既知の注意点・疑義

- `if step >= 6:` は step>=10 分岐の内側にあり**常に真の死んだ条件**（2箇所）。
  意図不明のため挙動保存のまま温存中 → [REFACTORING_PLAN.md](./REFACTORING_PLAN.md) の [!question]
- `Obj.Deg` のノイズ幅問題（Simulationと同様、実害なし）

## 7. 変更するときの作法

実機なしの回帰チェックで挙動不変を検証する（A=シミュ経路 / C=本番回避ラッパー経路、
緊急回避・通常回避の両分岐をカバー）:

```bash
uv run python tests/test_regression.py    # 全11項目がベースラインと一致すればOK
```

control_pc のソケット部分を触った場合は、3プロセス構成での手動 end-to-end 確認も行うこと。
詳細は [README.md](./README.md)・[REFACTORING_PLAN.md](./REFACTORING_PLAN.md) を参照。
