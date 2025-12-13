# -*- coding: utf-8 -*-
"""
Configuration File for Bat Echolocation Simulation
====================================
このファイルはシミュレーション全体で使用されるパラメータを一元管理します。

各パラメータには、使用されているファイルが記載されています。

パラメータの分類:
- 物理パラメータ: 音波、コウモリの身体的特徴など
- 時間パラメータ: エコー時間軸の設定
- 空間パラメータ: シミュレーション空間の設定
- 検出パラメータ: エコー検知の閾値とノイズ
- シミュレーションパラメータ: 試行回数、初期位置など
- ベイズ推論パラメータ: ベイズ更新のパラメータ
- 可視化パラメータ: プロット設定
- フォルダ設定: データ保存先
- フラグ設定: 動作モード切り替え
"""

import numpy as np

# ========================================
# 物理パラメータ (Physical Parameters)
# ========================================

h = 0.01              # 格子点間隔 [m]
                      # 使用: calc.py, bayesian.py, world.py

freq = 37000          # 周波数 [Hz] - コウモリが発する超音波の周波数
                      # 使用: calc.py
                      # 旧値: 40000

c = 340               # 音速 [m/s]
                      # 使用: calc.py, main.py, world.py

a = 0.0048            # 口の半径サイズ [m] - 超音波を発する口の大きさ
                      # 使用: calc.py
                      # 旧値: 0.005

ear_dist = 0.116      # 耳間距離 [m] - コウモリの左右の耳の間の距離
                      # 実機の値に合わせる（旧: 0.02, ベイズ定位点導入.py: 0.112m, 実機: 0.116m = 116mm）
                      # 使用: calc.py

# ========================================
# 時間パラメータ (Time Parameters)
# ========================================

dt = 100 * 10**-6     # エコー時間刻み幅 [s]
                      # 使用: calc.py, world.py

t_max = 64 * 10**-3   # エコー最大時間レンジ [s]
                      # 使用: world.py

Mt = int(np.round(t_max / dt))           # 時間刻み数
                      # 使用: calc.py

t_ax = np.linspace(0, t_max, Mt + 1)     # 時間軸の生成
                      # 使用: calc.py

eps_y = 1e-20         # エコー信号の背景レベル（微小値）
                      # 使用: calc.py

# ========================================
# 空間パラメータ (Spatial Parameters)
# ========================================

x_max = 3.5           # シミュレーション空間の最大x座標 [m]
                      # 使用: main.py, world.py

y_max = 3.5           # シミュレーション空間の最大y座標 [m]
                      # 使用: bayesian.py, main.py, world.py

margin_space = 0.5      # 境界マージン空間 [m]
                      # 使用: bayesian.py, main.py, world.py

Mx = My = int(np.round(y_max / h))       # 空間格子点数
                      # 使用: calc.py
                      # ※ world.pyでも同様の計算が行われています

# ========================================
# 検出パラメータ (Detection Parameters)
# ========================================

threshold = 0.0062        # エコー検知最小感度（さらに、conf1の半値境目）
                      # ベイズ定位点導入.pyに合わせる（旧: 1.2 * 0.1**3 = 0.0012）
                      # 使用: calc.py
                      # 旧値: 0.007, 0.0032

grad = 10**13             # conf1の勾配（sigmoid関数の急峻さ）
                      # ベイズ定位点導入.pyに合わせる（旧: 10**15）
                      # 値が小さいほど滑らかな遷移、大きいほど急峻（ステップ関数的）
                      # 使用: calc.py

k_r_noise = 60000         # 距離ノイズ係数
                      # 使用: calc.py

k_theta_noise = 360000    # 角度ノイズ係数
                      # 使用: calc.py

# ========================================
# シミュレーションパラメータ (Simulation Parameters)
# ========================================

trials = 20           # 試行回数
                      # 使用: calc.py, main.py, agent.py

# 初期位置パラメータ [x, y, fd, pd]
# x, y: 初期座標 [m]
# fd: 飛行方向（度数法）
# pd: パルス発射方向（度数法）
init_pos = [2.5, 2.5, 270, 270]
                      # 使用: main.py, agent.py

# ========================================
# ベイズ推論パラメータ (Bayesian Inference Parameters)
# ========================================

sigma2 = (0.0001 * c) ** 2  # ベイズ更新の分散パラメータ
                      # 使用: main.py, bayesian.py

min_p = -320                # 最小確率値（対数スケール）
                      # 使用: main.py, bayesian.py, visualize.py

# ========================================
# ダミー観測パラメータ (Dummy Detection Parameters)
# ========================================
"""
【2025年追加】検出なし時のダミー観測方式

目的:
  物体を検出しなかった場合、ワールド範囲外にダミーの観測を配置し、
  正規化による相対的な確率減衰を実現します。

理論的背景:
  - ワールド範囲外の固定位置にダミー観測を配置
  - 既存のベイズ更新ロジックがそのまま動作
  - 尤度のピークは範囲外、ワールド範囲内には裾野のみ影響
  - 正規化により、ワールド範囲内の確率が全体的に減衰

ワールド範囲:
  - x, y: 0 ~ 3.5m
  - 探索範囲: margin内（0.5 ~ 3.0m程度）
  - ダミー位置: (5.0m, 5.0m) = 範囲外

実装方式:
  案1: 負の尤度を明示的に計算（理論的に正しいが複雑）
  案2: ワールド範囲外にダミー観測を配置（実装簡単）← 採用
"""

use_dummy_detection = True    # ダミー観測を使用するか
                      # 使用: control_pc.py
                      # True: 検出なし時にダミー観測を配置
                      # False: 従来通り（検出なしの場合は更新なし）

dummy_detection_x = 0.0       # ダミー観測のX座標 [m]
                      # 使用: control_pc.py
                      # ワールド範囲外の座標（x_max=3.5m より大きい）

dummy_detection_y = 0.0       # ダミー観測のY座標 [m]
                      # 使用: control_pc.py
                      # ワールド範囲外の座標（y_max=3.5m より大きい）

# ========================================
# 相互相関直接処理パラメータ (Correlation Direct Processing)
# ========================================
"""
【相互相関直接方式】
物体定位（ピーク検出・左右マッチング・距離角度計算）をスキップし、
相互相関波形の閾値以上の時間点を直接ベイズ更新に使用する方式。

従来方式との違い:
  - 従来: 相関 → ピーク検出 → マッチング → 距離・角度 → エコー時間 → ベイズ
  - 新方式: 相関 → 閾値処理 → 時間点抽出 → ベイズ

利点:
  - ピーク検出・マッチングの不確実性を回避
  - 相互相関の情報をより直接的に活用
  - 実装がシンプル

注意点:
  - 時間点の数が10〜100倍に増加（計算コスト増）
  - 閾値設定が重要
"""

use_correlation_direct = True     # True: 相互相関を直接使用, False: 物体定位を使用
                      # 使用: control_pc.py
                      # デフォルトはFalse（従来の物体定位方式）

correlation_threshold = 0.15       # 相互相関の閾値（0〜1の正規化後の値）
                      # 使用: control_pc.py
                      # この値以上の相関値を持つ時間点を抽出
                      # 推奨値: 0.20〜0.30（高すぎると検出減、低すぎるとノイズ増）

direct_pulse_samples = 581         # 直達音のサンプル数（片道10cm = 往復20cm相当）
                      # 使用: control_pc.py
                      # この範囲の相関データをゼロにして直達音を除外
                      # 計算: 0.2m ÷ 344m/s × 1MHz = 581サンプル（温度21℃）

correlation_sampling_rate = 1000000  # 相互相関のサンプリング周波数 [Hz]
                      # 使用: control_pc.py
                      # ロボットからの相互相関データのサンプリングレート

# ========================================
# Negative Evidence パラメータ
# ========================================
"""
【観測なし時の処理方式】
閾値を超える相関値がなかった場合の処理を制御します。

シナリオ分類:
  1. 両方観測あり（閾値超え）: 両方で通常のベイズ更新
  2. 片方だけ観測あり: 観測ありの耳だけで更新、観測なしの耳は更新しない（事前分布を保持）
  3. 両方観測なし（閾値以下）: Negative Evidenceを適用

従来方式（use_negative_evidence = False）:
  - フォールバック処理で上位1個を強制取得
  - 必ず何らかの観測があることになる（シナリオ1に強制変換）

Negative Evidence方式（use_negative_evidence = True）:
  - フォールバック処理をスキップ
  - シナリオ2: 片方だけ観測 → その耳だけで更新、もう片方は事前分布を保持
  - シナリオ3: 両方観測なし → 「観測できたはずなのに検出しなかった」という情報を活用

ベイズ統計学の観点:
  - negative evidence（否定的証拠）: 「何も見えなかった」という事実自体が情報
  - P(障害物あり | 両方観測なし) は低いはず
  - observable領域（confidenceが高い領域）の確率を下げる
"""

use_negative_evidence = False  # True: negative evidence適用, False: フォールバック処理
                               # 使用: control_pc.py, bayesian.py

confidence_threshold = 0.3     # observableとunobservableの境界閾値
                               # 使用: bayesian.py
                               # この値以上のconfidenceを持つ領域は「観測できたはず」

negative_evidence_value = 0.0001  # observable領域での尤度値（両方観測なし時）
                                  # 使用: bayesian.py
                                  # 「見えるはずなのに見えなかった」= 障害物がない可能性が高い

unobservable_value = 1.0       # unobservable領域での尤度値（両方観測なし時）
                               # 使用: bayesian.py
                               #
                               # 【重要】正規化によって最大値が基準点（dB=0）になります：
                               #
                               # ケース1: unobservable_value = 1.0（推奨設定）
                               #   - unobservable領域の値（1.0）が最大
                               #   - 正規化後: unobservable領域 = 1.0（基準点）
                               #   - 正規化後: observable領域 = 0.0001 / 1.0 = 0.0001
                               #   - dB変換: observable領域 = -4（確率が下がる）
                               #   - dB変換: unobservable領域 = 0（変化なし）
                               #   → observable領域の確率を下げる（理論的に正しい）
                               #
                               # ケース2: unobservable_value = 0.1**20（実験設定）
                               #   - observable領域の値（0.0001）が最大
                               #   - 正規化後: observable領域 = 1.0（基準点）
                               #   - 正規化後: unobservable領域 = 10^-20 / 0.0001 = 10^-16
                               #   - dB変換: observable領域 = 0（変化なし）
                               #   - dB変換: unobservable領域 = -16（確率が下がる）
                               #   → unobservable領域も含めて確率を下げる（実験的）
                               #
                               # 推奨: 1.0（観測できない場所は情報なし）
                               # 実験用: 0.1**20（全空間の確率を下げる）

# ========================================
# 可視化パラメータ (Visualization Parameters)
# ========================================

c_percentile = 95     # カラーマップのパーセンタイル
                      # 使用: main.py, visualize.py

# ========================================
# フォルダ設定 (Folder Settings)
# ========================================

folder_name = "bayse_olddata2"  # データ保存フォルダ
                      # 使用: main.py, world.py, agent.py

# 出力ディレクトリの設定（全出力ファイルをoutput配下に統一）
output_dir = folder_name + "/output"  # 出力のベースディレクトリ
                      # 使用: main.py, agent.py

output_dir_movie_sim2 = output_dir + "/movie/sim2"  # sim2シミュレーション動画
                      # 使用: main.py, visualize.py

output_dir_movie_posterior = output_dir + "/movie/事後分布"  # 事後分布動画
                      # 使用: agent.py

# ========================================
# チェックポイント設定 (Checkpoint Settings)
# ========================================
"""
【途中再開機能】
ベイズ推定の状態を定期的に保存し、途中から再開できるようにします。

理論的背景:
  - ベイズ推定は逐次更新: 事後確率 → 次の事前確率
  - マルコフ性: 現在の事後分布に全情報が凝縮
  - 十分統計量: 過去の観測データは不要

保存されるデータ:
  - Px2L_log, Px2R_log: 基本モデルの事後確率分布
  - Px3L_log, Px3R_log: 記憶保持モデルの事後確率分布
  - confidence: confidence行列
  - convergence_history: 認知収束度合いの履歴
  - step_idx: ステップ番号

使用例:
  1. 新規開始: load_checkpoint_path = None
     → 0ステップから開始、20ステップごとに保存

  2. 途中再開（160ステップから）:
     load_checkpoint_path = "bayse_olddata2/checkpoints/state_step_0160.npz"
     → 161ステップから再開、180, 200, ...で保存

用途:
  - 長時間実験の中断・再開
  - 特定ステップから再実行
  - デバッグ時に同じ状態から何度も試行
"""

enable_state_checkpoint = True    # True: チェックポイント保存を有効化
                                  # 使用: control_pc.py, robot_simulator.py

checkpoint_interval = 20          # 保存間隔（ステップ数）
                                  # 使用: control_pc.py, robot_simulator.py
                                  # 20なら20, 40, 60, ...ステップごとに保存

checkpoint_dir = folder_name + "/checkpoints"  # チェックポイント保存先
                                                # 使用: control_pc.py, robot_simulator.py

load_checkpoint_path = None       # 読み込むチェックポイントのパス
                                  # 使用: control_pc.py, robot_simulator.py
                                  # None: 新規開始
                                  # 例: "bayse_olddata2/checkpoints/state_step_0160.npz"

# ========================================
# フラグ設定 (Flag Settings)
# ========================================

# 障害物を壁の線上に配置するかどうか
world_pole_wall = False
                      # 使用: calc.py, world.py

# 壁の外を認知-20にするかどうか
world_wall_pos = True
                      # 使用: bayesian.py, calc.py

# ========================================
# パラメータ使用サマリー
# ========================================
#
# calc.py で使用:
#   - 物理: h, freq, c, a, ear_dist
#   - 時間: dt, Mt, t_ax, eps_y
#   - 空間: Mx, My
#   - 検出: threshold, grad, k_r_noise, k_theta_noise
#   - シミュレーション: trials
#   - フラグ: world_pole_wall, world_wall_pos
#
# bayesian.py で使用:
#   - 物理: h
#   - 空間: y_max, margin_space
#   - ベイズ: sigma2, min_p (コンストラクタ引数として)
#   - フラグ: world_wall_pos
#
# world.py で使用:
#   - 物理: h, c
#   - 時間: dt, t_max
#   - 空間: x_max, y_max, margin_space
#   - フォルダ: folder_name
#   - フラグ: world_pole_wall
#
# main.py で使用:
#   - 全パラメータ（各クラスに渡すため）
#
# agent.py で使用:
#   - シミュレーション: trials, init_pos
#   - 空間: margin_space
#   - フォルダ: folder_name
#   - ※現在は引数で受け取る設計
#
# visualize.py で使用:
#   - 可視化: c_percentile
#   - ベイズ: min_p
#   - フォルダ: output_dir_movie
#   - ※現在は引数で受け取る設計
