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

freq = 40000          # 周波数 [Hz] - コウモリが発する超音波の周波数
                      # 使用: calc.py

c = 340               # 音速 [m/s]
                      # 使用: calc.py, main.py, world.py

a = 0.005             # 口の半径サイズ [m] - 超音波を発する口の大きさ
                      # 使用: calc.py

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

threshold = 0.0032        # エコー検知最小感度（さらに、conf1の半値境目）
                      # ベイズ定位点導入.pyに合わせる（旧: 1.2 * 0.1**3 = 0.0012）
                      # 使用: calc.py

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

dummy_detection_x = 5.0       # ダミー観測のX座標 [m]
                      # 使用: control_pc.py
                      # ワールド範囲外の座標（x_max=3.5m より大きい）

dummy_detection_y = 5.0       # ダミー観測のY座標 [m]
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

direct_pulse_samples = 1200        # 直達音のサンプル数
                      # 使用: control_pc.py
                      # この範囲の相関データをゼロにして直達音を除外
                      # 推奨値: 1000〜1500（音速と最短距離に依存）

correlation_sampling_rate = 1000000  # 相互相関のサンプリング周波数 [Hz]
                      # 使用: control_pc.py
                      # ロボットからの相互相関データのサンプリングレート

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
