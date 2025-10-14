# -*- coding: utf-8 -*-
"""
Bayesian Simulation for Bat Echolocation with Step-by-Step Execution
====================================
このプログラムはコウモリのエコロケーション（反響定位）システムのベイジアンシミュレーションを実装します。

シミュレーションの主な流れ:
1. コウモリと障害物の位置関係を設定
2. 理想的なエコー信号（音波の往復距離・時間）を計算
3. 現実的なノイズを加えた観測信号を生成
4. ベイジアン推論を使用して環境マップを更新
5. 結果の可視化と解析

ベイズ理論の適用:
P(位置|観測データ) ∝ P(観測データ|位置) × P(位置)
事後確率     ∝   尤度    × 事前確率

このシミュレーションでは、コウモリが送信する超音波パルスとその反射波（エコー）の
時間差などの情報から、ベイズ推論を用いて空間内の障害物の位置を推定します。

以下のシミュレーションでは、コウモリが発する超音波パルスが障害物に反射する様子と、
その反射波をコウモリが受信して空間認識を行うプロセスをモデル化しています。
これにより、コウモリが暗闇でも正確に障害物を検知し飛行できる仕組みを理解できます。

更新: このバージョンでは、シミュレーションを1ステップごとに実行できるように変更しています。
これにより、認知マップを使った動的な経路計画が可能になります。
"""

# 可視化用のカスタムクラス
from world import World
from agent import Agent
from bayesian import Bayesian
from visualize import BatVisualizer
import numpy as np
import copy
import csv
import os
import signal
import sys

# 設定ファイルから全パラメータをインポート
import config

def save_convergence_to_csv(folder_name, pattern, convergence_history):
    """認知収束度合いの履歴をCSVファイルに保存する関数"""
    os.makedirs(config.output_dir, exist_ok=True)
    convergence_csv_path = os.path.join(config.output_dir, f"cognitive_convergence_{pattern}.csv")
    try:
        with open(convergence_csv_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Step', 'Convergence_Value'])
            for i, value in enumerate(convergence_history):
                csv_writer.writerow([i, value])
        print(f"認知収束度合いのCSVファイル作成完了: {convergence_csv_path}")
        return True
    except Exception as e:
        print(f"CSVファイル作成に失敗しました: {e}")
        return False


def signal_handler(sig, frame):
    """Ctrl+Cが押された時のシグナルハンドラ"""
    print("\nプログラムが中断されました。認知収束度合いのCSVファイルを保存します...")
    if 'bayesian' in globals() and 'world' in globals() and 'agent' in globals():
        save_convergence_to_csv(world.folder_name, "sim2", bayesian.convergence_history)
    sys.exit(0)


if __name__ == "__main__":
    # Ctrl+Cシグナルハンドラを登録
    signal.signal(signal.SIGINT, signal_handler)

    # 認知の範囲は広くして、可視化の場所だけ狭くしてる。事後分布で計算がはみ出さない。
    world = World(
        x_max = config.x_max,
        y_max = config.y_max,
        margin_space = config.margin_space,
        h = config.h,
        t_max = config.t_max,
        dt = config.dt,
        c = config.c,
        folder_name = config.folder_name
    )

    bayesian = Bayesian(
        sigma2 = config.sigma2,
        min_p = config.min_p,
        c = config.c
    )

    agent = Agent(
        bayesian = bayesian,
        margin_space = config.margin_space,
        folder_name = config.folder_name,
        X = world.X,
        Y = world.Y,
        sim = {
            "trials": config.trials,
            "init_pos": config.init_pos
        },
        world = world
    )
# 現在はtrial数が必要なのでAgent後
    bayesian.Init(world, agent)

    visualizer = BatVisualizer(
        output_dir = config.output_dir_movie_sim2,
        X = world.X,
        Y = world.Y,
        c_percentile = config.c_percentile,
        min_p = config.min_p,
        x_max = world.x_max,
        y_max = world.y_max,
        wall_x = world.wall_x,
        wall_y = world.wall_y,
    )
    print("可視化初期化完了\n\n")

    # ステップごとの実行
    print("loop start...")
    for i in range(0, 200):
        
        print(f"trial {i} start")
        print("\n\n--------------------------------\n\n\n")
        if i != 0:
            flag = agent.one_step(i, visualizer)
            print(f"flag: {flag}")
        else:
            flag = True

        y_x, y_y, y_el_vec, y_er_vec, data1, data2, data3, data4 = agent.do_sensing(world)
        
        visualizer.plot_frame(
            frame_idx = i,
            current_bat_x = agent.PositionX,
            current_bat_y = agent.PositionY,
            current_pd = agent.pd,
            current_fd = agent.fd,
            pole_x = world.pole_x,
            pole_y = world.pole_y,
            y_x = y_x,
            y_y = y_y,
            data1 = data1,
            data2 = data2,
            data3 = data3,
            data4 = data4,
            t_ax = world.t_ax,
            y_el_vec = y_el_vec,
            y_er_vec = y_er_vec,
            flag = flag
        )
    # world_pole_wall: world.py(96, 97), calc.py(551, 553) で使用（障害物を壁の線上に配置するかどうか）
    # world_wall_pos: bayesian.py(175, 176), calc.py(552, 556) で使用（壁の外を認知-20にするかどうか）
    
    # 最終的なアニメーション作成
    start_trial = 0
    end_trial = agent.trials - 1
    gif_path = visualizer.create_gif_from_frames(start_trial, end_trial, duration=0.2)
    if gif_path:
        print(f"GIFアニメーション作成完了: {gif_path}")
    else:
        print("GIF作成に失敗しました")
    
    # 認知収束度合いの履歴をCSVファイルに保存
    save_convergence_to_csv(world.folder_name, "sim2", bayesian.convergence_history)
    
    print("シミュレーション完了")