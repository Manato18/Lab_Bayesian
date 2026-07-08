# -*- coding: utf-8 -*-
"""
リファクタリング用回帰チェックスクリプト（Lab_Bayesian）
========================================================
実機・marker_server・ソケット通信なしで、bayes_code の決定論的なコア
（calc → bayesian.update_belief → agent の回避ロジック）を数ステップ実行し、
数値状態（位置履歴・収束履歴・事後分布）をベースラインと比較する。

実機の本番経路（control_pc.py が Localizer 経由で update_belief を呼ぶ）とは
別だが、リファクタ対象の calc()/update_belief()/agent の回避計算を同じ関数群で
通すため、挙動不変の検証として十分機能する。

使い方:
    ベースライン保存:  uv run python tests/test_regression.py --save-baseline
    回帰チェック実行:  uv run python tests/test_regression.py

注意:
    - marker_server から取得する障害物は、ここでは固定の合成座標を world に直接セットする
      （control_pc.py が実機で pole_x/pole_y を上書きするのと同じ位置づけ）。
    - baseline (tests/baseline.npz) はローカル専用（.gitignore 対象）。
"""
import argparse
import io
import os
import sys
import contextlib

import matplotlib
matplotlib.use("Agg")  # ヘッドレスで plot するためのバックエンド

import numpy as np

# リポジトリルートを import パスに追加（bayes_code パッケージを読むため）
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(REPO_ROOT)
sys.path.insert(0, REPO_ROOT)

BASELINE_PATH = os.path.join(REPO_ROOT, "tests", "baseline.npz")
N_STEPS = 12  # 直線移動フェーズ(step<10)と回避フェーズ(step>=10)の両方を通す
SEED = 42

# 合成障害物（実機では marker_server が渡す。バット前方 -y 側に配置）
SYNTHETIC_POLE_X = np.array([2.5, 2.0, 3.0])
SYNTHETIC_POLE_Y = np.array([1.5, 1.2, 1.3])


def run_simulation(n_steps: int, seed: int) -> dict:
    """bayes_code のコアを n_steps 実行し、数値状態を返す（ハードウェア不要）。"""
    np.random.seed(seed)

    from bayes_code import config
    from bayes_code.world import World
    from bayes_code.bayesian import Bayesian
    from bayes_code.agent import Agent

    world = World(
        x_max=config.x_max, y_max=config.y_max, margin_space=config.margin_space,
        h=config.h, t_max=config.t_max, dt=config.dt, c=config.c,
        folder_name=config.folder_name,
    )
    # 実機では marker_server が障害物を渡す。ここでは固定の合成座標をセットする。
    world.pole_x = SYNTHETIC_POLE_X.copy()
    world.pole_y = SYNTHETIC_POLE_Y.copy()

    bayesian = Bayesian(sigma2=config.sigma2, min_p=config.min_p, c=world.c)
    agent = Agent(
        bayesian=bayesian,
        margin_space=world.margin_space,
        folder_name=world.folder_name,
        X=world.X, Y=world.Y,
        sim={"trials": n_steps, "init_pos": list(config.init_pos)},
        world=world,
    )
    bayesian.Init(world, agent)

    positions = []  # 各ステップの [x, y, fd, pd]
    flags = []
    for i in range(n_steps):
        if i != 0:
            flag = agent.one_step(i, None)  # _sim_flight2 は visualizer を使わない
        else:
            flag = True
        flags.append(bool(flag))
        agent.do_sensing(world)
        positions.append([agent.PositionX, agent.PositionY, agent.fd, agent.pd])

    return {
        "positions": np.array(positions),
        "flags": np.array(flags),
        "convergence": np.array(bayesian.convergence_history),
        "posterior": bayesian.Px_yn_log_current,
        "posterior_memory": bayesian.Px_yn_conf_log_current,
        "prior_L": bayesian.Px2L_log,
        "prior_memory_L": bayesian.Px3L_log,
    }


def compare(baseline: dict, current: dict) -> bool:
    """ベースラインと現在の実行結果を厳密比較する。"""
    all_ok = True
    for key in baseline.files:
        base_arr = baseline[key]
        curr_arr = current[key]
        if base_arr.shape != curr_arr.shape:
            print(f"  NG {key}: shape不一致 {base_arr.shape} != {curr_arr.shape}")
            all_ok = False
            continue
        equal = np.array_equal(base_arr, curr_arr, equal_nan=True) if base_arr.dtype.kind == "f" \
            else np.array_equal(base_arr, curr_arr)
        if equal:
            print(f"  OK {key}: 完全一致 (shape={base_arr.shape})")
        else:
            diff = np.abs(np.nan_to_num(base_arr.astype(float)) - np.nan_to_num(curr_arr.astype(float)))
            print(f"  NG {key}: 最大差分 = {diff.max():.6e}（不一致 {np.count_nonzero(diff)} 要素）")
            all_ok = False
    return all_ok


def main():
    parser = argparse.ArgumentParser(description="リファクタリング回帰チェック（Lab_Bayesian）")
    parser.add_argument("--save-baseline", action="store_true", help="ベースラインを保存する")
    parser.add_argument("--verbose", action="store_true", help="シミュレーションのログを表示する")
    args = parser.parse_args()

    print(f"シミュレーション実行中... (steps={N_STEPS}, seed={SEED})")
    if args.verbose:
        result = run_simulation(N_STEPS, SEED)
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            result = run_simulation(N_STEPS, SEED)

    if args.save_baseline:
        os.makedirs(os.path.dirname(BASELINE_PATH), exist_ok=True)
        np.savez_compressed(BASELINE_PATH, **result)
        size_mb = os.path.getsize(BASELINE_PATH) / 1024 / 1024
        print(f"ベースライン保存完了: {BASELINE_PATH} ({size_mb:.1f} MB)")
        print(f"  positions:\n{result['positions']}")
        return 0

    if not os.path.exists(BASELINE_PATH):
        print("エラー: ベースラインがありません。先に --save-baseline で保存してください。")
        return 1

    baseline = np.load(BASELINE_PATH)
    print("ベースラインと比較中...")
    ok = compare(baseline, result)
    if ok:
        print("\n✅ 回帰チェック合格: 全項目がベースラインと一致")
        return 0
    print("\n❌ 回帰チェック失敗: 挙動が変わっています")
    return 1


if __name__ == "__main__":
    sys.exit(main())
