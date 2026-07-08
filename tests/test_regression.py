# -*- coding: utf-8 -*-
"""
リファクタリング用回帰チェックスクリプト（Lab_Bayesian）
========================================================
実機・marker_server・ソケット通信なしで、bayes_code の決定論的なコアを
複数シナリオで実行し、数値状態をベースラインと比較する。

シナリオ構成（各22ステップ。12→22に延長した理由は下記）:
    A: 近接合成障害物 + agent.one_step 経路。
       step10-13 で緊急回避（危険角度→±60度、flag=False）と連続回避、
       step14以降で「危険なし→min_angle 選択」の通常回避分岐（flag=True）を通す。
       ※ 旧12ステップでは通常回避分岐が一度も実行されないことが
          レビューで判明したため22ステップに延長（初回分析時は前方が未照射で
          必ず危険扱いになり、旋回でパルスが前方を照射した後にのみ通常分岐に入る）。
    C: シナリオAと同じ障害物で、**本番ラッパー calculate_avoidance_command**
       （control_pc.py から呼ばれる回避エントリポイント）を駆動する。
       control_pc のループ（センシング→ベイズ更新→移動指令→移動）を模擬。
       ※ 本番経路の回避ラッパーがハーネス未カバーだったレビュー指摘への対応。

使い方:
    ベースライン保存:  uv run python tests/test_regression.py --save-baseline
    回帰チェック実行:  uv run python tests/test_regression.py

ベースラインの再現手順（第三者検証・環境再構築時）:
    ベースライン (tests/baseline.npz) は .gitignore 対象のローカル専用ファイル。
    紛失した場合は「リファクタ前のコード」から再生成して突合する:
        git worktree add /tmp/lb_main main
        cd /tmp/lb_main && uv venv --python 3.12 && uv pip install -r requirements.txt
        # main には本スクリプトが無いので tests/ をコピーしてから:
        uv run python tests/test_regression.py --save-baseline
    ※ NumPy のバージョンが変わると浮動小数の完全一致が崩れる可能性があるため、
       同一 venv 環境（requirements.txt）で生成・比較すること。
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
N_STEPS = 22
SEED = 42

# シナリオA/C: バット前方 -y 側の近接障害物（緊急回避→通常回避の順に発火する）
POLES_CLOSE = (np.array([2.5, 2.0, 3.0]), np.array([1.5, 1.2, 1.3]))


def _build_core(poles):
    """World/Bayesian/Agent 一式をハードウェアなしで構築して返す"""
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
    world.pole_x = poles[0].copy()
    world.pole_y = poles[1].copy()

    bayesian = Bayesian(sigma2=config.sigma2, min_p=config.min_p, c=world.c)
    agent = Agent(
        bayesian=bayesian,
        margin_space=world.margin_space,
        folder_name=world.folder_name,
        X=world.X, Y=world.Y,
        sim={"trials": N_STEPS, "init_pos": list(config.init_pos)},
        world=world,
    )
    bayesian.Init(world, agent)
    return world, bayesian, agent


def run_agent_scenario(n_steps: int, seed: int, poles) -> dict:
    """agent.one_step 経路（シミュレーションループ）を n_steps 実行する"""
    np.random.seed(seed)
    world, bayesian, agent = _build_core(poles)

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


def run_command_scenario(n_steps: int, seed: int, poles) -> dict:
    """本番ラッパー calculate_avoidance_command を control_pc と同じ流れで駆動する。

    control_pc.py のループ（センシング→ベイズ更新→移動指令計算→移動）を模擬:
    観測生成は Localizer の代わりに calc()（do_sensing）で行い、
    回避判断は本番と同じ calculate_avoidance_command に委ねる。
    """
    from bayes_code import config

    np.random.seed(seed)
    world, bayesian, agent = _build_core(poles)

    pos = {
        "x": float(config.init_pos[0]), "y": float(config.init_pos[1]),
        "fd": float(config.init_pos[2]), "pd": float(config.init_pos[3]),
    }
    positions = []   # 各ステップ後の [x, y, fd, pd]
    commands = []    # [avoidance_direction, move_distance, pulse_direction]
    emergencies = []
    for i in range(n_steps):
        # agent の状態を現在位置に同期（do_sensing / プロットが参照するため）
        agent.step_idx = i
        agent.PositionX, agent.PositionY = pos["x"], pos["y"]
        agent.fd, agent.pd = pos["fd"], pos["pd"]

        agent.do_sensing(world)  # ベイズ更新（Localizer の代替として calc() で観測生成）

        command, new_pos, emergency = agent.calculate_avoidance_command(pos, i)
        commands.append([
            command["avoidance_direction"],
            command["move_distance"],
            command["pulse_direction"],
        ])
        emergencies.append(bool(emergency))
        pos = new_pos
        positions.append([pos["x"], pos["y"], pos["fd"], pos["pd"]])

    return {
        "positions": np.array(positions),
        "commands": np.array(commands),
        "emergency": np.array(emergencies),
        "posterior_memory": bayesian.Px_yn_conf_log_current,
    }


def run_all_scenarios() -> dict:
    """シナリオA・Cを実行し、Cのキーに接頭辞 c_ を付けて統合 dict を返す"""
    result_a = run_agent_scenario(N_STEPS, SEED, POLES_CLOSE)
    result_c = run_command_scenario(N_STEPS, SEED, POLES_CLOSE)
    combined = dict(result_a)
    for key, value in result_c.items():
        combined[f"c_{key}"] = value
    return combined


def compare(baseline: dict, current: dict) -> bool:
    """ベースラインと現在の実行結果を厳密比較する"""
    all_ok = True
    for key in baseline.files:
        if key not in current:
            print(f"  NG {key}: 現在の実行結果にキーがありません")
            all_ok = False
            continue
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

    extra_keys = [k for k in current if k not in set(baseline.files)]
    if extra_keys:
        print(f"  警告: ベースラインに無いキー: {extra_keys}")
        print("        （ハーネスを拡張した場合は --save-baseline で更新してください）")
    return all_ok


def coverage_report(result: dict):
    """各シナリオが目的の分岐を通ったかを表示し、判定材料を返す"""
    a_emergency = [i for i, f in enumerate(result["flags"]) if not f]
    a_normal = [i for i, f in enumerate(result["flags"][10:], start=10) if f]
    c_emergency = [i for i, e in enumerate(result["c_emergency"]) if e]
    c_normal = [i for i, e in enumerate(result["c_emergency"][10:], start=10) if not e]
    print(f"カバレッジ: A 緊急回避={a_emergency} 通常回避(step>=10)={a_normal} / "
          f"C 緊急回避={c_emergency} 通常回避(step>=10)={c_normal}")
    return a_emergency, a_normal, c_emergency, c_normal


def main():
    parser = argparse.ArgumentParser(description="リファクタリング回帰チェック（Lab_Bayesian）")
    parser.add_argument("--save-baseline", action="store_true", help="ベースラインを保存する")
    parser.add_argument("--verbose", action="store_true", help="シミュレーションのログを表示する")
    args = parser.parse_args()

    print(f"シミュレーション実行中... (A/C 各 steps={N_STEPS}, seed={SEED})")
    if args.verbose:
        result = run_all_scenarios()
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            result = run_all_scenarios()

    a_emergency, a_normal, c_emergency, c_normal = coverage_report(result)

    if args.save_baseline:
        # 各シナリオが目的の分岐（緊急回避・通常回避の両方）を通っていることを保存条件にする
        if not a_emergency or not a_normal or not c_emergency or not c_normal:
            print("エラー: いずれかのシナリオが目的の分岐を通っていません。")
            print("       障害物配置(POLES_*)を調整してから保存してください。")
            return 1
        os.makedirs(os.path.dirname(BASELINE_PATH), exist_ok=True)
        np.savez_compressed(BASELINE_PATH, **result)
        size_mb = os.path.getsize(BASELINE_PATH) / 1024 / 1024
        print(f"ベースライン保存完了: {BASELINE_PATH} ({size_mb:.1f} MB)")
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
