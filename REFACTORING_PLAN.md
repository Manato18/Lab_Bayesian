# リファクタリング計画 — 可読性向上（refactor/readability）

## 目的

**人が読んでわかりやすいコードにする**こと。アルゴリズム（ベイズ更新・エコー計算・回避ロジック・実機通信）の挙動は一切変えない。既存設計（`bayes_code/` パッケージ構成、control_pc / marker_server / robot_simulator の3プロセス構成、日本語コメント文化）を尊重する。

姉妹リポジトリ `Lab_Bayesian_Simulation` で実施したリファクタリング（config一元化・dataclass化・記号対応表・print整理・回避ロジック分割）の知見を、実機向けの本リポジトリに適用する。

## Lab_Bayesian の構造（Simulation との違い）

- **`bayes_code/` パッケージ**: calc/bayesian/agent/world は Simulation とほぼ同構造
- **`control_pc.py`（1067行, ソケット）** が本番オーケストレータ。`bayesian.update_belief` を直接呼び（569行）、`calc` からは個別関数のみ import。障害物検出は `Localizer`（localization.py, .dat/相互相関からピーク検出）が担当
- **`calc()` メガ関数と `agent.do_sensing`** は agent 経由のシミュレーション経路でのみ使用（本番の control_pc 経路では未使用のレガシー寄り）
- **config.py は既に一元化済み**（calc.py も config から import 済み）→ Simulation の Phase 1 に相当する作業は**ほぼ不要**
- **CSV収束ログは無い**（`convergence_history` のみ）→ bayesian の副作用分離は壁マスクのみ

## 精読で見つけた課題（Simulation と共通）

| 課題 | 場所 | Simulation での対応 |
|---|---|---|
| `calc()` が11タプルを返す | `bayes_code/calc.py` | `SensingResult` dataclass |
| デバッグ print `check0`〜`check25`（11箇所） | `bayes_code/calc.py` | 削除 |
| `update_belief` が `data1〜4` を返す | `bayes_code/bayesian.py` | `BeliefSnapshot` dataclass |
| ベイズ数式変数の意味不明（`Px2L_log` 等） | `bayes_code/bayesian.py` | 記号対応表を docstring に |
| `calculate_convergence` の大量デバッグ print | `bayes_code/bayesian.py` | 削除 |
| `update_belief` に壁マスク副作用が混在 | `bayes_code/bayesian.py` | `_apply_wall_mask()` 抽出 |
| 回避ロジックの巨大関数・マジックナンバー | `bayes_code/agent.py` | ヘルパー分割＋命名定数 |

## リファクタリング方針（尊重すること）

1. **パッケージ構成・ファイル配置は変えない**
2. **日本語コメント・docstring は維持・充実**
3. **数式由来の変数名は変えず、記号対応表を追加**
4. **挙動は変えない** — 各 Phase 後に回帰チェック（シード固定・12ステップ）で検証
5. **control_pc.py は「dataclass 化の波及」に限定して触る**。ソケット通信を含む本番経路の深い再構成は範囲外（回帰ハーネスで守れないため）

## Phase 構成

### Phase 0: 安全網 ✅（このコミット）
- [x] `tests/test_regression.py`: 実機なしの決定論ハーネス（合成障害物 + シード固定 + 12ステップ）
- [x] ベースライン保存・再現性確認（全7項目一致、EXIT=0）
- [x] 計画書作成

### Phase 1: `calc.py` の整理 ✅ 完了 (commit 881d004)
- [x] `SensingResult`（dataclass）で11タプル返しを置換、障害物レイアウト抽出、セクション整理
- [x] デバッグ print（check0〜25、座標ダンプ等）を削除
- [x] 呼び出し側 `agent.do_sensing` を更新

### Phase 1.5: 第三者レビュー対応 ✅ 完了
モデル変更を機に両リポジトリの全差分を批判的レビュー（独立エージェント2本）した結果への対応。
- [x] ハーネスに本番ラッパー `calculate_avoidance_command` の駆動シナリオCを追加
      （control_pc のループを模擬。レビュー指摘「本番回避エントリが未カバー」への対応）
- [x] シナリオを22ステップに延長し、緊急回避(step10-13)と通常回避(step14以降)の
      **両分岐**をA/C両経路でカバー（旧12ステップでは通常回避分岐が未実行だった）
- [x] `SensingResult.r_noise` → `r_detected` に改名（ノイズ幅ローカル変数との同名2義解消）
- [x] `SensingResult` に `eq=False`（ndarray == 比較の footgun 防止）
- [x] calc.py の未使用 config import（x_max/y_max/Mt/t_max）を削除、孤立空白行を掃除
- [x] `do_sensing` に返り値契約の docstring を追加（belief は Phase 2 で BeliefSnapshot 化予定と明記）
- [x] ベースライン再現手順（main worktree からの再生成）をハーネス docstring に文書化

> [!question] 既存挙動の疑義（挙動不変の方針により温存・記録のみ）
> - `Obj.Deg` に角度ではなく「角度ノイズ幅」が入っている（`do_sensing` の
>   `Obj(Deg=theta_noise)`）。現状 Newobj は書き込みのみで実害なし
> - `agent.py` の `_sim_flight2` / `calculate_avoidance_command` 内の `if step >= 6:` は
>   step>=10 分岐の内側にあり**常に真の死んだ条件**。削除は挙動不変だが、
>   「step 6〜9 で使う予定だった名残」の可能性があるため削除前にユーザー確認を取る

### Phase 2: `bayesian.py` の整理 ✅ 完了
- [x] 記号対応表をモジュール docstring に追加
- [x] `BeliefSnapshot`（dataclass, eq=False）で `data1〜4` を置換
- [x] `calculate_convergence` のデバッグ print 削除、壁マスクを `_apply_wall_mask()` に分離
- [x] 呼び出し側 `agent.do_sensing` と `control_pc.py` を更新
- [x] control_pc.py の可視化用 dict（キー `data1`〜`data4`）はキー名を温存し、
      値の詰め替えだけ BeliefSnapshot 対応（可視化消費側への波及なし）
- [x] 回帰チェック合格（A/C 全11項目一致）＋ control_pc/robot_simulator の import 検証

### Phase 3: `agent.py` の整理 ✅ 完了
- [x] `_analyze_posterior_for_avoidance` をヘルパー分割（`_aggregate_posterior_by_direction` /
      `_decide_emergency_avoidance` / `_print_avoidance_table`）、詳細テーブルは `verbose` 制御
- [x] マジックナンバーを命名定数化（STRAIGHT_STEPS / STEP_DISTANCE(_MM) /
      INITIAL_PULSE_OFFSET_DEG / PULSE_AVOID_FACTOR / EMERGENCY_AVOID_ANGLE /
      DANGER_THRESHOLD / DANGER_DISTANCE）。m/mm の二重定義は通信仕様由来である旨をコメント
- [x] `_sim_flight2` と `calculate_avoidance_command` を**対称に**変更し、
      ハーネス（シナリオA/C・全11項目）で両経路の一致を確認
- [x] 死んだ条件 `if step >= 6:` は温存し、NOTE コメントで [!question] へ誘導

### Phase 4: 仕上げ
- [ ] `robot_visualize.py` のタイポ・重複整理（あれば）
- [ ] README 更新（回帰チェック・dataclass・記号対応表への言及）
- [ ] 最終回帰チェック

## 進め方

- 1 Phase = 1 コミット以上
- 各 Phase 完了時に回帰チェックを実行し、ベースラインと一致を確認してから次へ
- 挙動差分が出たら即停止して原因を報告

> [!note] 回帰ハーネスの守備範囲
> ハーネスは `calc()`/`update_belief()`/agent の回避計算を通すが、control_pc の
> ソケット本番経路（Localizer 経由）は通らない。よって control_pc.py の変更は
> **dataclass 波及の最小限**に留め、構文チェックと目視で確認する。
