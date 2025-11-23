# ロボット×ベイズ 実行クイックガイド

現場での起動順と最低限の確認事項をまとめました。詳細な仕様は各README/既存ドキュメントを参照してください。

## 1. ハード・ネットワーク準備
- Raspi4を2台用意（メイン：ヘッド/センサ用、サブ：足元用）。
- PCは時間取得用の簡易サーバーを立てるため、`Bats_Swarming_Robot/robot_pc_connection_test.py` を起動しておく:
  - `python robot_pc_connection_test.py`

## 2. メインRaspiでの事前チェック
- いつも通りメインRaspiへSSHし、コードを開く。
- 音響系チェック：`startcarbat.go` を実行し、マイク2本が入力するか確認。
- サーモホン（超音波センサ）もこのタイミングで発音を確認。

## 3. 足元用Raspiのセッティング
- メインRaspiから足元用RaspiへSSH接続。
- `motor_code/README.md` を参照しつつ、以下を実行（デフォルト値は `--dis-scale=1`, `--turn-scale=1`）:
  - 前進調整: `python3 roboclaw_test.py --mode distance --dis-scale <前進スケール>`
  - 旋回調整: `python3 roboclaw_test.py --mode angle --turn-scale <回転スケール>`
- パラメータが決まったら、受信サーバーを起動したままにする:
  - `python3 udp_roboclaw_server.py --dis-scale <決定値> --turn-scale <決定値>`
  - これによりメインRaspiからの移動距離・回転・パルス放射方向の指令を受け取れる。

## 4. モーションキャプチャ準備（Motive）
- マーカー配置:
  - ヘッド（サーモホン）に3点。
  - ボディに3点以上。
  - 障害物も事前に配置しておく。
- Motiveでリジッドボディ名を設定:
  - `robot_head`, `robot_body`, `obstacles`（大量の障害物での動作は未検証）。

## 5. MoCapリアルタイム取得の確認
- `Lab_Bayesian/marker_server.py` を実行（デフォルトは本番モードの配信サーバーとして動作。必要に応じ `--mode test` でダミー確認も可）。本番時は起動したままにする。
- 別ターミナルで `Lab_Bayesian/marker_test.py` を実行し、リアルタイムにデータが取れているか確認。
- 詳細オプションやデータ仕様は `Lab_Bayesian/marker.md` を参照。

## 6. ベイズ初期化と本番起動
- 本番で `startcarbat.go` を走らせる前に、PC側で `Lab_Bayesian/control_pc.py` を起動:
  - `python control_pc.py`
  - モーションキャプチャからのリアルタイム位置を受け取り、ベイズの初期化を行う。`startcarbat.go` から呼び出される前提なので必ず先に起動。
- その後、メインRaspiで `startcarbat.go` を本番実行。
