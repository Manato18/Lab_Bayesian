# -*- coding: utf-8 -*-
"""
Robot Simulator - 実機ロボット模擬版（Golang風）
====================================
実機ロボット（Golang）のコードに似せたシミュレータ
bayes_serverと実際にTCP通信を行います。

動作フロー（Golangロボットと同じ）:
1. エコーセンシング（DoSensing）→ 相互相関データをダミー生成
2. サーバーに相互相関データを送信 [[crosscor_l], [crosscor_r]]
3. サーバーから移動指令を受信 {"Time": ..., "NextMove": ..., "NextAngle": ..., "PulseDirection": ...}
4. 移動実行（DoSingleMove）→ sleep
5. パルス発生（CreateCall）→ ログ出力

通信先:
- control_pc.py (localhost:6001)
"""

import time
import json
import socket
import sys
import os
import numpy as np


class RobotSimulator:
    """
    実機ロボット（Golang）の動作を模擬するシミュレータ
    """

    def __init__(self, server_host='localhost', server_port=6001):
        """
        ロボットシミュレータの初期化

        Args:
            server_host (str): control_pcのホスト
            server_port (int): control_pcのポート
        """
        self.server_addr = (server_host, server_port)
        self.step = 0

        print("=" * 60)
        print("ロボットシミュレータ（Golang風）を初期化しました")
        print(f"サーバー: {server_host}:{server_port}")
        print("=" * 60)

    def do_sensing(self, step):
        """
        エコーセンシング（DoSensing相当）

        相互相関データを生成します。
        実機では音響センシングを行いますが、ここでは：
        - 既存の _data_Multi.dat ファイルから読み込み
        - または、ダミーデータを生成

        Args:
            step (int): ステップ番号（1から開始）

        Returns:
            tuple: (crosscor_l, crosscor_r) - 左右の相互相関データ（配列）
        """
        # ステップ番号は1から始まるが、ファイル名は0000から始まる
        file_index = step - 1
        file_name = f"{file_index:04d}_data_Multi.dat"
        file_path = os.path.join("robot_data", "wall_000", "goldorak", str(file_index), file_name)

        if os.path.exists(file_path):
            # _data_Multi.dat ファイルから相互相関データを読み込み
            return self._read_crosscor_from_file(file_path)
        else:
            # ファイルが見つからない場合はダミーデータを生成
            print(f"    警告: {file_path} が見つかりません。ダミーデータを生成します。")
            return self._generate_dummy_crosscor()

    def _read_crosscor_from_file(self, file_path):
        """
        _data_Multi.dat ファイルから相互相関データを読み込む

        Args:
            file_path (str): ファイルパス

        Returns:
            tuple: (crosscor_l, crosscor_r)
        """
        crosscor_l = []
        crosscor_r = []

        try:
            with open(file_path, 'r') as f:
                lines = f.read().split("\n")
                for i in range(1, len(lines) - 1):
                    parts = lines[i].replace('"', '').split(" ")
                    if len(parts) >= 4:
                        # parts[2]: corrL, parts[3]: corrR
                        crosscor_l.append(float(parts[2]))
                        crosscor_r.append(float(parts[3]))

            print(f"    ファイル読み込み成功: {file_path} ({len(crosscor_l)} samples)")
            return crosscor_l, crosscor_r

        except Exception as e:
            print(f"    エラー: ファイル読み込み失敗: {e}")
            return self._generate_dummy_crosscor()

    def _generate_dummy_crosscor(self):
        """
        ダミーの相互相関データを生成

        Returns:
            tuple: (crosscor_l, crosscor_r)
        """
        # 30000サンプルのダミーデータ（ノイズ + いくつかのピーク）
        crosscor_l = np.random.normal(0, 0.1, 30000).tolist()
        crosscor_r = np.random.normal(0, 0.1, 30000).tolist()

        # いくつかのピークを追加（物体をシミュレート）
        peak_positions = [5000, 10000, 15000]
        for pos in peak_positions:
            if pos < len(crosscor_l):
                crosscor_l[pos] += 0.5
                crosscor_r[pos + 100] += 0.4  # 時間差を付ける

        print(f"    ダミーデータ生成: {len(crosscor_l)} samples")
        return crosscor_l, crosscor_r

    def send_data_to_server(self, crosscor_l, crosscor_r):
        """
        サーバーに相互相関データを送信して、移動指令を受信

        Golangロボットと同じプロトコル：
        - 送信: [[crosscor_l], [crosscor_r]] (JSON)
        - 受信: {"Time": str, "NextMove": float (m), "NextAngle": float (rad), "PulseDirection": float (rad)}

        Args:
            crosscor_l (list): 左の相互相関データ
            crosscor_r (list): 右の相互相関データ

        Returns:
            dict: 移動指令 {'Time': str, 'NextMove': float, 'NextAngle': float, 'PulseDirection': float}
        """
        try:
            # TCPソケット接続
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(30.0)
            sock.connect(self.server_addr)

            # データをJSON形式で送信
            data = [crosscor_l, crosscor_r]
            data_json = json.dumps(data)
            sock.sendall(data_json.encode('utf-8'))
            sock.sendall(b'\n')  # 改行で終端を示す（Golangサーバーと同じ）

            print(f"    データ送信完了: {len(data_json)} bytes")

            # 移動指令を受信
            response_bytes = sock.recv(4096)
            sock.close()

            response = json.loads(response_bytes.decode('utf-8'))
            return response

        except socket.timeout:
            print(f"  ✗ サーバーへの接続がタイムアウトしました")
            raise
        except ConnectionRefusedError:
            print(f"  ✗ サーバーに接続できません（{self.server_addr}）")
            print("    control_pc.pyが起動していることを確認してください")
            raise
        except Exception as e:
            print(f"  ✗ サーバーとの通信エラー: {e}")
            raise

    def do_single_move(self, next_move, next_angle, pulse_direction=0.0):
        """
        移動実行（DoSingleMove相当）

        実機では実際にモーター制御を行いますが、ここではsleepで模擬

        Args:
            next_move (float): 移動距離 [m]
            next_angle (float): 回避角度 [rad]
            pulse_direction (float): パルス放射方向 [rad]
        """
        print(f"    [移動実行] 距離={next_move:.3f}m, 角度={next_angle:.3f}rad ({np.degrees(next_angle):.1f}度), "
              f"パルス方向={pulse_direction:.3f}rad ({np.degrees(pulse_direction):.1f}度)")
        time.sleep(1.0)  # 移動時間をシミュレート
        print(f"    移動完了")


    def run(self, max_steps=20):
        """
        メインループ（Golangのfor文に相当）

        Args:
            max_steps (int): 最大ステップ数
        """
        print("\n" + "=" * 60)
        print("ロボットシミュレータ起動（Golang風）")
        print("=" * 60 + "\n")

        try:
            for step in range(1, max_steps + 1):
                print(f"\n--- ステップ {step} ---")

                # 1. エコーセンシング（DoSensing）
                print("[1] エコーセンシング実行中...")
                crosscor_l, crosscor_r = self.do_sensing(step)

                # 2. サーバーに送信して移動指令を受信
                print("[2] サーバーに問い合わせ中...")
                response = self.send_data_to_server(crosscor_l, crosscor_r)

                if 'NextMove' in response and 'NextAngle' in response:
                    next_move = response['NextMove']
                    next_angle = response['NextAngle']
                    pulse_direction = response.get('PulseDirection', 0.0)  # デフォルト0.0
                    print(f"    指令受信: NextMove={next_move:.3f}m, NextAngle={next_angle:.3f}rad, "
                          f"PulseDirection={pulse_direction:.3f}rad")

                    # 3. 移動実行（DoSingleMove）
                    print("[3] 移動中...")
                    self.do_single_move(next_move, next_angle, pulse_direction)


                else:
                    print(f"    ✗ エラー: {response}")

            print("\n" + "=" * 60)
            print("ロボットシミュレータ終了")
            print("=" * 60)

        except KeyboardInterrupt:
            print("\n中断されました")
        except Exception as e:
            print(f"\n✗ エラー: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║     ロボットシミュレータ（Golang風）                     ║
║                                                          ║
║  事前に以下を起動してください:                           ║
║    1. marker_server.py --mode test --port 6000           ║
║    2. control_pc.py                                      ║
║                                                          ║
║  このシミュレータは実機ロボット（Golang）の動作を        ║
║  模擬し、control_pc.pyとTCP通信（ポート6001）します。    ║
╚══════════════════════════════════════════════════════════╝
    """)

    max_steps = 20
    if len(sys.argv) > 1:
        try:
            max_steps = int(sys.argv[1])
            print(f"ステップ数: {max_steps}\n")
        except ValueError:
            print("デフォルト: 20ステップ\n")

    robot = RobotSimulator(server_host='localhost', server_port=6001)
    robot.run(max_steps=max_steps)
