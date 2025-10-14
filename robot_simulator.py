# -*- coding: utf-8 -*-
"""
Robot Simulator - 固定データ版
====================================
このプログラムは実際のロボットの動作をシミュレートします。
ロボットは制御PCとだけ通信します。

オプションB: 固定のテストデータを返す
- 位置情報は持たない（env_serverが管理）
- 固定の物体定位データを制御PCに送信
- 制御PCから移動指令を受信

メインループの流れ:
1. 固定の物体定位データを準備
2. 制御PCに「ステップ番号と物体定位結果」を送信
3. 制御PCから「回避方向、移動距離、パルス方向」を受信
"""

import time
import json
import socket
import sys

# 既存のモジュールをインポート
import config


class RobotSimulator:
    """
    シンプルなロボットシミュレータ（固定データ版）
    制御PCとだけ通信する
    """

    def __init__(self, control_pc_host='localhost', control_pc_port=5001):
        """
        ロボットシミュレータの初期化

        Args:
            control_pc_host (str): 制御PCのホスト
            control_pc_port (int): 制御PCのポート
        """
        # 通信設定
        self.control_pc_addr = (control_pc_host, control_pc_port)

        print("=" * 60)
        print("ロボットシミュレータを初期化しました（固定データ版）")
        print("=" * 60)

    def get_fixed_detections(self, step):
        """
        固定の物体定位データを返す（テスト用）

        実機では、超音波センサーから取得するデータ

        Args:
            step (int): ステップ番号

        Returns:
            list: [{'distance': float, 'angle': float}, ...]
                  distance: 物体までの距離 [mm]
                  angle: 物体の角度 [度] (-180 ~ 180)
        """
        # 固定のテストデータ
        # ステップごとに少しずつ変化させる（シミュレーション的に）
        base_detections = [
            {"distance": 1500.0 + step * 10, "angle": 15.0},
            {"distance": 2300.0 + step * 15, "angle": -22.0},
            {"distance": 1800.0 + step * 12, "angle": 5.0},
        ]

        return base_detections

    def request_to_control_pc(self, step, detections):
        """
        制御PCに問い合わせて移動指令を受信

        Args:
            step (int): ステップ番号
            detections (list): 物体定位結果

        Returns:
            dict: 移動指令 {'avoidance_direction', 'move_distance', 'pulse_direction'}
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(30.0)  # 30秒のタイムアウト
            sock.connect(self.control_pc_addr)

            # 送信データ：ステップ番号と物体定位結果のみ
            request = {
                'step': step,
                'detections': detections
            }

            sock.send(json.dumps(request).encode())

            # 受信：回避方向、移動距離、パルス方向
            response = sock.recv(4096).decode()
            sock.close()

            return json.loads(response)

        except socket.timeout:
            print(f"  ✗ 制御PCへの接続がタイムアウトしました")
            raise
        except ConnectionRefusedError:
            print(f"  ✗ 制御PCに接続できません（{self.control_pc_addr}）")
            print("    control_pc.pyが起動していることを確認してください")
            raise
        except Exception as e:
            print(f"  ✗ 制御PCとの通信エラー: {e}")
            raise

    def run(self, max_steps=20):
        """
        メインループ

        Args:
            max_steps (int): 最大ステップ数
        """
        print("\n" + "=" * 60)
        print("ロボットシミュレータ起動")
        print("=" * 60 + "\n")

        try:
            for step in range(1, max_steps + 1):  # ステップ1から開始
                print(f"\n--- ステップ {step} ---")

                # 1. 固定の物体定位データを取得
                print("[1] 物体定位中...")
                detections = self.get_fixed_detections(step)
                print(f"    {len(detections)}個の障害物を検出（固定データ）")
                for det in detections:
                    print(f"      距離={det['distance']:.1f}mm, 角度={det['angle']:.1f}度")

                # 2. 制御PCに送信して指令を待つ
                print("[2] 制御PCに問い合わせ中...")
                response = self.request_to_control_pc(step, detections)

                if response.get('status') == 'ok':
                    command = response['command']
                    print(f"    指令受信: 回避={command['avoidance_direction']:.1f}°, "
                          f"移動={command['move_distance']:.1f}mm, パルス={command['pulse_direction']:.1f}°")
                    
                    # 3. 移動実行（1秒）
                    print("[3] 移動中...")
                    time.sleep(1.0)
                    print("    移動完了")
                else:
                    print(f"    ✗ エラー: {response.get('error', 'Unknown')}")

            print("" + "=" * 60)
            print("ロボットシミュレータ終了")
            print("=" * 60)

        except KeyboardInterrupt:
            print("中断されました")
        except Exception as e:
            print(f"\n✗ エラー: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║     ロボットシミュレータ（固定データ版）                 ║
║                                                          ║
║  事前に以下を起動してください:                           ║
║    1. env_server.py                                      ║
║    2. control_pc.py                                      ║
╚══════════════════════════════════════════════════════════╝
    """)

    max_steps = 200
    if len(sys.argv) > 1:
        try:
            max_steps = int(sys.argv[1])
            print(f"ステップ数: {max_steps}\n")
        except ValueError:
            print("デフォルト: 20ステップ\n")

    robot = RobotSimulator(control_pc_host='localhost', control_pc_port=5001)
    robot.run(max_steps=max_steps)
