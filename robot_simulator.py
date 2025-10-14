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
import os
import base64

# 既存のモジュールをインポート
from bayes_code import config


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
        print("ロボットシミュレータを初期化しました")
        print("=" * 60)

    def load_data_multi_file(self, step):
        """
        指定されたステップの _data_Multi.dat ファイルを読み込む

        Args:
            step (int): ステップ番号（1から開始）

        Returns:
            dict: {'file_path': str, 'content_base64': str} or None (ファイルが存在しない場合)
        """
        # ステップ番号は1から始まるが、ファイル名は0000から始まる
        file_index = step - 1
        file_name = f"{file_index:04d}_data_Multi.dat"
        file_path = os.path.join("robot_data", "wall_000", "goldorak", str(file_index), file_name)

        if not os.path.exists(file_path):
            print(f"    警告: ファイルが見つかりません: {file_path}")
            return None

        try:
            # ファイルをバイナリモードで読み込み
            with open(file_path, 'rb') as f:
                content_bytes = f.read()

            # Base64エンコード（JSON送信のため、特殊文字をエスケープ）
            # 2MBのファイルでも安全にJSON文字列として送信可能
            content_base64 = base64.b64encode(content_bytes).decode('ascii')

            print(f"    ファイル読み込み成功: {file_path} ({len(content_bytes)} bytes → {len(content_base64)} bytes Base64)")
            return {
                'file_path': file_path,
                'content_base64': content_base64
            }
        except Exception as e:
            print(f"    エラー: ファイル読み込み失敗: {file_path}, {e}")
            return None

    def request_to_control_pc(self, step, data_multi_file=None):
        """
        制御PCに問い合わせて移動指令を受信

        Args:
            step (int): ステップ番号
            data_multi_file (dict): {'file_path': str, 'content_base64': str} or None

        Returns:
            dict: 移動指令 {'avoidance_direction', 'move_distance', 'pulse_direction'}
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(30.0)  # 30秒のタイムアウト
            sock.connect(self.control_pc_addr)

            # 送信データ：ステップ番号、ファイルデータ（Base64エンコード済み）
            request = {
                'step': step,
                'data_multi_file': data_multi_file  # None または {'file_path': str, 'content_base64': str}
            }

            # JSONをバイト列に変換
            request_json = json.dumps(request)
            request_bytes = request_json.encode('utf-8')

            # 【大容量データ送信プロトコル】
            # 1. データサイズを先に送信（4バイト、ビッグエンディアン）
            #    受信側がデータ全体を受信するまでループできるようにする
            data_size = len(request_bytes)
            sock.sendall(data_size.to_bytes(4, byteorder='big'))

            # 2. 実際のJSONデータを送信（sendallで確実に全送信）
            sock.sendall(request_bytes)

            print(f"    データ送信完了: {data_size} bytes")

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

                # 1. _data_Multi.dat ファイルを読み込む
                print("[1] _data_Multi.dat ファイル読み込み中...")
                data_multi_file = self.load_data_multi_file(step)

                # 2. 制御PCに送信して指令を待つ
                print("[2] 制御PCに問い合わせ中...")
                response = self.request_to_control_pc(step, data_multi_file)

                if response.get('status') == 'ok':
                    command = response['command']
                    print(f"    指令受信: 回避={command['avoidance_direction']:.1f}°, "
                          f"移動={command['move_distance']:.1f}mm, パルス={command['pulse_direction']:.1f}°")

                    # 3. 移動実行（1秒）
                    print("  [3] 移動中...")
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
