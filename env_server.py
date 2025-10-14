# -*- coding: utf-8 -*-
"""
Environment Server - 環境情報サーバー
====================================
このプログラムは環境のグランドトゥルース（正解データ）を管理します。

機能:
1. 障害物の位置情報を管理
2. ロボットの実際の位置を記録
3. 制御PCからの問い合わせに応答
"""

import socket
import json
import pandas as pd
from bayes_code import config


class EnvServer:
    """
    環境情報サーバー
    ロボットの位置と障害物情報を管理する
    """

    def __init__(self, host='localhost', port=5000):
        """
        環境情報サーバーの初期化

        Args:
            host (str): サーバーのホスト
            port (int): サーバーのポート
        """
        self.host = host
        self.port = port

        # ロボットの現在位置（初期値）
        self.robot_position = {
            'x': config.init_pos[0],
            'y': config.init_pos[1],
            'fd': config.init_pos[2],
            'pd': config.init_pos[3]
        }

        # 障害物データを読み込み
        self.obstacles = self.load_obstacles()

        print("=" * 60)
        print("環境情報サーバーを初期化しました")
        print(f"ロボット初期位置: x={self.robot_position['x']:.3f}m, "
              f"y={self.robot_position['y']:.3f}m")
        print(f"障害物数: {len(self.obstacles['pole_x'])}個")
        print("=" * 60)

    def load_obstacles(self):
        """障害物データをCSVから読み込み"""
        try:
            csv_path = f"{config.folder_name}/chain_position_y.csv"
            chain_loc = pd.read_csv(csv_path, header=0)

            obs_x = chain_loc["X"].values + config.margin_space
            obs_y = chain_loc["Y"].values + config.margin_space

            print(f"障害物データを読み込みました: {csv_path}")

            return {
                'pole_x': obs_x.tolist(),
                'pole_y': obs_y.tolist()
            }
        except Exception as e:
            print(f"警告: 障害物データの読み込みに失敗: {e}")
            return {'pole_x': [], 'pole_y': []}

    def handle_request(self, request):
        """
        要求を処理

        Args:
            request (dict): クライアントからの要求

        Returns:
            dict: 応答
        """
        command = request.get('command')

        if command == 'get_robot_position':
            # ロボットの現在位置を返す
            return self.robot_position

        elif command == 'get_obstacles':
            # 障害物情報を返す
            return self.obstacles

        elif command == 'update_position':
            # ロボットの位置を更新
            new_pos = request.get('data')
            if new_pos:
                self.robot_position = new_pos
                print(f"位置更新: x={new_pos['x']:.3f}m, y={new_pos['y']:.3f}m, "
                      f"fd={new_pos['fd']:.1f}度, pd={new_pos['pd']:.1f}度")
                return {'status': 'ok'}
            else:
                return {'status': 'error', 'message': 'No position data provided'}

        else:
            return {'status': 'error', 'message': f'Unknown command: {command}'}

    def run(self):
        """サーバーを起動"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(5)

        print(f"\n{'='*60}")
        print("環境情報サーバー起動")
        print(f"待機中: {self.host}:{self.port}")
        print(f"{'='*60}\n")

        try:
            while True:
                conn, addr = server.accept()

                try:
                    # データを受信
                    data = conn.recv(16384).decode()
                    request = json.loads(data)

                    # 要求を処理
                    response = self.handle_request(request)

                    # 応答を送信
                    conn.send(json.dumps(response).encode())

                except json.JSONDecodeError as e:
                    print(f"✗ JSONデコードエラー: {e}")
                    error_response = {'status': 'error', 'message': 'Invalid JSON'}
                    conn.send(json.dumps(error_response).encode())

                except Exception as e:
                    print(f"✗ エラー: {e}")
                    error_response = {'status': 'error', 'message': str(e)}
                    conn.send(json.dumps(error_response).encode())

                finally:
                    conn.close()

        except KeyboardInterrupt:
            print("\n\n" + "=" * 60)
            print("環境情報サーバー終了")
            print("=" * 60)

        finally:
            server.close()


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║            環境情報サーバー                              ║
║                                                          ║
║  このプログラムを最初に起動してください                  ║
╚══════════════════════════════════════════════════════════╝
    """)

    server = EnvServer(host='localhost', port=5000)
    server.run()
