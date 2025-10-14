# -*- coding: utf-8 -*-
"""
Control PC - 制御PC
====================================
このプログラムはベイズ推論と回避計算を担当します。

機能:
1. env_serverからロボットの位置を取得
2. ロボットからの物体定位情報を受信
3. ベイズ推論で事後確率を更新
4. 回避方向を計算
5. env_serverにロボットの新しい位置を更新
6. 移動指令を返す
"""

import socket
import json
import numpy as np
import os
import base64
from bayes_code import config
from bayes_code.world import World
from bayes_code.bayesian import Bayesian
from bayes_code.agent import Agent, Obj
from bayes_code.calc import r_theta_matrix, real_dist_goback_matrix, dist_attenuation, direc_attenuation, sigmoid, ear_posit, r_theta_to_XY_calc, real_dist_goback, space_echo_translater
from bayes_code.localization import Localizer


class ControlPC:
    """
    制御PC
    ベイズ推論と回避計算を担当する
    """

    def __init__(self, host='localhost', port=5001, env_server_host='localhost', env_server_port=5000):
        """
        制御PCの初期化

        Args:
            host (str): 制御PCのホスト
            port (int): 制御PCのポート
            env_server_host (str): 環境情報サーバーのホスト
            env_server_port (int): 環境情報サーバーのポート
        """
        self.host = host
        self.port = port
        self.env_server_addr = (env_server_host, env_server_port)

        print("=" * 60)
        print("制御PCを初期化中...")
        print("=" * 60)

        # World初期化
        print("World初期化中...")
        self.world = World(
            x_max=config.x_max,
            y_max=config.y_max,
            margin_space=config.margin_space,
            h=config.h,
            t_max=config.t_max,
            dt=config.dt,
            c=config.c,
            folder_name=config.folder_name
        )
        print("✓ World初期化完了")

        # Bayesian初期化
        print("Bayesian初期化中...")
        self.bayesian = Bayesian(
            sigma2=config.sigma2,
            min_p=config.min_p,
            c=config.c
        )
        print("✓ Bayesian初期化完了")

        # env_serverからロボットの初期位置を取得
        print("env_serverからロボット初期位置を取得中...")
        init_pos_data = self.get_robot_position_from_env_server()
        init_pos = [init_pos_data['x'], init_pos_data['y'],
                    init_pos_data['fd'], init_pos_data['pd']]
        print(f"✓ ロボット初期位置: x={init_pos[0]:.3f}m, y={init_pos[1]:.3f}m, "
              f"fd={init_pos[2]:.1f}度, pd={init_pos[3]:.1f}度")

        # Agent初期化（事後分布の計算に必要）
        print("Agent初期化中...")
        self.agent = Agent(
            bayesian=self.bayesian,
            margin_space=config.margin_space,
            folder_name=config.folder_name,
            X=self.world.X,
            Y=self.world.Y,
            sim={
                "trials": config.trials,
                "init_pos": init_pos  # env_serverから取得した位置
            },
            world=self.world
        )
        print("✓ Agent初期化完了")

        # Bayesianの初期化（事後分布の準備）
        print("Bayesian事後分布初期化中...")
        self.bayesian.Init(self.world, self.agent)
        print("✓ Bayesian事後分布初期化完了")

        # ファイル保存先ディレクトリの作成
        self.save_dir = os.path.join("bayse_olddata2", "output", "robot_data")
        os.makedirs(self.save_dir, exist_ok=True)
        print(f"✓ ファイル保存先ディレクトリ: {self.save_dir}")

        # Localizer初期化（物体定位計算）
        print("Localizer初期化中...")
        self.localizer = Localizer(
            mic_distance=116,  # マイク間距離 [mm]
            temperature=21,     # 温度 [℃]
            threshold=0.15      # ピーク検出閾値
        )
        print("✓ Localizer初期化完了")

        print("=" * 60)
        print("制御PC初期化完了")
        print("=" * 60)

    def get_robot_position_from_env_server(self):
        """
        env_serverからロボット位置を取得

        Returns:
            dict: {'x': float, 'y': float, 'fd': float, 'pd': float}
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect(self.env_server_addr)
            print(f"[接続成功] env_server ({self.env_server_addr[0]}:{self.env_server_addr[1]}) に接続しました")

            request = {'command': 'get_robot_position'}
            sock.send(json.dumps(request).encode())
            response = sock.recv(4096).decode()
            sock.close()

            return json.loads(response)

        except Exception as e:
            print(f"✗ env_serverからの位置取得エラー: {e}")
            raise

    def update_robot_position_to_env_server(self, new_position):
        """
        env_serverにロボットの新しい位置を更新

        Args:
            new_position (dict): {'x': float, 'y': float, 'fd': float, 'pd': float}

        Returns:
            bool: 成功/失敗
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect(self.env_server_addr)

            request = {
                'command': 'update_position',
                'data': new_position
            }
            sock.send(json.dumps(request).encode())
            response = sock.recv(1024).decode()
            sock.close()

            result = json.loads(response)
            return result.get('status') == 'ok'

        except Exception as e:
            print(f"✗ env_serverへの位置更新エラー: {e}")
            return False

    def save_data_multi_file(self, step, file_path, content_base64):
        """
        受信した _data_Multi.dat ファイルを保存

        Args:
            step (int): ステップ番号
            file_path (str): 元のファイルパス（例: robot_data/wall_000/goldorak/0/0000_data_Multi.dat）
            content_base64 (str): Base64エンコードされたファイル内容

        Returns:
            str or None: 保存先のファイルパス（成功時）、None（失敗時）
        """
        try:
            # Base64デコード（送信時にエンコードされたデータを元に戻す）
            content_bytes = base64.b64decode(content_base64)

            # ファイル名を取得（例: 0000_data_Multi.dat）
            file_name = os.path.basename(file_path)

            # 保存先ディレクトリにステップ番号のサブディレクトリを作成
            step_dir = os.path.join(self.save_dir, f"step_{step:04d}")
            os.makedirs(step_dir, exist_ok=True)

            # 保存先のフルパス
            save_path = os.path.join(step_dir, file_name)

            # ファイルを保存（バイナリモード）
            with open(save_path, 'wb') as f:
                f.write(content_bytes)

            print(f"  ✓ ファイル保存完了: {save_path} ({len(content_bytes)} bytes)")
            return save_path  # 保存先パスを返す

        except Exception as e:
            print(f"  ✗ ファイル保存エラー: {e}")
            return None

    def calculate_movement(self, step, current_position, detections):
        """
        物体定位結果から移動指令を計算

        Args:
            step (int): ステップ番号
            current_position (dict): {'x', 'y', 'fd', 'pd'}
            detections (list): [{'distance', 'angle'}, ...]

        Returns:
            dict: 移動指令 {'avoidance_direction', 'move_distance', 'pulse_direction'}
                  と新しい位置 {'x', 'y', 'fd', 'pd'}
        """
        # Agentの位置を更新
        self.agent.PositionX = current_position['x']
        self.agent.PositionY = current_position['y']
        self.agent.fd = current_position['fd']
        self.agent.pd = current_position['pd']
        self.agent.step_idx = step

        # 物体定位情報をObjインスタンスに変換
        self.agent.Newobj = []
        for det in detections:
            self.agent.Newobj.append(Obj(Dis=det['distance'], Deg=det['angle']))

        # 耳の位置を計算
        earL_x, earL_y, earR_x, earR_y = ear_posit(
            self.agent.PositionX, self.agent.PositionY, self.agent.pd
        )

        # detectionsからy_el, y_erを計算
        if len(detections) > 0:
            # 距離と角度を配列に変換
            distances = np.array([det['distance'] for det in detections]) / 1000.0  # mm -> m
            angles = np.array([det['angle'] for det in detections])  # 度
            
            # 障害物のXY座標を計算
            obs_x, obs_y = r_theta_to_XY_calc(
                distances, angles,
                self.agent.PositionX, self.agent.PositionY, self.agent.pd
            )
            
            # 各耳までの往復距離を計算
            goback_L = real_dist_goback(
                self.agent.PositionX, self.agent.PositionY,
                earL_x, earL_y,
                obs_x, obs_y
            )
            goback_R = real_dist_goback(
                self.agent.PositionX, self.agent.PositionY,
                earR_x, earR_y,
                obs_x, obs_y
            )
            
            # エコー到達時間に変換
            y_el, y_er = space_echo_translater(goback_L, goback_R)
            
            # bayesianが期待する形状に変換: (1, n)
            y_el = y_el.reshape(1, -1)
            y_er = y_er.reshape(1, -1)
            
            print(f"  計算完了: y_el形状={y_el.shape}, y_er形状={y_er.shape}")
        else:
            # 検出がない場合は空の2D配列
            y_el = np.array([[]]).reshape(1, 0)
            y_er = np.array([[]]).reshape(1, 0)

        # ベイズ更新のための空間行列を計算
        current_r_2vec, current_theta_2vec_rad = r_theta_matrix(
            np.array([self.agent.PositionX]), np.array([self.agent.PositionY]),
            self.world.X, self.world.Y, np.array([self.agent.pd])
        )
        current_obs_goback_dist_matrix_L = real_dist_goback_matrix(
            np.array([self.agent.PositionX]), np.array([self.agent.PositionY]),
            np.array([earL_x]), np.array([earL_y]), self.world.X, self.world.Y
        )
        current_obs_goback_dist_matrix_R = real_dist_goback_matrix(
            np.array([self.agent.PositionX]), np.array([self.agent.PositionY]),
            np.array([earR_x]), np.array([earR_y]), self.world.X, self.world.Y
        )
        current_attenuation_matrix = dist_attenuation(current_r_2vec) * \
                                      direc_attenuation(current_theta_2vec_rad)
        current_confidence_matrix = sigmoid(current_attenuation_matrix,
                                           config.threshold, config.grad)

        # ベイズ更新
        self.bayesian.update_belief(
            step, y_el, y_er,
            current_obs_goback_dist_matrix_L,
            current_obs_goback_dist_matrix_R,
            current_confidence_matrix
        )

        # 回避方向の計算
        posterior_sel, X_sel, Y_sel = self.agent._plot_posterior_distribution(
            posx=self.agent.PositionX,
            posy=self.agent.PositionY,
            pd=self.agent.pd,
            fd=self.agent.fd
        )

        angle_results, avoid_angle, value, flag = \
            self.agent._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel)

        # 新しい方向を計算
        new_fd = self.agent.normalize_angle_deg(self.agent.fd - avoid_angle)
        new_pd = self.agent.normalize_angle_deg(self.agent.pd - avoid_angle)
        if step >= 6:
            new_pd = self.agent.normalize_angle_deg(self.agent.fd - (avoid_angle * 2))

        # 移動距離を決定
        if flag:
            move_distance = 150.0  # mm
        else:
            move_distance = 50.0  # mm

        # 新しい位置を計算
        move_distance_m = move_distance / 1000.0  # mm -> m
        new_x = self.agent.PositionX + move_distance_m * np.cos(np.deg2rad(new_fd))
        new_y = self.agent.PositionY + move_distance_m * np.sin(np.deg2rad(new_fd))

        # 移動指令
        command = {
            'avoidance_direction': float(avoid_angle),
            'move_distance': float(move_distance),
            'pulse_direction': float(new_pd)
        }

        # 新しい位置
        new_position = {
            'x': float(new_x),
            'y': float(new_y),
            'fd': float(new_fd),
            'pd': float(new_pd)
        }

        print(f"  → 計算完了: 回避={avoid_angle:.1f}度, 移動={move_distance:.1f}mm")
        print(f"  → 新位置: ({new_x:.3f}, {new_y:.3f}), fd={new_fd:.1f}度, pd={new_pd:.1f}度")

        return command, new_position

    def handle_request(self, request):
        """
        要求を処理

        Args:
            request (dict): ロボットからの要求

        Returns:
            dict: 応答
        """
        try:
            step = request['step']
            data_multi_file = request.get('data_multi_file')  # None または {'file_path': str, 'content_base64': str}

            print(f"\nステップ{step}: ロボットから要求受信")

            # _data_Multi.dat ファイルが送信されている場合は保存して定位計算を実行
            detections = []
            if data_multi_file is not None:
                print(f"  _data_Multi.dat ファイル受信: {data_multi_file['file_path']}")
                save_path = self.save_data_multi_file(step, data_multi_file['file_path'], data_multi_file['content_base64'])

                # 定位計算を実行
                if save_path is not None:
                    print("  [定位計算] 物体定位を実行中...")
                    detections = self.localizer.localize(save_path)
                    if len(detections) > 0:
                        print(f"  [定位計算] {len(detections)}個の物体を検出しました")
                        for idx, det in enumerate(detections):
                            print(f"    物体{idx+1}: 距離={det['distance']:.1f}mm, "
                                  f"角度={det['angle']:.1f}度, "
                                  f"強度={det['intensity']:.3f}")
                    else:
                        print("  [定位計算] 物体は検出されませんでした")
                else:
                    print("  [定位計算] ファイル保存失敗のため定位計算をスキップ")
            else:
                print("  _data_Multi.dat ファイルなし（ファイルが見つからなかった可能性あり）")

            # env_serverから現在位置を取得
            print("  env_serverから現在位置を取得中...")
            current_position = self.get_robot_position_from_env_server()
            print(f"  現在位置: ({current_position['x']:.3f}, {current_position['y']:.3f})")

            # 移動指令を計算（実際のdetectionsを渡す）
            command, new_position = self.calculate_movement(step, current_position, detections)

            # env_serverに新しい位置を更新
            print("  env_serverに新しい位置を更新中...")
            success = self.update_robot_position_to_env_server(new_position)
            if success:
                print("  ✓ 位置更新完了")
            else:
                print("  ✗ 位置更新失敗")

            return {
                'status': 'ok',
                'command': command
            }

        except Exception as e:
            print(f"  ✗ エラー: {e}")
            import traceback
            traceback.print_exc()
            return {
                'status': 'error',
                'error': str(e)
            }

    def run(self):
        """サーバーを起動"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(5)

        print(f"\n{'='*60}")
        print("制御PC起動")
        print(f"待機中: {self.host}:{self.port}")
        print(f"{'='*60}\n")

        try:
            while True:
                conn, addr = server.accept()
                print(f"\n[接続受信] robot_simulator から接続 ({addr[0]}:{addr[1]})")

                try:
                    # 【大容量データ受信プロトコル】
                    # 1. データサイズを先に受信（4バイト、ビッグエンディアン）
                    size_bytes = conn.recv(4)
                    if len(size_bytes) < 4:
                        raise ValueError("データサイズを受信できませんでした")

                    data_size = int.from_bytes(size_bytes, byteorder='big')
                    print(f"  受信予定データサイズ: {data_size} bytes")

                    # 2. 実際のデータを受信（サイズ分を確実に受信）
                    #    2MBのデータでもrecv()は一度に全て受信できないため、ループで受信
                    data_bytes = b''
                    remaining = data_size
                    while remaining > 0:
                        chunk = conn.recv(min(remaining, 65536))  # 64KBずつ受信
                        if not chunk:
                            raise ValueError("データ受信が途中で終了しました")
                        data_bytes += chunk
                        remaining -= len(chunk)

                    print(f"  データ受信完了: {len(data_bytes)} bytes")

                    # 3. JSONデコード
                    data = data_bytes.decode('utf-8')
                    request = json.loads(data)

                    # 要求を処理
                    response = self.handle_request(request)

                    # 応答を送信
                    conn.send(json.dumps(response).encode())

                except json.JSONDecodeError as e:
                    print(f"✗ JSONデコードエラー: {e}")
                    error_response = {'status': 'error', 'error': 'Invalid JSON'}
                    conn.send(json.dumps(error_response).encode())

                except Exception as e:
                    print(f"✗ エラー: {e}")
                    import traceback
                    traceback.print_exc()
                    error_response = {'status': 'error', 'error': str(e)}
                    conn.send(json.dumps(error_response).encode())

                finally:
                    conn.close()

        except KeyboardInterrupt:
            print("\n\n" + "=" * 60)
            print("制御PC終了")
            print("=" * 60)

        finally:
            server.close()


if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║                  制御PC                                  ║
║                                                          ║
║  このプログラムを2番目に起動してください                 ║
║  (1番目: env_server.py)                                  ║
╚══════════════════════════════════════════════════════════╝
    """)

    control_pc = ControlPC(
        host='localhost',
        port=5001,
        env_server_host='localhost',
        env_server_port=5000
    )
    control_pc.run()
