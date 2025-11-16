# -*- coding: utf-8 -*-
"""
Control PC - 制御PC
====================================
このプログラムはベイズ推論と回避計算を担当します。

機能:
1. marker_trackerからロボットの位置を取得
2. ロボットからの物体定位情報を受信
3. ベイズ推論で事後確率を更新
4. 回避方向を計算
5. 内部で位置を管理・更新
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
from bayes_code.robot_visualize import BatVisualizer
from marker_tracker_client import MarkerTrackerClient


class ControlPC:
    """
    制御PC
    ベイズ推論と回避計算を担当する
    """

    def __init__(self, host='localhost', port=6001,
                 marker_tracker_host='localhost', marker_tracker_port=6000):
        """
        制御PCの初期化

        Args:
            host (str): 制御PCのホスト
            port (int): 制御PCのポート
            marker_tracker_host (str): marker_trackerのホスト
            marker_tracker_port (int): marker_trackerのポート
        """
        self.host = host
        self.port = port

        print("=" * 60)
        print("制御PCを初期化中...")
        print("=" * 60)

        # MarkerTrackerClient初期化
        print("MarkerTrackerClient初期化中...")
        self.marker_tracker = MarkerTrackerClient(
            host=marker_tracker_host,
            port=marker_tracker_port
        )
        print("✓ MarkerTrackerClient初期化完了")

        # 接続テスト
        print("marker_trackerへの接続を確認中...")
        if not self.marker_tracker.test_connection():
            print("警告: marker_trackerへの接続に失敗しました")
            print("  marker_tracker.pyが起動していることを確認してください")
            print("  テストモード: python marker_tracker.py --mode test --port 6000")

        # World初期化（障害物データはまだ読み込まない）
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

        # 障害物をmarker_trackerから取得（'obstacles'マーカーセットから）
        print("marker_trackerから障害物情報を取得中...")
        obs_x, obs_y = self.marker_tracker.get_obstacles('obstacles')

        if len(obs_x) > 0:
            # marker_trackerから取得した障害物データで上書き
            self.world.pole_x = obs_x
            self.world.pole_y = obs_y
            print(f"✓ 障害物データ取得完了: {len(obs_x)}個")
        else:
            print("警告: marker_trackerから障害物を取得できませんでした")
            print("  障害物全てを選択して'obstacles'という名前のマーカーセットを作成してください")
            # Worldが既に読み込んでいるCSVデータを使用（もしあれば）
            if self.world.pole_x is not None and len(self.world.pole_x) > 0:
                print(f"  フォールバック: CSVから読み込んだ障害物データを使用（{len(self.world.pole_x)}個）")
            else:
                print("  警告: 障害物データがありません")

        print("✓ World初期化完了")

        # Bayesian初期化
        print("Bayesian初期化中...")
        self.bayesian = Bayesian(
            sigma2=config.sigma2,
            min_p=config.min_p,
            c=config.c
        )
        print("✓ Bayesian初期化完了")

        # marker_trackerからロボットの初期位置を取得
        print("marker_trackerからロボット初期位置を取得中...")
        init_pos_data = self.get_robot_position_from_marker_tracker(
            body_marker_set='robot_body',
            head_marker_set='robot_head'
        )

        if init_pos_data is None:
            # フォールバック: configの初期値を使用
            print("警告: marker_trackerから位置取得失敗、config初期値を使用")
            init_pos = config.init_pos
        else:
            init_pos = [init_pos_data['x'], init_pos_data['y'],
                        init_pos_data['fd'], init_pos_data['pd']]

        print(f"✓ ロボット初期位置: x={init_pos[0]:.3f}m, y={init_pos[1]:.3f}m, "
              f"fd={init_pos[2]:.1f}度, pd={init_pos[3]:.1f}度")

        # 現在位置を内部管理
        self.current_position = {
            'x': init_pos[0],
            'y': init_pos[1],
            'fd': init_pos[2],
            'pd': init_pos[3]
        }

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
                "init_pos": init_pos
            },
            world=self.world
        )
        print("✓ Agent初期化完了")

        # Bayesianの初期化（事後分布の準備）
        print("Bayesian事後分布初期化中...")
        self.bayesian.Init(self.world, self.agent)
        print("✓ Bayesian事後分布初期化完了")

        # ファイル保存先ディレクトリの作成
        self.save_dir = os.path.join(config.folder_name, "output", "robot_data")
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

        # BatVisualizer初期化（可視化）
        print("BatVisualizer初期化中...")
        # 壁の座標（正方形の部屋を想定）
        wall_x = np.array([0, config.x_max, config.x_max, 0])
        wall_y = np.array([0, 0, config.y_max, config.y_max])

        self.visualizer = BatVisualizer(
            X=self.world.X,
            Y=self.world.Y,
            c_percentile=config.c_percentile,
            min_p=config.min_p,
            x_max=config.x_max,
            y_max=config.y_max,
            wall_x=wall_x,
            wall_y=wall_y,
            show_echo_plots=False  # エコープロット非表示
        )
        print("✓ BatVisualizer初期化完了")

        # 可視化用の出力ディレクトリ
        self.viz_output_dir = os.path.join(config.folder_name, "output", "visualization")
        os.makedirs(self.viz_output_dir, exist_ok=True)
        print(f"✓ 可視化出力ディレクトリ: {self.viz_output_dir}")

        # 初期状態の可視化
        print("初期状態を可視化中...")
        self.plot_initial_state()
        print("✓ 初期状態の可視化完了")

        # ステップカウンター（実機ロボット用）
        self.step_counter = 0

        print("=" * 60)
        print("制御PC初期化完了")
        print("  ※ 新形式（相互相関配列）に対応")
        print("=" * 60)

    def get_robot_position_from_marker_tracker(self, body_marker_set='robot_body',
                                               head_marker_set='robot_head'):
        """
        marker_trackerからロボット位置を取得

        Args:
            body_marker_set (str): ロボット本体のマーカーセット名
            head_marker_set (str): ロボット頭部のマーカーセット名

        Returns:
            dict or None: {'x': float, 'y': float, 'fd': float, 'pd': float}
        """
        try:
            position = self.marker_tracker.get_robot_position(
                body_marker_set=body_marker_set,
                head_marker_set=head_marker_set
            )

            if position is None:
                return None

            # z座標は無視（2D平面での動作を想定）
            return {
                'x': position['x'],
                'y': position['y'],
                'fd': position['fd'],
                'pd': position['pd']
            }

        except Exception as e:
            print(f"✗ marker_trackerからの位置取得エラー: {e}")
            return None

    def update_current_position(self, new_position):
        """
        内部で管理している現在位置を更新

        Args:
            new_position (dict): {'x': float, 'y': float, 'fd': float, 'pd': float}
        """
        self.current_position = new_position
        print(f"位置更新: x={new_position['x']:.3f}m, y={new_position['y']:.3f}m, "
              f"fd={new_position['fd']:.1f}度, pd={new_position['pd']:.1f}度")

    def plot_initial_state(self):
        """
        初期状態を可視化して保存

        ロボット位置、頭部方向、放射方向、壁、障害物を図示します。
        """
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        fig, ax = plt.subplots(figsize=(10, 10))

        # 壁を描画（四角形）
        wall_x_min = self.world.wall_x[0]
        wall_x_max = self.world.wall_x[1]
        wall_y_min = self.world.wall_y[0]
        wall_y_max = self.world.wall_y[1]

        wall_rect = patches.Rectangle(
            (wall_x_min, wall_y_min),
            wall_x_max - wall_x_min,
            wall_y_max - wall_y_min,
            linewidth=3,
            edgecolor='black',
            facecolor='none',
            label='Wall'
        )
        ax.add_patch(wall_rect)

        # 障害物を描画
        if len(self.world.pole_x) > 0:
            ax.scatter(
                self.world.pole_x,
                self.world.pole_y,
                c='red',
                s=100,
                marker='o',
                label=f'Obstacles ({len(self.world.pole_x)})',
                zorder=3
            )

        # ロボット位置を描画
        robot_x = self.current_position['x']
        robot_y = self.current_position['y']
        fd = self.current_position['fd']
        pd = self.current_position['pd']

        # ロボット本体（青い点）
        ax.scatter(
            robot_x, robot_y,
            c='blue',
            s=200,
            marker='o',
            label='Robot Position',
            zorder=5
        )

        # 矢印の長さ
        arrow_length = 0.3

        # 頭部方向（pd）を緑の矢印で描画
        pd_rad = np.radians(pd)
        ax.arrow(
            robot_x, robot_y,
            arrow_length * np.cos(pd_rad),
            arrow_length * np.sin(pd_rad),
            head_width=0.1,
            head_length=0.1,
            fc='green',
            ec='green',
            linewidth=2,
            label=f'Head Direction (pd)={pd:.1f}°',
            zorder=6
        )

        # 放射方向（fd）を紫の矢印で描画
        fd_rad = np.radians(fd)
        ax.arrow(
            robot_x, robot_y,
            arrow_length * np.cos(fd_rad),
            arrow_length * np.sin(fd_rad),
            head_width=0.1,
            head_length=0.1,
            fc='purple',
            ec='purple',
            linewidth=2,
            linestyle='--',
            label=f'Pulse Direction (fd)={fd:.1f}°',
            zorder=6
        )

        # グラフの設定
        ax.set_xlim(-0.2, config.x_max + 0.2)
        ax.set_ylim(-0.2, config.y_max + 0.2)
        ax.set_xlabel('X [m]', fontsize=12)
        ax.set_ylabel('Y [m]', fontsize=12)
        ax.set_title('Control PC - Initial State', fontsize=14, fontweight='bold')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)

        # 位置情報をテキストで表示
        info_text = (
            f'Robot Position: ({robot_x:.3f}, {robot_y:.3f}) m\n'
            f'Head Direction (pd): {pd:.1f}°\n'
            f'Pulse Direction (fd): {fd:.1f}°\n'
            f'Obstacles: {len(self.world.pole_x)}'
        )
        ax.text(
            0.02, 0.98,
            info_text,
            transform=ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        )

        # 保存
        output_path = os.path.join(self.viz_output_dir, 'initial_state.png')
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        plt.close()

        print(f"  初期状態の図を保存: {output_path}")

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

    def calculate_posterior_from_detections(self, step, current_position, detections):
        """
        物体定位結果から事後分布を計算・更新（ベイズ定位点導入.pyと同じ計算フロー）

        このメソッドは、ロボットから受信した物体定位結果（detections）を使って、
        ベイズ推論による事後確率分布を更新します。

        Args:
            step (int): ステップ番号
            current_position (dict): 現在位置 {'x': float, 'y': float, 'fd': float, 'pd': float}
            detections (list): 物体定位結果 [{'distance': mm, 'angle': 度}, ...]

        処理フロー:
            1. detections を XY座標に変換
            2. 耳の位置を計算
            3. 往復距離を計算
            4. エコー到達時間を計算
            5. 空間行列を計算
            6. ベイズ更新を実行
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

        print(f"  [事後分布計算] {len(detections)}個の検出結果から事後分布を更新")

        # detectionsからy_el, y_erを計算
        if len(detections) > 0:
            # 距離と角度を配列に変換
            distances = np.array([det['distance'] for det in detections]) / 1000.0  # mm -> m
            angles = np.array([det['angle'] for det in detections])  # 度

            # calc.pyの関数が期待する2次元配列形式に変換
            distances_2d = distances.reshape(1, -1)
            angles_2d = angles.reshape(1, -1)
            bat_x_2d = np.array([current_position['x']])
            bat_y_2d = np.array([current_position['y']])
            pd_2d = np.array([current_position['pd']])

            # 障害物のXY座標を計算（ベイズ定位点導入.py と同じ）
            obs_x, obs_y = r_theta_to_XY_calc(
                distances_2d, angles_2d,
                bat_x_2d, bat_y_2d, pd_2d
            )

            # 耳の位置を計算
            earL_x, earL_y, earR_x, earR_y = ear_posit(
                current_position['x'], current_position['y'], current_position['pd']
            )

            # 各耳までの往復距離を計算
            goback_L = real_dist_goback(
                current_position['x'], current_position['y'],
                earL_x, earL_y,
                obs_x, obs_y
            )
            goback_R = real_dist_goback(
                current_position['x'], current_position['y'],
                earR_x, earR_y,
                obs_x, obs_y
            )

            # エコー到達時間に変換
            y_el, y_er = space_echo_translater(goback_L, goback_R)

            # bayesianが期待する形状に変換: (1, n)
            y_el = y_el.reshape(1, -1)
            y_er = y_er.reshape(1, -1)

            print(f"  [事後分布計算] エコー時間計算完了: y_el形状={y_el.shape}, y_er形状={y_er.shape}")
        else:
            # 検出がない場合は空の2D配列
            y_el = np.array([[]]).reshape(1, 0)
            y_er = np.array([[]]).reshape(1, 0)
            print(f"  [事後分布計算] 検出なし: 空の観測データで更新")

        # 空間行列を計算
        print(f"  [事後分布計算] 空間行列を計算中...")
        current_r_2vec, current_theta_2vec_rad = r_theta_matrix(
            np.array([current_position['x']]), np.array([current_position['y']]),
            self.world.X, self.world.Y, np.array([current_position['pd']])
        )

        # 耳の位置を再計算（空間行列計算用）
        earL_x, earL_y, earR_x, earR_y = ear_posit(
            current_position['x'], current_position['y'], current_position['pd']
        )

        current_obs_goback_dist_matrix_L = real_dist_goback_matrix(
            np.array([current_position['x']]), np.array([current_position['y']]),
            np.array([earL_x]), np.array([earL_y]),
            self.world.X, self.world.Y
        )
        current_obs_goback_dist_matrix_R = real_dist_goback_matrix(
            np.array([current_position['x']]), np.array([current_position['y']]),
            np.array([earR_x]), np.array([earR_y]),
            self.world.X, self.world.Y
        )

        current_attenuation_matrix = dist_attenuation(current_r_2vec) * \
                                      direc_attenuation(current_theta_2vec_rad)
        current_confidence_matrix = sigmoid(current_attenuation_matrix,
                                           config.threshold, config.grad)

        print(f"  [事後分布計算] 空間行列計算完了")

        # ベイズ更新（戻り値を受け取る）
        print(f"  [事後分布計算] ベイズ更新を実行中...")
        data1, data2, data3, data4 = self.bayesian.update_belief(
            step, y_el, y_er,
            current_obs_goback_dist_matrix_L,
            current_obs_goback_dist_matrix_R,
            current_confidence_matrix
        )
        print(f"  [事後分布計算] ベイズ更新完了")
        
        # エコーベクトルの作成（可視化用）
        y_el_vec = np.ones(len(self.world.t_ax)) * config.eps_y
        y_er_vec = np.ones(len(self.world.t_ax)) * config.eps_y
        
        if len(detections) > 0 and len(y_el[0]) > 0:
            for tl, tr in zip(y_el[0], y_er[0]):
                time_idx_L = np.argmin(np.abs(self.world.t_ax - tl))
                time_idx_R = np.argmin(np.abs(self.world.t_ax - tr))
                y_el_vec[time_idx_L] = 1.0
                y_er_vec[time_idx_R] = 1.0
        
        # 観測点の座標を計算
        if len(detections) > 0:
            # obs_x, obs_yはすでに計算済み（行286-290）
            obs_x_mean = np.mean(obs_x) if obs_x.size > 0 else current_position['x']
            obs_y_mean = np.mean(obs_y) if obs_y.size > 0 else current_position['y']
        else:
            # 検出がない場合はロボット位置を使用
            obs_x_mean = current_position['x']
            obs_y_mean = current_position['y']
        
        # 可視化に必要なデータを返す
        return {
            'data1': data1,
            'data2': data2,
            'data3': data3,
            'data4': data4,
            'y_el_vec': y_el_vec,
            'y_er_vec': y_er_vec,
            'obs_x': obs_x_mean,
            'obs_y': obs_y_mean
        }

    def calculate_movement_command(self, step, current_position):
        """
        更新された事後分布から移動指令を計算

        このメソッドは、ベイズ推論で更新された事後確率分布を解析して、
        ロボットの回避方向と移動距離を決定します。

        実装はAgentクラスのcalculate_avoidance_commandメソッドに委譲されています。
        これにより、コードの重複を削減し、保守性を向上させています。

        Args:
            step (int): ステップ番号
            current_position (dict): 現在位置 {'x': float, 'y': float, 'fd': float, 'pd': float}

        Returns:
            tuple: (command, new_position, emergency_avoidance)
                - command (dict): 移動指令 {'avoidance_direction', 'move_distance', 'pulse_direction'}
                - new_position (dict): 新しい位置 {'x', 'y', 'fd', 'pd'}
                - emergency_avoidance (bool): 緊急回避フラグ
        """
        print(f"  [移動指令計算] 事後分布から回避方向を計算中...")

        # Agentクラスのメソッドに処理を委譲
        command, new_position, emergency_avoidance = self.agent.calculate_avoidance_command(
            current_position, step
        )

        return command, new_position, emergency_avoidance

    def plot_current_state(self, step, current_position, posterior_data, emergency_avoidance=False):
        """
        現在の状態を可視化して画像として保存

        Args:
            step (int): ステップ番号
            current_position (dict): 現在位置 {'x': float, 'y': float, 'fd': float, 'pd': float}
            posterior_data (dict): 事後分布計算結果
                - data1, data2, data3, data4: ベイズ推論データ
                - y_el_vec, y_er_vec: エコーベクトル
                - obs_x, obs_y: 観測点座標

        Returns:
            str: 保存した画像のパス（失敗時はNone）
        """
        print(f"  [可視化] ステップ{step}の状態を可視化中...")

        try:
            # plot_single_step()を呼び出して可視化
            image_path = self.visualizer.plot_single_step(
                step_idx=step,
                bat_x=current_position['x'],
                bat_y=current_position['y'],
                fd=current_position['fd'],
                pd=current_position['pd'],
                pole_x=self.world.pole_x,
                pole_y=self.world.pole_y,
                obs_x=posterior_data['obs_x'],
                obs_y=posterior_data['obs_y'],
                data1=posterior_data['data1'],
                data2=posterior_data['data2'],
                data3=posterior_data['data3'],
                data4=posterior_data['data4'],
                t_ax=self.world.t_ax,
                y_el_vec=posterior_data['y_el_vec'],
                y_er_vec=posterior_data['y_er_vec'],
                output_dir=self.viz_output_dir,
                emergency_avoidance=emergency_avoidance
            )
            
            print(f"  ✓ 可視化完了: {image_path}")
            return image_path
            
        except Exception as e:
            print(f"  ✗ 可視化エラー: {e}")
            import traceback
            traceback.print_exc()
            return None

    @staticmethod
    def _mm_to_m(mm_value):
        """mm → m 変換"""
        return mm_value / 1000.0

    @staticmethod
    def _m_to_mm(m_value):
        """m → mm 変換"""
        return m_value * 1000.0

    @staticmethod
    def _deg_to_rad(deg_value):
        """度 → ラジアン 変換"""
        return np.radians(deg_value)

    @staticmethod
    def _rad_to_deg(rad_value):
        """ラジアン → 度 変換"""
        return np.degrees(rad_value)

    def handle_real_robot_request(self, crosscor_l, crosscor_r):
        """
        新形式の要求を処理（実機ロボット用）

        Args:
            crosscor_l (list): 左耳の相互相関データ
            crosscor_r (list): 右耳の相互相関データ

        Returns:
            dict: 応答（実機ロボット形式）
                {'Time': str, 'NextMove': float (m), 'NextAngle': float (rad)}
        """
        try:
            # ステップカウンターをインクリメント
            self.step_counter += 1
            step = self.step_counter

            print(f"\nステップ{step}: 実機ロボットから要求受信")
            print(f"  相互相関データ: L={len(crosscor_l)} samples, R={len(crosscor_r)} samples")

            # 物体定位を実行（新メソッド）
            print("  [定位計算] 物体定位を実行中...")
            detections = self.localizer.localize_from_crosscor(crosscor_l, crosscor_r)

            if len(detections) > 0:
                print(f"  [定位計算] {len(detections)}個の物体を検出しました")
                for idx, det in enumerate(detections):
                    print(f"    物体{idx+1}: 距離={det['distance']:.1f}mm, "
                          f"角度={det['angle']:.1f}度, "
                          f"強度={det['intensity']:.3f}")
            else:
                print("  [定位計算] 物体は検出されませんでした")

            # marker_trackerから現在位置を取得（リアルタイム更新）
            print("  marker_trackerから現在位置を取得中...")
            marker_position = self.get_robot_position_from_marker_tracker(
                body_marker_set='robot_body',
                head_marker_set='robot_head'
            )

            if marker_position is not None:
                # marker_trackerから取得できた場合は更新
                current_position = marker_position
                self.update_current_position(current_position)
            else:
                # 取得失敗の場合は内部管理の位置を使用
                print("  警告: marker_tracker取得失敗、前回の位置を使用")
                current_position = self.current_position

            print(f"  現在位置: ({current_position['x']:.3f}, {current_position['y']:.3f})")

            # 事後分布を計算・更新
            posterior_data = self.calculate_posterior_from_detections(step, current_position, detections)

            # 移動指令を計算
            command, new_position, emergency_avoidance = self.calculate_movement_command(step, current_position)

            # 新しい位置を内部管理
            self.update_current_position(new_position)

            # 現在の状態を可視化
            self.plot_current_state(step, current_position, posterior_data, emergency_avoidance)

            # 実機ロボット形式で応答を作成
            import datetime
            response = {
                'Time': datetime.datetime.now().isoformat(),
                'NextMove': self._mm_to_m(command['move_distance']),  # mm → m
                'NextAngle': self._deg_to_rad(command['avoidance_direction']),  # 度 → rad
                'PulseDirection': self._deg_to_rad(command['pulse_direction'])  # 度 → rad
            }

            print(f"  応答送信: NextMove={response['NextMove']:.3f}m, "
                  f"NextAngle={response['NextAngle']:.3f}rad ({command['avoidance_direction']:.1f}度), "
                  f"PulseDirection={response['PulseDirection']:.3f}rad ({command['pulse_direction']:.1f}度)")

            return response

        except Exception as e:
            print(f"  ✗ エラー: {e}")
            import traceback
            traceback.print_exc()
            # エラー時も形式を合わせる
            import datetime
            return {
                'Time': datetime.datetime.now().isoformat(),
                'NextMove': 0.0,
                'NextAngle': 0.0,
                'PulseDirection': 0.0
            }

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

            # marker_trackerから現在位置を取得（リアルタイム更新）
            print("  marker_trackerから現在位置を取得中...")
            marker_position = self.get_robot_position_from_marker_tracker(
                body_marker_set='robot_body',
                head_marker_set='robot_head'
            )

            if marker_position is not None:
                # marker_trackerから取得できた場合は更新
                current_position = marker_position
                self.update_current_position(current_position)
            else:
                # 取得失敗の場合は内部管理の位置を使用
                print("  警告: marker_tracker取得失敗、前回の位置を使用")
                current_position = self.current_position

            print(f"  現在位置: ({current_position['x']:.3f}, {current_position['y']:.3f})")

            # 事後分布を計算・更新（戻り値を受け取る）
            posterior_data = self.calculate_posterior_from_detections(step, current_position, detections)

            # 移動指令を計算
            command, new_position, emergency_avoidance = self.calculate_movement_command(step, current_position)

            # 新しい位置を内部管理（env_serverへの更新は不要）
            self.update_current_position(new_position)

            # 現在の状態を可視化
            self.plot_current_state(step, current_position, posterior_data, emergency_avoidance)

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
        """サーバーを起動（新形式：相互相関配列）"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(5)

        print(f"\n{'='*60}")
        print("制御PC起動（新形式：相互相関配列）")
        print(f"待機中: {self.host}:{self.port}")
        print(f"{'='*60}\n")

        try:
            while True:
                conn, addr = server.accept()
                print(f"\n[接続受信] robot_simulator から接続 ({addr[0]}:{addr[1]})")

                try:
                    # 【新形式受信プロトコル】
                    # 改行終端のJSONデータを受信
                    data_bytes = b''
                    while True:
                        chunk = conn.recv(4096)
                        if not chunk:
                            break
                        data_bytes += chunk
                        # 改行が見つかったら終了
                        if b'\n' in data_bytes:
                            break

                    print(f"  データ受信完了: {len(data_bytes)} bytes")

                    # JSONデコード
                    data = data_bytes.decode('utf-8').strip()
                    request = json.loads(data)

                    # リクエスト形式を確認
                    if isinstance(request, list) and len(request) == 2:
                        # 新形式: [[crosscor_l], [crosscor_r]]
                        crosscor_l = request[0]
                        crosscor_r = request[1]
                        response = self.handle_real_robot_request(crosscor_l, crosscor_r)
                    else:
                        # 不明な形式
                        raise ValueError(f"不明なリクエスト形式: {type(request)}")

                    # 応答を送信
                    response_json = json.dumps(response)
                    conn.send(response_json.encode('utf-8'))

                except json.JSONDecodeError as e:
                    print(f"✗ JSONデコードエラー: {e}")
                    import datetime
                    error_response = {
                        'Time': datetime.datetime.now().isoformat(),
                        'NextMove': 0.0,
                        'NextAngle': 0.0,
                        'PulseDirection': 0.0
                    }
                    conn.send(json.dumps(error_response).encode())

                except Exception as e:
                    print(f"✗ エラー: {e}")
                    import traceback
                    traceback.print_exc()
                    import datetime
                    error_response = {
                        'Time': datetime.datetime.now().isoformat(),
                        'NextMove': 0.0,
                        'NextAngle': 0.0,
                        'PulseDirection': 0.0
                    }
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
║  marker_tracker.pyを起動してから実行してください         ║
║  テストモード: python marker_tracker.py --mode test --port 6000
║                                                          ║
║  Motive側で以下を設定:                                   ║
║  - robot_body: ロボット本体のマーカーセット              ║
║                (z座標最大の点をロボット位置とする)       ║
║  - robot_head: ロボット頭部のマーカーセット（3点）       ║
║                (真ん中の点をヘッド位置とする)            ║
║  - obstacles: 障害物全てを含む1つのマーカーセット        ║
║               (各マーカーが個別の障害物を表す)           ║
╚══════════════════════════════════════════════════════════╝
    """)

    control_pc = ControlPC(
        host='localhost',
        port=6001,
        marker_tracker_host='localhost',
        marker_tracker_port=6000
    )
    control_pc.run()
