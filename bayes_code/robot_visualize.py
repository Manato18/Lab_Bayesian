import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import imageio

class BatVisualizer:
    def __init__(self, X, Y, c_percentile, min_p, x_max, y_max, wall_x, wall_y, show_echo_plots=True):
        """
        コウモリのエコロケーションデータを可視化するクラス

        Parameters:
        -----------
        X, Y : np.ndarray
            メッシュグリッド
        c_percentile : float
            カラーマップのパーセンタイル
        min_p : float
            最小確率値
        x_max, y_max : float
            プロット範囲
        wall_x, wall_y : np.ndarray
            壁の座標
        show_echo_plots : bool, optional
            エコー受信タイミングのプロットを表示するかどうか (デフォルト: True)
        """
        self.X = X
        self.Y = Y
        self.c_percentile = c_percentile
        self.min_p = min_p
        self.x_max = x_max
        self.y_max = y_max
        self.wall_x = wall_x
        self.wall_y = wall_y
        self.show_echo_plots = show_echo_plots

        # 壁の描画用
        # 壁の座標を閉じたポリゴンにするために最初の点を最後にも追加
        if len(self.wall_x) > 0 and len(self.wall_y) > 0:
            if self.wall_x[0] != self.wall_x[-1] or self.wall_y[0] != self.wall_y[-1]:
                self.wall_x = np.append(self.wall_x, self.wall_x[0])
                self.wall_y = np.append(self.wall_y, self.wall_y[0])

        self.wall_x_draw = self.wall_x
        self.wall_y_draw = self.wall_y

    def create_figure(self):
        """基本的な図の作成"""
        if self.show_echo_plots:
            fig = plt.figure(figsize=(45, 18), constrained_layout=False)
            # サブプロットの作成（5列グリッドに変更）
            ax_y = fig.add_subplot(3, 5, (1, 6))   # 実topview データ
            ax_y2 = fig.add_subplot(3, 5, (2, 7))  # 実topview データ
            ax_y3 = fig.add_subplot(3, 5, (3, 8))  # 実topview データ
            ax_y4 = fig.add_subplot(3, 5, (4, 9))  # 実topview データ
            ax_histogram = fig.add_subplot(3, 5, (5, 10))  # 角度評価ヒストグラム
            ax_py_xL = fig.add_subplot(615)        # エコーデータL
            ax_py_xR = fig.add_subplot(616)        # エコーデータR
        else:
            fig = plt.figure(figsize=(45, 12), constrained_layout=False)
            # エコープロットなしの場合は上部のプロットのみ作成
            ax_y = fig.add_subplot(1, 5, 1)   # 実topview データ
            ax_y2 = fig.add_subplot(1, 5, 2)  # 実topview データ
            ax_y3 = fig.add_subplot(1, 5, 3)  # 実topview データ
            ax_y4 = fig.add_subplot(1, 5, 4)  # 実topview データ
            ax_histogram = fig.add_subplot(1, 5, 5)  # 角度評価ヒストグラム
            ax_py_xL = None
            ax_py_xR = None

        plt.rcParams["font.size"] = 18

        # タイトル設定
        ax_y.set_title("likely_hood_newL")
        ax_y2.set_title("likely_hood_newR")
        ax_y3.set_title("confidence_matrix")
        ax_y4.set_title("posterior(with mmemory)")
        ax_histogram.set_title("Avoidance Angle Evaluation")

        return fig, (ax_y, ax_y2, ax_y3, ax_y4, ax_histogram, ax_py_xL, ax_py_xR)

    def setup_colormap(self, ax, data, cmap="GnBu"):
        """カラーマップの設定"""
        # カラーマップの範囲計算
        c_min = np.min(data[np.nonzero(data)])
        c_max = np.percentile(data[np.nonzero(data)], q=self.c_percentile)

        # カラーマップ適用
        pcm = ax.pcolormesh(self.X, self.Y, data, cmap=cmap, vmin=c_min, vmax=c_max)
        return pcm, c_min, c_max

    def setup_axes(self, ax, pcm):
        """軸の設定"""
        ax.set_xlim([0, self.x_max])
        ax.set_ylim([0, self.y_max])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_aspect('equal')

        colorbar = plt.colorbar(pcm, ax=ax, orientation="vertical")
        return colorbar

    def plot_elements(self, ax, bat_x, bat_y, body_x, body_y, pulse_x, pulse_y, pole_x, pole_y, obs_x=None, obs_y=None, bat_color='k'):
        """基本要素のプロット"""
        # コウモリ、ポール、壁のプロット
        bat = ax.plot(bat_x, bat_y, f'{bat_color}o', markersize=20, label="bat")[0]
        pole = ax.plot(pole_x, pole_y, 'ro', markersize=15, markerfacecolor='None', markeredgecolor='r', label="pole")[0]

        # 方向線を短くする（元の長さの1/3に）
        # 体の方向（赤線）
        body_dir_x = body_x - bat_x
        body_dir_y = body_y - bat_y
        body_length = np.sqrt(body_dir_x**2 + body_dir_y**2)
        body_dir_x_short = bat_x + (body_dir_x / body_length) * (body_length / 3)
        body_dir_y_short = bat_y + (body_dir_y / body_length) * (body_length / 3)
        fd = ax.plot(np.array([bat_x, body_dir_x_short]), np.array([bat_y, body_dir_y_short]), 'r-', markersize=5, label="fd")[0]

        # パルス方向（黒線）
        pulse_dir_x = pulse_x - bat_x
        pulse_dir_y = pulse_y - bat_y
        pulse_length = np.sqrt(pulse_dir_x**2 + pulse_dir_y**2)
        pulse_dir_x_short = bat_x + (pulse_dir_x / pulse_length) * (pulse_length / 3)
        pulse_dir_y_short = bat_y + (pulse_dir_y / pulse_length) * (pulse_length / 3)
        pd = ax.plot(np.array([bat_x, pulse_dir_x_short]), np.array([bat_y, pulse_dir_y_short]), 'y-', markersize=5, label="pd")[0]

        # 壁のプロット
        wall = ax.plot(self.wall_x_draw, self.wall_y_draw, 'k-', linewidth=2, label="wall")[0]

        # オブザベーションのプロット（配列に対応、空配列の場合は何もプロットしない）
        if obs_x is not None and obs_y is not None and len(obs_x) > 0:
            # 複数の物体を検出した場合、全ての物体位置をプロット
            obs_now = ax.plot(obs_x, obs_y, 'm+', markersize=20, markeredgewidth=5, label=r"observation $y$")[0]
            return bat, pole, fd, pd, wall, obs_now

        return bat, pole, fd, pd, wall

    def plot_echo_timing(self, ax, t_ax, y_e_vec, label_name):
        """エコー受信タイミングのプロット"""
        ax.set_xlim([0, 10])
        ax.set_ylim([0, 1])
        ax.set_ylabel(label_name)

        echo_line = ax.plot(t_ax*10**3, y_e_vec, 'm-', markersize=20, label=r"$y^{echo}$")[0]
        ax.legend(loc="best")

        return echo_line

    def plot_frame(self, frame_idx, bat_x, bat_y, body_x, body_y, pulse_x, pulse_y,
                  pole_x, pole_y, y_x, y_y, data1, data2, data3, data4,
                  t_ax, y_el_vec, y_er_vec, output_dir):
        """単一フレームのプロットと保存"""
        # 出力ディレクトリの確認/作成
        os.makedirs(output_dir, exist_ok=True)

        # 図の作成
        fig, (ax_y, ax_y2, ax_y3, ax_y4, ax_py_xL, ax_py_xR) = self.create_figure()

        # データプロット
        pcm1, c_min1, c_max1 = self.setup_colormap(ax_y, data1[frame_idx])
        pcm2, c_min2, c_max2 = self.setup_colormap(ax_y2, data2[frame_idx])
        pcm3, c_min3, c_max3 = self.setup_colormap(ax_y3, data3[frame_idx])

        # data4の特殊なカラースケール処理
        c_min4 = np.min(data4[frame_idx][data4[frame_idx] > 2*self.min_p])
        data_temp4 = data4[frame_idx][data4[frame_idx] > c_min4]
        c_max4 = np.percentile(data_temp4[data_temp4 < np.max(data_temp4)], q=self.c_percentile)
        pcm4 = ax_y4.pcolormesh(self.X, self.Y, data4[frame_idx], cmap="GnBu", vmin=c_min4, vmax=c_max4)

        # 軸の設定
        cb1 = self.setup_axes(ax_y, pcm1)
        cb2 = self.setup_axes(ax_y2, pcm2)
        cb3 = self.setup_axes(ax_y3, pcm3)
        cb4 = self.setup_axes(ax_y4, pcm4)

        # 各パネルに要素をプロット
        self.plot_elements(
            ax_y, bat_x[frame_idx], bat_y[frame_idx], body_x[frame_idx], body_y[frame_idx],
            pulse_x[frame_idx], pulse_y[frame_idx], pole_x, pole_y,
            y_x[frame_idx], y_y[frame_idx]
        )

        # 他のパネルにも同様の要素をプロット
        self.plot_elements(
            ax_y2, bat_x[frame_idx], bat_y[frame_idx], body_x[frame_idx], body_y[frame_idx],
            pulse_x[frame_idx], pulse_y[frame_idx], pole_x, pole_y,
            y_x[frame_idx], y_y[frame_idx]
        )

        self.plot_elements(
            ax_y3, bat_x[frame_idx], bat_y[frame_idx], body_x[frame_idx], body_y[frame_idx],
            pulse_x[frame_idx], pulse_y[frame_idx], pole_x, pole_y,
            y_x[frame_idx], y_y[frame_idx]
        )

        self.plot_elements(
            ax_y4, bat_x[frame_idx], bat_y[frame_idx], body_x[frame_idx], body_y[frame_idx],
            pulse_x[frame_idx], pulse_y[frame_idx], pole_x, pole_y,
            y_x[frame_idx], y_y[frame_idx]
        )

        # エコータイミングのプロット（表示オプションが有効な場合のみ）
        if self.show_echo_plots and ax_py_xL is not None and ax_py_xR is not None:
            self.plot_echo_timing(ax_py_xL, t_ax, y_el_vec[frame_idx], "Left received echo")
            echo_right = self.plot_echo_timing(ax_py_xR, t_ax, y_er_vec[frame_idx], "Right received echo")
            ax_py_xR.set_xlabel("time [ms]")

        # フレームのインデックス表示
        ax_y.text(0.1, 0.8, f"sensing_n = {frame_idx}", transform=ax_y.transAxes)

        # 画像を保存
        fig.savefig(f"{output_dir}/frame_{frame_idx:04d}.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        return f"{output_dir}/frame_{frame_idx:04d}.png"


    def plot_single_step(self, step_idx,
                        bat_x, bat_y, fd, pd,
                        pole_x, pole_y,
                        obs_x, obs_y,
                        data1, data2, data3, data4, data5,
                        t_ax, y_el_vec, y_er_vec,
                        output_dir,
                        emergency_avoidance=False,
                        angle_evaluation=None):
        """
        単一ステップのプロットと保存（履歴配列不要、過去の観測点なし）

        Parameters:
        -----------
        step_idx : int
            ステップ番号
        bat_x, bat_y : float
            ロボットのX, Y位置（スカラー値）
        fd : float
            飛行方向（度）
        pd : float
            パルス方向（度）
        pole_x, pole_y : np.ndarray
            ポール（障害物）の座標
        obs_x, obs_y : float or np.ndarray
            現在の観測点の座標（スカラー値または配列）
        data1, data2, data3, data4, data5 : np.ndarray
            ベイズ推論データ（2D配列: (Mx+1, My+1)）
        t_ax : np.ndarray
            時間軸
        y_el_vec, y_er_vec : np.ndarray
            左右のエコーベクトル（1D配列: (len(t_ax),)）
        output_dir : str
            出力ディレクトリ
        
        Returns:
        --------
        str: 保存した画像のパス
        """
        # 出力ディレクトリの確認/作成
        os.makedirs(output_dir, exist_ok=True)
        
        # 方向矢印の終点座標を計算（矢印の長さ = 1m）
        arrow_length = 1.0
        body_x = arrow_length * np.cos(np.deg2rad(fd)) + bat_x
        body_y = arrow_length * np.sin(np.deg2rad(fd)) + bat_y
        pulse_x = arrow_length * np.cos(np.deg2rad(pd)) + bat_x
        pulse_y = arrow_length * np.sin(np.deg2rad(pd)) + bat_y
        
        # 緊急回避時はロボットの色を黄色に変更
        bat_color = 'y' if emergency_avoidance else 'k'

        # 図の作成
        fig, (ax_y, ax_y2, ax_y3, ax_y4, ax_histogram, ax_py_xL, ax_py_xR) = self.create_figure()
        
        # データプロット
        pcm1, c_min1, c_max1 = self.setup_colormap(ax_y, data1)
        pcm2, c_min2, c_max2 = self.setup_colormap(ax_y2, data5)
        pcm3, c_min3, c_max3 = self.setup_colormap(ax_y3, data2)
        
        # data4の特殊なカラースケール処理
        if np.any(data4 > 2*self.min_p):
            c_min4 = np.min(data4[data4 > 2*self.min_p])
            data_temp4 = data4[data4 > c_min4]
            if data_temp4.size > 0 and np.any(data_temp4 < np.max(data_temp4)):
                filtered_data = data_temp4[data_temp4 < np.max(data_temp4)]
                if filtered_data.size > 0:
                    c_max4 = np.percentile(filtered_data, q=self.c_percentile)
                else:
                    c_max4 = np.max(data_temp4)
            else:
                c_max4 = np.max(data_temp4) if data_temp4.size > 0 else 0
        else:
            c_min4 = self.min_p
            c_max4 = 0
        
        pcm4 = ax_y4.pcolormesh(self.X, self.Y, data4, cmap="GnBu", vmin=c_min4, vmax=c_max4)
        
        # 軸の設定
        cb1 = self.setup_axes(ax_y, pcm1)
        cb2 = self.setup_axes(ax_y2, pcm2)
        cb3 = self.setup_axes(ax_y3, pcm3)
        cb4 = self.setup_axes(ax_y4, pcm4)
        
        # 各パネルに要素をプロット（過去の観測点なし）
        self.plot_elements(
            ax_y, bat_x, bat_y, body_x, body_y,
            pulse_x, pulse_y, pole_x, pole_y,
            obs_x, obs_y, bat_color=bat_color
        )
        
        self.plot_elements(
            ax_y2, bat_x, bat_y, body_x, body_y,
            pulse_x, pulse_y, pole_x, pole_y,
            obs_x, obs_y, bat_color=bat_color
        )
        
        self.plot_elements(
            ax_y3, bat_x, bat_y, body_x, body_y,
            pulse_x, pulse_y, pole_x, pole_y,
            obs_x, obs_y, bat_color=bat_color
        )
        
        self.plot_elements(
            ax_y4, bat_x, bat_y, body_x, body_y,
            pulse_x, pulse_y, pole_x, pole_y,
            obs_x, obs_y, bat_color=bat_color
        )
        
        # エコータイミングのプロット（表示オプションが有効な場合のみ）
        if self.show_echo_plots and ax_py_xL is not None and ax_py_xR is not None:
            self.plot_echo_timing(ax_py_xL, t_ax, y_el_vec, "Left received echo")
            self.plot_echo_timing(ax_py_xR, t_ax, y_er_vec, "Right received echo")
            ax_py_xR.set_xlabel("time [ms]")
        
        # ステップ番号表示
        ax_y.text(0.1, 0.8, f"sensing_n = {step_idx}", transform=ax_y.transAxes)

        # 角度評価ヒストグラムのプロット
        if angle_evaluation is not None and angle_evaluation['angle_results'] is not None:
            angle_results = angle_evaluation['angle_results']
            selected_angle = angle_evaluation['selected_angle']

            # 角度と合計値のリストを作成
            angles = sorted(angle_results.keys())
            totals = [angle_results[angle]['total'] for angle in angles]

            # selected_angleがanglesに含まれているかチェック
            if selected_angle in angles:
                # 通常モード：ヒストグラムを表示
                colors = ['green' if angle == selected_angle else 'blue' for angle in angles]
                bars = ax_histogram.bar(angles, totals, width=4, color=colors, edgecolor='black', alpha=0.7)

                # 選択された角度に目立つマーカーを追加
                selected_idx = angles.index(selected_angle)
                ax_histogram.plot(selected_angle, totals[selected_idx], 'r*', markersize=20,
                                label=f'Selected: {selected_angle:.0f}°')

                ax_histogram.set_xlabel('Angle [deg] (fd-based)', fontsize=14)
                ax_histogram.set_ylabel('Total Posterior Value', fontsize=14)
                ax_histogram.grid(True, alpha=0.3)
                ax_histogram.legend(fontsize=12)
                ax_histogram.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
            else:
                # 緊急回避モード：メッセージを表示
                ax_histogram.text(0.5, 0.5, f'Emergency Avoidance Mode\nSelected angle: {selected_angle:.0f}°',
                                ha='center', va='center', transform=ax_histogram.transAxes,
                                fontsize=16, color='red', weight='bold')
                ax_histogram.set_xlabel('Angle [deg]')
                ax_histogram.set_ylabel('Total Posterior Value')
        else:
            # angle_evaluationがない場合は空のプロット
            ax_histogram.text(0.5, 0.5, 'No avoidance analysis\n(step < 8)',
                            ha='center', va='center', transform=ax_histogram.transAxes,
                            fontsize=16)
            ax_histogram.set_xlabel('Angle [deg]')
            ax_histogram.set_ylabel('Total Posterior Value')

        # 画像を保存
        fig.savefig(f"{output_dir}/frame_{step_idx:04d}.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        return f"{output_dir}/frame_{step_idx:04d}.png"


    def create_gif_from_frames(self, output_dir, start_trial, end_trial, duration=0.2):
        """
        指定ディレクトリ内のフレーム画像からGIFアニメーションを作成する

        Parameters:
        -----------
        output_dir : str
            画像ファイルがあるディレクトリパス
        start_trial : int
            開始試行番号（GIFファイル名用）
        end_trial : int
            終了試行番号（GIFファイル名用）
        duration : float
            フレーム間の表示時間（秒）

        Returns:
        --------
        str:
            作成したGIFファイルのパス、またはエラー時は空文字列
        """
        # 画像ファイルを取得してソート
        image_files = sorted(glob.glob(os.path.join(output_dir, "frame_*.png")))

        if not image_files:
            print("画像ファイルが見つかりません")
            return ""

        # GIFファイル名を設定
        gif_path = os.path.join(output_dir, f"bat_visualization_{start_trial}-{end_trial}.gif")

        # GIFアニメーションを作成
        print(f"フレーム数: {len(image_files)}、GIF作成中...")
        try:
            with imageio.get_writer(gif_path, mode='I', duration=duration) as writer:
                for img_file in image_files:
                    image = imageio.imread(img_file)
                    writer.append_data(image)

            print(f"GIFアニメーション作成完了: {gif_path}")
            return gif_path
        except Exception as e:
            print(f"GIF作成エラー: {e}")
            return ""
