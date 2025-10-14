import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import imageio
import shutil

class BatVisualizer:
    def __init__(self, output_dir, X, Y, c_percentile, min_p, x_max, y_max, wall_x, wall_y):
        """
        コウモリのエコロケーションデータを可視化するクラス
        
        Parameters:
        -----------
        output_dir : str
            出力ディレクトリ
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
        """
        self.output_dir = output_dir
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        self.X = X
        self.Y = Y
        self.c_percentile = c_percentile
        self.min_p = min_p
        self.x_max = x_max
        self.y_max = y_max
        self.wall_x = wall_x
        self.wall_y = wall_y
        
        # 壁の描画用データ準備（固定）
        wall_x_draw_id = np.array([0, 0, 1, 1, 0])
        wall_y_draw_id = np.array([0, 1, 1, 0, 0])
        self.wall_x_draw = wall_x[wall_x_draw_id]
        self.wall_y_draw = wall_y[wall_y_draw_id]
    
    def create_figure(self):
        """基本的な図の作成"""
        fig = plt.figure(figsize=(36, 18), constrained_layout=False)
        plt.rcParams["font.size"] = 18
        
        # サブプロットの作成
        ax_y = fig.add_subplot(3, 4, (1, 5))   # 実topview データ
        ax_y2 = fig.add_subplot(3, 4, (2, 6))  # 実topview データ
        ax_y3 = fig.add_subplot(3, 4, (3, 7))  # 実topview データ
        ax_y4 = fig.add_subplot(3, 4, (4, 8))  # 実topview データ
        ax_py_xL = fig.add_subplot(615)        # エコーデータL
        ax_py_xR = fig.add_subplot(616)        # エコーデータR
        
        # タイトル設定
        ax_y.set_title("likely_hood_newL")
        ax_y2.set_title("confidence_matrix")
        ax_y3.set_title("posterior(without mmemory)")
        ax_y4.set_title("posterior(with mmemory)")
        
        return fig, (ax_y, ax_y2, ax_y3, ax_y4, ax_py_xL, ax_py_xR)
    
    def setup_colormap(self, ax, data, cmap="GnBu", vmin=None, vmax=None):
        """カラーマップの設定（vmin/vmaxを指定可能に）"""
        if vmin is not None and vmax is not None:
            c_min = vmin
            c_max = vmax
        else:
            c_min = np.min(data[np.nonzero(data)])
            c_max = np.percentile(data[np.nonzero(data)], q=self.c_percentile)
        pcm = ax.pcolormesh(self.X, self.Y, data, cmap=cmap, vmin=c_min, vmax=c_max)
        return pcm, c_min, c_max
    
    def setup_axes(self, ax, pcm):
        """軸の設定"""
        # ax.set_xlim([0, self.x_max])
        # ax.set_ylim([0, self.y_max])
        ax.set_xlim([1.5, 7])
        ax.set_ylim([1.5, 7])
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_aspect('equal')
        
        colorbar = plt.colorbar(pcm, ax=ax, orientation="vertical")
        return colorbar
    
    def plot_elements(self, ax, bat_x, bat_y, body_x, body_y, pulse_x, pulse_y, pole_x, pole_y, obs_x=None, obs_y=None, flag=True):
        """基本要素のプロット"""
        # コウモリ、ポール、壁のプロット
        # flagがFalseの場合は黄色のマーカーを使用
        bat_color = 'ko' if flag else 'yo'
        bat = ax.plot(bat_x, bat_y, bat_color, markersize=20, label="bat")[0]
        pole = ax.plot(pole_x, pole_y, 'ro', markersize=15, markerfacecolor='None', markeredgecolor='r', label="pole")[0]
        fd = ax.plot(np.array([bat_x, body_x]), np.array([bat_y, body_y]), 'r-', markersize=20, label="fd")[0]
        pd = ax.plot(np.array([bat_x, pulse_x]), np.array([bat_y, pulse_y]), 'k-', markersize=20, label="pd")[0]
        wall = ax.plot(self.wall_x_draw, self.wall_y_draw, 'k-', linewidth=2, label="wall")[0]
        
        # オブザベーションのプロット（あれば）
        if obs_x is not None and obs_y is not None:
            # 配列の場合は全てプロット
            if hasattr(obs_x, 'shape') and obs_x.shape != ():
                obs_x_plot = obs_x.flatten()
                obs_y_plot = obs_y.flatten()
            else:
                obs_x_plot = [obs_x]
                obs_y_plot = [obs_y]
            obs_now = ax.plot(obs_x_plot, obs_y_plot, 'm+', markersize=20, markeredgewidth=5, label=r"observation $y$")
        
        return bat, pole, fd, pd, wall
    
    def plot_echo_timing(self, ax, t_ax, y_e_vec, label_name):
        """エコー受信タイミングのプロット"""
        ax.set_xlim([0, 10])
        ax.set_ylim([0, 1])
        ax.set_ylabel(label_name)
        
        # y_e_vecがスカラー値の場合の処理
        if np.isscalar(y_e_vec) or (hasattr(y_e_vec, 'shape') and y_e_vec.shape == ()):
            # スカラー値の場合、t_axと同じ長さの配列を作成
            y_e_vec_plot = np.full_like(t_ax, y_e_vec)
        else:
            # 配列の場合、t_axと同じ形状にする
            y_e_vec_plot = np.full_like(t_ax, y_e_vec.flatten()[0] if y_e_vec.size > 0 else y_e_vec)
        
        echo_line = ax.plot(t_ax*10**3, y_e_vec_plot, 'm-', markersize=20, label=r"$y^{echo}$")[0]
        ax.legend(loc="best")
        
        return echo_line
    
    def plot_frame(self, frame_idx, current_bat_x, current_bat_y, current_pd, current_fd,
                  pole_x, pole_y, y_x, y_y, data1, data2, data3, data4, 
                  t_ax, y_el_vec, y_er_vec, flag=True):
        """単一フレームのプロットと保存"""

        pulse_x, pulse_y = direc_arrow(current_bat_x, current_bat_y, current_pd)
        body_x, body_y = direc_arrow(current_bat_x, current_bat_y, current_fd)

        # 図の作成
        fig, (ax_y, ax_y2, ax_y3, ax_y4, ax_py_xL, ax_py_xR) = self.create_figure()
        # 左2つはカラーバー範囲を固定
        pcm1, _, _ = self.setup_colormap(ax_y, data1, vmin=0, vmax=5)
        pcm2, _, _ = self.setup_colormap(ax_y2, data2, vmin=0, vmax=1)
        # 右2つは従来通り
        pcm3, _, _ = self.setup_colormap(ax_y3, data3)
        # データが存在する場合のみ最小値を計算、そうでなければデフォルト値を使用
        if np.any(data4 > 2*self.min_p):
            c_min4 = np.min(data4[data4 > 2*self.min_p])
            data_temp4 = data4[data4 > c_min4]
            # データが存在し、最大値より小さい値がある場合のみパーセンタイルを計算
            if data_temp4.size > 0 and np.any(data_temp4 < np.max(data_temp4)):
                filtered_data = data_temp4[data_temp4 < np.max(data_temp4)]
                if filtered_data.size > 0:
                    c_max4 = np.percentile(filtered_data, q=self.c_percentile)
                else:
                    c_max4 = np.max(data_temp4)  # フィルタリング後のデータが空の場合
            else:
                c_max4 = np.max(data_temp4) if data_temp4.size > 0 else 0  # データが空の場合
        else:
            # データがない場合はデフォルト値を使用
            c_min4 = self.min_p
            c_max4 = 0
        pcm4 = ax_y4.pcolormesh(self.X, self.Y, data4, cmap="GnBu", vmin=c_min4, vmax=c_max4)
        cb1 = self.setup_axes(ax_y, pcm1)
        cb2 = self.setup_axes(ax_y2, pcm2)
        cb3 = self.setup_axes(ax_y3, pcm3)
        cb4 = self.setup_axes(ax_y4, pcm4)
        # 各パネルに要素をプロット
        self.plot_elements(ax_y, current_bat_x, current_bat_y, body_x, body_y, pulse_x, pulse_y, pole_x, pole_y, y_x, y_y, flag)
        self.plot_elements(ax_y2, current_bat_x, current_bat_y, body_x, body_y, pulse_x, pulse_y, pole_x, pole_y, y_x, y_y, flag)
        self.plot_elements(ax_y3, current_bat_x, current_bat_y, body_x, body_y, pulse_x, pulse_y, pole_x, pole_y, y_x, y_y, flag)
        self.plot_elements(ax_y4, current_bat_x, current_bat_y, body_x, body_y, pulse_x, pulse_y, pole_x, pole_y, y_x, y_y, flag)
        self.plot_echo_timing(ax_py_xL, t_ax, y_el_vec, "Left received echo")
        self.plot_echo_timing(ax_py_xR, t_ax, y_er_vec, "Right received echo")
        ax_py_xR.set_xlabel("time [ms]")
        ax_y.text(0.1, 0.8, f"sensing_n = {frame_idx}", transform=ax_y.transAxes)
        fig.savefig(f"{self.output_dir}/frame_{frame_idx:04d}.png", dpi=150, bbox_inches='tight')
        plt.close(fig)
        return f"{self.output_dir}/frame_{frame_idx:04d}.png"


    def create_gif_from_frames(self, start_trial, end_trial, duration=0.2):
        """
        指定ディレクトリ内のフレーム画像からGIFアニメーションを作成する
        
        Parameters:
        -----------
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
        image_files = sorted(glob.glob(os.path.join(self.output_dir, "frame_*.png")))
        
        if not image_files:
            print("画像ファイルが見つかりません")
            return ""
        
        # GIFファイル名を設定
        gif_path = os.path.join(self.output_dir, f"bat_visualization_{start_trial}-{end_trial}.gif")
        
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


###### （パルス・飛行）方向印字関数 #############
def direc_arrow(bat_x, bat_y, direc):
    """
    コウモリの位置から特定の方向を指す矢印の終点座標を計算する関数
    
    視覚化のために、コウモリの現在位置から指定された方向（飛行方向やパルス発射方向）に
    矢印を描画するための終点座標を計算します。
    
    Args:
        bat_x (ndarray): コウモリのx座標配列
        bat_y (ndarray): コウモリのy座標配列
        direc (ndarray): 方向角度（度数法）の配列
        
    Returns:
        tuple: (arrow_end_x, arrow_end_y) - 矢印の終点のx, y座標配列
    """
    # 矢印の長さを定義（メートル単位）
    arrow_length = 1
    
    # 方向角度（度）をラジアンに変換し、三角関数で矢印の終点座標を計算
    arrow_end_x = arrow_length * np.cos(np.deg2rad(direc)) + bat_x
    arrow_end_y = arrow_length * np.sin(np.deg2rad(direc)) + bat_y
    
    return arrow_end_x, arrow_end_y
