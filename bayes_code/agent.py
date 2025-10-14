import pandas
import numpy as np
import math
import matplotlib.pyplot as plt
import os
import csv

from bayes_code import config
from bayes_code.calc import calc

class Obj:
    def __init__(self, Dis=None, Deg=None):
        ## Intens of Extract peak (degree of confidence)
        self.Intens = 1 # ひとまず1で固定
        # Distance of Obj [mm]
        self.Dis = Dis
        # Degree of Obj (-pi ~ pi)
        self.Deg = Deg

class Agent:
    def __init__(self, bayesian, margin_space, folder_name, X, Y, sim=None, world=None):
        self.trials = None
        self.PositionX_all = None
        self.PositionY_all = None
        self.fd_all = None
        self.pd_all = None
        self.Newobj = []  # Objインスタンスのリストとして障害物情報を管理
        
        self.step_idx = 0
        self.PositionX = None
        self.PositionY = None
        self.fd = None
        self.pd = None
        self.margin_space = margin_space
        self.folder_name = folder_name
        self.X = X
        self.Y = Y
        
        # CSVファイルの初期化（位置情報保存用）
        os.makedirs(config.output_dir, exist_ok=True)
        self.csv_filename = os.path.join(config.output_dir, 'position_data.csv')
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['step', 'position_x', 'position_y', 'head_direction', 'pulse_direction'])
            
        # 前回の回避方向を記録する変数（初期値はNone）
        self.last_avoidance_direction = None
        # 連続回避カウンター
        self.consecutive_avoidance_count = 0
        
        self.trials = sim["trials"]
        self.PositionX = sim["init_pos"][0]
        self.PositionY = sim["init_pos"][1]
        self.fd = sim["init_pos"][2]
        self.pd = sim["init_pos"][3]

        self.bayesian = bayesian
    
    def normalize_angle_deg(self, angle_deg):
        """
        角度を-180°～180°の範囲に正規化する関数
        
        Args:
            angle_deg (float or ndarray): 正規化する角度（度数法）
            
        Returns:
            float or ndarray: -180°～180°の範囲に正規化された角度（度数法）
        """
        # 角度を-180°～180°の範囲に正規化
        normalized_angle = ((angle_deg + 180) % 360) - 180
        return normalized_angle
    
    def do_sensing(self, world):
        r_noise, theta_noise, y_el, y_er, y_x, y_y, y_el_vec, y_er_vec, current_obs_goback_dist_matrix_L, current_obs_goback_dist_matrix_R, current_confidence_matrix = calc(world, self.PositionX, self.PositionY, self.fd, self.pd, self.X, self.Y)
        
        # 単位を合わせる
        r_noise = r_noise * 1000
        theta_noise = theta_noise * 180 / math.pi

        # 障害物情報をObjインスタンスで保持
        self.Newobj = []
        if r_noise is not None and theta_noise is not None:
            # r_noise, theta_noiseが配列の場合すべて格納
            if hasattr(r_noise, 'shape') and hasattr(theta_noise, 'shape') and r_noise.shape == theta_noise.shape:
                for r, t in zip(r_noise.flatten(), theta_noise.flatten()):
                    if not (np.isnan(r) or np.isnan(t)):
                        self.Newobj.append(Obj(Dis=r, Deg=t))
            else:
                # スカラー値の場合
                if not (math.isnan(r_noise) or math.isnan(theta_noise)):
                    self.Newobj.append(Obj(Dis=r_noise, Deg=theta_noise))
        
        data1, data2, data3, data4 = self.bayesian.update_belief(self.step_idx, y_el, y_er, current_obs_goback_dist_matrix_L, current_obs_goback_dist_matrix_R, current_confidence_matrix)
        return y_x, y_y, y_el_vec, y_er_vec, data1, data2, data3, data4
    
    def one_step(self, step_idx, visualizer):
        self.step_idx = step_idx
        
        # 現在の位置情報をCSVに保存
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([self.step_idx, self.PositionX, self.PositionY, self.fd, self.pd])

        self.PositionX, self.PositionY, self.fd, self.pd, flag = self._sim_flight2(self.PositionX, self.PositionY, self.fd, self.pd, visualizer)
        
        return flag


    def _sim_flight2(self, posx, posy, fd, pd, visualizer):
        """
        事後確率に応じたコウモリの飛行経路を生成する関数
        ベイズ推論による事後確率分布を利用して障害物を検知し、回避行動を行う
        障害物がないところへと移動するため、指向性範囲内で一番確率が低い場所に移動する。

        Args:
            posx (float): コウモリのx座標
            posy (float): コウモリのy座標
            fd (float): コウモリの飛行方向（度数法）
            pd (float): コウモリのパルス発射方向（度数法）
        
        Returns:
            posx (float): 更新されたコウモリのx座標
            posy (float): 更新されたコウモリのy座標
            fd (float): 更新されたコウモリの飛行方向
            pd (float): 更新されたコウモリのパルス発射方向
        """
        
        # 事後分布をプロットしてmovieフォルダに保存（移動前）
        posterior_sel, X_sel, Y_sel = self._plot_posterior_distribution(
            posx = self.PositionX,
            posy = self.PositionY,
            pd = self.pd,
            fd = self.fd,
        )

        # 回避のための事後分布分析：前方左右30度を5度ごとに、1.5mまで0.1mごとに分析
        print(f"\n=== ステップ {self.step_idx}: 回避のための事後分布分析 ===")
        angle_results, avoid_angle, value, flag = self._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel)

        # 最も安全な角度に移動する
        new_fd = self.normalize_angle_deg(fd -avoid_angle)
        new_pd = self.normalize_angle_deg(pd -(avoid_angle))
        if self.step_idx >= 6:
            new_pd = self.normalize_angle_deg(fd -(avoid_angle*2))
        print(f"{fd}度から{-avoid_angle}度があって{new_fd}度へ移動")
        print(f"pd: {-fd -(avoid_angle*3/2)}度があって{new_pd}度")

        if flag == True:
            # 事後確率に応じたコウモリの飛行経路を生成する
            new_posx = posx + 0.15 * np.cos(np.deg2rad(new_fd))
            new_posy = posy + 0.15 * np.sin(np.deg2rad(new_fd))
        else:
            new_posx = posx + 0.05 * np.cos(np.deg2rad(new_fd))
            new_posy = posy + 0.05 * np.sin(np.deg2rad(new_fd))
        
        return new_posx, new_posy, new_fd, new_pd, flag

    def calculate_avoidance_command(self, current_position, step):
        """
        事後分布から移動指令を計算
        
        このメソッドは、ベイズ推論で更新された事後確率分布を解析して、
        ロボットの回避方向と移動距離を決定します。
        control_pc.pyとagent.pyで重複していたロジックを統合しました。
        
        Args:
            current_position (dict): 現在位置 {'x': float, 'y': float, 'fd': float, 'pd': float}
            step (int): ステップ番号
            
        Returns:
            tuple: (command, new_position)
                - command (dict): 移動指令 {'avoidance_direction', 'move_distance', 'pulse_direction'}
                - new_position (dict): 新しい位置 {'x', 'y', 'fd', 'pd'}
        """
        # 事後分布をプロット
        posterior_sel, X_sel, Y_sel = self._plot_posterior_distribution(
            posx=current_position['x'],
            posy=current_position['y'],
            pd=current_position['pd'],
            fd=current_position['fd']
        )
        
        # 回避角度を計算
        angle_results, avoid_angle, value, flag = \
            self._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel)
        
        print(f"  [移動指令計算] 回避角度: {avoid_angle:.1f}度, フラグ: {flag}")
        
        # 新しい方向を計算
        new_fd = self.normalize_angle_deg(current_position['fd'] - avoid_angle)
        new_pd = self.normalize_angle_deg(current_position['pd'] - avoid_angle)
        if step >= 6:
            new_pd = self.normalize_angle_deg(current_position['fd'] - (avoid_angle * 2))
        
        # 移動距離を決定
        if flag:
            move_distance = 150.0  # mm
        else:
            move_distance = 50.0  # mm
        
        # 新しい位置を計算
        move_distance_m = move_distance / 1000.0  # mm -> m
        new_x = current_position['x'] + move_distance_m * np.cos(np.deg2rad(new_fd))
        new_y = current_position['y'] + move_distance_m * np.sin(np.deg2rad(new_fd))
        
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
        
        print(f"  [移動指令計算] 完了: 回避={avoid_angle:.1f}度, 移動={move_distance:.1f}mm")
        print(f"  [移動指令計算] 新位置: ({new_x:.3f}, {new_y:.3f}), fd={new_fd:.1f}度, pd={new_pd:.1f}度")
        
        return command, new_position

    def _analyze_posterior_for_avoidance(self, X_sel, Y_sel, posterior_sel):
        """
        回避のための事後分布分析
        前方の左右30度までを5度ごとに、1.5mまでの値を0.1mごとに足して
        それぞれの角度での合計を表示する
        """
        # 角度範囲の設定（左右30度、5度ごと）
        angles = np.arange(-30, 30, 5)  # -30, -25, -20, ..., 25, 30
        distances = np.arange(0.1, 1.5, 0.1)  # 0.2, ..., 1.5
        
        # 結果を格納する辞書
        angle_results = {}
        
        # 表形式で表示（横が距離、縦が角度）
        print("事後分布値（対数確率）の表（横：距離[m]、縦：角度[度]）:")
        
        # ヘッダー行（距離）
        header = "角度[度] |"
        for distance in distances:
            header += f" {distance:5.1f} |"
        print(header)
        
        # 区切り線
        separator = "---------|"
        for _ in distances:
            separator += "-------|"
        print(separator)
        
        # 各角度の行
        for angle in angles:
            angle_rad = np.deg2rad(angle)
            angle_results[angle] = {}
            cumulative_sum = 0.0
            
            # 角度の行を開始
            row = f"{angle:7.0f} |"
            
            for distance in distances:
                # 指定角度・距離の範囲内のデータを抽出
                # 角度の計算（arctan2を使用）
                angle_mask = np.abs(np.arctan2(X_sel, Y_sel) - angle_rad) < np.deg2rad(2.5)  # ±2.5度の範囲
                dist_mask = np.abs(Y_sel - distance) < 0.05  # ±0.05mの範囲
                combined_mask = angle_mask & dist_mask
                
                if np.any(combined_mask):
                    values_at_angle_dist = posterior_sel[combined_mask]
                    avg_value = np.mean(values_at_angle_dist)
                    cumulative_sum += avg_value
                    angle_results[angle][distance] = avg_value
                    row += f" {avg_value:5.2f} |"
                else:
                    angle_results[angle][distance] = 0.0
                    row += f" {'N/A':>5} |"
            
            # 行を表示
            print(row)
            
            # 各角度での合計を保存
            angle_results[angle]['total'] = cumulative_sum
        
        # 各角度での合計を表示
        print("\n各角度での事後分布値の合計:")
        print("角度[度] | 合計値（対数確率）")
        print("---------|------------------")
        for angle in angles:
            total = angle_results[angle]['total']
            print(f"{angle:7.0f} | {total:16.2f}")
        
        # 最も安全な角度（合計値が最も低い角度）を特定
        # angles の中で一番最初にその最大値を持った a が min_angle に選ばれる。
        min_angle = min(angles, key=lambda a: angle_results[a]['total'])
        min_value = angle_results[min_angle]['total']
        # 最も危険な角度（合計値が最も高い角度）を特定
        # angles の中で一番最初にその最小値を持った a が max_angle に選ばれる。
        max_angle = max(angles, key=lambda a: angle_results[a]['total'])
        max_value = angle_results[max_angle]['total']
        print(f"\n最も危険な角度: {max_angle}度 (合計値: {max_value:.2f})")
        print(f"最も安全な角度: {min_angle}度 (合計値: {min_value:.2f})")
        print("=" * 50)
        
        
        # 新しい回避機能：既存の計算結果を活用して角度-30度から30度の範囲で距離0.5m以内に事後分布の値が-10以上のものがあれば左右で数を数えて少ない方に回避する
        print("\n=== 新しい回避機能のチェック ===")
        
        # 距離0.5m以内のデータをチェック（既存の計算結果を活用）
        check_distances = [d for d in distances if d <= 0.4]  # 0.3m以下の距離のみ
        
        dangerous_angles = []
        
        # 既存の計算結果から-100以上の値をチェック
        for angle in angles:
            for distance in check_distances:
                if distance in angle_results[angle]:
                    value = angle_results[angle][distance]
                    if value >= -50:
                        dangerous_angles.append(angle)
                        print(f"危険な角度を発見: {angle}度 (距離: {distance}m, 値: {value:.2f})")
        
        if dangerous_angles:
            # 左右で数を数える
            left_count = len([angle for angle in dangerous_angles if angle < 0])
            right_count = len([angle for angle in dangerous_angles if angle > 0])
            
            print(f"左側（負の角度）の危険な角度数: {left_count}")
            print(f"右側（正の角度）の危険な角度数: {right_count}")
            
            # 連続回避の場合は前回と同じ方向に回避する
            if self.last_avoidance_direction is not None and self.consecutive_avoidance_count > 0:
                # 前回と同じ方向に回避
                avoidance_angle = self.last_avoidance_direction
                print(f"連続回避: 前回と同じ方向({avoidance_angle}度)に回避")
                self.consecutive_avoidance_count += 1
            else:
                # 通常の回避ロジック（少ない方に回避）
                if left_count <= right_count:
                    # 左側が少ない場合、右側に回避
                    avoidance_angle = 60
                    print(f"左側が少ないため、右側（{avoidance_angle}度）に回避")
                else:
                    # 右側が少ない場合、左側に回避
                    avoidance_angle = -60
                    print(f"右側が少ないため、左側（{avoidance_angle}度）に回避")
                # 回避を開始したので、カウンターを1に設定
                self.consecutive_avoidance_count = 1
            
            # 今回の回避方向を記録
            self.last_avoidance_direction = avoidance_angle
            
            print("=" * 50)
            # 危険な角度で60度回る場合は、Falseを返して進行しないようにする。
            return angle_results, avoidance_angle, -10.0, False  # 新しい回避角度を返す
        
        print("危険な角度は見つかりませんでした。従来の回避方法を使用します。")
        print("=" * 50)
        
        # 危険な角度がない場合は連続回避カウンターをリセット
        self.consecutive_avoidance_count = 0
        
        return angle_results, min_angle, min_value, True

    def _plot_posterior_distribution(self, posx, posy, fd, pd):
        """
        コウモリの位置posx,posy、頭部方向pd（度数法）を使い、
        コウモリ前方2m範囲だけを「コウモリが下中央・上向き」でプロットし、
        その範囲の事後分布値もprintで出力する
        """
        def rotate_points(x, y, origin_x, origin_y, angle_rad):
            x_shifted = x - origin_x
            y_shifted = y - origin_y
            x_rot = x_shifted * np.cos(angle_rad) - y_shifted * np.sin(angle_rad)
            y_rot = x_shifted * np.sin(angle_rad) + y_shifted * np.cos(angle_rad)
            return x_rot, y_rot

        # movieフォルダが存在しない場合は作成
        movie_dir = config.output_dir_movie_posterior
        if not os.path.exists(movie_dir):
            os.makedirs(movie_dir)

        posterior_data = self.bayesian.Px_yn_conf_log_current

        # X, Yはmeshgrid想定
        X_flat = self.X.flatten()
        Y_flat = self.Y.flatten()
        posterior_flat = posterior_data.flatten()

        # コウモリ位置・頭部方向
        bat_x = posx
        bat_y = posy
        bat_angle_deg = pd  # 度数法
        
        # 度数法をラジアンに変換し、コウモリが上向きになるように回転
        # pdは進行方向（度数法）なので、これを上向き（90度）に合わせる
        bat_angle_rad = np.deg2rad(-bat_angle_deg+90)  # 90度引いて上向きに調整

        # コウモリ中心・頭部方向上向きに回転
        X_rot, Y_rot = rotate_points(X_flat, Y_flat, bat_x, bat_y, bat_angle_rad)

        # 前方2m範囲だけ抽出（x方向±1m, y方向0〜2m）
        mask = (Y_rot >= 0) & (Y_rot <= 2.0) & (np.abs(X_rot) <= 2.0)
        X_sel = X_rot[mask]
        Y_sel = Y_rot[mask]
        posterior_sel = posterior_flat[mask]

        # 非有限値を除外
        finite_mask = np.isfinite(posterior_sel)
        X_sel = X_sel[finite_mask]
        Y_sel = Y_sel[finite_mask]
        posterior_sel = posterior_sel[finite_mask]

        # 前方2m範囲の事後分布値をprint
        print("コウモリ前方2m範囲の事後分布値:")
        print(posterior_sel.shape)
        print(X_sel.shape)

        # プロット
        plt.figure(figsize=(8, 8))
        if len(X_sel) > 0:
            # 等高線図の描画
            contour = plt.tricontourf(X_sel, Y_sel, posterior_sel, levels=100, cmap='viridis')
            
            # カラーバーの詳細設定（3桁固定フォーマット）
            cbar = plt.colorbar(contour, ax=plt.gca(), shrink=0.8, aspect=20)
            cbar.ax.tick_params(labelsize=10)
            
            # カラーバーの目盛りを3桁固定フォーマットで設定
            tick_values = np.linspace(np.min(posterior_sel), np.max(posterior_sel), 6)
            cbar.set_ticks(tick_values)
            # 固定幅フォーマット（例: -45.2, -36.1, -27.0, -17.9, -8.8, 0.3）
            tick_labels = []
            for val in tick_values:
                if val < 0:
                    tick_labels.append(f'{val:5.1f}')  # 負の数は5文字幅
                else:
                    tick_labels.append(f' {val:4.1f}')  # 正の数は6文字幅（空白含む）
            cbar.set_ticklabels(tick_labels)
        else:
            print("プロットするデータがありません")
            
        plt.plot(0, 0, 'ro', markersize=8)  # コウモリ位置（下中央）

        plt.xlim(-3, 3)
        plt.ylim(-1, 3)
        plt.grid(True, alpha=0.3)

        # ファイル名の生成（ステップ番号を含む）
        filename = f"{movie_dir}/posterior_step_{self.step_idx:04d}.png"
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"事後分布プロットを保存しました: {filename}")

        return posterior_sel, X_sel, Y_sel
