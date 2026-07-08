import numpy as np
import math
import matplotlib.pyplot as plt
import os
import csv

from bayes_code import config
from bayes_code.calc import calc

# --- 飛行・回避チューニング定数 ---
# （物理定数ではなく回避アルゴリズム固有の値なので config.py ではなくここに置く。
#   _sim_flight2（シミュレーション経路）と calculate_avoidance_command（本番経路）の
#   両方で同じ値を使う。単位が m と mm で分かれているのは通信仕様（ロボットへは mm）のため）
STRAIGHT_STEPS = 10              # 直線移動する初期ステップ数（この間は回避なし）
STEP_DISTANCE = 0.15             # 通常時の 1 ステップ移動距離 [m]（シミュレーション経路）
EMERGENCY_STEP_DISTANCE = 0.05   # 緊急回避時の 1 ステップ移動距離 [m]（慎重に小さく進む）
STEP_DISTANCE_MM = 150.0         # 通常時の移動指令距離 [mm]（本番経路: ロボットへ送る値）
EMERGENCY_STEP_DISTANCE_MM = 50.0  # 緊急回避時の移動指令距離 [mm]
INITIAL_PULSE_OFFSET_DEG = 50.0  # 初期直進フェーズでパルスを進行方向から左にずらす角度 [deg]
PULSE_AVOID_FACTOR = 1.3         # 回避時のパルス方向オフセット倍率（fd - avoid_angle*1.3）
EMERGENCY_AVOID_ANGLE = 60       # 緊急回避時の旋回角 [deg]
DANGER_THRESHOLD = -50           # 危険とみなす事後分布値のしきい値
DANGER_DISTANCE = 0.4            # 危険判定を行う近距離の上限 [m]


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
        # 回避分析の詳細テーブルを標準出力に表示するか（デバッグ用）
        self.verbose = False
        
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
        """1 ステップ分のセンシングとベイズ更新を実行する（シミュレーション経路）。

        実機経路（control_pc.py）では Localizer が観測を生成して update_belief を
        直接呼ぶため、このメソッドは使われない。

        Returns:
            tuple: (y_x, y_y, y_el_vec, y_er_vec, belief)
                - y_x, y_y: 観測点の座標（可視化用）
                - y_el_vec, y_er_vec: 左右エコーの時間軸ベクトル（可視化用）
                - belief: BeliefSnapshot（bayesian.py 参照。旧 data1〜4 の4タプルを
                  Phase 2 で dataclass 化したもの）
        """
        # calc() は SensingResult（各値の意味は calc.py の定義を参照）を返す
        sensing = calc(world, self.PositionX, self.PositionY, self.fd, self.pd, self.X, self.Y)

        # 単位を合わせる（距離: m→mm、角度: rad→deg）
        r_noise = sensing.r_detected * 1000
        theta_noise = sensing.theta_noise * 180 / math.pi

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

        belief = self.bayesian.update_belief(
            self.step_idx,
            sensing.y_el, sensing.y_er,
            sensing.goback_dist_matrix_L, sensing.goback_dist_matrix_R,
            sensing.confidence_matrix,
        )
        return sensing.y_x, sensing.y_y, sensing.y_el_vec, sensing.y_er_vec, belief
    
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

        # 最初の STRAIGHT_STEPS ステップは直線移動（回避なし）
        print(f"\n=== ステップ {self.step_idx}: 回避のための事後分布分析 ===")
        if self.step_idx < STRAIGHT_STEPS:
            avoid_angle = 0.0
            flag = True
            print(f"ステップ{self.step_idx}: 直線移動モード（回避なし）")
        else:
            # 回避のための事後分布分析（範囲は _analyze_posterior_for_avoidance 参照）
            angle_results, avoid_angle, value, flag = self._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel)

        # 最も安全な角度に移動する
        new_fd = self.normalize_angle_deg(fd - avoid_angle)

        # パルス放射方向の計算
        if self.step_idx < STRAIGHT_STEPS:
            # 初期直進フェーズはパルスを左に INITIAL_PULSE_OFFSET_DEG 度固定
            new_pd = self.normalize_angle_deg(fd + INITIAL_PULSE_OFFSET_DEG)
            print(f"ステップ{self.step_idx}: パルス放射方向を左{INITIAL_PULSE_OFFSET_DEG:.0f}度固定 (fd={fd:.1f}° → pd={new_pd:.1f}°)")
        else:
            # ステップ STRAIGHT_STEPS 以降は通常の計算
            new_pd = self.normalize_angle_deg(pd - avoid_angle)
            # NOTE: この条件は step>=10 分岐の内側にあるため常に真（死んだ条件）。
            #       意図（step6〜9用の名残?）が不明なため挙動保存のまま温存している。
            #       詳細は REFACTORING_PLAN.md の [!question] を参照
            if self.step_idx >= 6:
                new_pd = self.normalize_angle_deg(fd - (avoid_angle * PULSE_AVOID_FACTOR))

        print(f"{fd}度から{-avoid_angle}度があって{new_fd}度へ移動")
        print(f"pd: {new_pd}度")

        if flag == True:
            # 通常移動
            new_posx = posx + STEP_DISTANCE * np.cos(np.deg2rad(new_fd))
            new_posy = posy + STEP_DISTANCE * np.sin(np.deg2rad(new_fd))
        else:
            # 緊急回避（慎重に小さく進む）
            new_posx = posx + EMERGENCY_STEP_DISTANCE * np.cos(np.deg2rad(new_fd))
            new_posy = posy + EMERGENCY_STEP_DISTANCE * np.sin(np.deg2rad(new_fd))

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

        # 最初の STRAIGHT_STEPS ステップは直線移動（回避なし）
        if step < STRAIGHT_STEPS:
            avoid_angle = 0.0
            flag = True
            print(f"  [移動指令計算] ステップ{step}: 直線移動モード（回避なし）")
        else:
            # 回避角度を計算
            angle_results, avoid_angle, value, flag = \
                self._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel)
            print(f"  [移動指令計算] 回避角度: {avoid_angle:.1f}度, フラグ: {flag}")

        # 新しい方向を計算
        new_fd = self.normalize_angle_deg(current_position['fd'] - avoid_angle)

        # パルス放射方向の計算
        if step < STRAIGHT_STEPS:
            # 初期直進フェーズはパルスを左に INITIAL_PULSE_OFFSET_DEG 度固定
            new_pd = self.normalize_angle_deg(current_position['fd'] + INITIAL_PULSE_OFFSET_DEG)
            print(f"  [移動指令計算] ステップ{step}: パルス放射方向を左{INITIAL_PULSE_OFFSET_DEG:.0f}度固定 (fd={current_position['fd']:.1f}° → pd={new_pd:.1f}°)")
        else:
            # ステップ STRAIGHT_STEPS 以降は通常の計算
            new_pd = self.normalize_angle_deg(current_position['pd'] - avoid_angle)
            # NOTE: この条件は step>=10 分岐の内側にあるため常に真（死んだ条件）。
            #       _sim_flight2 側と同じ理由で挙動保存のまま温存している
            if step >= 6:
                new_pd = self.normalize_angle_deg(current_position['fd'] - (avoid_angle * PULSE_AVOID_FACTOR))

        # 移動距離を決定（ロボットへ送る指令は mm 単位）
        if flag:
            move_distance = STEP_DISTANCE_MM
        else:
            move_distance = EMERGENCY_STEP_DISTANCE_MM
        
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
        
        # 緊急回避フラグ（flagがFalseの場合が緊急回避）
        emergency_avoidance = not flag
        
        print(f"  [移動指令計算] 完了: 回避={avoid_angle:.1f}度, 移動={move_distance:.1f}mm, 緊急回避={emergency_avoidance}")
        print(f"  [移動指令計算] 新位置: ({new_x:.3f}, {new_y:.3f}), fd={new_fd:.1f}度, pd={new_pd:.1f}度")
        
        return command, new_position, emergency_avoidance

    def _analyze_posterior_for_avoidance(self, X_sel, Y_sel, posterior_sel):
        """回避のための事後分布分析。

        前方 -30〜+25 度（5度刻み、np.arange 由来で非対称）× 距離0.05〜0.70m
        （0.05m刻み）の各方向について事後分布値の平均を集計し、最も安全な
        （値が低い）方向を選ぶ。近距離に危険（値が高い）方向がある場合は、
        左右で数を比べて少ない側へ±60度の緊急回避を行う。

        _sim_flight2（シミュレーション経路）と calculate_avoidance_command
        （本番経路）の両方から呼ばれる共通の回避判断ロジック。

        Returns:
            tuple: (angle_results, avoid_angle, value, flag)
                - angle_results: {角度: {距離: 平均値, 'total': 合計}} の集計辞書
                - avoid_angle: 選択した回避角度
                - value: その角度での評価値
                - flag: True=通常回避 / False=緊急回避（前進を抑える）
        """
        # 角度範囲（-30〜+25度、5度ごと）と距離範囲（0.05〜0.70m、0.05mごと）
        angles = np.arange(-30, 30, 5)
        distances = np.arange(0.05, 0.75, 0.05)

        # 各(角度, 距離)での事後分布値の平均を集計
        angle_results = self._aggregate_posterior_by_direction(X_sel, Y_sel, posterior_sel, angles, distances)

        if self.verbose:
            self._print_avoidance_table(angles, distances, angle_results)

        # 最も安全な角度（合計値が最も低い角度）を特定
        min_angle = min(angles, key=lambda a: angle_results[a]['total'])
        min_value = angle_results[min_angle]['total']

        # 近距離に危険方向があれば緊急回避（左右の少ない側へ±60度）
        avoidance_angle = self._decide_emergency_avoidance(angles, distances, angle_results)
        if avoidance_angle is not None:
            # 危険な角度で60度回る場合は、Falseを返して進行しないようにする
            return angle_results, avoidance_angle, -10.0, False

        # 危険な角度がない場合は連続回避カウンターをリセットして通常回避
        self.consecutive_avoidance_count = 0
        return angle_results, min_angle, min_value, True

    def _aggregate_posterior_by_direction(self, X_sel, Y_sel, posterior_sel, angles, distances):
        """各(角度, 距離)の範囲に入る事後分布値の平均を集計して辞書で返す。

        Returns:
            dict: {角度: {距離: 平均値, ..., 'total': 距離方向の合計}}
        """
        angle_results = {}
        for angle in angles:
            angle_rad = np.deg2rad(angle)
            angle_results[angle] = {}
            cumulative_sum = 0.0
            for distance in distances:
                # 指定角度・距離の範囲内（角度±2.5度・距離±0.05m）のデータを抽出
                angle_mask = np.abs(np.arctan2(X_sel, Y_sel) - angle_rad) < np.deg2rad(2.5)
                dist_mask = np.abs(Y_sel - distance) < 0.05
                combined_mask = angle_mask & dist_mask

                if np.any(combined_mask):
                    avg_value = np.mean(posterior_sel[combined_mask])
                    cumulative_sum += avg_value
                    angle_results[angle][distance] = avg_value
                else:
                    angle_results[angle][distance] = 0.0

            # 各角度での距離方向の合計を保存
            angle_results[angle]['total'] = cumulative_sum
        return angle_results

    def _decide_emergency_avoidance(self, angles, distances, angle_results):
        """近距離(DANGER_DISTANCE以内)に危険方向(値>=DANGER_THRESHOLD)があれば緊急回避角度を決める。

        危険がなければ None を返す。危険があれば左右の危険数を比べて
        少ない側へ ±EMERGENCY_AVOID_ANGLE 度回避し、self.last_avoidance_direction /
        consecutive_avoidance_count を更新する（連続回避は前回方向を維持）。
        """
        # 近距離(DANGER_DISTANCE 以内)で事後分布値が DANGER_THRESHOLD 以上の方向を「危険」とみなす
        check_distances = [d for d in distances if d <= DANGER_DISTANCE]
        dangerous_angles = []
        for angle in angles:
            for distance in check_distances:
                if distance in angle_results[angle]:
                    if angle_results[angle][distance] >= DANGER_THRESHOLD:
                        dangerous_angles.append(angle)

        if not dangerous_angles:
            return None

        # 左右で危険方向の数を数える
        left_count = len([angle for angle in dangerous_angles if angle < 0])
        right_count = len([angle for angle in dangerous_angles if angle > 0])

        # 連続回避中は前回と同じ方向を維持、そうでなければ危険の少ない側へ回避
        if self.last_avoidance_direction is not None and self.consecutive_avoidance_count > 0:
            avoidance_angle = self.last_avoidance_direction
            self.consecutive_avoidance_count += 1
        else:
            avoidance_angle = EMERGENCY_AVOID_ANGLE if left_count <= right_count else -EMERGENCY_AVOID_ANGLE
            self.consecutive_avoidance_count = 1

        # 今回の回避方向を記録
        self.last_avoidance_direction = avoidance_angle
        return avoidance_angle

    def _print_avoidance_table(self, angles, distances, angle_results):
        """回避分析の詳細テーブルを標準出力に表示する（self.verbose 時のみ）。"""
        print("事後分布値（対数確率）の表（横：距離[m]、縦：角度[度]）:")
        header = "角度[度] |"
        for distance in distances:
            header += f" {distance:5.2f} |"
        print(header)
        separator = "---------|" + "-------|" * len(distances)
        print(separator)
        for angle in angles:
            row = f"{angle:7.0f} |"
            for distance in distances:
                value = angle_results[angle][distance]
                row += f" {value:5.2f} |" if value != 0.0 else f" {'N/A':>5} |"
            print(row)

        print("\n各角度での事後分布値の合計:")
        for angle in angles:
            print(f"{angle:7.0f} | {angle_results[angle]['total']:16.2f}")

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
