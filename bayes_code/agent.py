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

        # 前回の位置を保存（可視化用）
        self.prev_x = None
        self.prev_y = None
        self.prev_head_x = None
        self.prev_head_y = None
        self.prev_fd = None
        self.prev_pd = None

        # 危険判定閾値を保存（可視化用）
        self.last_danger_threshold = None

        # エージェントの物理的な大きさを考慮した衝突判定用パラメータ
        self.agent_radius = 0.03  # エージェントの半径 [m]
        self.collision_threshold = -50  # 衝突とみなす事後分布の閾値（初期値、動的に更新される）

        self.PositionX = sim["init_pos"][0]
        self.PositionY = sim["init_pos"][1]
        self.fd = sim["init_pos"][2]
        self.pd = sim["init_pos"][3]

        self.bayesian = bayesian
        self.world = world  # 可視化で障害物情報を使用するため保存
    
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
    

    def _decide_emergency_avoidance_angle(self, dangerous_angles, candidate_angles):
        """
        緊急回避時の回避角度を決定する共通ロジック

        Args:
            dangerous_angles (list): 危険と判定された角度のリスト
            candidate_angles (list): 候補角度のリスト（危険がない場合の参考用）

        Returns:
            float: 回避角度（-60度または60度）
        """
        if dangerous_angles:
            # 左右で数を数える
            left_count = len([angle for angle in dangerous_angles if angle < 0])
            right_count = len([angle for angle in dangerous_angles if angle > 0])

            print(f"  左側（負の角度）の危険な角度数: {left_count}")
            print(f"  右側（正の角度）の危険な角度数: {right_count}")

            # 連続回避の場合は前回と同じ方向に回避する
            if self.last_avoidance_direction is not None and self.consecutive_avoidance_count > 0:
                avoid_angle = self.last_avoidance_direction
                print(f"  連続回避: 前回と同じ方向({avoid_angle}度)に回避")
                self.consecutive_avoidance_count += 1
            else:
                # 通常の回避ロジック（少ない方に回避）
                if left_count <= right_count:
                    # 左側の危険が少ない（左側が安全）→ 左側に回避
                    avoid_angle = -60
                    print(f"  左側の危険が少ない（左側が安全）ため、左側（{avoid_angle}度）に回避")
                else:
                    # 右側の危険が少ない（右側が安全）→ 右側に回避
                    avoid_angle = 60
                    print(f"  右側の危険が少ない（右側が安全）ため、右側（{avoid_angle}度）に回避")
                # 回避を開始したので、カウンターを1に設定
                self.consecutive_avoidance_count = 1

            # 今回の回避方向を記録
            self.last_avoidance_direction = avoid_angle
        else:
            # 危険な角度が見つからなかった場合も±60度で回避
            if self.last_avoidance_direction is not None and self.consecutive_avoidance_count > 0:
                avoid_angle = self.last_avoidance_direction
                print(f"  連続回避: 前回と同じ方向({avoid_angle}度)に回避")
                self.consecutive_avoidance_count += 1
            else:
                # 最も安全な候補角度の方向を参考にする
                if len(candidate_angles) > 0 and candidate_angles[0] < 0:
                    avoid_angle = -60
                    print(f"  危険判定なし。最も安全な角度が左側のため、左側（{avoid_angle}度）に回避")
                else:
                    avoid_angle = 60
                    print(f"  危険判定なし。最も安全な角度が右側のため、右側（{avoid_angle}度）に回避")
                self.consecutive_avoidance_count = 1

            self.last_avoidance_direction = avoid_angle

        return avoid_angle

    def _check_collision_at_position(self, new_x, new_y, X_sel, Y_sel, posterior_sel):
        """
        指定位置にエージェントが移動した場合の衝突チェック（回転座標系）
        エージェントの半径を考慮し、移動先を中心とした範囲内の事後分布値を確認

        重要: すべての座標は回転座標系（原点=ロボット位置、Y軸=fd方向）で指定すること

        Args:
            new_x (float): 移動後のx座標（回転座標系）
            new_y (float): 移動後のy座標（回転座標系）
            X_sel (np.ndarray): 事後分布のX座標（回転座標系）
            Y_sel (np.ndarray): 事後分布のY座標（回転座標系）
            posterior_sel (np.ndarray): 事後分布の値

        Returns:
            is_safe (bool): True=安全, False=衝突の危険あり
            max_value (float): エージェント範囲内の最大事後分布値
        """
        # エージェント範囲内のデータを抽出
        # (new_x, new_y)を中心に、agent_radius以内のすべての点をチェック
        distance_from_center = np.sqrt((X_sel - new_x)**2 + (Y_sel - new_y)**2)
        within_agent_area = distance_from_center <= self.agent_radius

        if np.any(within_agent_area):
            values_in_area = posterior_sel[within_agent_area]
            max_value = np.max(values_in_area)

            # 閾値以上の値があれば衝突の危険あり
            is_safe = max_value < self.collision_threshold
            return is_safe, max_value
        else:
            # 範囲内にデータがない場合は安全とみなす
            return True, -np.inf


    def calculate_avoidance_command(self, current_position, step):
        """
        事後分布から移動指令を計算

        このメソッドは、ベイズ推論で更新された事後確率分布を解析して、
        ロボットの回避方向と移動距離を決定します。
        control_pc.pyとagent.pyで重複していたロジックを統合しました。

        Args:
            current_position (dict): 現在位置 {'x': float, 'y': float, 'fd': float, 'pd': float,
                                               'head_x': float, 'head_y': float, 'head_z': float}
            step (int): ステップ番号

        Returns:
            tuple: (command, new_position)
                - command (dict): 移動指令 {'avoidance_direction', 'move_distance', 'pulse_direction'}
                - new_position (dict): 新しい位置 {'x', 'y', 'fd', 'pd'}
        """
        # 回避計算用: 前方2m範囲の事後分布データを取得（head位置から）
        posterior_sel, X_sel, Y_sel = self._get_forward_posterior_data(
            posx=current_position['head_x'],
            posy=current_position['head_y'],
            pd=current_position['pd'],
            fd=current_position['fd']
        )

        # 衝突判定閾値を計算（環境全体の事後分布から上位5%を使用、最大-50）
        all_values = self.bayesian.Px_yn_conf_log_current[self.bayesian.Px_yn_conf_log_current <= -21].flatten()
        if len(all_values) >= 5:
            percentile_95 = np.percentile(all_values, 95)
            danger_threshold = min(percentile_95, -50)  # 最大でも-50に制限
            if percentile_95 > -50:
                print(f"  [閾値計算] 上位5%: {percentile_95:.2f} → 上限-50を適用 (データ点数: {len(all_values)})")
            else:
                print(f"  [閾値計算] 危険判定閾値（環境全体の上位5%）: {danger_threshold:.2f} (データ点数: {len(all_values)})")
        else:
            danger_threshold = -50
            print(f"  [閾値計算] 警告: データが{len(all_values)}点のみ、固定閾値{danger_threshold}を使用")

        # 可視化用と衝突判定用に閾値を保存
        self.last_danger_threshold = danger_threshold
        self.collision_threshold = danger_threshold

        # 最初の8ステップは直線移動（回避なし）
        candidate_angles = []  # 可視化用に初期化
        if step < 8:
            avoid_angle = 0.0
            flag = True
            angle_results = None  # ステップ8未満は回避分析なし
            print(f"  [移動指令計算] ステップ{step}: 直線移動モード（回避なし）")
        else:
            # 回避角度を計算（fd基準、座標系もfd基準なので変換不要）
            angle_results, avoid_angle, value, flag, candidate_angles = \
                self._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel, danger_threshold)
            print(f"  [移動指令計算] 回避角度: {avoid_angle:.1f}度（fd基準）, フラグ: {flag}")

        # 通常時: 候補角度リストを順次チェック
        if step >= 8 and flag:
            print("  [移動指令計算] 候補角度の衝突チェック")
            selected_angle = None

            for candidate_angle in candidate_angles:
                # 回転座標系で移動後の位置を計算
                # 回転座標系では：原点(0,0)=ロボット位置, Y軸=fd方向
                # candidate_angleはfd基準なので、そのまま使用可能
                relative_angle = candidate_angle

                # 回転座標系での移動後の位置（移動距離0.07m）
                new_x_rot = 0.07 * np.sin(np.deg2rad(relative_angle))
                new_y_rot = 0.07 * np.cos(np.deg2rad(relative_angle))

                # 衝突チェック（回転座標系で実行）
                is_safe, max_posterior = self._check_collision_at_position(
                    new_x_rot, new_y_rot, X_sel, Y_sel, posterior_sel
                )

                if is_safe:
                    print(f"  角度{candidate_angle}度は安全（最大事後分布: {max_posterior:.2f}）")
                    selected_angle = candidate_angle
                    break
                else:
                    print(f"  角度{candidate_angle}度は衝突の危険（最大事後分布: {max_posterior:.2f}）")

            if selected_angle is None:
                print("  警告: すべての候補角度で衝突の危険があります。緊急回避モードに切り替えます。")
                flag = False

                # 緊急回避角度を決定（fd基準）
                fd_relative_angles = np.arange(-30, 30, 5)
                distances = np.arange(0.10, 0.75, 0.05)
                check_distances = [d for d in distances if d < 0.2]

                # 閾値以上の箇所を危険として認定（壁の外-20も含む）
                dangerous_angles = []
                for fd_angle in fd_relative_angles:
                    for distance in check_distances:
                        if distance in angle_results[fd_angle]:
                            value_temp = angle_results[fd_angle][distance]
                            if value_temp >= danger_threshold:
                                dangerous_angles.append(fd_angle)
                                break

                # 共通メソッドで回避角度を決定
                avoid_angle = self._decide_emergency_avoidance_angle(dangerous_angles, candidate_angles)
            else:
                avoid_angle = selected_angle
                # 通常移動に戻ったので連続回避カウンターをリセット
                self.consecutive_avoidance_count = 0
                self.last_avoidance_direction = None

        # 新しい方向を計算
        new_fd = self.normalize_angle_deg(current_position['fd'] - avoid_angle)

        # パルス放射方向の計算（回避後の方向 new_fd を基準にする）
        if step < 8:
            # 最初の8ステップはパルス放射方向を 0, -30, 30, 0, -30, 30, 0, -30 のパターンで変化
            pattern = [0, -30, 30, 0, -30, 30, 0, -30]
            pulse_offset = pattern[step]
            new_pd = self.normalize_angle_deg(new_fd + pulse_offset)
            print(f"  [移動指令計算] ステップ{step}: パルス放射方向を回避後の方向から{pulse_offset:+.0f}度 (new_fd={new_fd:.1f}° → pd={new_pd:.1f}°)")
        else:
            # ステップ8以降は回避後の進行方向を基準に、回避角度の半分だけ放射方向をずらす
            new_pd = self.normalize_angle_deg(new_fd + (avoid_angle * 0.5))
            print(f"  [移動指令計算] ステップ{step}: パルス放射方向を回避後の方向から {avoid_angle * 0.5:.1f}度調整 (new_fd={new_fd:.1f}°, avoid_angle={avoid_angle:.1f}° → pd={new_pd:.1f}°)")

        # 緊急回避時は飛行方向と放射方向を一致させる
        if not flag:
            new_pd = new_fd
            print("  緊急回避モード: 放射方向を飛行方向に合わせました")

        # 移動距離を決定
        if flag:
            move_distance = 70.0  # mm (通常移動: 0.07m)
        else:
            move_distance = 30.0  # mm (緊急回避: 0.03m)

        # 新しい位置を計算
        move_distance_m = move_distance / 1000.0  # mm -> m

        # body位置の計算（回転軸として扱う）
        new_x = current_position['x'] + move_distance_m * np.cos(np.deg2rad(new_fd))
        new_y = current_position['y'] + move_distance_m * np.sin(np.deg2rad(new_fd))

        # head位置も同じ移動ベクトルで計算（bodyとheadは一緒に移動する）
        new_head_x = current_position['head_x'] + move_distance_m * np.cos(np.deg2rad(new_fd))
        new_head_y = current_position['head_y'] + move_distance_m * np.sin(np.deg2rad(new_fd))

        # 移動指令
        command = {
            'avoidance_direction': float(avoid_angle),
            'move_distance': float(move_distance),
            'pulse_direction': float(new_pd)
        }

        # 新しい位置（head情報を含む）
        new_position = {
            'x': float(new_x),
            'y': float(new_y),
            'head_x': float(new_head_x),
            'head_y': float(new_head_y),
            'head_z': float(current_position.get('head_z', 0.0)),
            'fd': float(new_fd),
            'pd': float(new_pd)
        }
        
        # 緊急回避フラグ（flagがFalseの場合が緊急回避）
        emergency_avoidance = not flag

        print(f"  [移動指令計算] 完了: 回避={avoid_angle:.1f}度, 移動={move_distance:.1f}mm, 緊急回避={emergency_avoidance}")
        print(f"  [移動指令計算] 新body位置: ({new_x:.3f}, {new_y:.3f}), 新head位置: ({new_head_x:.3f}, {new_head_y:.3f})")
        print(f"  [移動指令計算] 新方向: fd={new_fd:.1f}度, pd={new_pd:.1f}度")

        # 可視化: 現在位置と次の位置をプロット（候補角度も描画）
        self._plot_posterior_global(current_position, new_position, move_distance_m, candidate_angles)

        # 現在位置を前回位置として保存（次のステップ用、head情報も保存）
        self.prev_x = current_position['x']
        self.prev_y = current_position['y']
        self.prev_head_x = current_position.get('head_x', current_position['x'])
        self.prev_head_y = current_position.get('head_y', current_position['y'])
        self.prev_fd = current_position['fd']
        self.prev_pd = current_position['pd']

        # 角度評価結果（可視化用）
        angle_evaluation = {
            'angle_results': angle_results,  # 各角度での合計値
            'selected_angle': float(avoid_angle)  # 選択された回避角度
        }

        return command, new_position, emergency_avoidance, angle_evaluation

    def _analyze_posterior_for_avoidance(self, X_sel, Y_sel, posterior_sel, danger_threshold, fd_offset=0.0):
        """
        回避のための事後分布分析（fd基準）
        fdの方向を中心に左右30度までを5度ごとに、0.55mまでの値を0.05mごとに足して
        それぞれの角度での合計を表示する

        Args:
            X_sel (np.ndarray): 事後分布のX座標（回転座標系、fd基準）
            Y_sel (np.ndarray): 事後分布のY座標（回転座標系、fd基準）
            posterior_sel (np.ndarray): 事後分布の値
            danger_threshold (float): 危険判定閾値
            fd_offset (float): 予備パラメータ（現在は使用されず、常に0.0）

        Returns:
            tuple: (angle_results, min_angle, min_value, flag, candidate_angles)
                   角度はすべてfd基準で返される
        """
        # fd基準での角度範囲（-30～+30度）
        fd_relative_angles = np.arange(-30, 30, 5)  # -30, -25, -20, ..., 25, 30
        # 座標系もfd基準なので変換は不要
        angles = fd_relative_angles + fd_offset
        distances = np.arange(0.10, 0.55, 0.05)  # 0.10, 0.15, 0.20, ..., 0.50
        
        # 結果を格納する辞書（fd基準の角度をキーにする）
        angle_results = {}

        # 表形式で表示（横が距離、縦が角度）
        print("事後分布値（対数確率）の表（横：距離[m]、縦：角度[度、fd基準]）:")

        # ヘッダー行（距離）
        header = "fd角度[°]|"
        for distance in distances:
            header += f" {distance:5.1f} |"
        print(header)

        # 区切り線
        separator = "---------|"
        for _ in distances:
            separator += "-------|"
        print(separator)

        # 各角度の行
        for fd_angle, angle in zip(fd_relative_angles, angles):
            angle_rad = np.deg2rad(angle)  # fd基準の角度をラジアンに
            angle_results[fd_angle] = {}  # fd基準の角度をキーにする
            cumulative_sum = 0.0

            # 角度の行を開始（fd基準で表示）
            row = f"{fd_angle:7.0f} |"
            
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
                    angle_results[fd_angle][distance] = avg_value
                    row += f" {avg_value:5.2f} |"
                else:
                    angle_results[fd_angle][distance] = 0.0
                    row += f" {'N/A':>5} |"

            # 行を表示
            print(row)

            # 各角度での合計を保存
            angle_results[fd_angle]['total'] = cumulative_sum
        
        # 各角度での合計を表示
        print("\n各角度での事後分布値の合計（fd基準）:")
        print("fd角度[°]| 合計値（対数確率）")
        print("---------|------------------")
        for fd_angle in fd_relative_angles:
            total = angle_results[fd_angle]['total']
            print(f"{fd_angle:7.0f} | {total:16.2f}")

        # 最も安全な角度（合計値が最も低い角度）を特定
        # fd_relative_angles の中で一番最初にその最小値を持った a が min_angle に選ばれる。
        min_angle = min(fd_relative_angles, key=lambda a: angle_results[a]['total'])
        min_value = angle_results[min_angle]['total']
        # 最も危険な角度（合計値が最も高い角度）を特定
        # fd_relative_angles の中で一番最初にその最大値を持った a が max_angle に選ばれる。
        max_angle = max(fd_relative_angles, key=lambda a: angle_results[a]['total'])
        max_value = angle_results[max_angle]['total']
        print(f"\n最も危険な角度（fd基準）: {max_angle}度 (合計値: {max_value:.2f})")
        print(f"最も安全な角度（fd基準）: {min_angle}度 (合計値: {min_value:.2f})")
        print("=" * 50)
        

        # 新しい回避機能：既存の計算結果を活用して角度-30度から30度の範囲で距離0.1~0.3m未満に閾値以上のものがあれば左右で数を数えて少ない方に回避する
        print("\n=== 緊急回避判定（fd基準） ===")
        print(f"使用する危険判定閾値: {danger_threshold:.2f}")

        # 距離0.1~0.3m未満のデータをチェック（既存の計算結果を活用）
        check_distances = [d for d in distances if 0.1 <= d < 0.3]  # 0.1~0.3m未満の距離

        # 閾値以上の箇所を危険として認定（壁の外-20も含む）
        dangerous_angles = []
        for fd_angle in fd_relative_angles:
            for distance in check_distances:
                if distance in angle_results[fd_angle]:
                    value = angle_results[fd_angle][distance]
                    if value >= danger_threshold:
                        dangerous_angles.append(fd_angle)
                        print(f"危険な角度を発見（fd基準）: {fd_angle}度 (距離: {distance}m, 値: {value:.2f})")

        if dangerous_angles:
            # 通常時の候補角度リストを作成（安全な順にソート、fd基準）
            sorted_angles = sorted(fd_relative_angles, key=lambda a: angle_results[a]['total'])

            # 共通メソッドで回避角度を決定
            avoidance_angle = self._decide_emergency_avoidance_angle(dangerous_angles, sorted_angles)

            print("=" * 50)
            # 危険な角度で60度回る場合は、Falseを返して進行しないようにする。
            return angle_results, avoidance_angle, -10.0, False, []  # 緊急回避時は空リストを返す
        
        print("危険な角度は見つかりませんでした。従来の回避方法を使用します。")
        print("=" * 50)

        # 通常時の候補角度リストを作成（安全な順にソート、fd基準）
        sorted_angles = sorted(fd_relative_angles, key=lambda a: angle_results[a]['total'])

        return angle_results, min_angle, min_value, True, sorted_angles

    def _get_forward_posterior_data(self, posx, posy, fd, pd):
        """
        回避計算用: ロボット前方2m範囲の事後分布データを取得

        ロボットの位置と頭部方向fdを使い、前方2m範囲の事後分布データを
        ロボット中心の回転座標系で取得する（可視化はしない）

        Args:
            posx (float): ロボットのx座標
            posy (float): ロボットのy座標
            fd (float): 飛行方向（度数法）
            pd (float): パルス放射方向（度数法）

        Returns:
            tuple: (posterior_sel, X_sel, Y_sel)
                - posterior_sel: 前方2m範囲の事後分布値
                - X_sel: 回転座標系でのx座標
                - Y_sel: 回転座標系でのy座標
        """
        def rotate_points(x, y, origin_x, origin_y, angle_rad):
            x_shifted = x - origin_x
            y_shifted = y - origin_y
            x_rot = x_shifted * np.cos(angle_rad) - y_shifted * np.sin(angle_rad)
            y_rot = x_shifted * np.sin(angle_rad) + y_shifted * np.cos(angle_rad)
            return x_rot, y_rot

        posterior_data = self.bayesian.Px_yn_conf_log_current

        # X, Yはmeshgrid想定
        X_flat = self.X.flatten()
        Y_flat = self.Y.flatten()
        posterior_flat = posterior_data.flatten()

        # ロボット位置・頭部方向
        bat_x = posx
        bat_y = posy
        bat_angle_deg = fd  # 度数法（頭部方向を使用）

        # 度数法をラジアンに変換し、ロボットが上向きになるように回転
        bat_angle_rad = np.deg2rad(-bat_angle_deg+90)  # 90度引いて上向きに調整

        # ロボット中心・頭部方向上向きに回転
        X_rot, Y_rot = rotate_points(X_flat, Y_flat, bat_x, bat_y, bat_angle_rad)

        # 前方2m範囲だけ抽出（x方向±2m, y方向0〜2m）
        mask = (Y_rot >= 0) & (Y_rot <= 2.0) & (np.abs(X_rot) <= 2.0)
        X_sel = X_rot[mask]
        Y_sel = Y_rot[mask]
        posterior_sel = posterior_flat[mask]

        # 非有限値を除外
        finite_mask = np.isfinite(posterior_sel)
        X_sel = X_sel[finite_mask]
        Y_sel = Y_sel[finite_mask]
        posterior_sel = posterior_sel[finite_mask]

        return posterior_sel, X_sel, Y_sel

    def _plot_posterior_global(self, current_pos, next_pos, move_distance_m, candidate_angles=[]):
        """
        事後分布を全体表示（回転なし、ワールド座標系）で可視化

        前回位置→現在位置→次の位置（回避角度分回転+移動距離分直進）を
        頭部方向（矢印）と放射方向（線）付きで表示する
        また、全候補角度のロボット位置を円で描画（デバッグ用）

        Args:
            current_pos (dict): 現在位置 {'x', 'y', 'fd', 'pd', 'head_x', 'head_y'}
            next_pos (dict): 次の位置（回避後） {'x', 'y', 'fd', 'pd', 'head_x', 'head_y'}
            move_distance_m (float): 実際の移動距離 [m]
            candidate_angles (list): 候補角度リスト（fd基準、度数法）
        """
        # ロボットの半径を計算（bodyとheadの距離）
        robot_radius = np.sqrt((current_pos['x'] - current_pos['head_x'])**2 +
                              (current_pos['y'] - current_pos['head_y'])**2)
        print(f"  [Visualization] ロボット半径（body-head距離）: {robot_radius:.3f}m")

        # 事後分布データ取得
        posterior_data = self.bayesian.Px_yn_conf_log_current

        # 出力ディレクトリの作成
        movie_dir = config.output_dir_movie_posterior
        os.makedirs(movie_dir, exist_ok=True)

        # プロット作成
        plt.figure(figsize=(12, 12))

        # 事後分布の等高線図（全体、回転なし）
        contour = plt.contourf(self.X, self.Y, posterior_data, levels=100, cmap='viridis')
        cbar = plt.colorbar(contour, shrink=0.8, aspect=20)
        cbar.set_label('Log Posterior Probability', fontsize=12)

        # 障害物をプロット
        if self.world is not None and self.world.pole_x is not None and len(self.world.pole_x) > 0:
            plt.scatter(self.world.pole_x, self.world.pole_y,
                       c='red', marker='x', s=100, linewidths=3,
                       label='Obstacles', zorder=5)

        # 特定の角度（0度、-10度、30度）での衝突チェック範囲を円で描画
        # test_angles = [0, -10, 30]  # fd基準の角度
        # from matplotlib.patches import Circle
        # for i, test_angle in enumerate(test_angles):
        #     # fd基準の角度から移動先を計算（ワールド座標系）
        #     new_fd_test = self.normalize_angle_deg(current_pos['fd'] - test_angle)
        #     test_x = current_pos['x'] + move_distance_m * np.cos(np.deg2rad(new_fd_test))
        #     test_y = current_pos['y'] + move_distance_m * np.sin(np.deg2rad(new_fd_test))
        #
        #     # 衝突チェック範囲を青色の円で描画（body中心、robot_radius使用）
        #     circle = Circle((test_x, test_y), robot_radius,
        #                    fill=False, edgecolor='blue', linewidth=2,
        #                    linestyle='-', alpha=0.8, zorder=6)
        #     plt.gca().add_patch(circle)
        #
        #     # ラベルを追加（最初の円のみ）
        #     if i == 0:
        #         plt.plot([], [], color='blue', linestyle='-', linewidth=2,
        #                 label=f'Collision Check Area (r={robot_radius:.3f}m)')
        #
        # print(f"  [Visualization] {len(test_angles)} collision check circles plotted at angles: {test_angles}")

        # 危険領域をグレーの散布図でプロット
        if self.last_danger_threshold is not None:
            # 事後分布全体から閾値以上の場所を抽出
            danger_mask = posterior_data >= self.last_danger_threshold
            X_danger = self.X[danger_mask]
            Y_danger = self.Y[danger_mask]

            if len(X_danger) > 0:
                plt.scatter(X_danger, Y_danger,
                           c='gray', marker='.', s=10, alpha=0.5,
                           label=f'Danger Zone (threshold≥{self.last_danger_threshold:.1f})', zorder=4)
                print(f"  [Visualization] Danger zone: {len(X_danger)} points plotted (threshold: {self.last_danger_threshold:.2f})")

        # 矢印と線の長さ
        arrow_len = 0.4  # 頭部方向（矢印）の長さ
        line_len = 0.35  # 放射方向（線のみ）の長さ

        # 前回位置（もしあれば）- 薄めの水色
        if self.prev_x is not None:
            color_prev = 'lightblue'

            # body中心の円を描画
            circle_prev = Circle((self.prev_x, self.prev_y), robot_radius,
                                fill=False, edgecolor=color_prev, linewidth=2,
                                linestyle='-', alpha=0.6, zorder=3,
                                label='Previous Position')
            plt.gca().add_patch(circle_prev)

            # 前回の頭部方向（fd）：矢印
            dx_fd = arrow_len * np.cos(np.deg2rad(self.prev_fd))
            dy_fd = arrow_len * np.sin(np.deg2rad(self.prev_fd))
            plt.arrow(self.prev_x, self.prev_y, dx_fd, dy_fd,
                     head_width=0.15, head_length=0.1,
                     fc=color_prev, ec=color_prev, linewidth=2.5,
                     label='Previous Head Direction', zorder=4)

            # 前回の放射方向（pd）：線のみ
            dx_pd = line_len * np.cos(np.deg2rad(self.prev_pd))
            dy_pd = line_len * np.sin(np.deg2rad(self.prev_pd))
            plt.plot([self.prev_x, self.prev_x + dx_pd],
                    [self.prev_y, self.prev_y + dy_pd],
                    color=color_prev, linewidth=2.5, linestyle='-',
                    label='Previous Radiation Direction', zorder=4)

        # 現在位置 - 赤色
        color_current = 'red'

        # body中心の円を描画
        circle_current = Circle((current_pos['x'], current_pos['y']), robot_radius,
                               fill=False, edgecolor=color_current, linewidth=2,
                               linestyle='-', alpha=0.6, zorder=3,
                               label='Current Position')
        plt.gca().add_patch(circle_current)

        # 現在の頭部方向（fd）：矢印
        dx_fd = arrow_len * np.cos(np.deg2rad(current_pos['fd']))
        dy_fd = arrow_len * np.sin(np.deg2rad(current_pos['fd']))
        plt.arrow(current_pos['x'], current_pos['y'], dx_fd, dy_fd,
                 head_width=0.15, head_length=0.1,
                 fc=color_current, ec=color_current, linewidth=2.5,
                 label='Current Head Direction', zorder=4)

        # 現在の放射方向（pd）：線のみ
        dx_pd = line_len * np.cos(np.deg2rad(current_pos['pd']))
        dy_pd = line_len * np.sin(np.deg2rad(current_pos['pd']))
        plt.plot([current_pos['x'], current_pos['x'] + dx_pd],
                [current_pos['y'], current_pos['y'] + dy_pd],
                color=color_current, linewidth=2.5, linestyle='-',
                label='Current Radiation Direction', zorder=4)

        # 次の位置（回避角度分回転+移動距離分直進）- 薄めの緑色
        color_next = 'lightgreen'

        # body中心の円を描画
        circle_next = Circle((next_pos['x'], next_pos['y']), robot_radius,
                            fill=False, edgecolor=color_next, linewidth=2,
                            linestyle='-', alpha=0.6, zorder=3,
                            label='Next Position (After Avoidance)')
        plt.gca().add_patch(circle_next)

        # 次の頭部方向（fd）：矢印
        dx_fd = arrow_len * np.cos(np.deg2rad(next_pos['fd']))
        dy_fd = arrow_len * np.sin(np.deg2rad(next_pos['fd']))
        plt.arrow(next_pos['x'], next_pos['y'], dx_fd, dy_fd,
                 head_width=0.15, head_length=0.1,
                 fc=color_next, ec=color_next, linewidth=2.5,
                 label='Next Head Direction', zorder=4)

        # 次の放射方向（pd）：線のみ
        dx_pd = line_len * np.cos(np.deg2rad(next_pos['pd']))
        dy_pd = line_len * np.sin(np.deg2rad(next_pos['pd']))
        plt.plot([next_pos['x'], next_pos['x'] + dx_pd],
                [next_pos['y'], next_pos['y'] + dy_pd],
                color=color_next, linewidth=2.5, linestyle='-',
                label='Next Radiation Direction', zorder=4)

        # 移動経路を線で結ぶ
        if self.prev_x is not None:
            # 前回→現在→次
            plt.plot([self.prev_x, current_pos['x'], next_pos['x']],
                    [self.prev_y, current_pos['y'], next_pos['y']],
                    'k--', alpha=0.6, linewidth=2.5, label='Movement Path', zorder=3)
        else:
            # 現在→次
            plt.plot([current_pos['x'], next_pos['x']],
                    [current_pos['y'], next_pos['y']],
                    'k--', alpha=0.6, linewidth=2.5, label='Movement Path', zorder=3)

        # グラフの設定
        # plt.legend(loc='upper right', fontsize=11, framealpha=0.9)
        plt.grid(True, alpha=0.3)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.xlabel('X [m]', fontsize=12)
        plt.ylabel('Y [m]', fontsize=12)
        plt.title(f'Step {self.step_idx}: Posterior Distribution and Movement Plan (Global View)', fontsize=14, fontweight='bold')

        # 保存
        filename = f"{movie_dir}/posterior_step_{self.step_idx:04d}.png"
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"Posterior distribution plot (global view) saved: {filename}")
