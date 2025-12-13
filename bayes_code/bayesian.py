import numpy as np
import copy
import csv
import os

# 設定ファイルから必要なパラメータをインポート
from bayes_code.config import x_max, y_max, margin_space, h, world_wall_pos
from bayes_code import config

class Bayesian:
    def __init__(self, sigma2, min_p, c):
        self.sigma2 = sigma2
        self.min_p = min_p
        self.c = c
        ## ベイズ更新用
        # 初期事前確率分布
        self.Px = None
        # ベイズ更新用の状態変数（基本的な事前確率分布）
        self.Px2L_log = None
        self.Px2R_log = None
        # confidence_matrix
        self.confidence = None
        # ベイズ更新用の状態変数（記憶保持モデルの事前確率分布）
        self.Px3L_log = None
        self.Px3R_log = None
        # 現在のステップ用の配列（過去のデータは保存しない）
        self.Pyn_x_L_current = None
        self.Pyn_x_R_current = None
        self.Px_yn_log_current = None
        self.Px_ynL_log_current = None
        self.Px_ynR_log_current = None
        self.Pyn_x_conf_L_log_current = None
        self.Pyn_x_conf_R_log_current = None
        self.Px_ynL_conf_log_current = None
        self.Px_ynR_conf_log_current = None
        self.Px_yn_conf_log_current = None
        # 認知収束度合いの履歴を保存する配列
        self.convergence_history = []
        # 認知収束度合いのCSVファイルパス
        self.convergence_csv_path = None
        self.convergence_step = 0

    def Init(self, world, agent):

        # 初期事前確率分布
        print("事前確率分布を初期化中...")
        self.Px = np.ones((world.Mx + 1, world.My + 1))
        
        # 初期化方法は一旦事前確率分布と同じ
        self.confidence = self.dB_trans(self.Px)

        # ベイズ更新用の初期化
        self.Px2L_log = self.dB_trans(self.Px)
        self.Px2R_log = self.dB_trans(self.Px)
        self.Px3L_log = self.dB_trans(self.Px)
        self.Px3R_log = self.dB_trans(self.Px)
        
        # 現在のステップ用の配列の初期化（過去のデータは保存しない）
        self.Pyn_x_L_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Pyn_x_R_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Px_yn_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Px_ynL_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Px_ynR_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Pyn_x_conf_L_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Pyn_x_conf_R_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Px_ynL_conf_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Px_ynR_conf_log_current = np.zeros((world.Mx + 1, world.My + 1))
        self.Px_yn_conf_log_current = np.zeros((world.Mx + 1, world.My + 1))
    
    def dB_trans(self, data):
        """
        データをデシベル（dB）スケールに変換する関数
        
        リニアスケールの値を対数スケール（10を底とした対数）に変換します。
        単位をデシベルに変換する場合は、定義により10を乗じる必要があります（パワーの場合）。
        この関数は簡易的な対数変換のみを行います。
        
        Args:
            data (ndarray or float): デシベルスケールに変換する入力データ
            
        Returns:
            ndarray or float: 対数変換されたデータ
        """
        # 対数スケールに変換（10を底とした対数）
        # デシベル値にするには、この結果に10を掛ける必要がある（例: 10 * np.log10(data)）
        return np.log10(data)
        
    def calculate_convergence(self):
        """
        事後確率分布から認知収束度合いを計算する（単純に合計値を返す）

        Returns:
            float: 認知収束度合いの値（単純な合計値）
        """
        # 壁の座標を設定
        wall_x = np.array([margin_space, x_max - margin_space])
        wall_y = np.array([margin_space, y_max - margin_space])
        wall_corner_x, wall_corner_y = np.meshgrid(wall_x, wall_y)
        wall_corner_x = wall_corner_x.flatten()
        wall_corner_y = wall_corner_y.flatten()

        # wall_cornerの範囲を取得
        min_x = np.min(wall_corner_x)
        max_x = np.max(wall_corner_x)
        min_y = np.min(wall_corner_y)
        max_y = np.max(wall_corner_y)

        # 座標とインデックスの対応: h = 0.01なので、座標値 / h = インデックス
        min_idx_x = int(min_x / h)
        max_idx_x = int(max_x / h)
        min_idx_y = int(min_y / h)
        max_idx_y = int(max_y / h)

        # インデックスの範囲チェック
        if min_idx_x < 0 or min_idx_y < 0 or max_idx_x >= self.Px_yn_conf_log_current.shape[0] or max_idx_y >= self.Px_yn_conf_log_current.shape[1]:
            # インデックスを配列の範囲内に制限
            min_idx_x = max(0, min_idx_x)
            min_idx_y = max(0, min_idx_y)
            max_idx_x = min(self.Px_yn_conf_log_current.shape[0] - 1, max_idx_x)
            max_idx_y = min(self.Px_yn_conf_log_current.shape[1] - 1, max_idx_y)

        try:
            # 壁の内側の領域を抽出
            inner_posterior = self.Px_yn_conf_log_current[min_idx_x:max_idx_x+1, min_idx_y:max_idx_y+1]

            # NaNを含む場合は0で置き換える
            if np.isnan(inner_posterior).any():
                inner_posterior = np.nan_to_num(inner_posterior, nan=0)

            # 単純に合計値を返す
            convergence = np.sum(inner_posterior)

            if inner_posterior.size == 0:
                convergence = -1000  # デフォルト値を設定

            return convergence
        except Exception as e:
            return -1000  # エラー時のデフォルト値

    def init_convergence_csv(self, folder_name, pattern):
        """
        認知収束度合いのCSVファイルを初期化する

        Args:
            folder_name: 出力先のフォルダ名
            pattern: パターン名（ファイル名に使用）
        """
        # 出力ディレクトリの作成
        csv_output_dir = os.path.join(folder_name, "output", pattern)
        os.makedirs(csv_output_dir, exist_ok=True)

        # CSVファイルパスの設定
        self.convergence_csv_path = os.path.join(csv_output_dir, f"cognitive_convergence_{pattern}.csv")

        # CSVファイルの作成とヘッダーの書き込み
        try:
            with open(self.convergence_csv_path, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Step', 'Convergence_Value'])
            print(f"認知収束度合いCSVファイル初期化完了")
            self.convergence_step = 0
            return True
        except Exception as e:
            print(f"CSVファイル初期化に失敗しました")
            self.convergence_csv_path = None
            return False

    def new_likelyhood_2D(self, tau_n, d, sigma2):
        """
        2次元空間上の尗度（尤度、likelihood）計算関数
        
        エコーの到達時間と空間上の各点までの距離に基づいて、ベイズ更新に必要な
        尗度行列を計算します。正規分布に基づき、観測されたエコー時間が各格子点の距離から
        予測される到達時間と一致する確率を計算します。
        
        Args:
            tau_n (ndarray): 観測されたエコーの到達時間の配列
            d (ndarray): 空間全体の各点までの距離の2次元配列
            sigma2 (float): 正規分布の分散パラメータ（ノイズの強さに相当）
            
        Returns:
            ndarray: 各格子点と各エコー時間に対する尗度の3次元配列
        """
        # 到達時間と距離の格子データを作成
        Tau_n, D = np.meshgrid(tau_n, d)  # 2次元的な格子を生成
        
        # 3次元の形状に変形して、各格子点で各エコー時間を考慮できるようにする
        Tau_n = np.reshape(Tau_n, (d.shape[0], d.shape[1], tau_n.shape[0]))  # 時間格子
        D = np.reshape(D, (d.shape[0], d.shape[1], tau_n.shape[0]))  # 距離格子

        # 正規分布に基づく尗度計算
        # 計算式: (1/√(2πσ2)) * e^(-((cτ - d)^2)/(2σ2))
        # cτは予測される距離、dは実際の距離、σ2は分散パラメータ
        Pyn_2Dxy_each = np.nan_to_num(1 / np.sqrt(2 * np.pi * sigma2) * np.exp(-((self.c * Tau_n - D) ** 2) / (2 * sigma2)))
        
        # 代替実装のコメントアウト例
        #    Pyn_2Dxy_each = np.nan_to_num(1/np.sqrt(2*np.pi*sigma2)*np.exp(-(Tau_n-D/c)**2/(2*sigma2)))
        #    Pyn_2Dxy_sum = np.sum(Pyn_2Dxy_each, axis = 2) #使わない
        
        return Pyn_2Dxy_each


    def update_belief(self, step_idx, y_el, y_er, current_obs_goback_dist_matrix_L, current_obs_goback_dist_matrix_R, current_confidence_matrix):
        ## ベイズ更新の実行

        # 観測の有無をチェック
        has_observation_L = len(y_el[0][~np.isnan(y_el[0])]) > 0
        has_observation_R = len(y_er[0][~np.isnan(y_er[0])]) > 0

        # 左耳の尤度計算
        if has_observation_L:
            # 通常の尤度計算
            Pyn_x_L_each = self.new_likelyhood_2D(
                y_el[0][~np.isnan(y_el[0])],
                current_obs_goback_dist_matrix_L[0],
                self.sigma2
            ).transpose(2, 0, 1)
            self.Pyn_x_L_current = np.clip(np.sum(Pyn_x_L_each, axis=0), 0.1**20, None)
        else:
            # 観測なし時の処理
            if config.use_negative_evidence and not has_observation_R:
                # シナリオ3: 両方観測なし → Negative Evidenceを適用
                observable_mask = current_confidence_matrix[0] > config.confidence_threshold
                self.Pyn_x_L_current = np.where(
                    observable_mask,
                    config.negative_evidence_value,  # observable領域: 低い値
                    config.unobservable_value         # unobservable領域
                )
                print(f"  左耳: Negative Evidence適用（両方観測なし）")
            else:
                # シナリオ2: 片方だけ観測あり、または従来方式 → 全域1.0で更新しない
                self.Pyn_x_L_current = np.ones_like(current_confidence_matrix[0])
                if has_observation_R:
                    print(f"  左耳: 観測なし（右耳のみで更新）")

        # 右耳の尤度計算
        if has_observation_R:
            # 通常の尤度計算
            Pyn_x_R_each = self.new_likelyhood_2D(
                y_er[0][~np.isnan(y_er[0])],
                current_obs_goback_dist_matrix_R[0],
                self.sigma2
            ).transpose(2, 0, 1)
            self.Pyn_x_R_current = np.clip(np.sum(Pyn_x_R_each, axis=0), 0.1**20, None)
        else:
            # 観測なし時の処理
            if config.use_negative_evidence and not has_observation_L:
                # シナリオ3: 両方観測なし → Negative Evidenceを適用
                observable_mask = current_confidence_matrix[0] > config.confidence_threshold
                self.Pyn_x_R_current = np.where(
                    observable_mask,
                    config.negative_evidence_value,  # observable領域: 低い値
                    config.unobservable_value         # unobservable領域
                )
                print(f"  右耳: Negative Evidence適用（両方観測なし）")
            else:
                # シナリオ2: 片方だけ観測あり、または従来方式 → 全域1.0で更新しない
                self.Pyn_x_R_current = np.ones_like(current_confidence_matrix[0])
                if has_observation_L:
                    print(f"  右耳: 観測なし（左耳のみで更新）")

        # 同時確率分布の計算
        PxynL_log = self.dB_trans(self.Pyn_x_L_current / np.max(self.Pyn_x_L_current)) + self.Px2L_log
        PxynR_log = self.dB_trans(self.Pyn_x_R_current / np.max(self.Pyn_x_R_current)) + self.Px2R_log
        
        PxynL_log = np.clip(PxynL_log, self.min_p, None)
        PxynR_log = np.clip(PxynR_log, self.min_p, None)
        
        PxynL_temp = np.where(PxynL_log > self.min_p, 10 ** (PxynL_log), 0)
        PxynR_temp = np.where(PxynR_log > self.min_p, 10 ** (PxynR_log), 0)
        
        PxynL_log_sum = np.log10(np.sum(PxynL_temp))
        PxynR_log_sum = np.log10(np.sum(PxynR_temp))
        
        # 事後確率の計算
        self.Px_ynL_log_current = PxynL_log - PxynL_log_sum
        self.Px_ynR_log_current = PxynR_log - PxynR_log_sum
        
        # 次ステップの事前確率として使用
        self.Px2L_log = copy.deepcopy(self.Px_ynL_log_current)
        self.Px2R_log = copy.deepcopy(self.Px_ynR_log_current)
        
        # 左右の耳の事後確率を統合
        self.Px_yn_log_current = self.Px_ynL_log_current + self.Px_ynR_log_current
        
        # 記憶保持メカニズムによる確率更新
        PxynL_conf_log = (
            current_confidence_matrix[0] * self.dB_trans(self.Pyn_x_L_current / np.max(self.Pyn_x_L_current)) + self.Px3L_log
        )
        PxynR_conf_log = (
            current_confidence_matrix[0] * self.dB_trans(self.Pyn_x_R_current / np.max(self.Pyn_x_R_current)) + self.Px3R_log
        )
        
        PxynL_conf_log = np.clip(PxynL_conf_log, self.min_p, None)
        PxynR_conf_log = np.clip(PxynR_conf_log, self.min_p, None)
        PxynL_conf_temp = np.where(PxynL_conf_log > self.min_p, 10 ** (PxynL_conf_log), 0)
        PxynR_conf_temp = np.where(PxynR_conf_log > self.min_p, 10 ** (PxynR_conf_log), 0)
        
        PxynL_conf_log_sum = np.log10(np.sum(PxynL_conf_temp))
        PxynR_conf_log_sum = np.log10(np.sum(PxynR_conf_temp))
        
        self.Px_ynL_conf_log_current = PxynL_conf_log - PxynL_conf_log_sum
        self.Px_ynR_conf_log_current = PxynR_conf_log - PxynR_conf_log_sum
        
        self.Px3L_log = copy.deepcopy(self.Px_ynL_conf_log_current)
        self.Px3R_log = copy.deepcopy(self.Px_ynR_conf_log_current)
        
        self.Px_yn_conf_log_current = self.Px_ynL_conf_log_current + self.Px_ynR_conf_log_current

        # world_wall_posはconfig.pyから読み込まれます
        if world_wall_pos:
            # 壁の範囲外を-20に設定
            wall_x = np.array([margin_space, x_max - margin_space])
            wall_y = np.array([margin_space, y_max - margin_space])
            wall_corner_x, wall_corner_y = np.meshgrid(wall_x, wall_y)
            wall_corner_x = wall_corner_x.flatten()
            wall_corner_y = wall_corner_y.flatten()
            
            # wall_cornerの範囲を取得
            min_x = np.min(wall_corner_x)  # 2.0
            max_x = np.max(wall_corner_x)  # 6.5
            min_y = np.min(wall_corner_y)  # 2.0
            max_y = np.max(wall_corner_y)  # 6.5
            
            # 座標とインデックスの対応: h = 0.01なので、座標値 / h = インデックス
            min_idx = int(min_x / h)  # 200
            max_idx = int(max_x / h)  # 650
            
            # 壁の範囲外を-20に設定
            # 1. x < 2.0 または x > 6.5 の領域
            self.Px_yn_conf_log_current[:min_idx, :] = -20  # x < 2.0
            self.Px_yn_conf_log_current[max_idx+1:, :] = -20  # x > 6.5
            
            # 2. y < 2.0 または y > 6.5 の領域
            self.Px_yn_conf_log_current[:, :min_idx] = -20  # y < 2.0
            self.Px_yn_conf_log_current[:, max_idx+1:] = -20  # y > 6.5

        # 認知収束度合いを計算してCSVに保存
        convergence_value = self.calculate_convergence()
        self.convergence_history.append(convergence_value)

        if self.convergence_csv_path is not None:
            try:
                with open(self.convergence_csv_path, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow([step_idx, convergence_value])
                self.convergence_step += 1
            except Exception as e:
                pass  # エラーは無視

        data1 = self.Pyn_x_L_current
        data2 = current_confidence_matrix[0]
        data3 = self.Px_yn_log_current
        data4 = self.Px_yn_conf_log_current
        data5 = self.Pyn_x_R_current
        return data1, data2, data3, data4, data5

    def save_state(self, file_path, step_idx):
        """
        ベイズ推定の状態を保存（途中再開用）

        保存するデータ:
        - Px2L_log, Px2R_log: 基本モデルの事後確率分布（対数スケール）
        - Px3L_log, Px3R_log: 記憶保持モデルの事後確率分布（対数スケール）
        - confidence: confidence行列
        - convergence_history: 認知収束度合いの履歴
        - step_idx: 現在のステップ番号

        理論的背景:
        ベイズ推定では、事後確率が次のステップの事前確率になります。
        これらの状態変数には過去の全観測データの情報が凝縮されているため、
        これらを保存すれば途中から再開できます（マルコフ性）。

        Args:
            file_path (str): 保存先のファイルパス（.npz形式を推奨）
            step_idx (int): 現在のステップ番号
        """
        # ディレクトリが存在しない場合は作成
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        # 圧縮形式でnumpy配列を保存
        np.savez_compressed(
            file_path,
            Px2L_log=self.Px2L_log,
            Px2R_log=self.Px2R_log,
            Px3L_log=self.Px3L_log,
            Px3R_log=self.Px3R_log,
            confidence=self.confidence,
            convergence_history=np.array(self.convergence_history),
            step_idx=step_idx
        )
        print(f"チェックポイント保存完了: ステップ {step_idx}")

    def load_state(self, file_path):
        """
        ベイズ推定の状態を読み込み（途中再開用）

        保存されたチェックポイントから以下のデータを復元:
        - Px2L_log, Px2R_log: 基本モデルの事後確率分布
        - Px3L_log, Px3R_log: 記憶保持モデルの事後確率分布
        - confidence: confidence行列
        - convergence_history: 認知収束度合いの履歴

        理論的背景:
        ステップnの事後分布P(x|y₁,...,yₙ)には、過去の全観測データの情報が
        内包されています。したがって、この分布さえあればステップn+1以降の
        計算を継続できます。これがベイズ推定における十分統計量の性質です。

        Args:
            file_path (str): 読み込むチェックポイントファイルのパス

        Returns:
            int: 保存時のステップ番号（次のステップはstep_idx+1から開始）

        Raises:
            FileNotFoundError: チェックポイントファイルが存在しない場合
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"チェックポイントファイルが見つかりません: {file_path}")

        print(f"チェックポイント読み込み中...")

        # numpy形式のファイルを読み込み
        data = np.load(file_path)

        # 状態変数を復元
        self.Px2L_log = data['Px2L_log']
        self.Px2R_log = data['Px2R_log']
        self.Px3L_log = data['Px3L_log']
        self.Px3R_log = data['Px3R_log']
        self.confidence = data['confidence']
        self.convergence_history = list(data['convergence_history'])
        step_idx = int(data['step_idx'])

        print(f"チェックポイント復元完了: ステップ {step_idx} → 次のステップ {step_idx + 1} から実行を再開します")

        return step_idx