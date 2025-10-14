import numpy as np
import copy

# 設定ファイルから必要なパラメータをインポート
from config import y_max, margin_space, h, world_wall_pos

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

    def Init(self, world, agent):

        # 初期事前確率分布
        print("prior initialization...")
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
        # デバッグ情報: Px_yn_conf_log_currentの状態を確認
        print("\n=== calculate_convergence デバッグ情報 ===")
        print(f"Px_yn_conf_log_current shape: {self.Px_yn_conf_log_current.shape}")
        print(f"Px_yn_conf_log_current contains NaN: {np.isnan(self.Px_yn_conf_log_current).any()}")
        if np.isnan(self.Px_yn_conf_log_current).any():
            nan_count = np.isnan(self.Px_yn_conf_log_current).sum()
            print(f"NaN count in Px_yn_conf_log_current: {nan_count}")
        
        # 壁の座標を設定
        wall_x = np.array([margin_space, y_max - margin_space])
        wall_y = np.array([margin_space, y_max - margin_space])
        wall_corner_x, wall_corner_y = np.meshgrid(wall_x, wall_y)
        wall_corner_x = wall_corner_x.flatten()
        wall_corner_y = wall_corner_y.flatten()

        # wall_cornerの範囲を取得
        min_x = np.min(wall_corner_x)
        max_x = np.max(wall_corner_x)
        min_y = np.min(wall_corner_y)
        max_y = np.max(wall_corner_y)
        print(f"min_x: {min_x}, max_x: {max_x}, min_y: {min_y}, max_y: {max_y}")

        # 座標とインデックスの対応: h = 0.01なので、座標値 / h = インデックス
        min_idx_x = int(min_x / h)
        max_idx_x = int(max_x / h)
        min_idx_y = int(min_y / h)
        max_idx_y = int(max_y / h)
        print(f"min_idx_x: {min_idx_x}, max_idx_x: {max_idx_x}, min_idx_y: {min_idx_y}, max_idx_y: {max_idx_y}")

        # nanで足し算できないそうなので0に置き換える
        # インデックスの範囲チェック
        if min_idx_x < 0 or min_idx_y < 0 or max_idx_x >= self.Px_yn_conf_log_current.shape[0] or max_idx_y >= self.Px_yn_conf_log_current.shape[1]:
            print("警告: インデックスが配列の範囲外です")
            print(f"配列の形状: {self.Px_yn_conf_log_current.shape}")
            # インデックスを配列の範囲内に制限
            min_idx_x = max(0, min_idx_x)
            min_idx_y = max(0, min_idx_y)
            max_idx_x = min(self.Px_yn_conf_log_current.shape[0] - 1, max_idx_x)
            max_idx_y = min(self.Px_yn_conf_log_current.shape[1] - 1, max_idx_y)
            print(f"修正後: min_idx_x: {min_idx_x}, max_idx_x: {max_idx_x}, min_idx_y: {min_idx_y}, max_idx_y: {max_idx_y}")

        try:
            # 壁の内側の領域を抽出
            inner_posterior = self.Px_yn_conf_log_current[min_idx_x:max_idx_x+1, min_idx_y:max_idx_y+1]
            print(f"inner_posterior shape: {inner_posterior.shape}")
            print(f"inner_posterior contains NaN: {np.isnan(inner_posterior).any()}")
            
            # NaNを含む場合は-100で置き換える
            if np.isnan(inner_posterior).any():
                print("警告: inner_posteriorにNaNが含まれています。-100で置き換えます。")
                inner_posterior = np.nan_to_num(inner_posterior, nan=0)
            
            # 単純に合計値を返す
            convergence = np.sum(inner_posterior)
            print(f"convergence: {convergence}, is NaN: {np.isnan(convergence)}")
            
            # 最大最小値を表示
            if inner_posterior.size > 0:
                print(f"最大値: {np.max(inner_posterior)}")
                print(f"最小値: {np.min(inner_posterior)}")
                print(f"合計値: {convergence}")
            else:
                print("警告: inner_posteriorが空です")
                convergence = -1000  # デフォルト値を設定
            
            return convergence
        except Exception as e:
            print(f"エラーが発生しました: {e}")
            return -1000  # エラー時のデフォルト値
    
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
        # 尤度関数の計算
        Pyn_x_L_each = self.new_likelyhood_2D(
            y_el[0][~np.isnan(y_el[0])],
            current_obs_goback_dist_matrix_L[0],  # 2次元
            self.sigma2
        ).transpose(2, 0, 1)
        Pyn_x_R_each = self.new_likelyhood_2D(
            y_er[0][~np.isnan(y_er[0])],
            current_obs_goback_dist_matrix_R[0],  # 2次元
            self.sigma2
        ).transpose(2, 0, 1)
        self.Pyn_x_L_current = np.clip(np.sum(Pyn_x_L_each, axis=0), 0.1**20, None)
        self.Pyn_x_R_current = np.clip(np.sum(Pyn_x_R_each, axis=0), 0.1**20, None)
        
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
        
        # 認知収束度合いを計算
        convergence_value = self.calculate_convergence()
        self.convergence_history.append(convergence_value)

        # world_wall_posはconfig.pyから読み込まれます
        if world_wall_pos:
            # 壁の範囲外を-20に設定
            wall_x = np.array([margin_space, y_max - margin_space])
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
        
        data1 = self.Pyn_x_L_current
        data2 = current_confidence_matrix[0]
        data3 = self.Px_yn_log_current
        data4 = self.Px_yn_conf_log_current
        return data1, data2, data3, data4