# -*- coding: utf-8 -*-
"""
ベイズ更新モジュール (Bayesian belief update)
=============================================
エコー観測（実機では Localizer、シミュレーションでは calc()）から、
空間上の障害物存在確率（信念分布）をベイズ更新で逐次推定する。

--------------------------------------------------------------------
記号対応表（変数名の読み方）
--------------------------------------------------------------------
確率分布の変数は数式に対応した命名になっている。以下の規則で読む:

    P...        確率分布
    x           空間上の位置（格子点）
    y_n         n 回目の観測（エコー到達時間）
    末尾 L / R  左耳 / 右耳
    末尾 _log   log10 スケール
    _current    現ステップのみ保持（過去ステップは残さない）
    中間の 2    基本モデル（confidence 重みなし）
    中間の 3    記憶保持モデル（confidence で重み付け）

主な状態変数:
    Px                     初期事前分布（全格子点 1 で初期化）
    Px2L_log / Px2R_log    基本モデルの事前分布（左/右耳, log）。毎ステップ事後で更新
    Px3L_log / Px3R_log    記憶保持モデルの事前分布（左/右耳, log）。confidence 重み付き
    Pyn_x_L_current / _R   現ステップの尤度 P(y_n | x)（左/右耳）
    Px_ynL_log_current /_R 現ステップの事後 P(x | y_n)（左/右耳, log）
    Px_yn_log_current      左右統合した事後（記憶なし）
    Px_yn_conf_log_current 左右統合した事後（記憶あり=記憶保持モデル）
    confidence             confidence 行列（エコーの信頼度重み）
--------------------------------------------------------------------
"""
import numpy as np
import copy
from dataclasses import dataclass

# 設定ファイルから必要なパラメータをインポート
from bayes_code.config import x_max, y_max, margin_space, h, world_wall_pos


@dataclass(eq=False)  # ndarray フィールドを持つため == 比較は不可(eq=False で明示)
class BeliefSnapshot:
    """1 ステップのベイズ更新 update_belief() の出力（可視化に渡す 4 つの分布）。

    以前は data1〜data4 の 4 タプルで返しており、どれが何か分かりづらかった。
    可視化パネルの並び（尤度L / confidence / 事後(記憶なし) / 事後(記憶あり)）に対応する。
    control_pc.py の可視化用 dict（キー data1〜data4）へはキー名を温存して詰め替える。

    フィールドと従来の対応:
        likelihood_L          : 左耳の尤度 P(y_n|x)（旧 data1 = Pyn_x_L_current）
        confidence            : confidence 行列（旧 data2 = confidence_matrix[0]）
        posterior             : 左右統合した事後・記憶なし（旧 data3 = Px_yn_log_current）
        posterior_with_memory : 左右統合した事後・記憶保持（旧 data4 = Px_yn_conf_log_current）
    """
    likelihood_L: np.ndarray
    confidence: np.ndarray
    posterior: np.ndarray
    posterior_with_memory: np.ndarray


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

        # 座標とインデックスの対応: 座標値 / h = インデックス
        min_idx_x = int(min_x / h)
        max_idx_x = int(max_x / h)
        min_idx_y = int(min_y / h)
        max_idx_y = int(max_y / h)

        # インデックスが配列の範囲外なら範囲内に制限する
        if min_idx_x < 0 or min_idx_y < 0 or max_idx_x >= self.Px_yn_conf_log_current.shape[0] or max_idx_y >= self.Px_yn_conf_log_current.shape[1]:
            min_idx_x = max(0, min_idx_x)
            min_idx_y = max(0, min_idx_y)
            max_idx_x = min(self.Px_yn_conf_log_current.shape[0] - 1, max_idx_x)
            max_idx_y = min(self.Px_yn_conf_log_current.shape[1] - 1, max_idx_y)

        try:
            # 壁の内側の領域を抽出
            inner_posterior = self.Px_yn_conf_log_current[min_idx_x:max_idx_x+1, min_idx_y:max_idx_y+1]

            # NaN は足し算できないので 0 に置き換える
            if np.isnan(inner_posterior).any():
                inner_posterior = np.nan_to_num(inner_posterior, nan=0)

            # 壁の内側の事後確率の合計を収束度合いとして返す
            if inner_posterior.size > 0:
                convergence = np.sum(inner_posterior)
            else:
                convergence = -1000  # 領域が空のときのデフォルト値

            return convergence
        except Exception as e:
            print(f"認知収束度合いの計算でエラーが発生しました: {e}")
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

        # 壁の範囲外を認知対象外（-20）にマスクする
        if world_wall_pos:
            self._apply_wall_mask(self.Px_yn_conf_log_current)

        # 可視化用の 4 分布を BeliefSnapshot にまとめて返す
        return BeliefSnapshot(
            likelihood_L=self.Pyn_x_L_current,
            confidence=current_confidence_matrix[0],
            posterior=self.Px_yn_log_current,
            posterior_with_memory=self.Px_yn_conf_log_current,
        )

    def _apply_wall_mask(self, posterior):
        """壁の範囲外の事後確率を -20 に設定する（posterior を破壊的に更新）。

        壁の内側（margin_space 〜 x_max - margin_space）だけを認知対象とし、
        外側は一律 -20（log スケールでほぼ 0 確率）に落とす。
        """
        wall_x = np.array([margin_space, x_max - margin_space])
        wall_y = np.array([margin_space, y_max - margin_space])
        wall_corner_x, wall_corner_y = np.meshgrid(wall_x, wall_y)
        wall_corner_x = wall_corner_x.flatten()
        wall_corner_y = wall_corner_y.flatten()

        # wall_cornerの範囲を取得
        min_x = np.min(wall_corner_x)
        max_x = np.max(wall_corner_x)

        # 座標とインデックスの対応: 座標値 / h = インデックス
        min_idx = int(min_x / h)
        max_idx = int(max_x / h)

        # x が範囲外の領域を -20 に
        posterior[:min_idx, :] = -20
        posterior[max_idx+1:, :] = -20
        # y が範囲外の領域を -20 に
        posterior[:, :min_idx] = -20
        posterior[:, max_idx+1:] = -20