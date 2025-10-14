# -*- coding: utf-8 -*-
"""
World Class for Bayesian Simulation
==================================
このクラスはシミュレーション環境（壁と障害物）の管理を担当します。

主な機能:
1. 壁の座標管理
2. 障害物（ポール）の座標管理
3. 環境の初期化と設定
4. 障害物データの読み込み
"""

import pandas
import numpy as np

# 設定ファイルから必要なパラメータをインポート
from bayes_code.config import world_pole_wall

class World:
    """
    シミュレーション環境を管理するクラス
    
    このクラスは以下の機能を提供します：
    1. 壁の座標設定と管理
    2. 障害物（ポール）の座標読み込みと管理
    3. 環境の境界設定
    4. 障害物の動的な取得
    
    Attributes:
        x_max (float): シミュレーション空間の最大x座標
        y_max (float): シミュレーション空間の最大y座標
        margin_space (float): 境界マージン空間
        wall_x (np.ndarray): 壁のx座標配列
        wall_y (np.ndarray): 壁のy座標配列
        wall_corner_x (np.ndarray): 壁の角のx座標配列
        wall_corner_y (np.ndarray): 壁の角のy座標配列
        pole_x (np.ndarray): ポールのx座標配列
        pole_y (np.ndarray): ポールのy座標配列
        folder_name (str): データファイルのディレクトリ名
    """
    
    def __init__(self, x_max: float, y_max: float, margin_space: float, h: float, t_max: float, dt: float, c: float, folder_name: str):
        """
        Worldクラスの初期化
        
        Args:
            x_max (float): シミュレーション空間の最大x座標 [m]
            y_max (float): シミュレーション空間の最大y座標 [m]
            margin_space (float): 境界マージン空間 [m]
            h (float): 格子点間隔 [m]
            c (float): 音速 [m/s]
            folder_name (str): データファイルのディレクトリ名
        """
        self.x_max = x_max
        self.y_max = y_max
        self.margin_space = margin_space
        self.Mx, self.My = int(np.round(x_max / h)), int(np.round(y_max / h))
        # 空間座標軸の生成
        self.x_axis = np.linspace(0, self.x_max, int(self.x_max / h) + 1)  # x軸の座標点の配列
        self.y_axis = np.linspace(0, self.y_max, int(self.y_max / h) + 1)  # y軸の座標点の配列
        self.X, self.Y = np.meshgrid(self.x_axis, self.y_axis)  # 2次元格子点の生成
        # エコー時間軸の生成
        self.t_max = t_max
        self.dt = dt
        self.Mt = int(np.round(self.t_max / self.dt))  # 時間刻み数
        self.t_ax = np.linspace(0, self.t_max, self.Mt + 1)  # 時間軸の生成
        
        self.c = c
        self.folder_name = folder_name

        self.wall_x = None
        self.wall_y = None
        self.wall_corner_x = None
        self.wall_corner_y = None
        self.pole_x = None
        self.pole_y = None
        
        # 壁の座標を設定
        self._setup_walls()
        print(f"壁の頂点初期化: {self.wall_corner_x}, {self.wall_corner_y}")

        # 障害物（ポール）の座標を読み込み
        self._real_obs()
        print(f"障害物読み込み完了： {self.pole_x.shape} poles")

        
    def _setup_walls(self):
        """壁の座標を設定する"""
        # 障害物を周囲に置くようにする
        # 四方の壁の座標設定
        self.wall_x = np.array([self.margin_space, self.x_max - self.margin_space])
        self.wall_y = np.array([self.margin_space, self.y_max - self.margin_space])
        
        # 壁の角の座標生成
        wall_corner_x, wall_corner_y = np.meshgrid(self.wall_x, self.wall_y)
        self.wall_corner_x = wall_corner_x.flatten()
        self.wall_corner_y = wall_corner_y.flatten()

        # world_pole_wallはconfig.pyから読み込まれます
        if world_pole_wall:
            """
            ここでは、壁の座標を設定している。
            壁の座標は、障害物を周囲に置くようにする。
            """
            # wall_cornerを使用して正方形の線上に障害物ポールを配置
            grid_spacing = 0.1  # ポール間の間隔 [m] 0.5 0.25 0.1
            
            # wall_cornerの範囲を取得
            min_x = np.min(self.wall_corner_x)
            max_x = np.max(self.wall_corner_x)
            min_y = np.min(self.wall_corner_y)
            max_y = np.max(self.wall_corner_y)
            
            # 正方形の各辺に沿ってポールを配置
            pole_positions = []
            
            # 下辺（y = min_y）
            x_bottom = np.arange(min_x, max_x + grid_spacing, grid_spacing)
            for x in x_bottom:
                pole_positions.append([x, min_y])
            
            # 上辺（y = max_y）
            x_top = np.arange(min_x, max_x + grid_spacing, grid_spacing)
            for x in x_top:
                pole_positions.append([x, max_y])
            
            # 左辺（x = min_x）
            y_left = np.arange(min_y + grid_spacing, max_y, grid_spacing)
            for y in y_left:
                pole_positions.append([min_x, y])
            
            # 右辺（x = max_x）
            y_right = np.arange(min_y + grid_spacing, max_y, grid_spacing)
            for y in y_right:
                pole_positions.append([max_x, y])
            
            # 重複を除去してNumPy配列に変換
            pole_positions = np.array(pole_positions)
            self.pole_x = pole_positions[:, 0]
            self.pole_y = pole_positions[:, 1]
    
    def _real_obs(self):
        """
        CSVファイルから障害物（ポール）の位置データを読み込む
        
        Returns:
            Tuple[np.ndarray, np.ndarray]: (pole_x, pole_y) ポールのx, y座標配列
        """
        try:
            # ポールの位置情報をCSVファイルから読み込む
            csv_path = f"{self.folder_name}/chain_position_y.csv"
            chain_loc = pandas.read_csv(csv_path, header=0)
            
            # マージンを考慮して座標を調整し、NumPy配列に変換
            obs_x = chain_loc["X"].values + self.margin_space
            obs_y = chain_loc["Y"].values + self.margin_space
            
            # すでにある場合は追加する
            if self.pole_x is not None:
                self.pole_x = np.concatenate([self.pole_x, np.array(obs_x)])
                self.pole_y = np.concatenate([self.pole_y, np.array(obs_y)])
            else:
                self.pole_x = np.array(obs_x)
                self.pole_y = np.array(obs_y)
                        
        except FileNotFoundError:
            print(f"Warning: Obstacle data file not found {csv_path}")
            print("Using default obstacle positions")
            
        except Exception as e:
            print(f"Error loading obstacles: {e}")

