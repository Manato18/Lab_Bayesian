import copy
import numpy as np
from scipy.special import jv

h = 0.01
freq = 40000    # 周波数 [Hz] - コウモリが発する超音波の周波数
c = 340
a = 0.005       # 口の半径サイズ [m] - 超音波を発する口の大きさ
ear_dist = 0.02   # 耳間距離 [m] - コウモリの左右の耳の間の距離
dt = 100*10**-6
t_max=64*10**-3
Mt = int(np.round(t_max / dt))  # 時間刻み数
t_ax = np.linspace(0, t_max, Mt + 1)  # 時間軸の生成
trials = 1
x_max = y_max = 8.5
Mx = My = int(np.round(y_max / h))

threshold = 1.2 * 0.1**3  # エコー検知最小感度　(さらに、conf1の半値境目)
# threshold = 0.0017  # エコー検知最小感度　(さらに、conf1の半値境目)
grad = 10**15            # conf1の勾配
k_r_noise = 60000
k_theta_noise = 360000

def round_angle(angle_rad_matrix):
    """
    角度をラジアン単位で正規化する関数
    
    角度を-π～πの範囲に正規化します。これはコウモリの頭の方向や
    目標物の相対角度を計算する際に重要です。
    
    Args:
        angle_rad_matrix (ndarray): 正規化する角度（ラジアン単位）の配列
        
    Returns:
        ndarray: -π～πの範囲に正規化された角度（ラジアン単位）の配列
    """
    # ラジアンから度に変換
    angle_deg_matrix = np.rad2deg(angle_rad_matrix)
    
    # 角度を-180°～180°の範囲に正規化するための計算
    angle_fit = (angle_deg_matrix - 180) // 360 + 1
    # 以下は代替実装のコメントアウト例
    #    angle_fit = np.where(angle_fit <= -1, angle_fit+1, angle_fit)
    
    # 正規化された角度を計算（度単位）
    angle_deg_matrix_improve = angle_deg_matrix - angle_fit * 360
    
    # 度からラジアンに戻して返す
    return np.deg2rad(angle_deg_matrix_improve)

def wall_ref_point(x, y):
    """
    壁面の反射点の座標格子を生成する関数
    
    与えられたx座標とy座標からメッシュグリッド（格子点）を生成します。
    これは壁面の反射点を計算するために使用されます。
    
    Args:
        x (ndarray): x座標の配列（通常は壁のx座標）
        y (ndarray): y座標の配列（通常は壁のy座標）
        
    Returns:
        tuple: (X_ref, Y_ref) - 格子点のx座標とy座標を含む2次元配列
    """
    # numpyのmeshgrid関数を使用して2次元格子点を生成
    # 例: x=[1,2], y=[3,4]の場合、X_ref=[[1,2],[1,2]], Y_ref=[[3,3],[4,4]]となる
    X_ref, Y_ref = np.meshgrid(x, y)
    return X_ref, Y_ref

def XY_to_r_theta_calc(bat_x, bat_y, obs_x, obs_y, pd):
    """
    デカルト座標（x, y）から極座標（距離r, 角度θ）への変換関数
    
    コウモリの位置から見た障害物の距離と角度を計算します。
    コウモリの向いている方向（パルス方向）に対する相対角度を算出します。
    
    Args:
        bat_x (ndarray): コウモリのx座標配列
        bat_y (ndarray): コウモリのy座標配列
        obs_x (ndarray): 障害物のx座標の2次元配列（各行がセンシングタイミング、各列が障害物）
        obs_y (ndarray): 障害物のy座標の2次元配列（各行がセンシングタイミング、各列が障害物）
        pd (ndarray): パルス方向の角度（度数法）配列
        
    Returns:
        tuple: (r_true, theta_true)
            - r_true: コウモリから障害物までの距離の2次元配列
            - theta_true: コウモリのパルス方向に対する障害物の相対角度（ラジアン）の2次元配列
    """
    # コウモリの座標を障害物の数に合わせて拡張（タイル状に複製）
    BX = np.tile(bat_x, (np.shape(obs_x)[1], 1)).T  # 各行が同じコウモリx座標、列数は障害物の数
    BY = np.tile(bat_y, (np.shape(obs_y)[1], 1)).T  # 同上（y座標）
    
    # 障害物の座標を変数名の簡略化のために代入
    OX = obs_x
    OY = obs_y

    # 結果格納用の配列を初期化
    r_true = np.zeros_like(OX, dtype=float)      # 距離計算用の配列 (12,2) - センシング回数×障害物数
    theta_true = np.zeros_like(OX, dtype=float)  # 角度計算用の配列 (12,2) - 同上
    
    # デバッグ情報の出力
    print("len(OX):", len(OX))      # センシング回数
    print("len(OX[0]:", len(OX[0])) # 障害物の数
    
    # コウモリから障害物へのベクトル成分を計算
    del_X = OX - BX  # x方向の差分
    del_Y = OY - BY  # y方向の差分
    
    # ベクトルの大きさ（距離）を計算
    r_true = np.linalg.norm(np.array([del_X, del_Y]), axis=(0))
    # 代替実装のコメントアウト例
    #    r_true = np.linalg.norm(np.array([del_X[:,:], del_Y[:,:]]), axis=(0))
    
    # パルス方向を障害物の数に合わせて拡張
    pd = np.tile(pd, (del_X.shape[1], 1)).T
    
    # 障害物の角度をコウモリのパルス方向に対する相対角度として計算
    # arctan2でベクトルの角度を計算し、パルス方向を引いて、-π～πの範囲に正規化
    theta_true = round_angle(np.arctan2(del_Y, del_X) - np.deg2rad(pd))
    
    return r_true, theta_true

def dist_attenuation(r):
    """
    距離による音波減衰計算関数
    
    音波は距離の増加とともに強度が弱まります。この関数は距離による音波の減衰を
    計算します。音波の減衰は距離の逆数に比例します。
    
    Args:
        r (ndarray): コウモリから障害物までの距離(メートル)の配列
        
    Returns:
        ndarray: 距離による減衰係数の配列。値が小さいほど減衰が大きい
    """
    print("check20")  # デバッグ用出力
    
    # 基準距離を設定（空間分解能の半分を基準値とする）
    r0 = h / 2  # 基準距離[m] - config.pyのh（空間分解能）の半分
    
    print("check21")  # デバッグ用出力
    
    # 音波強度は距離の逆数に比例する（球面波の特性）
    return r0 / r

def direc_attenuation(theta):
    """
    方向による音波減衰計算関数
    
    超音波の指向性に基づく減衰を計算します。コウモリが発する超音波は
    その発声器官（口）の形状により、方向によって音圧が異なります。
    これは「ピストンモデル」と呼ばれるパターンに従います。
    
    Args:
        theta (ndarray): コウモリの無音波発射方向からの相対角度(ラジアン)の配列
    
    Returns:
        ndarray: 方向による減衰係数の配列。中心（0度）で最大、側面で小さくなる
    """
    # デバッグ用出力
    print("check22")
    
    # 計算の安定性のため、角度を-90度～90度（-π/2～π/2）の範囲に制限
    theta_clip = np.clip(theta, -np.pi / 2, np.pi / 2)
    
    print("check23")  # デバッグ用出力
    
    # 波数の計算: k = 2π / 波長, 波長 = 音速 / 周波数
    k = 2 * np.pi * freq / c  # 単位: rad/m
    
    print("check24")  # デバッグ用出力
    
    # ピストン音源の指向性関数を計算
    # これは円形音源（コウモリの口）からの音波放射パターンを表す
    # jv(1, x)は1次のベッセル関数
    beam_pattern = abs(
        2 * jv(1, k * a * np.sin(theta_clip)) / (k * a * np.sin(theta_clip))
    )  # ピストンモデル(再度ローブ表現のためにabs必要)
    
    print("check25")  # デバッグ用出力
    
    return beam_pattern

def detection_judge(attenuation, trans_info):
    """
    エコー検出判定関数
    
    減衰があまりに大きくなると、コウモリはエコーを検出できなくなります。
    この関数は、減衰が閾値を超えた（音が小さすぎる）エコーを「検出されない」と
    マーキングします。
    
    Args:
        attenuation (ndarray): 減衰係数（距離減衰と指向性減衰の積）の配列
        trans_info (ndarray): 更新する情報配列（距離や角度などの物理量）
        
    Returns:
        tuple: (trans_info_updated, echo_n)
            - trans_info_updated: 検出不可能な点が「NaN」でマーキングされた更新後の配列
            - echo_n: 各センシングタイミングで検出されたエコーの数の配列
    """
    # 閾値未満の減衰係数（検出できないエコー）のインデックスを取得
    idx_thres_lower = np.where(attenuation < threshold)
    
    # 検出できないエコーの情報を「NaN」に設定
    trans_info[idx_thres_lower] = "NaN"
    
    # 各センシングタイミング（行方向）での検出エコー数をカウント
    echo_n = np.sum((attenuation > threshold), axis=1)
    
    return trans_info, echo_n

def ear_posit(bat_x, bat_y, pd):
    """
    コウモリの左右の耳の位置を計算する関数
    
    コウモリの頭部位置と向きから、左右の耳の座標を計算します。
    耳はコウモリの頭部から両側に距離ear_dist/2だけ離れた位置にあると仮定します。
    
    Args:
        bat_x (ndarray): コウモリの頭部中心x座標の配列
        bat_y (ndarray): コウモリの頭部中心y座標の配列
        pd (ndarray): コウモリの頭部方向（パルス発射方向）の角度（度数法）の配列
        
    Returns:
        tuple: (ear_xL, ear_yL, ear_xR, ear_yR)
            - ear_xL: 左耳のx座標の配列
            - ear_yL: 左耳のy座標の配列
            - ear_xR: 右耳のx座標の配列
            - ear_yR: 右耳のy座標の配列
    """
    # 頭部方向に対して左90度の方向（左耳の方向）をラジアンで計算
    pd_rad_L = np.deg2rad(pd + 90)
    # 頭部方向に対して右90度の方向（右耳の方向）をラジアンで計算
    pd_rad_R = np.deg2rad(pd - 90)

    # 左耳のx, y座標を計算（頭部中心から左方向にear_dist/2だけ離れた位置）
    ear_xL = (ear_dist / 2) * np.cos(pd_rad_L) + bat_x
    ear_yL = (ear_dist / 2) * np.sin(pd_rad_L) + bat_y
    
    # 右耳のx, y座標を計算（頭部中心から右方向にear_dist/2だけ離れた位置）
    ear_xR = (ear_dist / 2) * np.cos(pd_rad_R) + bat_x
    ear_yR = (ear_dist / 2) * np.sin(pd_rad_R) + bat_y
    
    return ear_xL, ear_yL, ear_xR, ear_yR

def real_dist_goback(speaker_x, speaker_y, ear_x, ear_y, obs_x, obs_y):
    """
    超音波の実際の往復距離を計算する関数
    
    コウモリの口から放射された超音波が障害物に反射し、耳に到達するまでの
    全距離（往路+復路）を正確に計算します。コウモリの口と耳は別の位置にあるため、
    単純な2倍ではなく、実際の往路と復路の距離を個別に計算する必要があります。
    
    Args:
        speaker_x (ndarray): コウモリの口（超音波発生源）のx座標の配列
        speaker_y (ndarray): コウモリの口（超音波発生源）のy座標の配列
        ear_x (ndarray): コウモリの耳（受音部）のx座標の配列
        ear_y (ndarray): コウモリの耳（受音部）のy座標の配列
        obs_x (ndarray): 障害物のx座標の配列
        obs_y (ndarray): 障害物のy座標の配列
        
    Returns:
        ndarray: 往路と復路の合計距離の配列（メートル単位）
    """
    def measure_dist(x0, y0, x1, y1):
        """
        2点間の距離を計算するローカル関数
        
        Args:
            x0, y0: 始点の座標配列
            x1, y1: 終点の座標配列
            
        Returns:
            ndarray: 2点間の距離の配列
        """
        # 始点座標を終点配列のサイズに合わせて拡張
        BXS = np.tile(x0, (x1.shape[-1], 1)).T
        BYS = np.tile(y0, (y1.shape[-1], 1)).T
        
        # 終点座標を変数名の簡略化のために代入
        SX = x1
        SY = y1

        # 2点間のベクトルを計算
        a = np.array([BXS, BYS])  # 始点座標ベクトル
        b = np.array([SX, SY])    # 終点座標ベクトル
        vec = b - a               # 2点を結ぶベクトル

        # ベクトルのノルム（大きさ）を計算して距離を求める
        dist = np.linalg.norm(vec, axis=0)
        return dist

    # 往路：口から障害物までの距離を計算
    go_dist = measure_dist(speaker_x, speaker_y, obs_x, obs_y)
    
    # 復路：障害物から耳までの距離を計算
    back_dist = measure_dist(ear_x, ear_y, obs_x, obs_y)

    # 往路と復路の距離の合計が超音波の全移動距離
    return go_dist + back_dist

def space_echo_translater(goback_distL, goback_distR):
    tl = goback_distL / c
    tr = goback_distR / c

    return tl, tr

def gauss_noise(mu, sigma, trials):
    """
    ガウス（正規）ノイズを生成する関数
    
    指定された平均値と標準偏差のパラメータに基づいて、指定された回数の
    ガウス分布（正規分布）に従う乱数を生成します。
    一次元の入力（単一パラメータ）と二次元の入力（複数の障害物など）の両方に対応しています。
    
    Args:
        mu (ndarray): ガウスノイズの平均値（期待値）
        sigma (ndarray): ガウスノイズの標準偏差
        trials (int): 生成するノイズサンプルの数
        
    Returns:
        ndarray: 生成されたガウスノイズの配列。返値の形状は(trials, len(MU))または(trials,)
    """
    # 入力が多次元の場合（複数の障害物など）
    if mu.ndim > 1:
        # 転置して处理しやすい形式に変換
        MU = mu.T
        SIGMA = sigma.T
        
        # 結果を格納するゼロ配列を初期化
        y = np.zeros((len(MU), trials))
        
        # 各障害物についてループでガウスノイズを生成
        for obs_n in range(len(MU)):
            # numpyのrandom.normal関数で正規分布に従う乱数を生成
            y[obs_n] = np.random.normal(loc=MU[obs_n], scale=SIGMA[obs_n], size=trials)
    # 入力が一次元の場合（単一パラメータ）
    else:
        # よりシンプルな方法でガウスノイズを生成
        y = np.random.normal(loc=mu, scale=sigma, size=trials)
    
    # 転置して返却（行がトライアル、列が障害物の形式に整える）
    return y.T

def r_theta_to_XY_calc(obs_r, obs_theta_deg, bat_x, bat_y, pd):
    """
    極座標（r, θ）からデカルト座標（X, Y）への変換関数
    
    コウモリから見た障害物の距離と角度（極座標）から、実際の座標系での
    障害物の位置（デカルト座標）を計算します。これはXY_to_r_theta_calcの逆変換です。
    
    Args:
        obs_r (ndarray): コウモリから障害物までの距離の配列
        obs_theta_deg (ndarray): コウモリのパルス方向に対する障害物の相対角度（度数法）の配列
        bat_x (ndarray): コウモリのx座標の配列
        bat_y (ndarray): コウモリのy座標の配列
        pd (ndarray): コウモリのパルス方向の角度（度数法）の配列
        
    Returns:
        tuple: (X, Y)
            - X: 障害物の絶対x座標の配列
            - Y: 障害物の絶対y座標の配列
    """
    # パルス方向の配列を障害物の数に合わせて拡張
    pd = np.tile(pd, (obs_theta_deg.shape[-1], 1)).T
    # コウモリX座標配列を障害物の数に合わせて拡張
    bat_x = np.tile(bat_x, (obs_theta_deg.shape[-1], 1)).T
    # コウモリY座標配列を障害物の数に合わせて拡張
    bat_y = np.tile(bat_y, (obs_theta_deg.shape[-1], 1)).T

    # 相対角度から絶対角度に変換（パルス方向を加算）
    abs_theta = obs_theta_deg + pd
    
    # 極座標からデカルト座標への変換式を適用
    # X = r * cos(θ) + bat_x
    X = obs_r * np.cos(np.deg2rad(abs_theta)) + bat_x
    # Y = r * sin(θ) + bat_y
    Y = obs_r * np.sin(np.deg2rad(abs_theta)) + bat_y
    
    return X, Y

def index_calc(y, attenuation):
    """
    観測されたエコー時間を時間軸上のインデックスに変換し、スパイク状の時間応答ベクトルを生成
    
    引数:
        y: エコー到達時間データ（単一値または配列）
        attenuation: 減衰係数（現在の実装では使用されていない）
    
    戻り値:
        i: エコー到達時間に対応する時間インデックス
        y_array: 時間軸上でのスパイク応答ベクトル（各エコー到達時刻で1、それ以外では微小値）
    """
    # エコー到達時間をインデックスに変換（サンプリング時間dtで正規化し、整数に丸める）
    i = np.round(y / dt).astype(int) + 1  # +1はインデックスオフセット調整
    
    if np.isscalar(y):  # 単一のエコー（単発センシング）の場合
        # すべての時間で微小値を持ち、エコー到達時刻のみ1にするベクトル
        y_array = np.ones(len(t_ax)) * eps_y  # 背景レベル（微小値）で初期化
        y_array[i] = 1  # エコー到達時刻のみ1（スパイク）に設定
    else:  # 複数回のセンシングデータ（配列）の場合
        print("y_dim = 1,  sensing_length is     :", len(y))  # センシング回数を表示
        
        # 各センシングごとの時間応答ベクトルを2次元配列として生成
        y_array = np.zeros((len(y), len(t_ax)))  # [センシング回数 x 時間軸長]の2次元配列
        
        # 各センシングについて処理
        for n in range(len(i)):
            i_over = np.where(i[n] >= 0)  # 負のインデックスを除外（有効な時間のみ）
            y_array[n][i[n][i_over]] = 1  # 該当時間インデックスをスパイク（1）に設定
    
    return i, y_array


def r_theta_matrix(bx_vec, by_vec, space_x, space_y, pd_vec):
    """
    空間全体の距離と角度の行列を計算する関数
    
    コウモリの位置から見た空間内の全ての格子点に対する距離と角度を計算します。
    これはシミュレーション空間全体に対するベイズ更新や確率計算のための基礎データとして使用されます。
    
    Args:
        bx_vec (ndarray): コウモリのx座標の配列（各センシングタイミングごと）
        by_vec (ndarray): コウモリのy座標の配列（各センシングタイミングごと）
        space_x (ndarray): 空間の格子点のx座標の2次元配列
        space_y (ndarray): 空間の格子点のy座標の2次元配列
        pd_vec (ndarray): コウモリのパルス方向（度数法）の配列
        
    Returns:
        tuple: (r_2vec, theta_2vec_pipi)
            - r_2vec: コウモリから空間内の各点までの距離の3次元配列 (trials, Mx+1, My+1)
            - theta_2vec_pipi: コウモリのパルス方向から見た空間内の各点の角度の3次元配列（-π～πの範囲に正規化済み）
    """
    print("check0")  # デバッグ用出力
    
    # 結果を格納するゼロ配列を初期化
    # trials: センシング形時数、Mx+1, My+1: 空間格子点の数
    r_2vec = np.zeros((trials, Mx + 1, My + 1))  # 距離格納用
    theta_2vec = np.zeros((trials, Mx + 1, My + 1))  # 角度格納用
    
    # 各センシングタイミングごとに計算
    i = 0
    for bx, by, pd in zip(bx_vec, by_vec, pd_vec):
        # コウモリから各格子点へのベクトル成分を計算
        del_x = space_x - bx  # x方向の差分
        del_y = space_y - by  # y方向の差分
        
        # 距離（ベクトルの大きさ）をピタゴラスの定理で計算
        r_2vec[i] = np.sqrt(del_x**2 + del_y**2)
        
        # 角度を計算（arctan2でベクトルの角度を求め、パルス方向を差し引いて相対角度に変換）
        theta_2vec[i] = np.arctan2(del_y, del_x) - np.deg2rad(pd)
        i = i + 1

    print("check01")  # デバッグ用出力
    
    # 角度を-π～πの範囲に正規化（第一段階：πより大きい場合は2πを引く）
    theta_2vec_pi = np.where(theta_2vec > np.pi, theta_2vec - 2 * np.pi, theta_2vec)
    # 角度を-π～πの範囲に正規化（第二段階：-πより小さい場合は2πを足す）
    theta_2vec_pipi = np.where(theta_2vec_pi < -np.pi, theta_2vec_pi + 2 * np.pi, theta_2vec_pi)
    
    print("check02")  # デバッグ用出力
    
    return r_2vec, theta_2vec_pipi

def real_dist_goback_matrix(speaker_x, speaker_y, ear_x, ear_y, space_x, space_y):
    """
    シミュレーション空間全体における超音波の往復距離行列を計算する関数
    
    コウモリの口（超音波の発生源）から空間内の各点までの距離（往路）と、
    そこから耳（受音部）までの距離（復路）を計算し、合計します。
    これはシミュレーション空間全体の各格子点に対する超音波の往復時間を計算するための基礎となります。
    
    Args:
        speaker_x (ndarray): コウモリの口（超音波の発生源）のx座標の配列
        speaker_y (ndarray): コウモリの口（超音波の発生源）のy座標の配列
        ear_x (ndarray): コウモリの耳（受音部）のx座標の配列
        ear_y (ndarray): コウモリの耳（受音部）のy座標の配列
        space_x (ndarray): 空間の格子点のx座標の2次元配列
        space_y (ndarray): 空間の格子点のy座標の2次元配列
        
    Returns:
        ndarray: 往路と復路の合計距離の3次元配列 (trials, Mx+1, My+1)
    """
    def measure_dist(x0, y0, x1, y1):
        """
        内部関数: 2点間の距離を空間全体にわたって計算する
        
        Args:
            x0, y0: 始点の座標配列（センシング数分）
            x1, y1: 終点の格子点座標配列（2次元）
            
        Returns:
            ndarray: 距離の3次元配列 (trials, Mx+1, My+1)
        """
        print("check10")  # デバッグ用出力
        
        # 結果を格納するゼロ配列を初期化
        dist = np.zeros((trials, Mx + 1, My + 1))
        
        # 各センシングタイミングごとに計算
        i = 0
        for bx, by in zip(x0, y0):  # 各センシングタイミングの始点座標を取得
            # 始点から空間内の各点へのベクトル成分
            del_x = space_x - bx  # x方向の差分
            del_y = space_y - by  # y方向の差分
            
            # 距離（ベクトルの大きさ）をピタゴラスの定理で計算
            dist[i] = np.sqrt(del_x**2 + del_y**2)
            i = i + 1
            
        print("check11")  # デバッグ用出力

        return dist

    # 往路：口（スピーカー）から空間内の各点までの距離
    go_dist = measure_dist(speaker_x, speaker_y, space_x, space_y)
    
    # 復路：空間内の各点から耳までの距離
    back_dist = measure_dist(ear_x, ear_y, space_x, space_y)

    # 往路と復路の合計距離を返却
    return go_dist + back_dist

def sigmoid(x, center, grad):
    """
    シグモイド関数
    
    S字型の曲線を描くシグモイド関数を計算します。これは0から1の間での滑らかな遷移を表し、
    確率の急な変化や阈値処理などに利用されます。ベイズ更新のプロセスで確率値の滑らかな
    変化を表現するために使用されます。
    
    Args:
        x (ndarray or float): 関数の入力値
        center (float): 関数の中心点（この値で出力は0.5になる）
        grad (float): 関数の勅配を決めるパラメータ（値が大きいほど急勅配になる）
        
    Returns:
        ndarray or float: 0から1の間の値にマッピングされた結果
    """
    # シグモイド関数の実装
    # 1 / (1 + e^(-4 * grad * (x - center)))
    # 4を掛けることで、grad=1のときに標準的な勅配になるよう調整されている
    return 1 / (1 + np.exp(-4 * grad * (x - center)))


def calc(world, current_bat_x, current_bat_y, current_fd, current_pd, X, Y):

    """
    障害物を周囲に置くように変更する。
    """
    world_pole_wall = False # 障害物を壁の線上に配置するかどうか
    world_wall_pos = True # 壁の外を認知-20にするかどうか
    if world_pole_wall:
        current_obs_x = world.pole_x.reshape(1, -1)
        current_obs_y = world.pole_y.reshape(1, -1)
    elif world_wall_pos:
        current_obs_x = world.pole_x.reshape(1, -1)
        current_obs_y = world.pole_y.reshape(1, -1)
    else:
        # --- 障害物座標の計算（毎ステップ） ---
        # ポール・壁角
        obs_x = np.hstack((world.pole_x, world.wall_corner_x))
        obs_y = np.hstack((world.pole_y, world.wall_corner_y))
        # 1行分にする
        obs_x = obs_x.flatten()
        obs_y = obs_y.flatten()
        # 壁面反射点
        wall_x_det_temp = copy.deepcopy(current_bat_x)
        wall_y_det_temp = copy.deepcopy(current_bat_y)
        wall_x_det_UD, wall_y_det_UD = wall_ref_point(wall_x_det_temp, world.wall_y)
        wall_x_det_LR, wall_y_det_LR = wall_ref_point(world.wall_x, wall_y_det_temp)
        wall_x_det_UD = wall_x_det_UD.T
        wall_y_det_UD = wall_y_det_UD.T
        # 1次元にして結合
        current_obs_x = np.hstack((obs_x, wall_x_det_UD.flatten(), wall_x_det_LR.flatten()))
        current_obs_y = np.hstack((obs_y, wall_y_det_UD.flatten(), wall_y_det_LR.flatten()))

        # 2次元配列に変換（XY_to_r_theta_calc関数の期待する形式）
        current_obs_x = current_obs_x.reshape(1, -1)  # (1, n_obstacles)
        current_obs_y = current_obs_y.reshape(1, -1)  # (1, n_obstacles)
        print(f"障害物座標x: {current_obs_x.shape}, {current_obs_x}")

    # コウモリの座標も2次元配列に変換
    current_bat_x_2d = np.array([current_bat_x]).reshape(1, 1)  # (1, 1)
    current_bat_y_2d = np.array([current_bat_y]).reshape(1, 1)  # (1, 1)
    current_pd_2d = np.array([current_pd]).reshape(1, 1)  # (1, 1)
    print(f"コウモリ座標x: {current_bat_x_2d.shape}, {current_bat_x_2d}")

    # ターゲット極座標の計算（ステップごと）
    r_true, theta_true = XY_to_r_theta_calc(current_bat_x_2d, current_bat_y_2d, current_obs_x, current_obs_y, current_pd_2d)

    # エコー音圧計算（ステップごと）
    attenuation_obs = dist_attenuation(r_true) * direc_attenuation(theta_true)

    # 検知ポール判定（ステップごと）
    r_true, echo_n = detection_judge(attenuation_obs, r_true)
    theta_true, echo_n = detection_judge(attenuation_obs, theta_true)

    # ターゲットの厳密往復距離の計算（ステップごと）
    earL_x, earL_y, earR_x, earR_y = ear_posit(current_bat_x, current_bat_y, current_pd)

    obs_goback_dist_L = real_dist_goback(current_bat_x, current_bat_y, earL_x, earL_y, current_obs_x, current_obs_y)
    obs_goback_dist_R = real_dist_goback(current_bat_x, current_bat_y, earR_x, earR_y, current_obs_x, current_obs_y)

    y_el_true, y_er_true = space_echo_translater(obs_goback_dist_L, obs_goback_dist_R)

    # 真の揺らぎの計算（ステップごと）
    r_noise_rate = -20 * np.log10(attenuation_obs) / k_r_noise
    theta_noise_rate = -20 * np.log10(attenuation_obs) / k_theta_noise

    r_noise = np.ones_like(r_true) * r_noise_rate
    theta_noise = np.ones_like(r_noise) * theta_noise_rate

    # 現実世界に対応する観測信号生成（ステップごと）
    noisy_dist = gauss_noise(r_true, r_noise, 1)  # 1回の試行
    noisy_theta_rad = gauss_noise(theta_true, theta_noise, 1)  # 1回の試行
    noisy_theta_deg = np.rad2deg(noisy_theta_rad)

    noisy_obs_x, noisy_obs_y = r_theta_to_XY_calc(noisy_dist, noisy_theta_deg, current_bat_x, current_bat_y, current_pd)

    noisy_goback_L = real_dist_goback(current_bat_x, current_bat_y, earL_x, earL_y, noisy_obs_x, noisy_obs_y)
    noisy_goback_R = real_dist_goback(current_bat_x, current_bat_y, earR_x, earR_y, noisy_obs_x, noisy_obs_y)

    y_el, y_er = space_echo_translater(noisy_goback_L, noisy_goback_R)
    print(f"ノイズありのエコー到達時間：y_el: {y_el}, y_er: {y_er}")

    # 観測点の座標計算（ステップごと）
    if noisy_dist.ndim == 2:
        bat_x_array = np.tile(current_bat_x, (noisy_dist.shape[-1], 1)).T
        bat_y_array = np.tile(current_bat_y, (noisy_dist.shape[-1], 1)).T
        pd_array = np.tile(current_pd, (noisy_dist.shape[-1], 1)).T
    elif noisy_dist.ndim == 1:
        bat_x_array = current_bat_x
        bat_y_array = current_bat_y
        pd_array = current_pd

    y_x = bat_x_array + noisy_dist * np.cos(noisy_theta_rad + np.deg2rad(pd_array))
    y_y = bat_y_array + noisy_dist * np.sin(noisy_theta_rad + np.deg2rad(pd_array))

    # 可視化用データの準備（ステップごと）
    i_y_el, y_el_vec = index_calc(y_el, attenuation_obs)
    i_y_er, y_er_vec = index_calc(y_er, attenuation_obs)

    # 現在のステップの空間マトリックスを更新
    current_r_2vec, current_theta_2vec_rad = r_theta_matrix(
        np.array([current_bat_x]), np.array([current_bat_y]), X, Y, np.array([current_pd])
    )
    current_obs_goback_dist_matrix_L = real_dist_goback_matrix(
        np.array([current_bat_x]), np.array([current_bat_y]), np.array([earL_x]), np.array([earL_y]), X, Y
    )
    current_obs_goback_dist_matrix_R = real_dist_goback_matrix(
        np.array([current_bat_x]), np.array([current_bat_y]), np.array([earR_x]), np.array([earR_y]), X, Y
    )
    current_attenuation_matrix = dist_attenuation(current_r_2vec) * direc_attenuation(current_theta_2vec_rad)
    current_confidence_matrix = sigmoid(current_attenuation_matrix, threshold, grad)

    return r_true, theta_noise, y_el, y_er, y_x, y_y, y_el_vec, y_er_vec, current_obs_goback_dist_matrix_L, current_obs_goback_dist_matrix_R, current_confidence_matrix
