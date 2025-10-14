# -*- coding: utf-8 -*-
"""
Created on Tue May 30 19:31:55 2023

@author: yasuf
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from scipy.special import jv

import pandas
import math

import matplotlib.animation as animation
from matplotlib.colors import LogNorm
import os
import copy

from numba import jit

#記憶の再実装
# P(x|yn)=(P(yn|x)**S)*P(x)/P(y) s→0,1で考えてみな（Sはシグモイド関数）
#ただし、P(yn|x)もSも0に行くので収束スピード勝負にSが負けるとバグる
#なのでP(yn|x)は現在clipしている(これが、見え方を決行悪くするので理想的にはすごく小さくしたい)


#### 20240624 できたこと ####

# アンダーフロー問題解決
#　カラー調整もブラッシュアップ（最小閾値にへばりつく下限確率達を排除し、最小値を探索（color_min）、その最小値以上の中から上位X％の値を(color_max）

### 20240624 やること #####
#無指向性(極限閾値)でテスト→OK



### 余裕があれば ####
#   ポテンシャルモデルも入れてみる


##### カラー設定 #######################
c_percentile = 95 #10 #75 #95 #カラー中心を決める(分位点%[0~100])　(カラーの白黒割合を決める)


threshold = 1.2*0.1**3 #1.2*0.1**3 #エコー検知最小感度　(さらに、conf1の半値境目)
grad = 10**15 #2 * 10**3 #10**3(conf1の勾配)
threshold_conf = 0.5 #conf2_のx軸の半値境目（conf1の値の閾値）


################## 2024_6月に達成したこと ######################################
#   実データの取り込み
#   エコー強度によるばらつきの実装(おそらくOK)
#   自信度confidenceを用いた表現拡張 #   観測信号の一次加工（予想されるエコー強度から、情報の信頼性を多々見込むイメージ）
#   →記憶の消失防止

#   観測信号yをガウス分布を仮定し、yの空間分布表現に拡張 #空間尤度関数への表現拡張(計算が簡便化)
#   0_underflowが起きるのでlogで計算＋描画(OK)

#   ノイズ付きtau(tau: y_el, y_er)
#   厳密距離d (d: obs_goback_dist_matrix_L, obs_goback_dist_matrix_R)
#   2次元尤度(Pyn_x_L, Pyn_x_R)


#   カラーの良いレンジを探す
    #n分位点を実装することで見栄えが良くなった（最上位数パーセントの表示で濃淡がよく見える）
    # gradの急峻さ調整（そもそも検知を1,0で制御しているので急峻であればあるほど整合性は取れる）

######## #20240613(今は必要ない懸案事項) ######################
#   指向性調整！！(90°こえで1/inf)口が大きければ90度で0に落ちないので強制的に切る

#裾野の所が広がってしまう。。。モデル修正案
# conf 1 * P1 +(1-conf1)* np.max(P1) でもいいのでは？→すそ野がどうなるか注意は必要

#########################################################

#最尤推定の参考HP
#https://math-note.xyz/statistics/maximum-likelihood-estimation/



##### 2次元拡張 #######################
c = 340 #音速　[m/s]

##### コウモリ設定 ##############
ear_dist = 0.02 #耳間距離[m]
a = 0.005#0.005 #mouth radius size
freq = 40000 #40000


###### コウモリ実データ用 #####################
d_to_wall_x=0.24#x軸の補正（フレームから左の壁まで[m]）
d_to_wall_y=0.28#y軸の補正（フレームから奥の壁まで[m])
fps = 100 #[Hz]


######## 時間設定 ###################
t_max =64*10**-3 #[s] #エコー最大時間レンジ
dt = 100*10**-6 #[s] #エコー時間刻み幅
Mt = int(np.round(t_max/dt)) #時間刻み数
t_ax = np.linspace(0, t_max, Mt+1)

######## 空間設定 ###################
x_max = y_max = 5.5 ##最大距離 [m](左右上下マージンがここから各0.5m)
h=0.01 #空間刻み幅 dy
Mx = My = int(np.round(y_max / h)) #空間刻み数
y_ax = x_ax = np.linspace(0, y_max, My+1) #描画用縦軸・横軸刻み
margin_space = 0.5
######### 空間構築 #####################
x_axis=np.linspace(0, x_max, int(x_max / h)+1)
y_axis=np.linspace(0, y_max, int(y_max / h)+1)
X, Y = np.meshgrid(x_axis, y_axis)


######## 障害物位置・揺らぎ量の設定 ###################

##### 閉壁面 ##############################
wall_x = np.array([margin_space, x_max-margin_space])
wall_y = np.array([margin_space, y_max-margin_space])

wall_corner_x, wall_corner_y = np.meshgrid(wall_x, wall_y)

wall_corner_x = wall_corner_x.flatten()
wall_corner_y = wall_corner_y.flatten()

##### ポール ##############################

def real_obs():
    chain_loc=pandas.read_csv(f'chain_position_y.csv',header=0)
#    chain_loc=pandas.read_csv(f'chain_position_y.csv',header=0,index_col=0)
#    chain_loc=pandas.read_csv(f'chain_position.csv',header=6,index_col=0).T
    #chain_loc=pandas.read_csv(f'D:/{fname}/chain_position.csv',header=6,index_col=0)
    obs_x = chain_loc['X'].values + margin_space #pd.seriesだと後ろでエラー吐いたためnparrayで
    obs_y = chain_loc['Z'].values + margin_space#pd.seriesだと後ろでエラー吐いたためnparrayで

    return np.array(obs_x), np.array(obs_y)
            
pole_x, pole_y = real_obs()
#pole_x = np.array([3, 4]) ####正解obs座標x
#pole_y = np.array([3, 4.5]) ####正解obs座標y


###### コウモリ座標計算 #############################

bat_x0 = 2   #初期x_座標
bat_y0 = 2.2 #初期y_座標

bat_xe = 2 #1.4142 #1.4142 #1 #終期x_座標
bat_ye = 2 #2 #3 #終期y_座標

speed = 4 #[m/s]
IPI=0.1 #[s]


print("path making...")

def real_flight():
#    bat_loc = pandas.read_csv(f'601_no1.csv',header=6,index_col=0,usecols=range(0,5)) #コウモリ全軌跡
#    bat_loc = pandas.read_csv(f'D:/{fname}/{bat_num}/{bat_num}_no1.csv',header=6,index_col=0,usecols=range(0,5)) #コウモリ全軌跡
#    bat_loc['X'] = bat_loc['X'] + d_to_wall_x + margin_space
#    bat_loc['Z'] = bat_loc['Z'] + d_to_wall_y + margin_space
    pulse_timing = pandas.read_csv(f'601_pulse_timing_500.csv',index_col=0) #パルスタイミングとその座標
#    pulse_timing = pandas.read_csv(f'601_pulse_timing_cut.csv',index_col=0) #パルスタイミングとその座標
#    pulse_timing = pandas.read_csv(f'D:/{fname}/{bat_num}/{bat_num}_pulse_timing.csv',index_col=0) #パルスタイミングとその座標
    trials = len(pulse_timing) #放射回数
    bat_x = pulse_timing['x'].values #pd.seriesだと後ろでエラー吐いたためnparrayで
    bat_y = pulse_timing['y'].values
    
    bat_x_pre = np.roll(bat_x, 1)
    bat_y_pre = np.roll(bat_y, 1)
    head_direc = np.arctan2(bat_y-bat_y_pre, bat_x-bat_x_pre)
    head_direc[0] = head_direc[1]
    head_direc_deg = np.rad2deg(head_direc)
    fd = head_direc_deg # flight direction [deg]
    pd = head_direc_deg # pulse direction [deg]

    return trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd



def straight_flight_to_goal(bat_x0, bat_y0, bat_xe, bat_ye):
    head_direc = np.arctan2(bat_ye - bat_y0, bat_xe - bat_x0) 
    head_direc_deg = np.rad2deg(head_direc) 
    fd = head_direc_deg # flight direction [deg]
    pd = head_direc_deg # pulse direction [deg]
    
    trials = int(np.round(np.sqrt((bat_xe-bat_x0)**2 + (bat_ye-bat_y0)**2)/(speed*IPI)))+1

    epsilon=0.000001
    if bat_x0 == bat_xe:
        bat_x = np.linspace(bat_x0, bat_xe + epsilon, trials)
    else:    
        bat_x = np.linspace(bat_x0, bat_xe, trials)

    if bat_y0 == bat_ye:
        bat_y = np.linspace(bat_y0, bat_ye + epsilon, trials)
    else:
        bat_y = np.linspace(bat_y0, bat_ye, trials)
    return trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd


def hovering(bat_x0, bat_y0, trials):
    head_direc = np.arctan2(bat_ye - bat_y0, bat_xe - bat_x0) 
    head_direc_deg = np.rad2deg(head_direc) 
    fd = head_direc_deg # flight direction [deg]
    pd = head_direc_deg # pulse direction [deg]

    bat_x = np.ones(trials) * bat_x0

    bat_y = np.ones(trials) * bat_y0
    return trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd

def round_angle(angle_rad_matrix):
    angle_deg_matrix = np.rad2deg(angle_rad_matrix)
    angle_fit = (angle_deg_matrix-180) // 360 +1
#    angle_fit = np.where(angle_fit <= -1, angle_fit+1, angle_fit)
    angle_deg_matrix_improve = angle_deg_matrix-angle_fit * 360
    return np.deg2rad(angle_deg_matrix_improve)

def circular_flight_origin_pole(bat_x0, bat_y0, flight_distance):
    trials = int(np.round(flight_distance/(speed*IPI)))+1
    theta_init= np.arctan2(bat_y0-pole_y[0], bat_x0-pole_x[0])
    
    r = np.sqrt((pole_x[0]-bat_x0)**2 + (pole_y[0]-bat_y0)**2)
    theta_circ = flight_distance/r
    theta_array = np.linspace(theta_init, theta_init + theta_circ, trials)
    
    head_direc = round_angle(theta_array + np.pi/2)
    head_direc_deg = np.rad2deg(head_direc)

    fd = head_direc_deg # flight direction [deg]
    pd = head_direc_deg # pulse direction [deg]

    bat_x = pole_x[0] + r * np.cos(theta_array)
    bat_y = pole_y[0] + r * np.sin(theta_array)
    return trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd

#trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd = straight_flight_to_goal(bat_x0, bat_y0, bat_xe, bat_ye)
#trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd = hovering(bat_x0, bat_y0, 10)
#trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd = circular_flight_origin_pole(bat_x0, bat_y0, 60.1) #250.1 #15
trials, bat_x, bat_y, head_direc, head_direc_deg, fd, pd = real_flight()

###### （パルス・飛行）方向印字関数 #############
def direc_arrow(bat_x, bat_y, direc):
    arrow_length = 1
    arrow_end_x = arrow_length * np.cos(np.deg2rad(direc)) + bat_x
    arrow_end_y = arrow_length * np.sin(np.deg2rad(direc)) + bat_y
    return arrow_end_x, arrow_end_y

pulse_x, pulse_y = direc_arrow(bat_x, bat_y, pd)
body_x, body_y = direc_arrow(bat_x, bat_y, fd)

###### 壁面オブジェクトも加味した真の障害物反射座標 obs_x, obs_y の計算 #####################
print("reflection_point calc...")

wall_x_det_temp = copy.deepcopy(bat_x)
wall_y_det_temp = copy.deepcopy(bat_y)


def wall_ref_point(x, y):
    X_ref, Y_ref = np.meshgrid(x, y)
    return X_ref, Y_ref

wall_x_det_UD, wall_y_det_UD = wall_ref_point(wall_x_det_temp, wall_y)
wall_x_det_LR, wall_y_det_LR = wall_ref_point(wall_x, wall_y_det_temp)

wall_x_det_UD = wall_x_det_UD.T
wall_y_det_UD = wall_y_det_UD.T

obs_x = np.hstack((pole_x, wall_corner_x)) ####正解obs座標x
obs_y = np.hstack((pole_y, wall_corner_y)) ####正解obs座標y
obs_x = np.tile(obs_x, (len(bat_x), 1))
obs_y = np.tile(obs_y, (len(bat_y), 1))

obs_x = np.hstack((obs_x, wall_x_det_UD, wall_x_det_LR)) ####正解obs座標x
obs_y = np.hstack((obs_y, wall_y_det_UD, wall_y_det_LR)) ####正解obs座標y

obs_n = obs_x.shape[1]

####### ターゲット極座標（r_theta）の計算 ###########

def XY_to_r_theta_calc(bat_x, bat_y, obs_x, obs_y, pd):
    BX = np.tile(bat_x, (np.shape(obs_x)[1], 1)).T
    BY = np.tile(bat_y, (np.shape(obs_y)[1], 1)).T
    OX = obs_x
    OY = obs_y
    
    r_true = np.zeros_like(OX, dtype=float) #(12,2)
    theta_true = np.zeros_like(OX, dtype=float) #(12,2)
    print("len(OX):", len(OX))
    print("len(OX[0]:", len(OX[0]))
    del_X=OX-BX
    del_Y=OY-BY
    r_true = np.linalg.norm(np.array([del_X, del_Y]), axis=(0))
#    r_true = np.linalg.norm(np.array([del_X[:,:], del_Y[:,:]]), axis=(0))
    pd = np.tile(pd,(del_X.shape[1],1)).T
    theta_true = round_angle(np.arctan2(del_Y, del_X) - np.deg2rad(pd))

    return r_true, theta_true

r_true, theta_true = XY_to_r_theta_calc(bat_x, bat_y, obs_x, obs_y, pd)
theta_true_deg = np.rad2deg(theta_true) 

##### エコー音圧計算（obs用） #############################
print("attenuation calc...")
def dist_attenuation(r):
    print("check20")
    r0=h/2#基準距離[m]
    print("check21")
    return r0/r

def direc_attenuation(theta):
    print("check22")
    theta_clip = np.clip(theta, -np.pi/2, np.pi/2)
    print("check23")
    k=2*np.pi*freq/c
    print("check24")
    beam_pattern = abs(2*jv(1, k*a*np.sin(theta_clip))/(k*a*np.sin(theta_clip))) #ピストンモデル(再度ローブ表現のためにabs必要)
    print("check25")
    return beam_pattern

attenuation_obs = dist_attenuation(r_true)*direc_attenuation(theta_true)

##### 検知ポール判定 ####################
print("detection echo_pre_judging...")

def detection_judge(attenuation, trans_info):
    idx_thres_lower = np.where(attenuation < threshold)
    trans_info[idx_thres_lower]='NaN'
    echo_n = np.sum((attenuation> threshold), axis=1)
    return trans_info, echo_n

r_true, echo_n = detection_judge(attenuation_obs, r_true)
theta_true, echo_n = detection_judge(attenuation_obs, theta_true)
theta_true_deg = np.rad2deg(theta_true) 

#### ターゲットの厳密往復距離の計算　###################################
print("distance_calc...")

def ear_posit(bat_x, bat_y, pd):
    pd_rad_L = np.deg2rad(pd+90)
    pd_rad_R = np.deg2rad(pd-90)
    
    ear_xL = (ear_dist/2)*np.cos(pd_rad_L) + bat_x
    ear_yL = (ear_dist/2)*np.sin(pd_rad_L) + bat_y
    ear_xR = (ear_dist/2)*np.cos(pd_rad_R) + bat_x
    ear_yR = (ear_dist/2)*np.sin(pd_rad_R) + bat_y
    return ear_xL, ear_yL, ear_xR, ear_yR 

earL_x, earL_y, earR_x, earR_y = ear_posit(bat_x, bat_y, pd)

def real_dist_goback(speaker_x, speaker_y, ear_x, ear_y, obs_x, obs_y):
    
    def measure_dist(x0, y0, x1, y1):
        BXS = np.tile(x0, (x1.shape[-1],1)).T
        BYS = np.tile(y0, (y1.shape[-1],1)).T
        SX = x1
        SY = y1
    
        a = np.array([BXS, BYS])
        b = np.array([SX, SY])

        vec = b-a

        dist = np.linalg.norm(vec, axis = 0)
        return dist

    go_dist = measure_dist(speaker_x, speaker_y, obs_x, obs_y)
    back_dist = measure_dist(ear_x, ear_y, obs_x, obs_y)

    return go_dist + back_dist

obs_goback_dist_L = real_dist_goback(bat_x, bat_y, earL_x, earL_y, obs_x, obs_y)
obs_goback_dist_R = real_dist_goback(bat_x, bat_y, earR_x, earR_y, obs_x, obs_y)

######### 理想エコー遅れ時間の計算 ########################## 
def space_echo_translater(goback_distL, goback_distR):
    tl = goback_distL / c
    tr = goback_distR / c

    return tl, tr

y_el_true, y_er_true = space_echo_translater(obs_goback_dist_L, obs_goback_dist_R)

####### ターゲットの極座標依存ノイズ（r_noise, theta_noise）の計算 ###########################

##### 要検討 #############################
#test_dB_atte = 20*np.log10(attenuation_obs)

k_r_noise = 60000#600 #(大きい方がばらつき小さい)
k_theta_noise = 360000#3600 #(大きい方がばらつき小さい)

##### 真の揺らぎ（神のみぞ知る） ##############################
r_noise_rate = -20 * np.log10(attenuation_obs) / k_r_noise #0.1 #標準偏差
theta_noise_rate = -20 * np.log10(attenuation_obs) / k_theta_noise #0.01 #theta_true*0.05 #標準偏差

r_noise = np.ones_like(r_true)*r_noise_rate #標準偏差
theta_noise = np.ones_like(r_noise)*theta_noise_rate#(theta_true-pd)*0.05 #標準偏差

########## 観測信号一気に作成 ################
print("noisy signal making...")

def gauss_noise(mu, sigma, trials):
    if mu.ndim>1:
        MU=mu.T
        SIGMA=sigma.T
        y=np.zeros((len(MU), trials))
        for obs_n in range(len(MU)):
            y[obs_n] = np.random.normal(loc=MU[obs_n], scale = SIGMA[obs_n], size=trials)
    else:
        y = np.random.normal(loc=mu, scale = sigma, size=trials)
    return y.T

noisy_dist = gauss_noise(r_true, r_noise, trials) #trials 回 センシング分の揺らぎ付き距離情報 y_dist
noisy_theta_rad = gauss_noise(theta_true, theta_noise, trials) #trials 回 センシング分の揺らぎ付き方位情報 y_dist
noisy_theta_deg = np.rad2deg(noisy_theta_rad)

def r_theta_to_XY_calc(obs_r, obs_theta_deg, bat_x, bat_y, pd):
    pd = np.tile(pd, (obs_theta_deg.shape[-1],1)).T
    bat_x = np.tile(bat_x, (obs_theta_deg.shape[-1],1)).T
    bat_y = np.tile(bat_y, (obs_theta_deg.shape[-1],1)).T

    abs_theta = obs_theta_deg + pd
    X = obs_r * np.cos(np.deg2rad(abs_theta)) + bat_x
    Y = obs_r * np.sin(np.deg2rad(abs_theta)) + bat_y
    return X, Y

noisy_obs_x, noisy_obs_y = r_theta_to_XY_calc(noisy_dist, noisy_theta_deg, bat_x, bat_y, pd) #ノイズ付き　obs_X,Y座標

noisy_goback_L = real_dist_goback(bat_x, bat_y, earL_x, earL_y, noisy_obs_x, noisy_obs_y) #ノイズ付き厳密距離計算
noisy_goback_R = real_dist_goback(bat_x, bat_y, earR_x, earR_y, noisy_obs_x, noisy_obs_y) #ノイズ付き厳密距離計算


######### 時間情報に変換 #####################
y_el, y_er = space_echo_translater(noisy_goback_L, noisy_goback_R) #左右エコータイミング情報に変換（計算にはこれを使う）

########### 初期　事前確率P(x)作成　###############################
print("prior initialization...")
def prior(mu, sigma, x, h):
    if np.isscalar(mu): #単発センシング用
        p = norm.pdf(x, mu, sigma)*h
    else:
        p=np.zeros((len(mu),len(x)))
        if mu.ndim == 1:
            for i in range(len(mu)):
                p[i] = norm.pdf(x, mu[i], sigma[i])*h
        elif mu.ndim == 2:
            mu_size = mu.shape
            for j in range(mu_size[0]):
                for i in range(mu_size[1]):
                    p[j] = p[j] + norm.pdf(x, mu[j,i], sigma[j,i])*h                        
    return p

#Px_horiz = prior(x_max/2, x_max/2,x_ax, h)
#Px_vert = prior(y_max/2, y_max/2, y_ax, h)
#Px_h, Px_v = np.meshgrid(Px_horiz, Px_vert)
#Px = Px_h * Px_v

Px = np.ones((Mx+1, My+1))


######### ここから本来 loop 計算 ####################################
print("loop start...")
#### 極座標マトリックス作成 #####################
def r_theta_matrix(bx_vec, by_vec, space_x, space_y, pd_vec):
    print("check0")
    r_2vec = np.zeros((trials, Mx+1, My+1))
    theta_2vec = np.zeros((trials, Mx+1, My+1))
    i=0
    for bx, by, pd in zip(bx_vec, by_vec, pd_vec):
        del_x = space_x-bx
        del_y = space_y-by
        r_2vec[i] = np.sqrt(del_x**2 + del_y**2)
        theta_2vec[i] = np.arctan2(del_y, del_x) - np.deg2rad(pd)
        i=i+1
    
    print("check01")
    theta_2vec_pi = np.where(theta_2vec > np.pi, theta_2vec-2*np.pi, theta_2vec) 
    theta_2vec_pipi = np.where(theta_2vec_pi < -np.pi, theta_2vec_pi + 2*np.pi, theta_2vec_pi) 
    print("check02")
    return r_2vec, theta_2vec_pipi

print("matrix making...")

r_2vec, theta_2vec_rad = r_theta_matrix(bat_x, bat_y, X, Y, pd)
theta_2vec_deg = np.rad2deg(theta_2vec_rad)

print("check1")

#### 厳密距離マトリックスの作成 #####################
#@jit(cache=True)

def real_dist_goback_matrix(speaker_x, speaker_y, ear_x, ear_y, space_x, space_y):
    
    def measure_dist(x0, y0, x1, y1):
        print("check10")
        dist = np.zeros((trials, Mx+1, My+1))
        i=0
        for bx, by in zip(x0, y0):
            del_x = space_x-bx
            del_y = space_y-by
            dist[i] = np.sqrt(del_x**2 + del_y**2)
            i=i+1
        print("check11")

        return dist

    go_dist = measure_dist(speaker_x, speaker_y, space_x, space_y)
    back_dist = measure_dist(ear_x, ear_y, space_x, space_y)

    return go_dist + back_dist

obs_goback_dist_matrix_L = real_dist_goback_matrix(bat_x, bat_y, earL_x, earL_y, X, Y)
obs_goback_dist_matrix_R = real_dist_goback_matrix(bat_x, bat_y, earR_x, earR_y, X, Y)
print("check2")

####### エコー音圧matrix計算　################################
attenuation_matrix = dist_attenuation(r_2vec)*direc_attenuation(theta_2vec_rad)
print("check3")


###### sigmoid関数に通して、自信度へ変換　##########
def sigmoid(x, center, grad):
    return 1/(1+np.exp(-4*grad*(x-center)))

confidence_matrix = sigmoid(attenuation_matrix,threshold, grad)
print("check4")


###### sigmoid関数に通して、自信度へ変換2　##########
#アンダーフローを抑えた（上の関数と答えは一致しない）（p<S(x)<p_max）
def optimised_sigmoid_v2(x, center, bias_rate, value_of_zero_cross, p_max):
    p = value_of_zero_cross #value of x = 0
    alpha = bias_rate * p # must be 0 < bias_rate < 1 (alpha < p).

    am = center
    beta = p-alpha
    mu = am*(np.log(p_max-p)-np.log(beta))/(np.log(p_max+p-2*alpha)-np.log(beta))
    
    opt_sig = alpha + (p_max-alpha) / (1 + ((p_max-p)/beta)**(-(x-mu)/mu))
    opt_sig = np.nan_to_num(opt_sig, nan = p_max)
    return opt_sig

################## 20240624_更新_解析解ベイズ（logで計算、アンダーフロー改善） #################################

### P(tau_n|d(x,y)) すなわち　P(yn|x,y)の計算
#sigma2 = (0.001*c)**2 #0.001 **2  # 小原君と一緒(弄らない。ただし、向こうは標準偏差でこっちは分散で定義) #0.000003 #0.000001 #尤度（エコー時間）の分散
sigma2 = (0.0001*c)**2 #0.001 **2  # 小原君と一緒(弄らない。ただし、向こうは標準偏差でこっちは分散で定義) #0.000003 #0.000001 #尤度（エコー時間）の分散

def new_likelyhood_2D(tau_n, d, sigma2):

    Tau_n, D= np.meshgrid(tau_n, d)
    Tau_n = np.reshape(Tau_n, (d.shape[0], d.shape[1], tau_n.shape[0])) 
    D = np.reshape(D, (d.shape[0], d.shape[1], tau_n.shape[0])) 
    
    Pyn_2Dxy_each = np.nan_to_num(1/np.sqrt(2*np.pi*sigma2)*np.exp(-(c*Tau_n-D)**2/(2*sigma2)))
#    Pyn_2Dxy_each = np.nan_to_num(1/np.sqrt(2*np.pi*sigma2)*np.exp(-(Tau_n-D/c)**2/(2*sigma2)))
#    Pyn_2Dxy_sum = np.sum(Pyn_2Dxy_each, axis = 2) #使わない
    return Pyn_2Dxy_each

def dB_trans(data):
    return np.log10(data)

#事前分布
Px2L_log = dB_trans(Px)
Px2R_log = dB_trans(Px)
min_p = -320

#事前分布（記憶加工用）
Px3L_log = dB_trans(Px) #事前分布の初期化
Px3R_log = dB_trans(Px) #事前分布の初期化

#尤度関数
Pyn_x_L = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱
Pyn_x_R = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱

#事後分布(log)
Px_yn_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱
Px_ynL_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱
Px_ynR_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱

#記憶加工(尤度, log)
Pyn_x_conf_L_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱
Pyn_x_conf_R_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱

#記憶加工（事後分布(log)）
Px_ynL_conf_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱
Px_ynR_conf_log = np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱
Px_yn_conf_log =  np.zeros((trials, Mx+1, My+1)) #センシング回数分入れる箱

for i in range(trials):
    print(f"new_calc_sensing: {i}")
    Pyn_x_L_each = new_likelyhood_2D(y_el[i][~np.isnan(y_el[i])], obs_goback_dist_matrix_L[i], sigma2).transpose(2,0,1) #2次元空間尤度
    Pyn_x_R_each = new_likelyhood_2D(y_er[i][~np.isnan(y_er[i])], obs_goback_dist_matrix_R[i], sigma2).transpose(2,0,1) #2次元空間尤度

    Pyn_x_L[i] = np.clip(np.sum(Pyn_x_L_each, axis=0), 0.1**20, None) # 尤度関数　描画用　
    Pyn_x_R[i] = np.clip(np.sum(Pyn_x_R_each, axis=0), 0.1**20, None) # 尤度関数　描画用　
    
    PxynL_log = dB_trans(Pyn_x_L[i]/np.max(Pyn_x_L[i])) + Px2L_log  # 同時分布　描画用
    PxynR_log = dB_trans(Pyn_x_R[i]/np.max(Pyn_x_R[i])) + Px2R_log  # 同時分布　描画用
    PxynL_log = np.clip(PxynL_log, min_p, None) #アンダーフローを止める
    PxynR_log = np.clip(PxynR_log, min_p, None) #アンダーフローを止める
    PxynL_temp = np.where(PxynL_log > min_p, 10**(PxynL_log), 0) #アンダーフローぎりぎりの奴は0として扱う
    PxynR_temp = np.where(PxynR_log > min_p, 10**(PxynR_log), 0) #アンダーフローぎりぎりの奴は0として扱う
    PxynL_log_sum = np.log10(np.sum(PxynL_temp))
    PxynR_log_sum = np.log10(np.sum(PxynR_temp))
    Px_ynL_log[i] = PxynL_log - PxynL_log_sum
    Px_ynR_log[i] = PxynR_log - PxynR_log_sum
    
    Px2L_log = copy.deepcopy(Px_ynL_log[i]) #事前分布へ代入
    Px2R_log = copy.deepcopy(Px_ynR_log[i]) #事前分布へ代入
    Px_yn_log[i] = Px_ynL_log[i] + Px_ynR_log[i] #独立を仮定した左右同時事後確率
    

    ### 記憶保持加工(同時確率) ################################
    PxynL_conf_log = confidence_matrix[i]*dB_trans(Pyn_x_L[i]/np.max(Pyn_x_L[i])) + Px3L_log # 同時分布(confが0ならPxしか効かないのがみそ)
    PxynR_conf_log = confidence_matrix[i]*dB_trans(Pyn_x_R[i]/np.max(Pyn_x_R[i])) + Px3R_log # 同時分布(confが0ならPxしか効かないのがみそ)
    PxynL_conf_log = np.clip(PxynL_conf_log, min_p, None) #アンダーフローを止める
    PxynR_conf_log = np.clip(PxynR_conf_log, min_p, None) #アンダーフローを止める
    PxynL_conf_temp = np.where(PxynL_conf_log > min_p, 10**(PxynL_conf_log), 0) #アンダーフローぎりぎりの奴は0として扱う
    PxynR_conf_temp = np.where(PxynR_conf_log > min_p, 10**(PxynR_conf_log), 0) #アンダーフローぎりぎりの奴は0として扱う
    PxynL_conf_log_sum = np.log10(np.sum(PxynL_conf_temp))
    PxynR_conf_log_sum = np.log10(np.sum(PxynR_conf_temp))
    
    ### 記憶保持加工(事後分布) ################################
    Px_ynL_conf_log[i] = PxynL_conf_log - PxynL_conf_log_sum
    Px_ynR_conf_log[i] = PxynR_conf_log - PxynR_conf_log_sum

    Px3L_log = copy.deepcopy(Px_ynL_conf_log[i]) #事前分布へ代入
    Px3R_log = copy.deepcopy(Px_ynR_conf_log[i]) #事前分布へ代入
    Px_yn_conf_log[i] = Px_ynL_conf_log[i] + Px_ynR_conf_log[i] #独立を仮定した左右同時事後確率(記憶あり)



#######################################################################################################################

#これも正直いらない。描画用にだけ使う
####### t_idx, y(t)の(0,1)_vec の作成 #################
eps_y=0.0001 #微小入力バイアス

def index_calc(y, attenuation):
    i = np.round(y/dt).astype(int)+1
    if np.isscalar(y): #単発センシング用
        y_array = np.ones(len(t_ax))*eps_y
        y_array[i] = 1
    else:
        print("y_dim = 1,  sensing_length is     :",len(y)) #センシング回数
        y_array = np.zeros((len(y), len(t_ax)))
        for n in range(len(i)):
            i_over = np.where(i[n]>= 0)
            y_array[n][i[n][i_over]] = 1
    return i, y_array

i_y_el, y_el_vec = index_calc(y_el, attenuation_obs)
i_y_er, y_er_vec = index_calc(y_er, attenuation_obs)




######### fig_data_select ################################

data1 = Pyn_x_L
data2 = confidence_matrix
data3 = Px_yn_log
data4 = Px_yn_conf_log


################ fig作成 ############################
fig = plt.figure(figsize=(36,18), constrained_layout=False)
plt.rcParams["font.size"] = 18
ax_y = fig.add_subplot(3,4,(1,5)) #実topview データ
ax_y2 = fig.add_subplot(3,4,(2,6)) #実topview データ
ax_y3 = fig.add_subplot(3,4,(3,7)) #実topview データ
ax_y4 = fig.add_subplot(3,4,(4,8)) #実topview データ
ax_py_xL = fig.add_subplot(615) #エコーデータL
ax_py_xR = fig.add_subplot(616) #エコーデータR
#ax_px_y= fig.add_subplot(313)


######### ax_y (Top view figure) #################################


if noisy_dist.ndim == 2:
#    bat_x_array = np.tile(bat_x, (2, 1))
#    bat_y_array = np.tile(bat_y, (2, 1))
    bat_x_array = np.tile(bat_x, (noisy_dist.shape[-1], 1)).T
    bat_y_array = np.tile(bat_y, (noisy_dist.shape[-1], 1)).T
    pd_array =  np.tile(pd, (noisy_dist.shape[-1], 1)).T
elif noisy_dist.ndim == 1:
    bat_x_array = bat_x
    bat_y_array = bat_y
    pd_array = pd
    
y_x = bat_x_array + noisy_dist * np.cos(noisy_theta_rad + np.deg2rad(pd_array))
y_y = bat_y_array + noisy_dist * np.sin(noisy_theta_rad + np.deg2rad(pd_array))

#test=data1[0][np.nonzero(data1[0])]
c_min1 = np.min(data1[0])
c_max1 = np.percentile(data1[0][np.nonzero(data1[0])], q=c_percentile)
c_min2 = np.min(data2[0])
c_max2 = np.percentile(data2[0][np.nonzero(data2[0])], q=c_percentile)
c_min3 = np.min(data3[0][data3[0] > 2*min_p])
c_max3 = np.percentile(data3[0][data3[0] > c_min3], q=c_percentile)
c_min4 = np.min(data4[0][data4[0] > 2*min_p])
#c_max4 = np.percentile(data4[0][data4[0] > c_min4], q=c_percentile)
data_temp4 = data4[0][data4[0] > c_min4]
c_max4 = np.percentile(data_temp4[data_temp4 < np.max(data_temp4)], q=c_percentile)

px_y_draw =  ax_y.pcolormesh(X, Y, data1[0], cmap="GnBu", vmin = c_min1, vmax = c_max1) #, norm=Normalize(vmin=0, vmax=1)
px_y2_draw = ax_y2.pcolormesh(X, Y, data2[0], cmap="GnBu", vmin = c_min2, vmax = c_max2) #, norm=Normalize(vmin=0, vmax=1)
px_y3_draw = ax_y3.pcolormesh(X, Y, data3[0], cmap="GnBu", vmin = c_min3, vmax = c_max3) #, norm=Normalize(vmin=0, vmax=1)
px_y4_draw = ax_y4.pcolormesh(X, Y, data4[0], cmap="GnBu", vmin = c_min4, vmax = c_max4) #, norm=Normalize(vmin=0, vmax=1)

ax_y.set_title("likely_hood_newL")
ax_y2.set_title("confidence_matrix")
ax_y3.set_title("posterior(without mmemory)")
ax_y4.set_title("posterior(with mmemory)")

def pcolor_draw_basic(ax_name, p_draw_name):
    ax_name.set_xlim([0, x_max])
    ax_name.set_ylim([0, y_max])
    ax_name.set_xlabel("X-axis")
    ax_name.set_ylabel("Y-axis")
    ax_name.set_aspect('equal')
    
    bat_draw, = ax_name.plot(bat_x[0], bat_y[0], 'ko', markersize=20, label="bat")
    pole_draw, = ax_name.plot(pole_x, pole_y, 'ro', markersize=25, markerfacecolor='None', markeredgecolor='r', label="pole")
    fd_draw, = ax_name.plot(np.array([bat_x[0], body_x[0]]), np.array([bat_y[0], body_y[0]]), 'r-', markersize=20, label="fd")
    pd_draw, = ax_name.plot(np.array([bat_x[0], pulse_x[0]]), np.array([bat_y[0], pulse_y[0]]), 'k-', markersize=20, label="pd")
    pp = fig.colorbar(p_draw_name, ax = ax_name, orientation="vertical")
    
    global wall_x_draw, wall_y_draw 
    wall_x_draw_id = np.array([0,0,1,1,0])
    wall_y_draw_id = np.array([0,1,1,0,0])
    wall_x_draw = wall_x[wall_x_draw_id]
    wall_y_draw = wall_y[wall_y_draw_id]
    wall_draw, = ax_name.plot(wall_x_draw, wall_y_draw, 'k-', linewidth=2, label="wall")
    
    y_space_draw, = ax_name.plot(y_x[0], y_y[0], 'm+', markersize=20, markeredgewidth=5, label=r"observation $y$")
    y_space_draw_past, = ax_name.plot(y_x[0], y_y[0], '+',color="lightgrey", markersize=20, markeredgewidth=5, alpha = 0.02,  label=r"$y_{past}$")

    return ax_name, bat_draw, pole_draw, fd_draw, pd_draw, wall_draw, y_space_draw, y_space_draw_past, pp

ax_y,  bat_draw,  pole_draw,  fd_draw,  pd_draw,  wall_draw,  y_space_draw,  y_space_draw_past,  pp = pcolor_draw_basic(ax_y, px_y_draw)
ax_y2, bat2_draw, pole2_draw, fd2_draw, pd2_draw, wall2_draw, y_space2_draw, y_space2_draw_past, pp2 = pcolor_draw_basic(ax_y2, px_y2_draw)
ax_y3, bat3_draw, pole3_draw, fd3_draw, pd3_draw, wall3_draw, y_space3_draw, y_space3_draw_past, pp3 = pcolor_draw_basic(ax_y3, px_y3_draw)
ax_y4, bat4_draw, pole4_draw, fd4_draw, pd4_draw, wall4_draw, y_space4_draw, y_space4_draw_past, pp4 = pcolor_draw_basic(ax_y4, px_y4_draw)

#pp.set_label("probability", fontname="Arial", fontsize=20)
#pp2.set_label("Confidence", fontname="Arial", fontsize=20)


sensing_time = ax_y.text(0.1, 0.8, f"n = 0", transform = ax_y.transAxes)
#setting = ax_y.text(0.02, 1.1, r"2D model, $x_{true}(t) = (r, \theta), \qquad y^{space}_n(t) = x_{true}(r+\epsilon_{r}, \theta + \epsilon_{\theta}) \qquad$" + r"$ \epsilon_r \sim N(0,$ {Noise:.2}$(SD))\qquad$".format(Noise=r_noise_rate) + r"$ \epsilon_\theta \sim N(0,$ {thetaNoise:.2}$(SD))$".format(thetaNoise=theta_noise_rate), fontsize=22, transform = ax_y.transAxes)

#ax_y.legend(loc="best")
#ax_y.legend(loc=(0.05, 0.05))
#ax_y.legend(loc=loc, bbox_to_anchor=(0.05, 0, 0.05, 0.1))

######### ax_y (echo_arrival_timing figure) #################################
def t_seaquence_draw_basic(ax_name, y_e_vec, label_name):
    ax_name.set_xlim([0, 10])
    ax_name.set_ylim([0, 1])
    ax_name.set_ylabel(label_name)
    y_echo_past_draw, = ax_name.plot(t_ax*10**3, y_e_vec[0], 'c-', markersize=20, label=r"$y^{echo}_{past}$")
    y_echo_draw, = ax_name.plot(t_ax*10**3, y_e_vec[0], 'm-', markersize=20, label=r"$y^{echo}$")
    ax_name.legend(loc = "best")
    return ax_name, y_echo_past_draw, y_echo_draw

ax_py_xL, y_echo_L_past_draw, y_echo_L_draw = t_seaquence_draw_basic(ax_py_xL, y_el_vec, "Left received echo")
ax_py_xR, y_echo_R_past_draw, y_echo_R_draw = t_seaquence_draw_basic(ax_py_xR, y_er_vec, "Right received echo")
ax_py_xR.set_xlabel("time [ms]")


##############################################################

def init(): # only required for blitting to give a clean slate. 
    y_space_draw_past.set_data(y_x[0], y_y[0])
    y_space_draw.set_data(y_x[0], y_y[0])
    y_space2_draw_past.set_data(y_x[0], y_y[0])
    y_space2_draw.set_data(y_x[0], y_y[0])
    y_space3_draw_past.set_data(y_x[0], y_y[0])
    y_space3_draw.set_data(y_x[0], y_y[0])
    y_space4_draw_past.set_data(y_x[0], y_y[0])
    y_space4_draw.set_data(y_x[0], y_y[0])


    bat_draw.set_data(bat_x[0], bat_y[0])
    fd_draw.set_data(np.array([bat_x[0], body_x[0]]), np.array([bat_y[0], body_y[0]]))
    pd_draw.set_data(np.array([bat_x[0], pulse_x[0]]), np.array([bat_y[0], pulse_y[0]]))

    bat2_draw.set_data(bat_x[0], bat_y[0])
    fd2_draw.set_data(np.array([bat_x[0], body_x[0]]), np.array([bat_y[0], body_y[0]]))
    pd2_draw.set_data(np.array([bat_x[0], pulse_x[0]]), np.array([bat_y[0], pulse_y[0]]))
    
    bat3_draw.set_data(bat_x[0], bat_y[0])
    fd3_draw.set_data(np.array([bat_x[0], body_x[0]]), np.array([bat_y[0], body_y[0]]))
    pd3_draw.set_data(np.array([bat_x[0], pulse_x[0]]), np.array([bat_y[0], pulse_y[0]]))

    bat4_draw.set_data(bat_x[0], bat_y[0])
    fd4_draw.set_data(np.array([bat_x[0], body_x[0]]), np.array([bat_y[0], body_y[0]]))
    pd4_draw.set_data(np.array([bat_x[0], pulse_x[0]]), np.array([bat_y[0], pulse_y[0]]))


    px_y_draw.set_array(data1[0])
    px_y_draw.set_vmin = c_min1
    px_y_draw.set_vmax = c_max1
    px_y_draw.set_clim(vmin = c_min1, vmax = c_max1)


    px_y2_draw.set_array(data2[0])
    px_y_draw.set_vmin = c_min2
    px_y_draw.set_vmax = c_max2
    px_y_draw.set_clim(vmin = c_min2, vmax = c_max2)
    
    px_y3_draw.set_array(data3[0])
    px_y3_draw.set_vmin = c_min3
    px_y3_draw.set_vmax = c_max3
    px_y3_draw.set_clim(vmin = c_min3, vmax = c_max3)

    px_y4_draw.set_array(data4[0])
    px_y4_draw.set_vmin = c_min4
    px_y4_draw.set_vmax = c_max4
    px_y4_draw.set_clim(vmin = c_min4, vmax = c_max4)

    
    y_echo_L_past_draw.set_ydata(y_el_vec[0])
    y_echo_L_draw.set_ydata(y_el_vec[0])
    y_echo_R_past_draw.set_ydata(y_er_vec[0])
    y_echo_R_draw.set_ydata(y_er_vec[0])


#    return y_space3_draw, y_space3_draw_past, y_space4_draw, y_space4_draw_past, bat3_draw, fd3_draw, pd3_draw, bat4_draw, fd4_draw, pd4_draw, px_y3_draw, px_y4_draw, y_echo_L_past_draw, y_echo_L_draw, y_echo_R_past_draw, y_echo_R_draw        #, py_xL_draw, py_xR_draw
    return y_space_draw, y_space_draw_past, y_space2_draw, y_space2_draw_past, y_space3_draw, y_space3_draw_past, y_space4_draw, y_space4_draw_past, bat_draw, fd_draw, pd_draw, bat2_draw, fd2_draw, pd2_draw, bat3_draw, fd3_draw, pd3_draw, bat4_draw, fd4_draw, pd4_draw, px_y_draw, px_y2_draw, px_y3_draw, px_y4_draw, y_echo_L_past_draw, y_echo_L_draw, y_echo_R_past_draw, y_echo_R_draw        #, py_xL_draw, py_xR_draw

#@jit(cache=True)
def animate(i):
    print("sensing:", i)
    pole_draw.set_data(pole_x, pole_y)
    wall_draw.set_data(wall_x_draw, wall_y_draw)
        
    y_space_draw_past.set_data(y_x[:i], y_y[:i])
    y_space_draw.set_data(y_x[i], y_y[i])
    y_space2_draw_past.set_data(y_x[:i], y_y[:i])
    y_space2_draw.set_data(y_x[i], y_y[i])
    y_space3_draw_past.set_data(y_x[:i], y_y[:i])
    y_space3_draw.set_data(y_x[i], y_y[i])
    y_space4_draw_past.set_data(y_x[:i], y_y[:i])
    y_space4_draw.set_data(y_x[i], y_y[i])

    bat_draw.set_data(bat_x[i], bat_y[i])
    fd_draw.set_data(np.array([bat_x[i], body_x[i]]), np.array([bat_y[i], body_y[i]]))
    pd_draw.set_data(np.array([bat_x[i], pulse_x[i]]), np.array([bat_y[i], pulse_y[i]]))    
    
    bat2_draw.set_data(bat_x[i], bat_y[i])
    fd2_draw.set_data(np.array([bat_x[i], body_x[i]]), np.array([bat_y[i], body_y[i]]))
    pd2_draw.set_data(np.array([bat_x[i], pulse_x[i]]), np.array([bat_y[i], pulse_y[i]]))
  
    bat3_draw.set_data(bat_x[i], bat_y[i])
    fd3_draw.set_data(np.array([bat_x[i], body_x[i]]), np.array([bat_y[i], body_y[i]]))
    pd3_draw.set_data(np.array([bat_x[i], pulse_x[i]]), np.array([bat_y[i], pulse_y[i]]))

    bat4_draw.set_data(bat_x[i], bat_y[i])
    fd4_draw.set_data(np.array([bat_x[i], body_x[i]]), np.array([bat_y[i], body_y[i]]))
    pd4_draw.set_data(np.array([bat_x[i], pulse_x[i]]), np.array([bat_y[i], pulse_y[i]]))
    
    c_min1 = np.min(data1[i])
    c_max1 = np.percentile(data1[i][np.nonzero(data1[i])], q=c_percentile)
    c_min2 = np.min(data2[i])
    c_max2 = np.percentile(data2[i][np.nonzero(data2[i])], q=c_percentile)
    c_min3 = np.min(data3[i][data3[i] > 2*min_p])
    c_max3 = np.percentile(data3[i][data3[i] > c_min3], q=c_percentile)
    c_min4 = np.min(data4[i][data4[i] > 2*min_p])
    data_temp4 = data4[i][data4[i] > c_min4]
    c_max4 = np.percentile(data_temp4[data_temp4 < np.max(data_temp4)], q=c_percentile)
#    c_max4 = np.percentile(data4[i][data4[i] > c_min4], q=c_percentile)

    px_y_draw.set_array(data1[i])
    px_y_draw.set_vmin = c_min1
    px_y_draw.set_vmax = c_max1
    px_y_draw.set_clim(vmin = c_min1, vmax = c_max1)
    pp.update_normal(px_y_draw)
#    print("color: ", np.min(data1[i]), np.max(data1[i]))
   
    px_y2_draw.set_array(data2[i])
    px_y2_draw.set_vmin = c_min2
    px_y2_draw.set_vmax = c_max2
    px_y2_draw.set_clim(vmin = c_min2, vmax = c_max2)
    pp2.update_normal(px_y2_draw)


    px_y3_draw.set_array(data3[i])
    px_y3_draw.set_vmin = c_min3
    px_y3_draw.set_vmax = c_max3
    px_y3_draw.set_clim(vmin = c_min3, vmax = c_max3)
    pp3.update_normal(px_y3_draw)

    px_y4_draw.set_array(data4[i])  
    px_y4_draw.set_vmin = c_min4
    px_y4_draw.set_vmax = c_max4
    px_y4_draw.set_clim(vmin = c_min4, vmax = c_max4)
    pp4.update_normal(px_y4_draw)

    y_echo_L_draw.set_ydata(y_el_vec[i])
    y_echo_R_draw.set_ydata(y_er_vec[i])

    if i>=1:
        y_echo_L_past_draw.set_ydata(y_el_vec[i-1])
        y_echo_R_past_draw.set_ydata(y_er_vec[i-1])
        
    sensing_time.set_text(f"sensing_n = {i}")
#    return y_space3_draw, y_space3_draw_past, y_space4_draw, y_space4_draw_past, bat3_draw, fd3_draw, pd3_draw, bat4_draw, fd4_draw, pd4_draw, px_y3_draw, px_y4_draw, y_echo_L_past_draw, y_echo_L_draw, y_echo_R_past_draw, y_echo_R_draw        #, py_xL_draw, py_xR_draw
    return  y_space_draw_past, y_space2_draw, y_space2_draw_past, y_space3_draw, y_space3_draw_past, y_space4_draw, y_space4_draw_past, pole_draw, wall_draw, y_space_draw, bat_draw, fd_draw, pd_draw, bat2_draw, fd2_draw, pd2_draw, bat3_draw, fd3_draw, pd3_draw, bat4_draw, fd4_draw, pd4_draw, px_y_draw, px_y2_draw, px_y3_draw, px_y4_draw, y_echo_L_past_draw, y_echo_L_draw, y_echo_R_past_draw, y_echo_R_draw, sensing_time   #, py_xL_draw, py_xR_draw

ani = animation.FuncAnimation(fig, animate, init_func = init, frames=range(0,trials,1), interval=100, blit=True, save_count = trials)
f_name = 'movie/'
os.makedirs(f_name, exist_ok=True)
ani.save(f_name +f'bayes_2dim_model_most_likely(f={freq/1000}kHz, a={a}).mp4', writer="ffmpeg")

