#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Confidence Matrix Comparison Visualization

異なるthreshold値でのconfidence matrixを左右に比較表示します。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.special import jv

# ========================================
# パラメータ設定
# ========================================

# 物理パラメータ
h_grid = 0.05         # 格子点間隔 [m]
h_actual = 0.01       # 実際のコードで使われるh [m]
c = 340               # 音速 [m/s]
grad = 10**13         # sigmoid関数の勾配

# 比較する2つのパラメータセット
threshold_1 = 0.0032  # 左側のthreshold
threshold_2 = 0.007   # 右側のthreshold
a_1 = 0.005           # 左側の口の半径 [m]
a_2 = 0.005           # 右側の口の半径 [m]
freq_1 = 40000        # 左側の周波数 [Hz]
freq_2 = 37000        # 右側の周波数 [Hz]

# シミュレーション空間
x_min = -1.0
x_max = 1.0
y_min = -0.5
y_max = 2.0

# ロボットの位置と向き
bat_x = 0.0
bat_y = 0.0
bat_pd = 90

print("=" * 60)
print("Confidence Matrix Comparison")
print("=" * 60)
print(f"Left:  threshold = {threshold_1}, a = {a_1}, freq = {freq_1}")
print(f"Right: threshold = {threshold_2}, a = {a_2}, freq = {freq_2}")
print()

# ========================================
# 計算関数
# ========================================

def dist_attenuation(r):
    """距離による音波減衰"""
    r0 = h_actual / 2
    r_safe = np.where(r < 0.001, 0.001, r)
    return r0 / r_safe

def direc_attenuation(theta, a, freq):
    """方向による音波減衰（ピストンモデル）"""
    theta_clip = np.clip(theta, -np.pi / 2, np.pi / 2)
    k = 2 * np.pi * freq / c

    with np.errstate(divide='ignore', invalid='ignore'):
        beam_pattern = np.abs(
            2 * jv(1, k * a * np.sin(theta_clip)) / (k * a * np.sin(theta_clip))
        )
        beam_pattern = np.where(np.isnan(beam_pattern) | np.isinf(beam_pattern), 1.0, beam_pattern)

    return beam_pattern

def sigmoid(x, center, grad):
    """シグモイド関数"""
    z = -4 * grad * (x - center)
    z = np.clip(z, -100, 100)
    return 1 / (1 + np.exp(z))

def compute_confidence(threshold, a, freq):
    """指定されたthreshold、a、freqでconfidence matrixを計算"""
    # グリッド生成
    x = np.arange(x_min, x_max + h_grid, h_grid)
    y = np.arange(y_min, y_max + h_grid, h_grid)
    X, Y = np.meshgrid(x, y)

    # 距離と角度
    del_x = X - bat_x
    del_y = Y - bat_y
    r = np.sqrt(del_x**2 + del_y**2)

    angle_to_point = np.arctan2(del_y, del_x)
    pd_rad = np.deg2rad(bat_pd)
    theta = angle_to_point - pd_rad
    theta = np.arctan2(np.sin(theta), np.cos(theta))

    # 減衰計算
    dist_atten = dist_attenuation(r)
    direc_atten = direc_attenuation(theta, a, freq)
    combined_atten = dist_atten * direc_atten

    # Confidence計算
    confidence = sigmoid(combined_atten, threshold, grad)

    return X, Y, confidence

# ========================================
# 計算
# ========================================

print("計算中...")
X1, Y1, confidence_1 = compute_confidence(threshold_1, a_1, freq_1)
X2, Y2, confidence_2 = compute_confidence(threshold_2, a_2, freq_2)

print(f"\nLeft (threshold = {threshold_1}, a = {a_1}, freq = {freq_1}):")
print(f"  Confidence = 1.0: {100 * np.sum(confidence_1 >= 0.9999) / confidence_1.size:.2f}%")

print(f"\nRight (threshold = {threshold_2}, a = {a_2}, freq = {freq_2}):")
print(f"  Confidence = 1.0: {100 * np.sum(confidence_2 >= 0.9999) / confidence_2.size:.2f}%")

# ========================================
# 可視化
# ========================================

print("\n可視化中...")
fig, axes = plt.subplots(1, 2, figsize=(18, 9))

# パルス方向の矢印
arrow_length = 0.3
dx = arrow_length * np.cos(np.deg2rad(bat_pd))
dy = arrow_length * np.sin(np.deg2rad(bat_pd))

# 左側: threshold_1
ax1 = axes[0]
im1 = ax1.contourf(X1, Y1, confidence_1, levels=50, cmap='RdYlGn', vmin=0, vmax=1)
ax1.plot(bat_x, bat_y, 'r*', markersize=25, label='Robot',
         markeredgecolor='white', markeredgewidth=2)
ax1.arrow(bat_x, bat_y, dx, dy,
          head_width=0.12, head_length=0.1, fc='red', ec='red', linewidth=3)
contour1 = ax1.contour(X1, Y1, confidence_1, levels=[1.0],
                       colors='red', linewidths=4, linestyles='-')

ax1.set_xlabel('X [m]', fontsize=14, fontweight='bold')
ax1.set_ylabel('Y [m]', fontsize=14, fontweight='bold')
ax1.set_title(f'Confidence Matrix\nthreshold = {threshold_1}, freq = {freq_1} Hz',
              fontsize=16, fontweight='bold', pad=20)
ax1.set_xlim(x_min, x_max)
ax1.set_ylim(y_min, y_max)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)

cbar1 = plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
cbar1.set_label('Confidence', fontsize=12, fontweight='bold')

# 統計情報
info1 = (
    f"Parameters:\n"
    f"  threshold = {threshold_1}\n"
    f"  a = {a_1} m\n"
    f"  grad = {grad:.2e}\n"
    f"  freq = {freq_1} Hz\n"
    f"  h = {h_actual} m\n"
    f"  r0 = {h_actual/2} m\n\n"
    f"High confidence:\n"
    f"  = 1.0: {100 * np.sum(confidence_1 >= 0.9999) / confidence_1.size:.1f}%"
)
ax1.text(0.02, 0.98, info1, transform=ax1.transAxes,
         fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=1.5))

# 右側: threshold_2
ax2 = axes[1]
im2 = ax2.contourf(X2, Y2, confidence_2, levels=50, cmap='RdYlGn', vmin=0, vmax=1)
ax2.plot(bat_x, bat_y, 'r*', markersize=25, label='Robot',
         markeredgecolor='white', markeredgewidth=2)
ax2.arrow(bat_x, bat_y, dx, dy,
          head_width=0.12, head_length=0.1, fc='red', ec='red', linewidth=3)
contour2 = ax2.contour(X2, Y2, confidence_2, levels=[1.0],
                       colors='red', linewidths=4, linestyles='-')

ax2.set_xlabel('X [m]', fontsize=14, fontweight='bold')
ax2.set_ylabel('Y [m]', fontsize=14, fontweight='bold')
ax2.set_title(f'Confidence Matrix\nthreshold = {threshold_2}, freq = {freq_2} Hz',
              fontsize=16, fontweight='bold', pad=20)
ax2.set_xlim(x_min, x_max)
ax2.set_ylim(y_min, y_max)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)

cbar2 = plt.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
cbar2.set_label('Confidence', fontsize=12, fontweight='bold')

# 統計情報
info2 = (
    f"Parameters:\n"
    f"  threshold = {threshold_2}\n"
    f"  a = {a_2} m\n"
    f"  grad = {grad:.2e}\n"
    f"  freq = {freq_2} Hz\n"
    f"  h = {h_actual} m\n"
    f"  r0 = {h_actual/2} m\n\n"
    f"High confidence:\n"
    f"  = 1.0: {100 * np.sum(confidence_2 >= 0.9999) / confidence_2.size:.1f}%"
)
ax2.text(0.02, 0.98, info2, transform=ax2.transAxes,
         fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=1.5))

plt.tight_layout()

filename = 'confidence_matrix_comparison.png'
plt.savefig(filename, dpi=200, bbox_inches='tight')
print(f"\n図を保存しました: {filename}")
plt.close()

print("\n処理が完了しました。")
