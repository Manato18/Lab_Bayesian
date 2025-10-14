import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from math import atan2

# CSVファイルの読み込み
file_path = 'bayse_olddata2/650_pulse_timing_500.csv'
df = pd.read_csv(file_path)

# 最初の10パルスを削除
df = df.iloc[10:]

# 時間と位置データの取得
times = df['pulsetiming'].values
x = df['x'].values
y = df['y'].values
z = df['z'].values

# 時間間隔の計算
dt = np.diff(times)

# XY平面（2次元）での移動距離の計算
dx = np.diff(x)
dy = np.diff(y)
distances = np.sqrt(dx**2 + dy**2)  # 各ステップでの移動距離（XY平面のみ）

# 累積移動距離の計算
cumulative_distance = np.cumsum(distances)
total_distance = cumulative_distance[-1]  # 総移動距離

# 飛行速度の計算 (m/s)
speeds = distances / dt

# 移動方向の計算 (xy平面での角度、ラジアン)
directions = np.array([atan2(dy[i], dx[i]) for i in range(len(dx))])

# 旋回角速度の計算 (rad/s)
# 角度の変化を計算し、-πとπの間に正規化
direction_changes = np.diff(directions)
direction_changes = np.where(direction_changes > np.pi, direction_changes - 2*np.pi, direction_changes)
direction_changes = np.where(direction_changes < -np.pi, direction_changes + 2*np.pi, direction_changes)
angular_velocities = direction_changes / dt[1:]

# 旋回角度の度数への変換（ヒストグラム用）
direction_changes_deg = direction_changes * 180 / np.pi

# ラジアンから度への変換
directions_deg = directions * 180 / np.pi  # 移動方向を度に変換
angular_velocities_deg = angular_velocities * 180 / np.pi  # 角速度を度/秒に変換

# 1mあたりのパルス数の計算
total_pulses = len(times)
pulses_per_meter = total_pulses / total_distance

# 結果の表示
print(f"総移動距離: {total_distance:.2f} m")
print(f"総パルス数: {total_pulses}")
print(f"1mあたりのパルス数: {pulses_per_meter:.2f} pulses/m")

# 平滑化のためのSavitzky-Golayフィルタの適用（オプション）
window_length = 11  # 奇数である必要がある
polyorder = 3  # 多項式の次数

if len(speeds) > window_length:  # データ点が十分にある場合のみフィルタを適用
    speeds_smoothed = savgol_filter(speeds, window_length, polyorder)
else:
    speeds_smoothed = speeds

if len(angular_velocities) > window_length:
    angular_velocities_smoothed = savgol_filter(angular_velocities, window_length, polyorder)
else:
    angular_velocities_smoothed = angular_velocities

# プロットの作成
plt.figure(figsize=(18, 12))

# 飛行速度の時系列変化
plt.subplot(2, 3, 1)
plt.plot(times[1:], speeds, 'b-', alpha=0.5, label='Raw')
plt.plot(times[1:], speeds_smoothed, 'r-', label='Smoothed')
plt.title('Flight Speed over Time')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.grid(True)
plt.legend()

# 旋回角速度の時系列変化
plt.subplot(2, 3, 2)
plt.plot(times[2:], angular_velocities_deg, 'b-', alpha=0.5, label='Raw')
# 平滑化データも度に変換
angular_velocities_smoothed_deg = angular_velocities_smoothed * 180 / np.pi
plt.plot(times[2:], angular_velocities_smoothed_deg, 'r-', label='Smoothed')
plt.title('Angular Velocity over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.grid(True)
plt.legend()

# 飛行速度 vs 旋回角速度
plt.subplot(2, 3, 3)
plt.scatter(angular_velocities_deg, speeds[1:], alpha=0.5)
plt.title('Flight Speed vs Angular Velocity')
plt.xlabel('Angular Velocity (deg/s)')
plt.ylabel('Speed (m/s)')
plt.grid(True)

# 移動軌跡（XY平面）
plt.subplot(2, 3, 4)
plt.plot(x, y, 'g-')
plt.scatter(x[0], y[0], color='blue', s=100, label='Start')
plt.scatter(x[-1], y[-1], color='red', s=100, label='End')
plt.title('Flight Path (XY Plane)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid(True)
plt.axis('equal')
plt.legend()

# 旋回角度のヒストグラム
plt.subplot(2, 3, 5)
# ヒストグラムのビン（区間）の設定
bins = np.arange(-60, 61, 10)  # -180度から180度まで、10度間隔
plt.hist(direction_changes_deg, bins=bins, alpha=0.7, edgecolor='black')
plt.title('Histogram of Turn Angles')
plt.xlabel('Turn Angle (degrees)')
plt.ylabel('Frequency')
plt.grid(True)
plt.xlim(-60, 60)

# 旋回角度が-30度より小さいか30度より大きい場合のパルス回数目と角度を表示
print("\n大きな旋回角度を持つパルス:")
print("パルス番号\t旋回角度(度)")
print("-" * 30)

# 最初の10パルスは削除されているため、インデックスを調整
for i, angle in enumerate(direction_changes_deg):
    if angle < -20 or angle > 20:
        # 元のデータフレームのインデックスに10を足す（最初の10パルスを削除したため）
        pulse_idx = i + 10 + 1  # さらに+1は最初のパルスにはdirection_changesがないため
        print(f"{pulse_idx}\t\t{angle:.2f}")

# 空のサブプロット（バランス用）
plt.subplot(2, 3, 6)
plt.axis('off')

plt.tight_layout()
plt.savefig('flight_analysis.png', dpi=300)
plt.show()
