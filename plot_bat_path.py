import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

def plot_bat_path_and_pulse_direction():
    """
    コウモリの飛行経路とパルス放射方向をプロットする関数
    位置を点、放射方向を矢印で可視化
    """
    
    # データの読み込み
    print("データを読み込み中...")
    
    # パルスタイミングデータの読み込み
    pulse_timing_file = 'bayse_olddata2/650_pulse_timing_500.csv'
    pulse_timing = pd.read_csv(pulse_timing_file, index_col=0)
    
    # パルス方向データの読み込み
    pulse_direction_file = 'bayse_olddata2/20241016_650_pulse_direction.csv'
    df_pulse_direction = pd.read_csv(pulse_direction_file)
    
    print(f"パルスタイミングデータ: {len(pulse_timing)} 行")
    print(f"パルス方向データ: {len(df_pulse_direction)} 行")
    
    # データの長さを合わせる（短い方に合わせる）
    min_length = min(len(pulse_timing), len(df_pulse_direction))
    print(f"使用するデータ長: {min_length} 行")
    
    # 40-60パルスまでに制限
    start_idx = 40
    end_idx = 60
    data_length = end_idx - start_idx
    print(f"プロットするデータ長: {data_length} パルス (パルス{start_idx}から{end_idx}まで)")
    
    # データを切り詰める
    pulse_timing_trimmed = pulse_timing.iloc[start_idx:end_idx]
    pulse_direction_trimmed = df_pulse_direction.iloc[start_idx:end_idx]
    
    # コウモリの座標データを取得
    bat_x = pulse_timing_trimmed['x'].values
    bat_y = pulse_timing_trimmed['y'].values
    bat_z = pulse_timing_trimmed['z'].values
    pulse_timing_values = pulse_timing_trimmed['pulsetiming'].values
    
    # パルス方向データを取得
    pulse_directions = pulse_direction_trimmed['pulsedir'].values
    
    # マージンスペースを追加（元のコードと同じ）
    margin_space = 0.5
    bat_x_adjusted = bat_x + margin_space
    bat_y_adjusted = bat_y + margin_space
    
    # プロットの作成（左側の図のみ）
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # XY平面（2D）のプロット
    ax.set_title(f'Bat Flight Path and Pulse Direction (XY Plane)\nPulses {start_idx}-{end_idx}', fontsize=14, fontweight='bold')
    
    # 飛行経路を薄い赤色の線でプロット
    # パルス放射方向を一般的な矢印でプロット（すべての矢印を表示）
    arrow_length = 0.15  # 矢印の長さ
    for i in range(data_length):  # すべてのパルスに対して矢印を表示
        x_pos = bat_x_adjusted[i]
        y_pos = bat_y_adjusted[i]
        direction_rad = np.deg2rad(pulse_directions[i])
        
        # 矢印の終点を計算
        arrow_end_x = x_pos + arrow_length * np.cos(direction_rad)
        arrow_end_y = y_pos + arrow_length * np.sin(direction_rad)
        
        # 一般的な矢印を描画（quiverを使用、太くする）
        ax.quiver(x_pos, y_pos, 
                 arrow_end_x - x_pos, arrow_end_y - y_pos,
                 angles='xy', scale_units='xy', scale=1,
                 color='red', alpha=0.8, width=0.008, headwidth=2, headlength=4, zorder=2)
    
    # 飛行経路を薄い赤色の線でプロット（矢印の後に描画）
    ax.plot(bat_x_adjusted, bat_y_adjusted, color='lightcoral', linewidth=4, alpha=0.8, label='Flight Path', zorder=1)

    ax.set_xlabel('X Coordinate (m)', fontsize=12)
    ax.set_ylabel('Y Coordinate (m)', fontsize=12)
    ax.legend()
    ax.set_aspect('equal')
    
    # 表示範囲をx2-5、y0.5-2.5に設定
    ax.set_xlim(2, 5)
    ax.set_ylim(0, 2.5)
    
    plt.tight_layout()
    
    # 保存
    output_dir = 'bayse_olddata2'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    output_file = os.path.join(output_dir, 'bat_path_and_pulse_direction.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"プロットを保存しました: {output_file}")
    
    # 統計情報を表示
    print(f"\n=== 統計情報 ===")
    print(f"総パルス数: {data_length}")
    print(f"飛行時間: {pulse_timing_values[-1] - pulse_timing_values[0]:.2f} 秒")
    print(f"平均パルス間隔: {(pulse_timing_values[-1] - pulse_timing_values[0]) / data_length:.3f} 秒")
    print(f"X座標範囲: {bat_x_adjusted.min():.2f} ~ {bat_x_adjusted.max():.2f} m")
    print(f"Y座標範囲: {bat_y_adjusted.min():.2f} ~ {bat_y_adjusted.max():.2f} m")
    print(f"Z座標範囲: {bat_z.min():.2f} ~ {bat_z.max():.2f} m")
    print(f"パルス方向範囲: {pulse_directions.min():.1f} ~ {pulse_directions.max():.1f} 度")
    
    plt.show()

if __name__ == "__main__":
    plot_bat_path_and_pulse_direction()
