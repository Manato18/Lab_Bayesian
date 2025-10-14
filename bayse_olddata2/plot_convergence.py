import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_convergence(file_path):
    """
    CSVファイルからデータを読み込み、Stepを横軸、Convergence_Valueを縦軸としてプロットする
    
    Parameters:
    -----------
    file_path : str
        CSVファイルのパス
    """
    # ファイルが存在するか確認
    if not os.path.exists(file_path):
        print(f"エラー: ファイル '{file_path}' が見つかりません。")
        return
    
    try:
        # CSVファイルを読み込む
        df = pd.read_csv(file_path)
        
        # 必要な列があるか確認
        if 'Step' not in df.columns or 'Convergence_Value' not in df.columns:
            print("エラー: CSVファイルに 'Step' または 'Convergence_Value' 列がありません。")
            return
        
        # プロットの設定
        plt.figure(figsize=(10, 6))
        plt.plot(df['Step'], df['Convergence_Value'], marker='o', linestyle='-', markersize=3)
        
        # グラフのタイトルと軸ラベルを設定
        plt.title('Convergence Value vs Step')
        plt.xlabel('Step')
        plt.ylabel('Convergence Value')
        plt.grid(True, linestyle='--', alpha=0.7)
        
        # 科学的表記法を使用
        plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
        
        # グラフを表示
        plt.tight_layout()
        plt.show()
        
        print(f"'{file_path}' のプロットが完了しました。")
        
    except Exception as e:
        print(f"エラー: {e}")

if __name__ == "__main__":
    # CSVファイルのパス
    csv_file = "cognitive_convergence_isRealBat.csv"
    
    # 相対パスから絶対パスに変換
    current_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(current_dir, csv_file)
    
    # プロット実行
    plot_convergence(file_path)
