import os
import glob
from PIL import Image
import re

def natural_sort_key(s):
    """自然な順序でソートするためのキー関数"""
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('([0-9]+)', s)]

def png_to_gif(input_folder, output_file, duration=100):
    """
    PNGファイルをGIFに変換する関数
    
    Args:
        input_folder (str): PNGファイルが格納されているフォルダのパス
        output_file (str): 出力GIFファイルのパス
        duration (int): 各フレームの表示時間（ミリ秒）
    """
    # PNGファイルのパスを取得
    png_pattern = os.path.join(input_folder, "*.png")
    png_files = glob.glob(png_pattern)
    
    if not png_files:
        print(f"エラー: {input_folder} にPNGファイルが見つかりませんでした。")
        return
    
    # ファイル名で自然な順序にソート
    png_files.sort(key=natural_sort_key)
    
    print(f"見つかったPNGファイル数: {len(png_files)}")
    print("最初の5つのファイル:")
    for i, file in enumerate(png_files[:5]):
        print(f"  {i+1}: {os.path.basename(file)}")
    
    # 画像を読み込み
    images = []
    for png_file in png_files:
        try:
            img = Image.open(png_file)
            images.append(img)
            print(f"読み込み完了: {os.path.basename(png_file)}")
        except Exception as e:
            print(f"エラー: {png_file} の読み込みに失敗しました - {e}")
    
    if not images:
        print("エラー: 読み込める画像がありませんでした。")
        return
    
    # GIFとして保存
    try:
        print(f"GIFファイルを作成中: {output_file}")
        images[0].save(
            output_file,
            save_all=True,
            append_images=images[1:],
            duration=duration,
            loop=0
        )
        print(f"GIFファイルの作成が完了しました: {output_file}")
        print(f"フレーム数: {len(images)}")
        print(f"各フレームの表示時間: {duration}ms")
        
    except Exception as e:
        print(f"エラー: GIFファイルの作成に失敗しました - {e}")

if __name__ == "__main__":
    # 入力フォルダと出力ファイルのパスを設定
    input_folder = "bayse_olddata2/movie/isRealBat2"
    output_file = "isRealBat_animation.gif"
    
    # PNGからGIFに変換
    png_to_gif(input_folder, output_file, duration=100)
