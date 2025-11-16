# -*- coding: utf-8 -*-
"""
物体定位モジュール
====================================
_data_Multi.datファイルから物体の距離と角度を計算する
"""

import numpy as np
import math
import peakutils


class Localizer:
    """
    音響データから物体を定位するクラス

    ベイズ定位点導入.pyの定位計算ロジックを移植
    """

    def __init__(self, mic_distance=116, temperature=21, threshold=0.15):
        """
        初期化

        Args:
            mic_distance (float): マイク間距離 [mm]
            temperature (float): 温度 [℃]
            threshold (float): ピーク検出の閾値
        """
        self.mic_distance = mic_distance  # mm
        self.temperature = temperature
        self.threshold = threshold

        # 音速を計算
        self.sound_velocity = self._calculate_sound_velocity(temperature)

        # 最大時間差を計算
        self.max_delta_t = self._calculate_max_delta_t()

        print(f"Localizer初期化: mic_dist={mic_distance}mm, temp={temperature}℃, "
              f"sound_vel={self.sound_velocity:.3f}m/ms, max_delta_t={self.max_delta_t:.1f}µs")

    def _calculate_sound_velocity(self, temp):
        """
        温度から音速を計算 [m/ms]

        Args:
            temp (float): 温度 [℃]

        Returns:
            float: 音速 [m/ms]
        """
        return (331.3 + (0.606 * temp)) / 1000.0

    def _calculate_max_delta_t(self):
        """
        最大開き角での左右の信号の時間差を計算 [µs]

        Returns:
            float: 最大時間差 [µs]
        """
        # 120度の最大開き角
        max_delta_t = self.mic_distance * np.sin(120 * np.pi / 180.0) / self.sound_velocity
        return max_delta_t

    def localize(self, file_path):
        """
        _data_Multi.datファイルから物体を定位

        Args:
            file_path (str): _data_Multi.datファイルのパス

        Returns:
            list: [{'distance': float (mm), 'angle': float (度)}, ...]
                  定位した物体のリスト
        """
        try:
            # 1. ファイル読み込み
            lsig, rsig, lcorr, rcorr = self._read_multi_dat(file_path)

            # 2. 共通処理を呼び出し
            return self._localize_from_arrays(lcorr, rcorr)

        except Exception as e:
            print(f"  ✗ 定位計算エラー: {e}")
            import traceback
            traceback.print_exc()
            return []

    def localize_from_crosscor(self, crosscor_l, crosscor_r):
        """
        相互相関データから直接物体を定位（実機ロボット用）

        Args:
            crosscor_l (list or np.array): 左耳の相互相関データ
            crosscor_r (list or np.array): 右耳の相互相関データ

        Returns:
            list: [{'distance': float (mm), 'angle': float (度)}, ...]
                  定位した物体のリスト
        """
        try:
            # numpy配列に変換
            lcorr = np.array(crosscor_l)
            rcorr = np.array(crosscor_r)

            # 共通処理を呼び出し
            return self._localize_from_arrays(lcorr, rcorr)

        except Exception as e:
            print(f"  ✗ 定位計算エラー (crosscor): {e}")
            import traceback
            traceback.print_exc()
            return []

    def _localize_from_arrays(self, lcorr, rcorr):
        """
        相互相関配列から物体を定位（内部共通メソッド）

        Args:
            lcorr (np.array): 左耳の相互相関データ
            rcorr (np.array): 右耳の相互相関データ

        Returns:
            list: [{'distance': float (mm), 'angle': float (度)}, ...]
                  定位した物体のリスト
        """
        # 1. 相関データの正規化
        direct_pulse_time = 1000
        lcorr = lcorr / max(lcorr[:direct_pulse_time])
        rcorr = rcorr / max(rcorr[:direct_pulse_time])

        # 2. ピーク検出
        mindist = int(0.001 * 1000000)  # 1ms
        left_peaktime, left_peakIntens = self._extract_peaks_utils(lcorr, self.threshold, mindist)
        right_peaktime, right_peakIntens = self._extract_peaks_utils(rcorr, self.threshold, mindist)

        # 3. 左右のピークをマッチング
        LTime, RTime, Intens = self._get_new_obj(
            left_peaktime, left_peakIntens,
            right_peaktime, right_peakIntens,
            self.max_delta_t
        )

        # 4. 距離・角度を計算
        intens, objRad, objDeg, objDis, obj_X, obj_Y = self._calculate_object(
            LTime, RTime, Intens,
            self.sound_velocity,
            self.mic_distance
        )

        # 5. 辞書形式に変換
        detections = []
        for i in range(len(objDis)):
            detections.append({
                'distance': float(objDis[i]),
                'angle': float(objDeg[i]),
                'angle_rad': float(objRad[i]),
                'intensity': float(intens[i]),
                'obj_x': float(obj_X[i]),
                'obj_y': float(obj_Y[i])
            })

        return detections

    def _read_multi_dat(self, datapath):
        """
        _data_Multi.datファイルを読み込む

        Args:
            datapath (str): ファイルパス

        Returns:
            tuple: (sigL, sigR, corrL, corrR) - 各numpy配列
        """
        sigL = []
        sigR = []
        corrL = []
        corrR = []

        with open(datapath) as f:
            datalist = f.read().split("\n")
            for i in range(1, len(datalist) - 1):
                datalist[i] = (datalist[i].replace('"', '')).split(" ")
                sigL.append(float(datalist[i][0]))
                sigR.append(float(datalist[i][1]))
                corrL.append(float(datalist[i][2]))
                corrR.append(float(datalist[i][3]))

        return np.array(sigL), np.array(sigR), np.array(corrL), np.array(corrR)

    def _extract_peaks_utils(self, data, thresh, mindist):
        """
        peakutilsを使ってピークを検出

        Args:
            data (np.array): 信号データ
            thresh (float): 閾値
            mindist (int): 最小距離

        Returns:
            tuple: (indexes, peak_amplitudes) - ピーク位置と振幅
        """
        indexes = peakutils.indexes(data[:30000], thres=thresh, min_dist=mindist)
        peak_amplitudes = data[indexes]

        # 最初のピーク（直達音）を除外
        if len(indexes) > 0:
            return indexes[1:], peak_amplitudes[1:]
        else:
            return np.array([]), np.array([])

    def _get_new_obj(self, left_peaktime, left_peakIntens, right_peaktime, right_peakIntens, MaxDeltaT):
        """
        左右のピークをマッチング

        Args:
            left_peaktime: 左のピーク時間
            left_peakIntens: 左のピーク強度
            right_peaktime: 右のピーク時間
            right_peakIntens: 右のピーク強度
            MaxDeltaT: 最大時間差

        Returns:
            tuple: (LTime, RTime, Intens)
        """
        LTime = []
        RTime = []
        Intens = []

        for lIndex, lTime in enumerate(left_peaktime):
            count = 0
            intens = []
            for rIndex, rTime in enumerate(right_peaktime):
                if np.abs(lTime - rTime) < MaxDeltaT:
                    count = count + 1
                    LTime = np.append(LTime, lTime)
                    RTime = np.append(RTime, rTime)
                    intens = np.append(intens, left_peakIntens[lIndex] * right_peakIntens[rIndex])
            if count != 0:
                intens = intens / count
            Intens = np.append(Intens, intens)

        return LTime, RTime, Intens

    def _find_object(self, Soundvel, micDist, lTime, rTime):
        """
        距離と角度を計算

        Args:
            Soundvel: 音速 [m/ms]
            micDist: マイク間距離 [mm]
            lTime: 左のピーク時間
            rTime: 右のピーク時間

        Returns:
            tuple: (objDis, objDeg) - 距離[mm]と角度[rad]
        """
        objDis = (lTime + rTime) * Soundvel / 4.0
        objDeg = np.arcsin((rTime - lTime) * Soundvel / micDist)
        return objDis, objDeg

    def _calculate_object(self, LTime, RTime, Intens, Soundvel, MicDist):
        """
        物体情報を統合して計算（元のコードと同じ形式）

        Args:
            LTime: 左のピーク時間
            RTime: 右のピーク時間
            Intens: 強度
            Soundvel: 音速 [m/ms]
            MicDist: マイク間距離 [mm]

        Returns:
            tuple: (intens, objRad, objDeg, objDis, obj_X, obj_Y)
                - intens: 正規化された強度配列
                - objRad: 角度（ラジアン）配列
                - objDeg: 角度（度）配列
                - objDis: 距離（mm）配列
                - obj_X: X座標（相対）配列
                - obj_Y: Y座標（相対）配列
        """
        objDis = []
        objDeg = []
        objRad = []
        intens = []
        obj_X = []
        obj_Y = []

        if len(Intens) == 0:
            print('  定位結果: 物体なし')
            return intens, objRad, objDeg, objDis, obj_X, obj_Y

        MaxIntens = max(Intens)

        for i in range(len(Intens)):
            if Intens[i] / MaxIntens > 0.0:
                intens = np.append(intens, Intens[i] / MaxIntens)
                dis, rad = self._find_object(Soundvel, MicDist, LTime[i], RTime[i])
                deg = math.degrees(rad)
                objRad = np.append(objRad, rad)
                objDis = np.append(objDis, dis)
                objDeg = np.append(objDeg, deg)
                obj_X = np.append(obj_X, dis * math.sin(rad))
                obj_Y = np.append(obj_Y, dis * math.cos(rad))

        return intens, objRad, objDeg, objDis, obj_X, obj_Y


if __name__ == "__main__":
    # テスト用
    print("Localizerモジュール")
    localizer = Localizer()
    print("初期化完了")
