# -*- coding: utf-8 -*-
"""
MarkerTrackerClient - marker_tracker.pyからデータを取得するラッパークラス

このクラスはmarker_tracker.pyが提供するHTTP APIにアクセスし、
ロボットの位置情報や障害物情報を取得します。

marker_tracker.pyの仕様に準拠:
- GET /latest: 最新スナップショット全体を取得
- GET /marker_set?name=<name>: 特定マーカーセットを取得

データ構造:
- marker_sets: マーカーセット（名前付きマーカーグループ）
- rigid_bodies: 剛体（Motive側で定義された剛体オブジェクト）
- labeled_markers, unlabeled_markers, legacy_other_markers: その他のマーカー

使用例:
    client = MarkerTrackerClient(host='localhost', port=6000)

    # ロボット位置を取得（rigid_bodyまたはmarker_setから）
    position = client.get_robot_position('robot_head')

    # 障害物を取得
    obs_x, obs_y = client.get_obstacles('obstacles')
"""

import json
import urllib.request
import urllib.parse
import urllib.error
import numpy as np
from typing import Dict, List, Optional, Tuple


class MarkerTrackerClient:
    """
    marker_tracker.py (HTTPサーバー) からマーカーデータを取得するクライアント

    marker_test.pyの実装パターンに準拠したエラーハンドリングと
    urllib.parse.urlencode()を使用したURL構築を行います。
    """

    def __init__(self, host='localhost', port=6000, timeout=5.0):
        """
        Args:
            host (str): marker_trackerのホスト
            port (int): marker_trackerのポート
            timeout (float): HTTPリクエストのタイムアウト（秒）
        """
        self.host = host
        self.port = port
        self.base_url = f"http://{host}:{port}"
        self.timeout = timeout
        print(f"MarkerTrackerClient初期化: {self.base_url}")

    def get_latest(self) -> Optional[Dict]:
        """
        最新のスナップショット全体を取得（/latest エンドポイント）

        Returns:
            dict or None: {
                'ok': bool,
                'snapshot': {
                    'timestamp': float,
                    'frame': int,
                    'labeled_markers': [...],
                    'marker_sets': [...],
                    'unlabeled_markers': [...],
                    'legacy_other_markers': [...],
                    'rigid_bodies': [...]
                }
            }
        """
        url = f"{self.base_url}/latest"

        try:
            req = urllib.request.Request(url, method='GET')
            with urllib.request.urlopen(req, timeout=self.timeout) as response:
                if response.status != 200:
                    print(f"✗ /latest取得失敗: HTTP {response.status}")
                    return None

                data = response.read()
                text = data.decode('utf-8', errors='replace')
                result = json.loads(text)
                return result

        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ConnectionError) as e:
            print(f"✗ marker_tracker接続エラー: {e}")
            return None
        except json.JSONDecodeError as e:
            print(f"✗ JSONデコードエラー: {e}")
            return None
        except Exception as e:
            print(f"✗ 予期しないエラー: {e}")
            return None

    def get_marker_set(self, name: str) -> Optional[Dict]:
        """
        特定のマーカーセットを取得（/marker_set エンドポイント）

        marker_test.pyの実装パターンに準拠:
        - urllib.parse.urlencode()でクエリパラメータをエンコード
        - エラーハンドリングパターンを統一

        Args:
            name (str): マーカーセット名（例: 'robot_head', 'obstacles'）

        Returns:
            dict or None: {
                'ok': bool,
                'name': str,
                'timestamp': float,
                'frame': int,
                'count': int,
                'markers': [[x, y, z], ...],
                'found': bool
            }
        """
        # marker_test.pyと同じパターン: urllib.parse.urlencode()を使用
        qs = urllib.parse.urlencode({'name': name})
        url = f"{self.base_url}/marker_set?{qs}"

        try:
            req = urllib.request.Request(url, method='GET')
            with urllib.request.urlopen(req, timeout=self.timeout) as response:
                if response.status != 200:
                    print(f"✗ marker_set取得失敗: HTTP {response.status}")
                    return None

                data = response.read()
                text = data.decode('utf-8', errors='replace')
                result = json.loads(text)
                return result

        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ConnectionError) as e:
            print(f"✗ marker_tracker接続エラー: {e}")
            return None
        except json.JSONDecodeError as e:
            print(f"✗ JSONデコードエラー: {e}")
            return None
        except Exception as e:
            print(f"✗ 予期しないエラー: {e}")
            return None

    def get_robot_position(self, body_marker_set='robot_body',
                          head_marker_set='robot_head') -> Optional[Dict[str, float]]:
        """
        ロボット位置と方向を取得

        処理フロー:
        1. robot_bodyマーカーセットからz座標が最大のマーカーをロボット位置とする
        2. robot_headマーカーセットから3点の真ん中の点をヘッド位置とする
        3. pd（頭部方向）: bodyからheadへのベクトルの角度
        4. fd（放射方向）: headマーカー3点の並び方向

        Args:
            body_marker_set (str): ロボット本体のマーカーセット名（既定: 'robot_body'）
            head_marker_set (str): ロボット頭部のマーカーセット名（既定: 'robot_head'）

        Returns:
            dict or None: {'x': float, 'y': float, 'z': float, 'fd': float, 'pd': float}
                - x, y: ロボット位置（robot_bodyのz最大点のxy座標）[m]
                - z: ロボット位置のz座標 [m]
                - fd: 放射方向（headの3点の直線方向）[度、0-360]
                - pd: 頭部方向（bodyからheadへの方向）[度、0-360]
        """
        # 1. robot_bodyマーカーセットを取得
        body_result = self.get_marker_set(body_marker_set)

        if body_result is None or not body_result.get('ok'):
            print(f"✗ マーカーセット '{body_marker_set}' 取得失敗")
            return None

        if not body_result.get('found'):
            print(f"✗ マーカーセット '{body_marker_set}' が見つかりません")
            return None

        body_markers = body_result.get('markers', [])

        if len(body_markers) == 0:
            print(f"✗ マーカーセット '{body_marker_set}' にマーカーがありません")
            return None

        # z座標が最大のマーカーを見つける
        body_markers_array = np.array(body_markers)
        max_z_idx = np.argmax(body_markers_array[:, 2])
        body_pos = body_markers_array[max_z_idx]

        print(f"  robot_body: {len(body_markers)}個のマーカーからz最大点を選択")
        print(f"    選択された点（インデックス{max_z_idx}）: ({body_pos[0]:.3f}, {body_pos[1]:.3f}, {body_pos[2]:.3f})")

        # 2. robot_headマーカーセットを取得
        head_result = self.get_marker_set(head_marker_set)

        if head_result is None or not head_result.get('ok'):
            print(f"✗ マーカーセット '{head_marker_set}' 取得失敗")
            return None

        if not head_result.get('found'):
            print(f"✗ マーカーセット '{head_marker_set}' が見つかりません")
            return None

        head_markers = head_result.get('markers', [])

        if len(head_markers) != 3:
            print(f"✗ マーカーセット '{head_marker_set}' は3個のマーカーが必要ですが、{len(head_markers)}個です")
            return None

        # 3点の真ん中を見つける
        head_markers_array = np.array(head_markers)
        middle_idx = self._find_middle_point(head_markers_array)
        head_pos = head_markers_array[middle_idx]

        print(f"  robot_head: 3個のマーカーから真ん中の点を選択")
        print(f"    選択された点（インデックス{middle_idx}）: ({head_pos[0]:.3f}, {head_pos[1]:.3f}, {head_pos[2]:.3f})")

        # 3. pd（頭部方向）を計算: bodyからheadへの角度
        dx = head_pos[0] - body_pos[0]
        dy = head_pos[1] - body_pos[1]
        pd = np.degrees(np.arctan2(dy, dx))
        # 0-360度の範囲に正規化
        if pd < 0:
            pd += 360

        # 4. fd（放射方向）を計算: head3点の直線方向
        fd = self._calculate_head_direction(head_markers_array)

        position = {
            'x': float(body_pos[0]),
            'y': float(body_pos[1]),
            'z': float(body_pos[2]),
            'fd': float(fd),
            'pd': float(pd)
        }

        print(f"✓ ロボット位置取得: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
        print(f"  fd（放射方向）={position['fd']:.1f}度, pd（頭部方向）={position['pd']:.1f}度")

        return position

    def _find_middle_point(self, points: np.ndarray) -> int:
        """
        3点の中で真ん中の点のインデックスを返す

        3点が直線上に並んでいると仮定し、主成分分析（PCA）で主軸方向を求め、
        その方向に射影した値でソートして中央の点を選択します。

        Args:
            points (np.ndarray): (3, 3) の3次元座標配列

        Returns:
            int: 真ん中の点のインデックス（0, 1, 2のいずれか）
        """
        # 重心を計算
        centroid = np.mean(points, axis=0)
        # 重心を原点とする座標系に変換
        centered = points - centroid

        # 共分散行列を計算（3D）
        cov = np.cov(centered.T)

        # 固有値と固有ベクトルを計算
        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # 最大固有値に対応する固有ベクトル（主軸方向）
        principal_axis = eigenvectors[:, np.argmax(eigenvalues)]

        # 各点を主軸に射影
        projections = centered @ principal_axis

        # 射影値でソートして中央のインデックスを返す
        sorted_indices = np.argsort(projections)
        middle_idx = sorted_indices[1]  # 中央の点

        return middle_idx

    def _calculate_head_direction(self, head_markers: np.ndarray) -> float:
        """
        robot_headの3点から放射方向（fd）を計算

        3点が直線上に並んでいると仮定し、主成分分析（PCA）で主軸方向を求め、
        その方向を角度で返します。

        Args:
            head_markers (np.ndarray): (3, 3) の3次元座標配列

        Returns:
            float: 放射方向の角度 [度、0-360]
        """
        # 重心を計算
        centroid = np.mean(head_markers, axis=0)
        # 重心を原点とする座標系に変換
        centered = head_markers - centroid

        # xy平面での主軸を求める（z方向は無視）
        centered_2d = centered[:, :2]  # x, y座標のみ

        # 共分散行列を計算（2D）
        cov_2d = np.cov(centered_2d.T)

        # 固有値と固有ベクトルを計算
        eigenvalues, eigenvectors = np.linalg.eig(cov_2d)

        # 最大固有値に対応する固有ベクトル（主軸方向）
        principal_axis_2d = eigenvectors[:, np.argmax(eigenvalues)]

        # 方向を角度に変換
        fd = np.degrees(np.arctan2(principal_axis_2d[1], principal_axis_2d[0]))

        # 0-360度の範囲に正規化
        if fd < 0:
            fd += 360

        # PCAの主軸は180度の曖昧性があるため、
        # bodyからheadへの方向と整合性を取る必要がある場合は追加処理が必要
        # ここでは主軸の方向をそのまま返す

        return float(fd)

    def get_obstacles(self, marker_set_name='obstacles') -> Tuple[np.ndarray, np.ndarray]:
        """
        障害物位置をマーカーセットから取得

        Motive側で障害物全てを選択して「obstacles」という1つのマーカーセット（剛体）を作成。
        そのマーカーセット内の各マーカーを個別の障害物として取得します。

        Args:
            marker_set_name (str): 障害物のマーカーセット名（既定: 'obstacles'）

        Returns:
            Tuple[np.ndarray, np.ndarray]: (obs_x, obs_y) 障害物のx, y座標配列
        """
        # /marker_set エンドポイントで特定のマーカーセットを取得
        result = self.get_marker_set(marker_set_name)

        if result is None or not result.get('ok'):
            print(f"警告: marker_trackerから'{marker_set_name}'を取得できませんでした（空配列を返します）")
            return np.array([]), np.array([])

        if not result.get('found'):
            print(f"警告: マーカーセット '{marker_set_name}' が見つかりませんでした（空配列を返します）")
            return np.array([]), np.array([])

        markers = result.get('markers', [])

        if len(markers) == 0:
            print(f"警告: マーカーセット '{marker_set_name}' にマーカーがありません（空配列を返します）")
            return np.array([]), np.array([])

        # 各マーカーを個別の障害物として扱う
        markers_array = np.array(markers)
        obs_x = markers_array[:, 0]
        obs_y = markers_array[:, 1]

        print(f"✓ 障害物データ取得: {len(markers)}個")
        for i, (x, y, z) in enumerate(markers):
            print(f"  障害物{i+1}: 位置=({x:.3f}, {y:.3f}, {z:.3f})")

        return obs_x, obs_y

    def test_connection(self) -> bool:
        """
        marker_trackerへの接続テスト

        Returns:
            bool: 接続成功したかどうか
        """
        print(f"marker_tracker接続テスト: {self.base_url}")

        result = self.get_latest()

        if result is None:
            print("✗ 接続失敗")
            return False

        if not result.get('ok'):
            print("✗ サーバーエラー")
            return False

        snapshot = result.get('snapshot')
        if snapshot:
            frame = snapshot.get('frame')
            timestamp = snapshot.get('timestamp')
            print(f"✓ 接続成功: フレーム={frame}, タイムスタンプ={timestamp}")

            # デバッグ情報
            marker_sets = snapshot.get('marker_sets', [])
            rigid_bodies = snapshot.get('rigid_bodies', [])
            print(f"  マーカーセット数: {len(marker_sets)}")
            if len(marker_sets) > 0:
                print(f"  利用可能なマーカーセット: {[ms.get('name') for ms in marker_sets]}")
            print(f"  剛体数: {len(rigid_bodies)}")

            return True

        print("✗ スナップショットなし")
        return False


# テスト用コード
if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║        MarkerTrackerClient テスト                        ║
║                                                          ║
║  事前に marker_tracker.py をtestモードで起動:            ║
║  python marker_tracker.py --mode test --port 6000        ║
╚══════════════════════════════════════════════════════════╝
    """)

    # クライアント初期化
    client = MarkerTrackerClient(host='localhost', port=6000)

    # 接続テスト
    print("\n" + "=" * 60)
    print("1. 接続テスト")
    print("=" * 60)
    client.test_connection()

    # ロボット位置取得テスト
    print("\n" + "=" * 60)
    print("2. ロボット位置取得テスト")
    print("=" * 60)
    position = client.get_robot_position(
        body_marker_set='robot_body',
        head_marker_set='robot_head'
    )
    if position:
        print(f"位置: x={position['x']:.3f}m, y={position['y']:.3f}m, z={position['z']:.3f}m")
        print(f"方向: fd={position['fd']:.1f}度, pd={position['pd']:.1f}度")

    # 障害物取得テスト
    print("\n" + "=" * 60)
    print("3. 障害物取得テスト")
    print("=" * 60)
    obs_x, obs_y = client.get_obstacles('obstacles')
    print(f"障害物数: {len(obs_x)}")
    if len(obs_x) > 0:
        print(f"最初の障害物: x={obs_x[0]:.3f}m, y={obs_y[0]:.3f}m")
