# -*- coding: utf-8 -*-
"""
MarkerTrackerClient - marker_server.pyからデータを取得するラッパークラス

このクラスはmarker_server.pyが提供するHTTP APIにアクセスし、
ロボットの位置情報や障害物情報を取得します。

marker_server.pyの仕様に準拠:
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
    marker_server.py (HTTPサーバー) からマーカーデータを取得するクライアント

    marker_test.pyの実装パターンに準拠したエラーハンドリングと
    urllib.parse.urlencode()を使用したURL構築を行います。
    """

    def __init__(self, host='localhost', port=6000, timeout=5.0):
        """
        Args:
            host (str): marker_serverのホスト
            port (int): marker_serverのポート
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
            print(f"✗ marker_server接続エラー: {e}")
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
            print(f"✗ marker_server接続エラー: {e}")
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
        1. robot_bodyマーカーセット（3点以上）から:
           - z座標が最大のマーカーをロボット本体位置とする
           - z座標が上から2番目と3番目の2点から頭部方向（fd）を計算（2点間ベクトルの垂直方向）
        2. robot_headマーカーセット（3点）から:
           - z座標が最大のマーカーをhead中心（パルス放射位置）とする
           - 3点の直線に垂直な方向を放射方向（pd）として計算

        Args:
            body_marker_set (str): ロボット本体のマーカーセット名（既定: 'robot_body'）
            head_marker_set (str): ロボット頭部のマーカーセット名（既定: 'robot_head'）

        Returns:
            dict or None: {'x': float, 'y': float, 'z': float,
                          'head_x': float, 'head_y': float, 'head_z': float,
                          'fd': float, 'pd': float}
                - x, y, z: ロボット本体位置（robot_bodyのz最大点）[m]
                - head_x, head_y, head_z: パルス放射位置（robot_headのz最大点）[m]
                - fd: 頭部方向（bodyの上から2番目と3番目の2点間ベクトルに垂直な方向）[度、0-360]
                - pd: 放射方向（headの3点の直線に垂直な方向）[度、0-360]
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

        if len(body_markers) < 3:
            print(f"✗ マーカーセット '{body_marker_set}' は3個以上のマーカーが必要ですが、{len(body_markers)}個です")
            return None

        # z座標が最大のマーカーを見つける（ロボット本体位置）
        body_markers_array = np.array(body_markers)
        max_z_idx = np.argmax(body_markers_array[:, 2])
        body_pos = body_markers_array[max_z_idx]

        print(f"  robot_body: {len(body_markers)}個のマーカーからz最大点を選択")
        print(f"    ロボット本体位置（インデックス{max_z_idx}）: ({body_pos[0]:.3f}, {body_pos[1]:.3f}, {body_pos[2]:.3f})")

        # fd（頭部方向）を計算: 上から2番目と3番目の2点の垂直方向
        fd = self._calculate_fd_from_body(body_markers_array, max_z_idx)

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

        # z座標が最大のマーカーを見つける（パルス放射位置）
        head_markers_array = np.array(head_markers)
        head_max_z_idx = np.argmax(head_markers_array[:, 2])
        head_pos = head_markers_array[head_max_z_idx]

        print(f"  robot_head: 3個のマーカーからz最大点を選択")
        print(f"    パルス放射位置（インデックス{head_max_z_idx}）: ({head_pos[0]:.3f}, {head_pos[1]:.3f}, {head_pos[2]:.3f})")

        # pd（放射方向）を計算: 3点の直線に垂直な方向
        pd = self._calculate_pd_from_head(head_markers_array, body_pos)

        position = {
            'x': float(body_pos[0]),
            'y': float(body_pos[1]),
            'z': float(body_pos[2]),
            'head_x': float(head_pos[0]),
            'head_y': float(head_pos[1]),
            'head_z': float(head_pos[2]),
            'fd': float(fd),
            'pd': float(pd)
        }

        print(f"✓ ロボット本体位置: ({position['x']:.3f}, {position['y']:.3f}, {position['z']:.3f})")
        print(f"✓ パルス放射位置: ({position['head_x']:.3f}, {position['head_y']:.3f}, {position['head_z']:.3f})")
        print(f"  fd（頭部方向）={position['fd']:.1f}度, pd（放射方向）={position['pd']:.1f}度")

        return position


    def _calculate_fd_from_body(self, body_markers: np.ndarray, max_z_idx: int) -> float:
        """
        robot_bodyの3点から頭部方向（fd: head direction）を計算

        z座標が上から2番目と3番目の2点を見つけ、そのベクトルに垂直な方向を計算します。
        垂直方向はz最大点の方を向くように調整されます。

        Args:
            body_markers (np.ndarray): (N, 3) の3次元座標配列（N >= 3）
            max_z_idx (int): z座標が最大の点のインデックス

        Returns:
            float: 頭部方向の角度 [度、0-360]
        """
        # z最大点以外の点を取得
        other_indices = [i for i in range(len(body_markers)) if i != max_z_idx]

        if len(other_indices) < 2:
            print(f"  警告: fd計算に必要な点が不足しています。デフォルト値0度を返します。")
            return 0.0

        # z座標でソートして、z座標が大きい方から2点を選ぶ（上から2番目と3番目）
        other_markers = body_markers[other_indices]
        sorted_indices = np.argsort(other_markers[:, 2])[::-1]  # 降順ソート

        # 上から2番目と3番目の2点（B, C）
        B = other_markers[sorted_indices[0]]  # z最大点を除いた中で最大
        C = other_markers[sorted_indices[1]]  # z最大点を除いた中で2番目

        print(f"    上から2番目と3番目の2点: B=({B[0]:.3f}, {B[1]:.3f}, {B[2]:.3f}), C=({C[0]:.3f}, {C[1]:.3f}, {C[2]:.3f})")

        # B→Cのベクトル（xy平面のみ）
        BC_x = C[0] - B[0]
        BC_y = C[1] - B[1]

        # BCに垂直なベクトル（90度回転）: (x, y) → (-y, x) または (y, -x)
        # 2つの候補がある
        perp1_x = -BC_y
        perp1_y = BC_x
        perp2_x = BC_y
        perp2_y = -BC_x

        # z最大点（A）の位置
        A = body_markers[max_z_idx]

        # B-Cの中点M
        M_x = (B[0] + C[0]) / 2
        M_y = (B[1] + C[1]) / 2

        # M→Aのベクトル
        MA_x = A[0] - M_x
        MA_y = A[1] - M_y

        # perp1とperp2のどちらがM→Aに近いか判定（内積で判定）
        dot1 = perp1_x * MA_x + perp1_y * MA_y
        dot2 = perp2_x * MA_x + perp2_y * MA_y

        if dot1 > dot2:
            perp_x, perp_y = perp1_x, perp1_y
        else:
            perp_x, perp_y = perp2_x, perp2_y

        # 角度を計算
        fd = np.degrees(np.arctan2(perp_y, perp_x))

        # 0-360度の範囲に正規化
        if fd < 0:
            fd += 360

        print(f"    2点間ベクトルBC: ({BC_x:.3f}, {BC_y:.3f})")
        print(f"    垂直方向（fd）: {fd:.1f}度")

        return float(fd)

    def _calculate_pd_from_head(self, head_markers: np.ndarray, body_pos: np.ndarray) -> float:
        """
        robot_headの3点から放射方向（pd: pulse direction）を計算

        3点が直線上に並んでいると仮定し、主成分分析（PCA）で主軸方向を求め、
        その垂直方向を放射方向とします。垂直方向は前方（body_posから遠ざかる方向）に
        調整されます。

        Args:
            head_markers (np.ndarray): (3, 3) の3次元座標配列
            body_pos (np.ndarray): ロボット本体位置 [x, y, z]

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

        print(f"    head直線の主軸方向: ({principal_axis_2d[0]:.3f}, {principal_axis_2d[1]:.3f})")

        # 主軸に垂直な方向（90度回転）: 2つの候補
        perp1_x = -principal_axis_2d[1]
        perp1_y = principal_axis_2d[0]
        perp2_x = principal_axis_2d[1]
        perp2_y = -principal_axis_2d[0]

        # body_posから重心への方向
        body_to_head_x = centroid[0] - body_pos[0]
        body_to_head_y = centroid[1] - body_pos[1]

        # perp1とperp2のどちらがbody→headと同じ向きか判定（内積）
        dot1 = perp1_x * body_to_head_x + perp1_y * body_to_head_y
        dot2 = perp2_x * body_to_head_x + perp2_y * body_to_head_y

        # body_posから遠ざかる方向を選択
        if dot1 > dot2:
            perp_x, perp_y = perp1_x, perp1_y
        else:
            perp_x, perp_y = perp2_x, perp2_y

        # 方向を角度に変換
        pd = np.degrees(np.arctan2(perp_y, perp_x))

        # 0-360度の範囲に正規化
        if pd < 0:
            pd += 360

        print(f"    直線の垂直方向（pd）: {pd:.1f}度")

        return float(pd)


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
            print(f"警告: marker_serverから'{marker_set_name}'を取得できませんでした（空配列を返します）")
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
        marker_serverへの接続テスト

        Returns:
            bool: 接続成功したかどうか
        """
        print(f"marker_server接続テスト: {self.base_url}")

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
║  事前に marker_server.py をtestモードで起動:             ║
║  python marker_server.py --mode test --port 6000         ║
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
