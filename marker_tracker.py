#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OptiTrack マーカートラッキング＆HTTP提供ツール（サーバ／プリント表示）

概要:
- Motive（NatNet）からマーカー座標を受信し、以下の3モードで利用できます。
  1) プリント表示: 取得フレームの概要・内容を一定間隔でコンソール出力
  2) HTTPサーバ: 最新スナップショットを `GET /latest`、特定マーカーセットを `GET /marker_set?name=...` で返却
  3) テストモード: NatNet接続なしでダミーデータをHTTP提供（開発・テスト用）

前提:
- Motive 側でストリーミングが有効
- NatNetSDK の Python クライアントにパスが通っている（本リポの `NatNetSDK/Samples/PythonClient` を自動追加）

基本の使い方:
- サーバーモード（デフォルト）
    python robot_bayse/marker_tracker.py --server-ip <Motive_IP> --client-ip <THIS_PC_IP> [--port 6000]
  エンドポイント:
    - GET /latest                     例: http://localhost:6000/latest
    - GET /marker_set?name=robot_head 例: http://localhost:6000/marker_set?name=robot_head

- プリント表示（一定間隔で概要を出力）
    python robot_bayse/marker_tracker.py --server-ip <Motive_IP> --client-ip <THIS_PC_IP> --mode print [--interval 1.0]

- テストモード（ダミーデータをHTTP提供、Motive接続不要）
    python robot_bayse/marker_tracker.py --mode test [--port 6000]

引数の要点:
- --server-ip / --client-ip は省略可（未指定は localhost）。
  解決順: 引数 > 環境変数(NATNET_SERVER_IP / NATNET_CLIENT_IP) > "localhost"
- --mode: server | print | test（既定: server）
- --port: HTTPポート（既定: 6000）
- --multicast: マルチキャスト接続を有効化（既定はユニキャスト）
- --interval: プリントモード時の出力間隔（秒、既定: 1.0）

返却データ（/latest の snapshot 構造）:
{
  "timestamp": float,
  "frame": int,
  "labeled_markers": [{"id": int, "pos": [x,y,z]}],
  "marker_sets": [{"name": str, "markers": [[x,y,z], ...]}],
  "unlabeled_markers": [[x,y,z], ...],
  "legacy_other_markers": [[x,y,z], ...],
  "rigid_bodies": [{"id": int, "pos": [x,y,z], "tracking_valid": bool}]
}

運用メモ:
- Ctrl+C で HTTP と NatNet 受信スレッドを安全停止（完全に出力が止まります）。
- ライブラリの冗長ログは `print_level=0` により抑制済み。
- マーカーセット名が bytes で届く場合はUTF-8/CP932で文字列化して返却。
- テストモードでは Motive への接続は行わず、リアルな動きを模したダミーデータを120 FPS相当で生成します。

トラブルシューティング:
- JSON化エラー（bytes が混入）: 対策済み。発生時は /latest が 500 を返し、サーバは継続。
- 接続不可: Motive 側のストリーミング有効化と IP 設定、Firewall/ポート(1510/1511/HTTP)を確認。
"""

import time
import sys
import os
import argparse
import json
import copy
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

# NatNetSDK の Python クライアントへのパスを追加（リポジトリ構成前提）
# 例: <repo_root>/NatNetSDK/Samples/PythonClient/NatNetClient.py
_this_dir = os.path.dirname(os.path.abspath(__file__))
_repo_root = os.path.abspath(os.path.join(_this_dir, os.pardir))
_natnet_py_dir = os.path.join(_repo_root, 'NatNetSDK', 'Samples', 'PythonClient')
if _natnet_py_dir not in sys.path:
    sys.path.insert(0, _natnet_py_dir)

try:
    from NatNetClient import NatNetClient
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        'NatNetClient が見つかりません。NatNetSDK の Python クライアントパスが通っているか確認してください。\n'
        f'試したパス: {_natnet_py_dir}\n'
        '対処例: 環境変数 PYTHONPATH に NatNetSDK/Samples/PythonClient を追加するか、\n'
        'script 内の sys.path 追加処理を環境に合わせて修正してください。'
    ) from e

class SimpleMarkerTracker:
    def __init__(self, client_ip=None, server_ip=None, use_multicast=False, print_interval=1.0, mode='server', server_port=6000):
        # 接続設定（事前に設定済み / 上書き可能）
        # 優先度: 引数 > 環境変数 > 既定値
        env_client = os.environ.get("NATNET_CLIENT_IP")
        env_server = os.environ.get("NATNET_SERVER_IP")

        # 既定値は両方とも localhost
        self.server_ip = server_ip or env_server or "localhost"
        self.client_ip = client_ip or env_client or "localhost"

        self.use_multicast = bool(use_multicast)  # ユニキャスト接続が既定

        # 動作モード
        # 'print' または 'server'
        self.mode = mode
        self.server_port = int(server_port)
        self.print_enabled = (self.mode == 'print')

        # 時間管理
        self.last_print_time = time.time()
        self.print_interval = float(print_interval)  # 1秒間隔

        # ライブラリの詳細出力を抑止
        # NatNetClient には print_level があり、0 に設定すると詳細出力を抑制できる
        # （モジュールスコープの built-in print の置換は効果がないため採用しない）

        # スナップショット共有（サーバーモード用）
        self._snapshot_lock = threading.Lock()
        self._latest_snapshot = None
        self._http_server = None
        self._http_thread = None
        self._test_thread = None  # テストモード用データ更新スレッド

        # NatNetクライアント作成
        self.natnet_client = NatNetClient()
        self.setup_client()
        try:
            # 0: すべての詳細出力を抑制
            self.natnet_client.set_print_level(0)
        except Exception:
            pass

        
    def setup_client(self):
        """クライアントの設定"""
        # アドレス設定
        self.natnet_client.set_client_address(self.client_ip)
        self.natnet_client.set_server_address(self.server_ip)
        self.natnet_client.set_use_multicast(self.use_multicast)
        
        # コールバック関数設定（詳細データ付き）
        self.natnet_client.new_frame_with_data_listener = self.receive_new_frame
        
        if self.print_enabled:
            print(f"=== シンプルマーカートラッカー ===")
            print(f"クライアントIP: {self.client_ip}")
            print(f"サーバーIP: {self.server_ip}")
            print(f"接続方式: {'マルチキャスト' if self.use_multicast else 'ユニキャスト'}")
            print(f"出力間隔: {self.print_interval}秒")
            print("="*40)
        
    def receive_new_frame(self, data_dict):
        """新しいフレームを受信した時のコールバック関数"""
        current_time = time.time()

        # フレーム受信の簡単な確認（毎回）
        if hasattr(self, 'frame_count'):
            self.frame_count += 1
        else:
            self.frame_count = 1
            if self.print_enabled:
                print("データ受信開始...")

        # 最新スナップショットの更新（サーバ・プリント共通）
        try:
            snapshot = self.build_snapshot(data_dict)
            with self._snapshot_lock:
                self._latest_snapshot = snapshot
        except Exception:
            # スナップショット化に失敗しても受信自体は継続
            pass

        # 1秒経過したかチェック
        if self.print_enabled and (current_time - self.last_print_time >= self.print_interval):
            print(f"フレーム受信カウント: {self.frame_count}")
            self.print_marker_data(data_dict)
            self.last_print_time = current_time
    
    def print_marker_data(self, data_dict):
        """マーカーデータをプリント"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        frame_number = data_dict.get("frame_number", "不明")
        
        print(f"\n[{timestamp}] フレーム: {frame_number}")
        print("-" * 50)
        
        # MoCapDataオブジェクトを取得
        mocap_data = data_dict.get("mocap_data")
        if mocap_data is None:
            print("モーションキャプチャデータが見つかりません")
            return
        
        # ラベル付きマーカーデータを表示
        if mocap_data.labeled_marker_data is not None:
            labeled_markers = mocap_data.labeled_marker_data.labeled_marker_list
            print(f"ラベル付きマーカー数: {len(labeled_markers)}")
            for i, marker in enumerate(labeled_markers):
                if hasattr(marker, 'pos') and hasattr(marker, 'id_num'):
                    x, y, z = marker.pos[0], marker.pos[1], marker.pos[2]
                    marker_id = marker.id_num
                    print(f"  マーカー{i+1} (ID:{marker_id}): X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        
        # マーカーセットデータを表示
        if mocap_data.marker_set_data is not None:
            marker_sets = mocap_data.marker_set_data.marker_data_list
            total_markers = 0
            for marker_set in marker_sets:
                total_markers += len(marker_set.marker_pos_list)
            print(f"マーカーセット数: {len(marker_sets)}, 総マーカー数: {total_markers}")
            
            for set_idx, marker_set in enumerate(marker_sets):
                print(f"  セット{set_idx+1} '{marker_set.model_name}': {len(marker_set.marker_pos_list)}個")
                for i, pos in enumerate(marker_set.marker_pos_list):
                    x, y, z = pos[0], pos[1], pos[2]
                    print(f"    マーカー{i+1}: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
            
            # ラベルなしマーカー（unlabeled markers）を表示
            unlabeled_count = mocap_data.marker_set_data.get_unlabeled_marker_count()
            if unlabeled_count > 0:
                print(f"ラベルなしマーカー数: {unlabeled_count}")
                for i, pos in enumerate(mocap_data.marker_set_data.unlabeled_markers.marker_pos_list):
                    x, y, z = pos[0], pos[1], pos[2]
                    print(f"  ラベルなし{i+1}: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        
        # ラベルなしマーカー（レガシー）データを表示
        if mocap_data.legacy_other_markers is not None:
            other_markers = mocap_data.legacy_other_markers.marker_pos_list
            print(f"その他のマーカー数: {len(other_markers)}")
            for i, pos in enumerate(other_markers):
                x, y, z = pos[0], pos[1], pos[2]
                print(f"  その他{i+1}: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        
        # 剛体データも表示（参考）
        if mocap_data.rigid_body_data is not None:
            rigid_bodies = mocap_data.rigid_body_data.rigid_body_list
            print(f"剛体オブジェクト数: {len(rigid_bodies)}")
            for i, rb in enumerate(rigid_bodies):
                if hasattr(rb, 'pos') and hasattr(rb, 'id_num'):
                    x, y, z = rb.pos[0], rb.pos[1], rb.pos[2]
                    rb_id = rb.id_num
                    tracking = "追跡中" if rb.tracking_valid else "未追跡"
                    print(f"  剛体{i+1} (ID:{rb_id}) [{tracking}]: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
        
        print("-" * 50)

    # ---- サーバーモード: スナップショット生成/取得とHTTPサーバ ----
    def build_snapshot(self, data_dict):
        """JSONシリアライズ可能な最新スナップショットを生成"""
        ts = time.time()
        frame_number = data_dict.get("frame_number")
        mocap_data = data_dict.get("mocap_data")
        result = {
            "timestamp": ts,
            "frame": frame_number,
            "labeled_markers": [],
            "marker_sets": [],
            "unlabeled_markers": [],
            "legacy_other_markers": [],
            "rigid_bodies": [],
        }

        if mocap_data is None:
            return result

        # labeled markers
        try:
            if mocap_data.labeled_marker_data is not None:
                for marker in mocap_data.labeled_marker_data.labeled_marker_list:
                    if hasattr(marker, 'pos') and hasattr(marker, 'id_num'):
                        result["labeled_markers"].append({
                            "id": int(marker.id_num),
                            "pos": [float(marker.pos[0]), float(marker.pos[1]), float(marker.pos[2])],
                        })
        except Exception:
            pass

        # marker sets + unlabeled in set
        try:
            if mocap_data.marker_set_data is not None:
                for marker_set in mocap_data.marker_set_data.marker_data_list:
                    # model_name が bytes で届くことがあるため文字列化する
                    name = getattr(marker_set, 'model_name', None)
                    if isinstance(name, bytes):
                        try:
                            name = name.decode('utf-8')
                        except Exception:
                            # フォールバック（日本語Windows想定）
                            try:
                                name = name.decode('cp932', errors='replace')
                            except Exception:
                                name = str(name)
                    elif name is not None:
                        name = str(name)

                    entry = {
                        "name": name,
                        "markers": []
                    }
                    for pos in getattr(marker_set, 'marker_pos_list', []):
                        entry["markers"].append([float(pos[0]), float(pos[1]), float(pos[2])])
                    result["marker_sets"].append(entry)

                # unlabeled markers
                try:
                    unlabeled_list = getattr(mocap_data.marker_set_data, 'unlabeled_markers', None)
                    if unlabeled_list is not None:
                        for pos in getattr(unlabeled_list, 'marker_pos_list', []):
                            result["unlabeled_markers"].append([float(pos[0]), float(pos[1]), float(pos[2])])
                except Exception:
                    pass
        except Exception:
            pass

        # legacy other markers
        try:
            if mocap_data.legacy_other_markers is not None:
                for pos in mocap_data.legacy_other_markers.marker_pos_list:
                    result["legacy_other_markers"].append([float(pos[0]), float(pos[1]), float(pos[2])])
        except Exception:
            pass

        # rigid bodies
        try:
            if mocap_data.rigid_body_data is not None:
                for rb in mocap_data.rigid_body_data.rigid_body_list:
                    if hasattr(rb, 'pos') and hasattr(rb, 'id_num'):
                        result["rigid_bodies"].append({
                            "id": int(rb.id_num),
                            "pos": [float(rb.pos[0]), float(rb.pos[1]), float(rb.pos[2])],
                            "tracking_valid": bool(getattr(rb, 'tracking_valid', False)),
                        })
        except Exception:
            pass

        return result

    def get_latest_snapshot(self):
        # ディープコピーのみ行い、シリアライズは呼び出し側に任せる
        with self._snapshot_lock:
            return None if self._latest_snapshot is None else copy.deepcopy(self._latest_snapshot)

    def generate_test_snapshot(self, frame_number):
        """テストモード用のダミースナップショットを生成"""
        import math

        ts = time.time()
        # 周期的な動きをシミュレート（10秒周期）
        t = ts % 10.0
        offset_x = 0.02 * math.sin(2 * math.pi * t / 10.0)
        offset_y = 0.01 * math.cos(2 * math.pi * t / 10.0)
        offset_z = 0.015 * math.sin(2 * math.pi * t / 5.0)

        # robot_body マーカーセット（5個のマーカー、z座標が異なる）
        # z座標が最も高いマーカー（インデックス0）がロボット位置として使用される
        robot_body_markers = [
            [1.600 + offset_x, 0.200 + offset_y, 0.850 + offset_z],  # z最大（ロボット位置）
            [1.590 + offset_x, 0.210 + offset_y, 0.840 + offset_z],
            [1.610 + offset_x, 0.190 + offset_y, 0.835 + offset_z],
            [1.595 + offset_x, 0.205 + offset_y, 0.830 + offset_z],
            [1.605 + offset_x, 0.195 + offset_y, 0.825 + offset_z],
        ]

        # robot_head マーカーセット（3個のマーカー、直線上に配置）
        # 真ん中のマーカー（インデックス1）がヘッド位置として使用される
        robot_head_markers = [
            [1.617 + offset_x, 0.197 + offset_y, 0.903 + offset_z],
            [1.638 + offset_x, 0.192 + offset_y, 0.905 + offset_z],  # 真ん中
            [1.598 + offset_x, 0.192 + offset_y, 0.898 + offset_z],
        ]

        # obstacles マーカーセット（障害物全てを含む1つのマーカーセット）
        # 各マーカーが個別の障害物を表す
        obstacles_markers = [
            [2.5, 1.5, 0.1],   # 障害物1
            [1.0, 2.0, 0.1],   # 障害物2
            [3.0, 3.0, 0.1],   # 障害物3
            [0.5, 0.5, 0.1],   # 障害物4
            [3.5, 1.0, 0.1],   # 障害物5
        ]

        # その他のマーカー（legacy_other_markers用）
        other_markers = [
            [1.636 + offset_x * 0.5, 0.273 + offset_y * 0.8, 0.825 + offset_z * 0.6],
            [2.016 + offset_x * 0.7, -0.032 + offset_y * 0.9, 0.888 + offset_z * 0.7],
            [1.985 + offset_x * 0.6, -0.009 + offset_y * 0.85, 0.760 + offset_z * 0.5],
        ]

        # ラベル付きマーカー用のIDリスト
        labeled_markers_list = []

        # robot_bodyのマーカー（ID: 60001-60005）
        for i, marker in enumerate(robot_body_markers):
            labeled_markers_list.append({
                "id": 60001 + i,
                "pos": marker
            })

        # robot_headのマーカー（ID: 65537-65539）
        labeled_markers_list.extend([
            {"id": 65537, "pos": robot_head_markers[0]},
            {"id": 65538, "pos": robot_head_markers[1]},
            {"id": 65539, "pos": robot_head_markers[2]},
        ])
        
        # その他のマーカー
        labeled_markers_list.extend([
            {"id": 16027, "pos": other_markers[0]},
            {"id": 16029, "pos": other_markers[1]},
            {"id": 16454, "pos": other_markers[2]},
        ])
        
        # 障害物マーカー（IDは20001から連番）
        for i, obs_marker in enumerate(obstacles_markers):
            labeled_markers_list.append({
                "id": 20001 + i,
                "pos": obs_marker
            })

        result = {
            "timestamp": ts,
            "frame": frame_number,
            # ラベル付きマーカー
            "labeled_markers": labeled_markers_list,
            # マーカーセット
            "marker_sets": [
                {
                    "name": "robot_body",
                    "markers": robot_body_markers,
                },
                {
                    "name": "robot_head",
                    "markers": robot_head_markers,
                },
                {
                    "name": "obstacles",
                    "markers": obstacles_markers,
                },
                {
                    "name": "all",
                    "markers": robot_head_markers,  # この例では robot_head と同じ
                },
            ],
            # ラベルなしマーカー（テストでは空）
            "unlabeled_markers": [],
            # レガシーその他マーカー
            "legacy_other_markers": other_markers,
            # 剛体（robot_bodyとrobot_headの中心位置を計算）
            "rigid_bodies": [
                {
                    "id": 1,
                    "pos": [
                        sum(m[0] for m in robot_body_markers) / len(robot_body_markers),
                        sum(m[1] for m in robot_body_markers) / len(robot_body_markers),
                        sum(m[2] for m in robot_body_markers) / len(robot_body_markers),
                    ],
                    "tracking_valid": True,
                },
                {
                    "id": 2,
                    "pos": [
                        sum(m[0] for m in robot_head_markers) / 3,
                        sum(m[1] for m in robot_head_markers) / 3,
                        sum(m[2] for m in robot_head_markers) / 3,
                    ],
                    "tracking_valid": True,
                }
            ],
        }

        return result

    def update_test_data(self):
        """テストモード用：バックグラウンドでスナップショットを更新し続ける"""
        frame_number = 0
        # 120 FPS 相当でリアルタイム感を再現
        update_interval = 1.0 / 120.0

        print("テストデータ生成開始（120 FPS相当）...")
        try:
            while True:
                frame_number += 1
                snapshot = self.generate_test_snapshot(frame_number)

                with self._snapshot_lock:
                    self._latest_snapshot = snapshot

                time.sleep(update_interval)
        except Exception as e:
            print(f"テストデータ更新中にエラー: {e}")

    def serve_latest_http(self):
        """HTTPサーバを起動し、/latest と /marker_set で最新データを返す"""
        tracker = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                try:
                    parsed = urlparse(self.path)
                    path = parsed.path
                    qs = parse_qs(parsed.query)

                    if path == "/latest":
                        snapshot = tracker.get_latest_snapshot()
                        payload = {"ok": True, "snapshot": snapshot}
                        body = json.dumps(payload).encode('utf-8')
                        try:
                            print("HTTP GET /latest")
                        except Exception:
                            pass
                        self.send_response(200)
                        self.send_header("Content-Type", "application/json; charset=utf-8")
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                        return

                    if path == "/marker_set":
                        names = qs.get("name", [])
                        if not names:
                            msg = json.dumps({"ok": False, "error": "missing 'name' parameter"}).encode('utf-8')
                            try:
                                print("HTTP GET /marker_set (missing name)")
                            except Exception:
                                pass
                            self.send_response(400)
                            self.send_header("Content-Type", "application/json; charset=utf-8")
                            self.send_header("Content-Length", str(len(msg)))
                            self.end_headers()
                            self.wfile.write(msg)
                            return

                        name = names[0]
                        snap = tracker.get_latest_snapshot()
                        result = {
                            "ok": True,
                            "name": name,
                            "timestamp": None,
                            "frame": None,
                            "count": 0,
                            "markers": [],
                            "found": False,
                        }

                        if snap is not None:
                            result["timestamp"] = snap.get("timestamp")
                            result["frame"] = snap.get("frame")
                            for ms in snap.get("marker_sets", []) or []:
                                if ms.get("name") == name:
                                    markers = ms.get("markers") or []
                                    result["markers"] = markers
                                    result["count"] = len(markers)
                                    result["found"] = True
                                    break

                        body = json.dumps(result).encode('utf-8')
                        try:
                            print(f"HTTP GET /marker_set name={name} -> found={result['found']} count={result['count']} frame={result['frame']}")
                        except Exception:
                            pass
                        self.send_response(200)
                        self.send_header("Content-Type", "application/json; charset=utf-8")
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                        return

                    try:
                        print(f"HTTP 404 {path}")
                    except Exception:
                        pass
                    self.send_response(404)
                    self.end_headers()

                except Exception as e:
                    # 例外が発生してもスレッドを落とさず、500で返す
                    err = json.dumps({"ok": False, "error": str(e)}).encode('utf-8')
                    try:
                        print(f"HTTP 500 {getattr(e, '__class__', type('E',(),{})).__name__}: {e}")
                    except Exception:
                        pass
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.send_header("Content-Length", str(len(err)))
                    self.end_headers()
                    try:
                        self.wfile.write(err)
                    except Exception:
                        pass

            def log_message(self, fmt, *args):
                # 標準のアクセスログを抑制
                return

        server = ThreadingHTTPServer(("0.0.0.0", self.server_port), Handler)
        self._http_server = server
        print(f"HTTPサーバ起動: http://0.0.0.0:{self.server_port}/latest")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                server.shutdown()
            except Exception:
                pass
            try:
                server.server_close()
            except Exception:
                pass

    def start_tracking(self):
        """トラッキング開始"""
        try:
            if self.mode == 'print':
                print("接続中...")
                # データストリームモードで開始（'d'を指定）
                is_running = self.natnet_client.run('d')
                if not is_running:
                    print("エラー: ストリーミングクライアントを開始できませんでした。")
                    return
                # 接続確認（少し待ってから）
                time.sleep(1)
                if hasattr(self.natnet_client, 'connected') and not self.natnet_client.connected():
                    print("エラー: 正常に接続できませんでした。Motiveのストリーミングが有効になっているか確認してください。")
                    self.safe_shutdown()
                    return
                print("接続成功！マーカーデータを受信中...")
                print("停止するには Ctrl+C を押してください")
                # メインループ（キーボード割り込まで継続）
                while True:
                    time.sleep(0.1)
            elif self.mode == 'server':
                # 先にHTTPサーバをバックグラウンドで起動
                print("HTTPサーバを起動しています...")
                self._http_thread = threading.Thread(target=self.serve_latest_http, daemon=True)
                self._http_thread.start()

                # NatNet接続を開始
                print("NatNetに接続中...")
                is_running = self.natnet_client.run('d')
                if not is_running:
                    print("エラー: ストリーミングクライアントを開始できませんでした。HTTPは稼働中です。")
                else:
                    time.sleep(1)
                    if hasattr(self.natnet_client, 'connected') and not self.natnet_client.connected():
                        print("エラー: 正常に接続できませんでした。HTTPは稼働中です。Motiveの設定を確認してください。")
                    else:
                        print("接続成功！最新データをHTTPで提供します（/latest, /marker_set）")
                print("停止するには Ctrl+C を押してください")

                # メインループ（HTTPは別スレッドで提供、ここでは待機のみ）
                while True:
                    time.sleep(0.5)
            elif self.mode == 'test':
                # テストモード: NatNet接続なし、ダミーデータのみ
                print("テストモードで起動しています...")

                # HTTPサーバをバックグラウンドで起動
                print("HTTPサーバを起動しています...")
                self._http_thread = threading.Thread(target=self.serve_latest_http, daemon=True)
                self._http_thread.start()

                # テストデータ更新スレッドを起動
                self._test_thread = threading.Thread(target=self.update_test_data, daemon=True)
                self._test_thread.start()

                print(f"テストモード起動完了: http://0.0.0.0:{self.server_port}/latest")
                print("ダミーデータをHTTPで提供します（/latest, /marker_set）")
                print("停止するには Ctrl+C を押してください")

                # メインループ（HTTPとテストデータ更新は別スレッド）
                while True:
                    time.sleep(0.5)
            else:
                print(f"不明なモードです: {self.mode}")
                self.safe_shutdown()
                return
                
        except KeyboardInterrupt:
            print("\n\nトラッキングを停止しています...")
            # HTTPサーバ停止
            try:
                if self._http_server is not None:
                    self._http_server.shutdown()
            except Exception:
                pass
            # HTTPスレッドの終了を待つ
            try:
                if self._http_thread is not None and self._http_thread.is_alive():
                    self._http_thread.join(timeout=2.0)
            except Exception:
                pass
            self.safe_shutdown()
            print("プログラムを終了しました。")
        except Exception as e:
            print(f"エラーが発生しました: {e}")
            self.safe_shutdown()
    
    def safe_shutdown(self):
        """安全なシャットダウン処理"""
        try:
            # ソケットが初期化されている場合のみシャットダウン
            if (
                hasattr(self.natnet_client, 'command_socket') and self.natnet_client.command_socket is not None and
                hasattr(self.natnet_client, 'data_socket') and self.natnet_client.data_socket is not None
            ):
                self.natnet_client.shutdown()
            else:
                # ソケットが未初期化のときは停止フラグを立てて安全停止
                try:
                    self.natnet_client.stop_threads = True
                except Exception:
                    pass
                print("接続が確立されていないため、安全にシャットダウンしました。")
        except Exception as e:
            print(f"シャットダウン中にエラーが発生しました: {e}")
            # 強制的にスレッド停止フラグを設定
            try:
                self.natnet_client.stop_threads = True
            except:
                pass

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description="OptiTrack Simple Marker Tracker")
    parser.add_argument("--server-ip", dest="server_ip", default=None, help="Motive サーバーのIPv4。未指定時は 'localhost'")
    parser.add_argument("--client-ip", dest="client_ip", default=None, help="このPCのIPv4。未指定時は 'localhost'")
    parser.add_argument("--multicast", dest="multicast", action="store_true", help="マルチキャストで接続")
    parser.add_argument("--interval", dest="interval", type=float, default=None, help="出力間隔(秒) 例: 0.5")
    parser.add_argument("--mode", dest="mode", choices=["print", "server", "test"], default="server", help="動作モード: print, server, または test")
    parser.add_argument("--port", dest="port", type=int, default=6000, help="サーバーモード時のHTTPポート")
    args = parser.parse_args()

    tracker = SimpleMarkerTracker(
        client_ip=args.client_ip,
        server_ip=args.server_ip,
        use_multicast=args.multicast,
        print_interval=(args.interval if args.interval is not None else 1.0),
        mode=args.mode,
        server_port=args.port,
    )
    tracker.start_tracking()

if __name__ == "__main__":
    main() 
