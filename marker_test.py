#!/usr/bin/env python3
# -*- coding: utf-8 -*-

r"""
マーカーデータ取得テスター（/latest 全件表示／特定モデル抽出）

概要:
- `marker_tracker.py` をサーバモードで起動したホストに対して HTTP で問い合わせ、
  最新スナップショットを日本語でわかりやすく「全部」表示します。
- `--model` を指定すると、特定マーカーセット（Model Name）だけを `/marker_set` 経由で取得できます。

使い方（例）:
- 全データを整形表示（/latest）
    python robot_bayse/marker_test.py --host localhost --port 6000 [--interval 1.0]

- 生のJSONで出力
    python robot_bayse/marker_test.py --host localhost --port 6000 --raw

- 特定マーカーセット（例: robot_head）のみ
    python robot_bayse/marker_test.py --host localhost --port 6000 --model robot_head

引数:
- --host      サーバーホスト（既定: localhost）
- --port      サーバポート（既定: 6000）
- --interval  ポーリング間隔（秒, 既定: 1.0）
- --raw       スナップショットを生のJSONで出力
- --model     /marker_set?name=... で指定したモデルのマーカー座標のみ取得

注意:
- 利用前に `marker_tracker.py` をサーバモードで起動してください。
- 取得データ量が多い場合はコンソール出力が増えます。必要に応じて `--raw` で機械可読なJSONを保存してください。

出力例:

  1) /latest（全データ整形表示）

      (deskvenv) C:\Users\TeamRobot\Documents\mana_test>python robot_bayse\marker_test.py
      マーカーサーバへポーリング: http://localhost:6000/latest  （1.0秒ごと, Ctrl+Cで停止）
      [時刻 18:10:12] フレーム番号: 871510
      ラベル付きマーカー数: 6 | マーカーセット数: 2 | ラベルなしマーカー数: 0 | レガシーマーカー数: 3 | 剛体数: 1
      - ラベル付きマーカー一覧:
        [1] ID=65537 位置=(1.617, 0.197, 0.903)
        [2] ID=65538 位置=(1.638, 0.192, 0.905)
        [3] ID=65539 位置=(1.598, 0.192, 0.898)
        [4] ID=16027 位置=(1.636, 0.273, 0.825)
        [5] ID=16029 位置=(2.016, -0.032, 0.888)
        [6] ID=16454 位置=(1.985, -0.009, 0.760)
      - マーカーセット一覧:
        [1] セット名='robot_head' マーカー数=3
            マーカー1: X=1.617, Y=0.197, Z=0.903
            マーカー2: X=1.638, Y=0.192, Z=0.905
            マーカー3: X=1.598, Y=0.192, Z=0.898
        [2] セット名='all' マーカー数=3
            マーカー1: X=1.617, Y=0.197, Z=0.903
            マーカー2: X=1.638, Y=0.192, Z=0.905
            マーカー3: X=1.598, Y=0.192, Z=0.898
      - レガシーその他マーカー一覧:
        [1] X=1.636, Y=0.273, Z=0.825
        [2] X=2.016, Y=-0.032, Z=0.888
        [3] X=1.985, Y=-0.009, Z=0.760
      - 剛体一覧:
        [1] ID=1 位置=(1.618, 0.194, 0.902) 追跡有効=True

  2) /marker_set（特定モデルの抽出）

      (deskvenv) C:\Users\TeamRobot\Documents\mana_test>python robot_bayse\marker_test.py --model robot_head
      マーカーサーバへポーリング: http://localhost:6000/marker_set?name=robot_head  （1.0秒ごと, Ctrl+Cで停止）
      [時刻 18:10:35] マーカーセット='robot_head' マーカー数=3
        マーカー1: X=1.617, Y=0.197, Z=0.903
        マーカー2: X=1.638, Y=0.192, Z=0.905
        マーカー3: X=1.598, Y=0.192, Z=0.897
"""

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
import urllib.parse
from typing import Any, Dict, Optional, List


def fetch_latest(host: str, port: int, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
    url = f"http://{host}:{port}/latest"
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status != 200:
                return None
            data = resp.read()
            text = data.decode("utf-8", errors="replace")
            return json.loads(text)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ConnectionError):
        return None
    except Exception:
        return None


def fetch_marker_set(host: str, port: int, name: str, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
    qs = urllib.parse.urlencode({"name": name})
    url = f"http://{host}:{port}/marker_set?{qs}"
    req = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status != 200:
                return None
            data = resp.read()
            text = data.decode("utf-8", errors="replace")
            return json.loads(text)
    except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ConnectionError):
        return None
    except Exception:
        return None

def fmt_time(ts: Any) -> str:
    if isinstance(ts, (int, float)):
        try:
            return time.strftime("%H:%M:%S", time.localtime(float(ts)))
        except Exception:
            return str(ts)
    return "-"


def human_readable_full(snapshot: Dict[str, Any]) -> str:
    lines: List[str] = []
    ts = snapshot.get("timestamp")
    frame = snapshot.get("frame")
    labeled = snapshot.get("labeled_markers") or []
    marker_sets = snapshot.get("marker_sets") or []
    unlabeled = snapshot.get("unlabeled_markers") or []
    legacy = snapshot.get("legacy_other_markers") or []
    rigid = snapshot.get("rigid_bodies") or []

    # ヘッダ
    lines.append(f"[時刻 {fmt_time(ts)}] フレーム番号: {frame}")

    # サマリ
    lines.append(
        " | ".join(
            [
                f"ラベル付きマーカー数: {len(labeled)}",
                f"マーカーセット数: {len(marker_sets)}",
                f"ラベルなしマーカー数: {len(unlabeled)}",
                f"レガシーマーカー数: {len(legacy)}",
                f"剛体数: {len(rigid)}",
            ]
        )
    )

    # ラベル付きマーカー（全件）
    if labeled:
        lines.append("- ラベル付きマーカー一覧:")
        for i, m in enumerate(labeled, start=1):
            pos = m.get("pos") or [None, None, None]
            mid = m.get("id")
            try:
                lines.append(f"  [{i}] ID={mid} 位置=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            except Exception:
                lines.append(f"  [{i}] ID={mid} 位置={pos}")

    # マーカーセット（全件）
    if marker_sets:
        lines.append("- マーカーセット一覧:")
        for sidx, ms in enumerate(marker_sets, start=1):
            name = ms.get("name")
            markers = ms.get("markers") or []
            lines.append(f"  [{sidx}] セット名='{name}' マーカー数={len(markers)}")
            for midx, pos in enumerate(markers, start=1):
                try:
                    lines.append(f"      マーカー{midx}: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
                except Exception:
                    lines.append(f"      マーカー{midx}: {pos}")

    # ラベルなしマーカー（全件）
    if unlabeled:
        lines.append("- ラベルなしマーカー一覧:")
        for i, pos in enumerate(unlabeled, start=1):
            try:
                lines.append(f"  [{i}] X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
            except Exception:
                lines.append(f"  [{i}] {pos}")

    # レガシーその他マーカー（全件）
    if legacy:
        lines.append("- レガシーその他マーカー一覧:")
        for i, pos in enumerate(legacy, start=1):
            try:
                lines.append(f"  [{i}] X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
            except Exception:
                lines.append(f"  [{i}] {pos}")

    # 剛体（全件）
    if rigid:
        lines.append("- 剛体一覧:")
        for i, rb in enumerate(rigid, start=1):
            pos = rb.get("pos") or [None, None, None]
            rid = rb.get("id")
            tv = rb.get("tracking_valid")
            try:
                lines.append(f"  [{i}] ID={rid} 位置=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) 追跡有効={tv}")
            except Exception:
                lines.append(f"  [{i}] ID={rid} 位置={pos} 追跡有効={tv}")

    # スナップショットに未知キーがあれば最後にダンプ
    known = {"timestamp", "frame", "labeled_markers", "marker_sets", "unlabeled_markers", "legacy_other_markers", "rigid_bodies"}
    extras = {k: v for k, v in snapshot.items() if k not in known}
    if extras:
        lines.append("- 追加フィールド:")
        try:
            lines.append(json.dumps(extras, ensure_ascii=False))
        except Exception:
            lines.append(str(extras))

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Poll marker server and print latest snapshot")
    parser.add_argument("--host", default="localhost", help="Server host (default: localhost)")
    parser.add_argument("--port", type=int, default=6000, help="Server port (default: 6000)")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval seconds (default: 1.0)")
    parser.add_argument("--raw", action="store_true", help="Print raw JSON instead of formatted output")
    parser.add_argument("--model", default=None, help="Model name to fetch via /marker_set endpoint")
    args = parser.parse_args()

    host = args.host
    port = int(args.port)
    interval = float(args.interval)

    if args.model:
        print(f"マーカーサーバへポーリング: http://{host}:{port}/marker_set?name={args.model}  （{interval}秒ごと, Ctrl+Cで停止）")
    else:
        print(f"マーカーサーバへポーリング: http://{host}:{port}/latest  （{interval}秒ごと, Ctrl+Cで停止）")
    try:
        while True:
            if args.model:
                payload = fetch_marker_set(host, port, args.model)
                if not payload or not payload.get("ok", False):
                    print("データなし（サーバ未準備、またはモデル未検出）")
                else:
                    name = payload.get("name")
                    count = payload.get("count")
                    markers = payload.get("markers") or []
                    ts = payload.get("timestamp")
                    timestr = fmt_time(ts)
                    print(f"[時刻 {timestr}] マーカーセット='{name}' マーカー数={count}")
                    for i, pos in enumerate(markers, start=1):
                        try:
                            print(f"  マーカー{i}: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f}")
                        except Exception:
                            print(f"  マーカー{i}: {pos}")
            else:
                payload = fetch_latest(host, port)
                if not payload or not payload.get("ok", False):
                    print("データなし（サーバ未準備またはスナップショット未到達）")
                else:
                    snap = payload.get("snapshot")
                    if snap is None:
                        print("スナップショットなし")
                    else:
                        if args.raw:
                            print(json.dumps(snap, ensure_ascii=False))
                        else:
                            print(human_readable_full(snap))
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n停止しました。")
        return 0


if __name__ == "__main__":
    sys.exit(main())
