# 実ロボット実装ガイド

## 概要
このドキュメントは、現在のシミュレーションコード（sim2パターン）を実際のロボットで動作させるための変更点をまとめたものです。

---

## 現在のシステム構成（シミュレーション）

### アーキテクチャ
```
main.py (メインループ)
  ↓
  ├─ World (環境モデル)
  │   ├─ 壁・障害物の座標管理
  │   └─ 空間グリッド定義
  │
  ├─ Agent (ロボット行動制御)
  │   ├─ do_sensing(): センシング実行
  │   ├─ one_step(): 1ステップの実行
  │   └─ _sim_flight2(): sim2パターンの飛行制御
  │
  ├─ Bayesian (ベイズ推論)
  │   └─ update_belief(): 事後分布更新
  │
  └─ calc() (センシングシミュレーション)
      ├─ 障害物との距離計算
      ├─ エコー信号生成（ノイズ付加）
      └─ 減衰・検出判定
```

### データフロー（現在のシミュレーション）
1. **センシング** (`agent.do_sensing()` → `calc()`)
   - 入力: ロボット位置 (PositionX, PositionY)、姿勢 (fd, pd)
   - 処理:
     - world.pole_x, pole_y から障害物座標を取得
     - 距離・角度を計算
     - エコー信号を生成（ノイズ付加）
   - 出力: r_noise（距離）, theta_noise（角度）, エコー信号

2. **ベイズ推論** (`bayesian.update_belief()`)
   - 入力: エコー信号
   - 処理: 事後確率分布を更新
   - 出力: 更新された確率マップ

3. **行動決定** (`agent._sim_flight2()`)
   - 入力: 事後確率分布
   - 処理:
     - `_analyze_posterior_for_avoidance()`: 前方の安全性評価
     - 最も安全な方向を計算
   - 出力: 回避角度 (avoid_angle), 移動距離

4. **位置更新** (シミュレーション内)
   ```python
   new_posx = posx + 0.15 * np.cos(np.deg2rad(new_fd))
   new_posy = posy + 0.15 * np.sin(np.deg2rad(new_fd))
   ```

---

## 実ロボットシステムの構成

### ハードウェア構成
1. **メインPC（このコード実行）**
   - ベイズ推論・経路計画を実行
   - 環境PC・ロボットと通信

2. **環境PC**
   - 役割: 環境情報の提供
   - 提供データ:
     - ロボットの実際の位置情報 (x, y)
     - 障害物の情報 (位置座標)
   - 通信: メインPCからの要求に応じて送信

3. **ロボット**
   - 受信: 物体定位情報（ロボットからの距離・方向）
   - 送信可能: 回避方向、移動距離、パルス放射方向

---

## 必要な変更点

### 1. 通信インターフェースの実装

#### 1.1 環境PC通信モジュール (`environment_interface.py` - 新規作成)
```python
class EnvironmentInterface:
    """環境PCとの通信を管理するクラス"""

    def __init__(self, host, port):
        """
        Args:
            host: 環境PCのIPアドレス
            port: 通信ポート番号
        """
        pass

    def request_robot_position(self):
        """
        ロボットの現在位置を取得

        Returns:
            tuple: (x, y) ロボットの位置座標 [m]
        """
        pass

    def request_obstacle_positions(self):
        """
        障害物の位置情報を取得

        Returns:
            tuple: (obstacle_x_array, obstacle_y_array) 障害物座標の配列
        """
        pass
```

**実装方法の選択肢:**
- TCP/IP ソケット通信
- UDP通信（リアルタイム性重視の場合）
- RESTful API（HTTP）
- ROS（Robot Operating System）を使用している場合はトピック通信

#### 1.2 ロボット通信モジュール (`robot_interface.py` - 新規作成)
```python
class RobotInterface:
    """ロボットとの通信を管理するクラス"""

    def __init__(self, host, port):
        """
        Args:
            host: ロボットのIPアドレス
            port: 通信ポート番号
        """
        pass

    def receive_detection_info(self):
        """
        ロボットから物体定位情報を受信

        Returns:
            list: [(distance, angle), ...]
                  distance: 物体までの距離 [mm]
                  angle: 物体の方向 [度]
        """
        pass

    def send_command(self, avoidance_direction, move_distance, pulse_direction):
        """
        ロボットに制御命令を送信

        Args:
            avoidance_direction (float): 回避方向 [度]
            move_distance (float): 移動距離 [m]
            pulse_direction (float): パルス放射方向 [度]

        Returns:
            bool: 送信成功/失敗
        """
        pass
```

---

### 2. Agentクラスの修正 (`agent.py`)

#### 2.1 新しいパターン追加: `"realRobot"`

**変更箇所1: `__init__` メソッド**
```python
def __init__(self, ..., pattern, ..., env_interface=None, robot_interface=None):
    # 既存のコード
    self.pattern = pattern

    # 実ロボット用の追加
    if self.pattern == "realRobot":
        if env_interface is None or robot_interface is None:
            raise ValueError("realRobotパターンには通信インターフェースが必要です")
        self.env_interface = env_interface
        self.robot_interface = robot_interface
```

**変更箇所2: `do_sensing` メソッドの修正**

現在のコード（シミュレーション）:
```python
def do_sensing(self, world):
    r_noise, theta_noise, y_el, y_er, ... = calc(
        world, self.PositionX, self.PositionY, self.fd, self.pd, ...
    )
```

実ロボット版:
```python
def do_sensing(self, world):
    if self.pattern == "realRobot":
        # ロボットから物体定位情報を取得
        detection_list = self.robot_interface.receive_detection_info()

        # データ形式を変換
        if detection_list:
            r_noise = np.array([d[0] for d in detection_list])  # 距離 [mm]
            theta_noise = np.array([d[1] for d in detection_list])  # 角度 [度]
        else:
            r_noise = None
            theta_noise = None

        # 環境PCから実際の位置を取得
        actual_x, actual_y = self.env_interface.request_robot_position()
        self.PositionX = actual_x
        self.PositionY = actual_y

        # エコー信号は実ロボットのセンサーから直接計算
        # （またはロボット側で計算して送信してもらう）
        y_el, y_er = self._convert_detection_to_echo(r_noise, theta_noise)

        # ベイズ推論用の行列を計算（calc関数の一部を使用）
        current_obs_goback_dist_matrix_L, current_obs_goback_dist_matrix_R, current_confidence_matrix = \
            self._compute_observation_matrices(world)

    else:
        # 既存のシミュレーションコード
        r_noise, theta_noise, y_el, y_er, ... = calc(world, ...)
```

**変更箇所3: `one_step` メソッドの修正**

現在のコード:
```python
def one_step(self, step_idx, visualizer):
    # ...
    elif self.pattern == "sim2":
        self.PositionX, self.PositionY, self.fd, self.pd, flag = \
            self._sim_flight2(self.PositionX, self.PositionY, self.fd, self.pd, visualizer)
```

実ロボット版:
```python
def one_step(self, step_idx, visualizer):
    # ...
    elif self.pattern == "realRobot":
        self.PositionX, self.PositionY, self.fd, self.pd, flag = \
            self._real_robot_flight(self.PositionX, self.PositionY, self.fd, self.pd, visualizer)
```

#### 2.2 新メソッド: `_real_robot_flight` の実装

```python
def _real_robot_flight(self, posx, posy, fd, pd, visualizer):
    """
    実ロボット用の飛行制御

    sim2と同じロジックで回避方向を計算し、ロボットに送信する
    """
    # 事後分布を取得（sim2と同じ）
    posterior_sel, X_sel, Y_sel = self._plot_posterior_distribution(
        posx=self.PositionX,
        posy=self.PositionY,
        pd=self.pd,
        fd=self.fd,
    )

    # 回避方向の計算（sim2と同じ）
    angle_results, avoid_angle, value, flag = \
        self._analyze_posterior_for_avoidance(X_sel, Y_sel, posterior_sel)

    # 新しい方向を計算
    new_fd = self.normalize_angle_deg(fd - avoid_angle)
    new_pd = self.normalize_angle_deg(pd - (avoid_angle))
    if self.step_idx >= 6:
        new_pd = self.normalize_angle_deg(fd - (avoid_angle * 2))

    # 移動距離の決定
    if flag == True:
        move_distance = 0.15  # [m]
    else:
        move_distance = 0.05  # [m]

    # ロボットに命令を送信
    success = self.robot_interface.send_command(
        avoidance_direction=-avoid_angle,  # 回避方向
        move_distance=move_distance,        # 移動距離
        pulse_direction=new_pd              # パルス放射方向
    )

    if not success:
        print("警告: ロボットへの命令送信に失敗しました")
        return posx, posy, fd, pd, False

    # 実際の位置は環境PCから次のステップで取得するため、
    # ここでは予測位置を返す（参考値）
    predicted_x = posx + move_distance * np.cos(np.deg2rad(new_fd))
    predicted_y = posy + move_distance * np.sin(np.deg2rad(new_fd))

    print(f"ロボットに送信: 回避角度={-avoid_angle:.2f}度, 移動={move_distance:.2f}m, パルス方向={new_pd:.2f}度")

    return predicted_x, predicted_y, new_fd, new_pd, flag
```

---

### 3. Worldクラスの修正 (`world.py`)

実ロボットモードでは、障害物情報を環境PCから動的に取得する必要があります。

**変更箇所: 動的障害物更新メソッドの追加**

```python
class World:
    def update_obstacles_from_environment(self, env_interface):
        """
        環境PCから障害物情報を取得して更新

        Args:
            env_interface: EnvironmentInterfaceのインスタンス
        """
        obstacle_x, obstacle_y = env_interface.request_obstacle_positions()
        self.pole_x = obstacle_x
        self.pole_y = obstacle_y
        print(f"障害物情報を更新: {len(obstacle_x)}個の障害物")
```

---

### 4. calcモジュールの修正 (`calc.py`)

実ロボットモードでは、センシング部分はロボットの実際のセンサーから取得するため、
`calc`関数の一部機能のみを使用します。

**オプション1: 既存のcalc関数を分割**
- センシング部分: ロボットから直接取得に置き換え
- ベイズ推論用行列計算: そのまま使用

**オプション2: calc関数にモードフラグを追加**
```python
def calc(world, current_bat_x, current_bat_y, current_fd, current_pd, X, Y,
         mode="simulation", real_detections=None):
    """
    Args:
        mode: "simulation" or "real_robot"
        real_detections: 実ロボットの場合の検出情報
    """
    if mode == "real_robot":
        # 実ロボットからの検出情報を使用
        r_noise = real_detections['distances']
        theta_noise = real_detections['angles']
        # ... 以下、行列計算のみ実行
    else:
        # 既存のシミュレーションコード
        # ...
```

---

### 5. main.pyの修正

実ロボットモードでの実行設定:

```python
if __name__ == "__main__":
    # 通信インターフェースの初期化
    from environment_interface import EnvironmentInterface
    from robot_interface import RobotInterface

    # 環境PC接続
    env_interface = EnvironmentInterface(
        host="192.168.1.100",  # 環境PCのIP
        port=5000
    )

    # ロボット接続
    robot_interface = RobotInterface(
        host="192.168.1.101",  # ロボットのIP
        port=5001
    )

    # Worldの初期化（実ロボットモードでは初期障害物は環境PCから取得）
    world = World(
        x_max=8.5,
        y_max=8.5,
        margin_space=2,
        h=0.01,
        t_max=64*10**-3,
        dt=100*10**-6,
        c=340,
        folder_name="real_robot_data"
    )

    # 環境PCから障害物情報を取得
    world.update_obstacles_from_environment(env_interface)

    bayesian = Bayesian(
        sigma2=(0.0001 * world.c) ** 2,
        min_p=-320,
        c=world.c
    )

    agent = Agent(
        bayesian=bayesian,
        margin_space=world.margin_space,
        folder_name=world.folder_name,
        X=world.X,
        Y=world.Y,
        pattern="realRobot",  # 実ロボットモード
        sim={
            "trials": 200,
            "init_pos": [2.5, 6.05, 270, 270]
        },
        Avoider_k=0.005,
        Avoider_alpha=100.,
        world=world,
        env_interface=env_interface,      # 追加
        robot_interface=robot_interface    # 追加
    )

    # 環境PCから初期位置を取得
    agent.PositionX, agent.PositionY = env_interface.request_robot_position()

    bayesian.Init(world, agent)

    visualizer = BatVisualizer(...)

    # メインループ
    for i in range(0, 200):
        print(f"trial {i} start")

        if i != 0:
            flag = agent.one_step(i, visualizer)
        else:
            flag = True

        # センシング実行（ロボットから検出情報を受信）
        y_x, y_y, y_el_vec, y_er_vec, data1, data2, data3, data4 = \
            agent.do_sensing(world)

        # 可視化
        visualizer.plot_frame(...)

        # 定期的に環境情報を更新（必要に応じて）
        if i % 10 == 0:
            world.update_obstacles_from_environment(env_interface)
```

---

## 実装の優先順位

### Phase 1: 通信インターフェースの実装とテスト
1. `environment_interface.py` の実装
2. `robot_interface.py` の実装
3. 通信テストプログラムの作成
4. 接続確認

### Phase 2: Agentクラスの拡張
1. `realRobot` パターンの追加
2. `_real_robot_flight` メソッドの実装
3. `do_sensing` の実ロボット対応
4. シミュレーションとの動作比較テスト

### Phase 3: 統合とデバッグ
1. main.pyでの実ロボットモード実行
2. 通信遅延の測定と対策
3. エラーハンドリングの強化
4. ログ記録機能の追加

### Phase 4: 最適化
1. 通信頻度の最適化
2. リアルタイム性の改善
3. 安全機能の追加（緊急停止など）

---

## 注意点とリスク

### 1. 通信プロトコルの設計
- **問題**: 環境PC・ロボットとの通信プロトコルが未定義
- **対策**:
  - JSONベースのシンプルなプロトコルを推奨
  - タイムスタンプを必ず含める
  - 通信エラー時のリトライ機構

### 2. 座標系の一致
- **問題**: シミュレーションとロボットの座標系が異なる可能性
- **対策**:
  - 座標変換関数を実装
  - キャリブレーション手順の確立

### 3. タイミング同期
- **問題**: センシング→計算→命令送信のタイムラグ
- **対策**:
  - 各処理の実行時間を測定
  - 必要に応じて予測制御を導入

### 4. 単位の統一
- **問題**: 距離の単位（m vs mm）、角度（度 vs ラジアン）
- **現在のコード**:
  - 距離: 基本的に [m]、ロボットからは [mm] で受信する想定
  - 角度: 基本的に [度]
- **対策**:
  - インターフェース層で単位変換を実施
  - コメントで単位を明記

### 5. エラーハンドリング
- 通信エラー時の動作
- センサー異常時の対処
- ロボット応答なし時のタイムアウト

---

## テスト計画

### 1. 単体テスト
- [ ] EnvironmentInterface通信テスト
- [ ] RobotInterface通信テスト
- [ ] 座標変換の正確性テスト

### 2. 統合テスト
- [ ] シミュレーション vs 実ロボットの動作比較
- [ ] 長時間実行での安定性確認
- [ ] 通信エラー時の挙動確認

### 3. 実機テスト
- [ ] 静止状態でのセンシング精度確認
- [ ] 単純な直進移動テスト
- [ ] 障害物回避テスト

---

## まとめ

### 主な変更ファイル
1. **新規作成**:
   - `environment_interface.py`: 環境PC通信
   - `robot_interface.py`: ロボット通信

2. **修正**:
   - `agent.py`: realRobotパターン追加、新メソッド実装
   - `world.py`: 動的障害物更新メソッド追加
   - `main.py`: 実ロボットモードの実行設定
   - `calc.py`: （オプション）実ロボットモード対応

### キーポイント
- **センシング**: ロボットから直接取得（calc関数の一部を置き換え）
- **位置取得**: 環境PCから要求ベースで取得
- **行動制御**: sim2と同じロジックで計算し、ロボットに送信
- **ベイズ推論**: 既存のロジックをそのまま使用可能

### 次のステップ
1. 環境PC・ロボットとの通信プロトコル仕様を確認
2. 通信インターフェースの実装
3. 段階的な実装とテスト
