# StampFly flight_control.cpp loop_400Hz関数 詳細分析

## 現在の動作フロー

### 1. Mode定義（flight_control.hpp）
```
INIT_MODE (0)         - 初期化モード
AVERAGE_MODE (1)      - センサーオフセット計算モード
FLIGHT_MODE (2)       - 飛行モード
PARKING_MODE (3)      - 待機/停止モード
AUTO_LANDING_MODE (5) - 自動着陸モード
```

### 2. AutoFlightState定義（flight_control.cpp）
```
AUTO_INIT (0)     - 初期化
AUTO_WAIT (1)     - 待機（センサーキャリブレーション）
AUTO_TAKEOFF (2)  - 離陸
AUTO_HOVER (3)    - ホバリング
AUTO_LANDING (4)  - 着陸
AUTO_COMPLETE (5) - 完了
```

## 現在の状態遷移フロー

### 電源ON後の動作シーケンス

1. **INIT_MODE (Mode=0)**
   - モーター停止
   - 各種オフセットリセット
   - AVERAGE_MODEへ自動遷移

2. **AVERAGE_MODE (Mode=1)**
   - モーター停止
   - センサーオフセット計算（800回平均）
   - 完了後PARKING_MODEへ自動遷移
   - **auto_state変更なし**

3. **PARKING_MODE (Mode=3)**
   - **auto_state = AUTO_INIT → AUTO_WAIT**
     - タイマー開始
     - 3秒待機
   - **auto_state = AUTO_WAIT → AUTO_TAKEOFF**
     - 3秒経過後
     - AHRSリセット（20回）
     - Mode = FLIGHT_MODEへ遷移
     - Alt_ref = 0.5m設定

4. **FLIGHT_MODE (Mode=2)**
   - **auto_state = AUTO_TAKEOFF**
     - 50cmまで上昇
     - 到達後 → auto_state = AUTO_HOVER
   - **auto_state = AUTO_HOVER**
     - 5秒間ホバリング
     - 5秒後 → Mode = AUTO_LANDING_MODE, auto_state = AUTO_LANDING

5. **AUTO_LANDING_MODE (Mode=5)**
   - **auto_state = AUTO_LANDING**
     - auto_landing()関数実行
     - 着陸完了 → Mode = PARKING_MODE, auto_state = AUTO_COMPLETE

6. **PARKING_MODE (Mode=3)**
   - **auto_state = AUTO_COMPLETE**
     - モーター停止
     - 終了状態

## 問題点（冗長性）

### 1. 二重の状態管理
- **Mode**: 大まかな動作モード（飛行/停止/着陸など）
- **auto_state**: 自動飛行シーケンスの詳細状態

### 2. 状態遷移の複雑性
- ModeとAuto_stateが相互に影響
- PARKING_MODE内でFLIGHT_MODEへの遷移
- FLIGHT_MODE内でAUTO_LANDING_MODEへの遷移
- AUTO_LANDING_MODE内でPARKING_MODEへの遷移

### 3. 条件分岐の重複
- Mode判定とauto_state判定が混在
- 同じ処理が複数箇所に分散

## 改善案（auto_stateのみ使用）

### 新しい状態定義
```cpp
enum AutoFlightState {
    STATE_INIT = 0,        // 初期化
    STATE_CALIBRATION = 1, // センサーキャリブレーション
    STATE_WAIT = 2,        // 待機
    STATE_TAKEOFF = 3,     // 離陸
    STATE_HOVER = 4,       // ホバリング
    STATE_LANDING = 5,     // 着陸
    STATE_IDLE = 6         // アイドル（モーター停止）
};
```

### 新しい状態遷移
1. STATE_INIT → STATE_CALIBRATION（自動）
2. STATE_CALIBRATION → STATE_WAIT（800回完了後）
3. STATE_WAIT → STATE_TAKEOFF（3秒後）
4. STATE_TAKEOFF → STATE_HOVER（50cm到達）
5. STATE_HOVER → STATE_LANDING（5秒後）
6. STATE_LANDING → STATE_IDLE（着陸完了）

### 利点
- 単一の状態変数で管理
- シンプルな状態遷移
- 処理の重複削除
- デバッグが容易

## 実装時の注意点

### タイミング管理
- Control_period（400Hz制御周期）の維持
- phase_start_timeの適切な更新
- auto_timerの管理

### 安全機能の維持
- OverG_flag検出時の緊急停止
- MAX_FLIGHT_TIME超過時の自動着陸
- 電圧低下時の処理
- Range0flag処理

### 制御ロジックの統合
- angle_control()とrate_control()の呼び出しタイミング
- motor_stop()の呼び出し条件
- PIDリセットのタイミング
- AHRSリセットの実行

### 変数の初期化
- Auto_takeoff_counterの管理
- Landing_stateの管理
- 各種フィルターのリセット