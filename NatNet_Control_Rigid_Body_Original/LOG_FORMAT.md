# ホバリング制御ログフォーマット解説

`NatNet_Control_Imporoved/python_controller/hovering_controller.py` を実行すると、実行ディレクトリ配下に `flight_logs/log_YYYYMMDD_HHMMSS.csv` が生成されます。本書はこの CSV の列構造と意味を整理し、グラフ化や解析を行う際のリファレンスとして利用できるようにまとめたものです。

## 収録仕様
- **サンプリング周期**: 制御ループ 100 Hz（ループ時間を `loop_time_ms` 列に記録）
- **記録タイミング**: ホバリング制御が有効な間は常時。モーションキャプチャが得られない場合は 0 指令のままログを継続。
- **ファイル形式**: UTF-8 / カンマ区切り

## 列一覧

### 時刻・基本情報
| 列名 | 意味 / 単位 | 備考 |
| ---- | ----------- | ---- |
| `timestamp` | ISO8601 形式のローカル時刻 | 例: `2025-03-07T12:34:56.789123` |
| `elapsed_time` | ログ開始からの経過時間 [s] | float 文字列。後処理時は `float()` で使用 |

### 位置・誤差
| 列名 | 意味 / 単位 | 備考 |
| ---- | ----------- | ---- |
| `pos_x`, `pos_y`, `pos_z` | フィルタ後のドローン位置 [m] | ドローン座標系 (X: 前方, Y: 右, Z: 上) |
| `raw_pos_x`, `raw_pos_y`, `raw_pos_z` | フィルタ入力となった生データ [m] | リジッドボディ or マーカー平均 |
| `error_x`, `error_y` | 目標原点との差 [m] | Z 誤差は制御していないため記録なし |

### 角度指令・PID 成分
| 列名 | 意味 / 単位 | 備考 |
| ---- | ----------- | ---- |
| `roll_ref_rad`, `pitch_ref_rad` | ESP32 へ送信したロール/ピッチ角指令 [rad] | 信頼度によるスケーリング後の値 |
| `roll_ref_deg`, `pitch_ref_deg` | 上記を度単位で換算 [deg] | 可視化しやすいよう冗長記録 |
| `pid_x_p`, `pid_x_i`, `pid_x_d` | X 軸 PID の各成分 | ラジアン換算後の出力寄与 |
| `pid_y_p`, `pid_y_i`, `pid_y_d` | Y 軸 PID の各成分 | |

### フレームメタ情報
| 列名 | 意味 / 単位 | 備考 |
| ---- | ----------- | ---- |
| `frame_number` | NatNet 受信フレーム番号 | 0 の場合は未取得 |
| `marker_count` | 使用したマーカー数 | リジッドボディが有効な場合は rb 内マーカー数 |
| `send_success` | ESP32 送信結果 (1=成功, 0=失敗) | PySerial 書き込み例外を検知 |
| `control_active` | 制御ループが姿勢指令を出しているか | `start` 後に 1 となる |
| `loop_time_ms` | 制御ループ 1 周の実測時間 [ms] | 負荷監視用 |

### フィルタリング・信頼度
| 列名 | 意味 / 単位 | 備考 |
| ---- | ----------- | ---- |
| `is_outlier` | 外れ値判定 (1=True, 0=False) | 直近サンプルが閾値を超えたか |
| `used_prediction` | 予測による補完使用 (1=True, 0=False) | 連続欠測時に使用 |
| `confidence` | 信頼度 (0.0–1.0) | PID 出力スケールに使用 |
| `consecutive_outliers` | 連続外れ値カウント | 異常監視用 |
| `data_valid` | PID に有効データを渡したか | 0 の場合は roll/pitch=0 |
| `data_source` | 位置のソース種別 | `"rigid_body"`, `"markers"`, `"none"` |
| `filter_threshold` | 動的外れ値閾値 [m] | 空文字の場合は初期化直後 |
| `tracking_valid` | NatNet トラッキングフラグ (1/0) | リジッドボディ状態を反映 |

### リジッドボディ関連
| 列名 | 意味 / 単位 | 備考 |
| ---- | ----------- | ---- |
| `rb_error` | リジッドボディのマーカー RMS エラー | Motive 提供値。小さいほど良好 |
| `rb_marker_count` | リジッドボディに関連付くマーカー数 | |
| `rb_pos_x`, `rb_pos_y`, `rb_pos_z` | リジッドボディの平行移動 (ドローン座標) [m] | トラッキング無効時は空 |
| `rb_qx`, `rb_qy`, `rb_qz`, `rb_qw` | Motive からの四元数要素 | (x, y, z, w) |
| `rb_roll_deg`, `rb_pitch_deg`, `rb_yaw_deg` | 四元数を XYZ オイラー角に変換 [deg] | 解析・可視化向け |

## 解析時のヒント
- **姿勢安定性評価**: `error_x`, `error_y` の RMS や `roll_ref_deg`, `pitch_ref_deg` の時間推移をプロットすると応答の傾向が分かりやすい。
- **信頼度モニタ**: `confidence`、`data_source`、`consecutive_outliers` を組み合わせると計測欠落時の挙動やフォールバック動作を確認できる。
- **リジッドボディ姿勢**: `rb_roll_deg` などをプロットすることで光学姿勢推定の揺れを把握できる。IMU との比較にも活用可能。
- **ループ負荷**: `loop_time_ms` をヒストグラム化し、100 Hz 周期に対する余裕を確認する。

## グラフ化の例（擬似コード）
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("flight_logs/log_20250307_123456.csv")

fig, ax = plt.subplots()
ax.plot(df["elapsed_time"], df["pos_x"], label="pos_x (m)")
ax.plot(df["elapsed_time"], df["pos_y"], label="pos_y (m)")
ax.set_xlabel("Elapsed time [s]")
ax.set_ylabel("Position [m]")
ax.legend()
ax.grid(True)
plt.show()
```

必要に応じて本ドキュメントを更新し、追加列を導入した際はここに追記してください。
