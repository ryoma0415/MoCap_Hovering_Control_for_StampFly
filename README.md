# MoCap Hovering Control for StampFly

OptiTrack/Motive から取得したモーションキャプチャ情報を用いて StampFly を原点ホバリングさせるための、PC 側（Python）、中継 ESP32、StampFly（StampS3）までを含む統合プロジェクトです。PC で PID 制御を実行し、シリアル経由で ESP32 リレーへ角度指令を送信、ESP-NOW でフライトコントローラに転送してホバリングを実現します。

```
┌────────────┐   NatNet    ┌────────────────────┐   Serial USB   ┌────────────┐   ESP-NOW   ┌─────────────┐
│ OptiTrack  │ ─────────▶ │ Python Controller  │ ───────────────▶│ ESP32 Relay│────────────▶│ StampFly S3 │
│ Motive     │            │ (hovering_controller.py)             │             │            │ Flight Ctrl │
└────────────┘            │  PID + logging + telemetry           │◀────────────┤            └─────┬───────┘
                           └────────────────────┘    Feedback    │   Feedback  │                  │
                                                                ◀──────────────┘                  │
                                                                                目標姿勢角ログ   │
                                                                                                  ↓
                                                                                            ホバリング
```

## リポジトリ構成

- `NatNet_Control_Rigid_Body_Telemetry/` – Python 側の制御・ロギング一式。NatNet クライアント、PID、位置フィルタ、テレメトリなどを収録。
  - `python_controller/hovering_controller.py` – 運用時に実行するメインスクリプト。
  - `LOG_STRUCTURE.md` – 生成される CSV ログの列構成リファレンス。
  - `test_improved_system.py` – PID・フィルタのスタンドアロンテスト。
- `esp32_relay/` – PC ⇔ StampFly 間の中継を担う ESP32-DevKitC 用 Arduino スケッチ。シリアルで受けた角度指令を ESP-NOW でブロードキャストし、フィードバックを PC へ戻します。
- `StampFly_Hovering/` – StampS3 搭載 StampFly 用 PlatformIO プロジェクト。自動離着陸シーケンス、角度指令追従、姿勢・高度制御を実装。

## 必要環境

**ハードウェア**
- OptiTrack カメラシステム（Motive でリジッドボディを構築済み）
- StampFly（StampS3 フライトコントローラ搭載）
- ESP32-DevKitC（USB シリアル接続、中継用）
- PC（Windows/macOS/Linux） – NatNet クライアントと Python 制御を実行

**ソフトウェア**
- Motive（NatNet Streaming を有効化）
- Python 3.9+（推奨 3.10/3.11）
  - `pip install -r requirements.txt` 等は用意していません。`numpy`, `pyserial` を手動でインストールしてください。
- PlatformIO CLI または VSCode + PlatformIO 拡張機能（StampFly ファームウェア書き込み用）
- Arduino IDE / PlatformIO（ESP32 リレースケッチ書き込み用）

## セットアップ手順

### 1. OptiTrack / Motive 設定
1. Motive で StampFly に取り付けた 4 マーカーからリジッドボディを作成し、**Rigid Body ID** を控えます（デフォルトは `1`）。
2. `Edit → Settings → Streaming` を開き、以下を設定。
   - `Broadcast Frame Data` を有効化。
   - `Local Interface` に PC の NIC を指定。
   - 必要に応じて Multicast のアドレス/ポートを確認（Python 側は `239.255.42.99:1511/1510` を想定）。
3. 位置と姿勢が安定して取得できることを確認します。

### 2. Python 制御環境の準備
```bash
cd NatNet_Control_Rigid_Body_Telemetry
python -m venv .venv
source .venv/bin/activate  # Windows は .venv\Scripts\activate
pip install numpy pyserial
```

必要に応じて `python_controller/hovering_controller.py` 内の以下パラメータを調整します。

| 項目 | 設定箇所 | 説明 |
| ---- | -------- | ---- |
| シリアルポート | `HoveringController.__init__` | OS ごとのデフォルトを確認し、実機に合わせて変更 (`/dev/cu.usbserial-xxxx` など)。 |
| Rigid Body ID | `self.rigid_body_id` | Motive で付与した ID に合わせて変更。 |
| IP アドレス | `start_hovering()` 内 `set_server_address`, `set_client_address` | Motive PC と制御 PC が別の場合に実ネットワークアドレスへ設定。 |
| PID ゲイン | `XYPIDController` 初期化箇所 | ホバリング応答を調整する際に変更。 |

NatNet 接続確認や PID/フィルタ単体テストを行う場合は以下が利用できます。
- `python python_controller/test_hovering.py`（位置のみ／制御テストのメニューあり）
- `python test_improved_system.py`（シミュレーションベースのユニットテスト）
- `python rigid_body_pose_stream.py --server 192.168.x.x --client 192.168.x.y`（リジッドボディの生出力確認）

### 3. ESP32 リレーの書き込み
1. `esp32_relay/esp32_relay.ino` を Arduino IDE などで開きます。
2. `broadcastAddress` に **StampFly 側**（StampS3）の ESP-NOW MAC を設定します。
3. 開発ボードとして `ESP32 Dev Module`（または使用ボード相当）を選択し、115200bps で書き込みます。
4. シリアルモニタを開くと、受信した角度指令と送信統計が 5 秒ごとに表示されます。

### 4. StampFly（StampS3）ファームウェア
1. `StampFly_Hovering/` へ移動し、PlatformIO でビルド & 書き込み。
   ```bash
   cd StampFly_Hovering
   pio run -t upload
   ```
2. `src/main.cpp` 内 `relay_mac_address` が **ESP32 リレーの Wi-Fi MAC**（`WiFi.macAddress()` で取得）になっているか確認します。
3. ファームウェアは自動離陸 → ホバリング → 指令追従 → 自動着陸のシーケンスを持ち、`angle_command_active` が true の間は外部角度指令を優先します。

## 運用手順

1. **事前確認**
   - Motive でリジッドボディ追跡が安定している。
   - ESP32 リレーが PC へ USB 接続され、正しいシリアルポートを指定済み。
   - StampFly の電源 ON、フライトコントローラが待機状態。
   - 送信機など安全装置を確認し、十分な安全距離を確保する。

2. **Python コントローラ起動**
   ```bash
   cd NatNet_Control_Rigid_Body_Telemetry/python_controller
   python hovering_controller.py
   ```
   初期化メニューが表示されたら、NatNet 接続や位置データを自動確認します。

3. **飛行**
   - コンソールに `start` と入力すると以下が順に実行されます。
     1. NatNet 接続・モデル情報取得
     2. リジッドボディ／マーカーの現在位置検証（信頼度チェック）
     3. `flight_logs/log_YYYYMMDD_HHMMSS.csv` の生成開始
     4. ESP32 リレーとのフィードバックチャネル確立
     5. 制御ループ開始（100 Hz）
     6. ESP-NOW 経由で StampFly に離陸コマンド送信
     7. 外側 PID 制御を起動し原点ホバリングへ移行
   - 飛行中は 5 Hz で現在位置・誤差・指令角度・フィードバック遅延がコンソールへ表示されます。

4. **着陸**
   - コンソールに `stop` を入力すると制御が停止し、StampFly に着陸コマンドが送られます。
   - 制御スレッドと NatNet 接続、シリアルフィードバック、ログファイルが順次クローズします。
   - `exit` でプログラム終了。Ctrl+C の場合も安全停止ルーチンが走ります。

## ログとテレメトリ

- ログファイルは `NatNet_Control_Rigid_Body_Telemetry/python_controller/flight_logs/` に自動保存されます。
- 列構造の詳細は `NatNet_Control_Rigid_Body_Telemetry/LOG_STRUCTURE.md` を参照してください。位置誤差、PID 各項、送信成否、StampFly からのフィードバック（シーケンス番号／遅延／差分）などが記録されます。
- ESP32 リレー経由で StampFly は適用したロール・ピッチ指令とシーケンス番号を `CMD_FEEDBACK` フレームとして返送します。Python 側でコマンドに対する ACK と往復遅延が測定されます。

## 主なカスタマイズポイント

- **シリアル／ネットワーク**
  - `hovering_controller.py`: `serial_port`, `rigid_body_id`, Motive IP。
  - `esp32_relay.ino`: `broadcastAddress`（ターゲット StampFly）、必要ならボーレート。
  - `StampFly_Hovering/src/main.cpp`: `relay_mac_address`（リレーの MAC）。
- **制御パラメータ**
  - Python 側 PID ゲイン (`XYPIDController`) と出力制限 (`±0.087 rad ≒ ±5°`)。
  - フィルタの外れ値しきい値・ウィンドウサイズ (`PositionFilter`)。
  - StampFly の姿勢／高度 PID は `src/flight_control.cpp` の定数で調整可能。
- **安全機能**
  - `HoveringController.confidence_zero_threshold` によりモーションキャプチャ信頼度が低下した際に出力を自動ゼロ化。
  - StampFly ファームウェア側で 200 ms 以上新しい角度指令が届かない場合は水平姿勢へ戻ります。
  - `flight_control.cpp` では過負荷検出、飛行時間制限、レンジセンサ異常時の自動降下などの安全処理を実装。

## ユーティリティ / テスト

- `python_controller/test_hovering.py`: Motive 接続と PID 応答を人間が観察できる簡易 UI。条件によっては実機飛行なしで姿勢指令生成のみを確認できます。
- `test_improved_system.py`: 位置フィルタと PID の異常値ハンドリングを確認するスクリプト。
- `rigid_body_pose_stream.py`: 指定した Rigid Body の位置・姿勢をログ出力する CLI。動作確認や座標軸の検証に有用です。
- `StampFly_Hovering/flight_control_analysis.md`, `implementation_summary.md`: フライトコントローラの状態遷移や改修内容の詳細メモ。

## トラブルシューティング

- **Motive から位置が届かない**  
  - NatNet サーバ／クライアント IP が一致しているか確認。ファイアウォールで UDP 1510/1511 が遮断されていないかチェック。
  - Rigid Body の `tracking_valid` が false の場合、Python 側が信頼度不足と判定して角度を送らない設定です。

- **ESP32 リレーがコマンドを受け取らない**  
  - Python 側のシリアルポート設定を見直し、`✓ シリアルポート xxx に接続しました` が表示されているか確認。
  - Arduino シリアルモニタ上で `=> START command sent...` のような出力が出るか確認。

- **StampFly が傾かない / 追従しない**  
  - `relay_mac_address` と `broadcastAddress` が互いの MAC と一致しているか。
  - StampFly 側で `angle_command_active` が true になっているか（USBSerial モニタに角度更新が表示されます）。
  - Python ログで `feedback_match=0` の行が続く場合はシーケンス番号が一致していない可能性があります。

- **ログが作成されない**  
  - `flight_logs/` への書き込み権限を確認。開始時に `✓ ログファイル作成: ...` が表示されているか。

## ライセンス

- `NatNetClient.py` など NatNet 関連ファイルは NaturalPoint 社のプラグイン EULA に従います（冒頭コメントを参照）。
- `StampFly_Hovering` 以下のファームウェアコードは MIT License（`LICENSE` 参照）。
- README を含むその他ファイルはプロジェクトのライセンス方針に従ってください。

---

この README はプロジェクト一式を俯瞰し、セットアップから運用までを辿れるようまとめています。現場の機体・ネットワーク構成に合わせて各種アドレスやゲインを調整し、安全を最優先に運用してください。
