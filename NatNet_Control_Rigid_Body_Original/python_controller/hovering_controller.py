#!/usr/bin/env python3
"""
Optitrack原点ホバリング制御プログラム
NatNetでドローン位置を取得し、PID制御で角度指令を生成してESP32へ送信
"""

import serial
import struct
import time
import threading
import sys
import os
import csv
import atexit
from datetime import datetime
from math import asin, atan2, copysign, degrees, pi

# 親ディレクトリのパスを追加（NatNetClient等のインポート用）
# 現在のプロジェクト内のNatNet_Controlディレクトリを参照
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

from NatNetClient import NatNetClient
from pid_controller import XYPIDController
from position_filter import PositionFilter

class HoveringController:
    """Optitrackを使用した原点ホバリング制御クラス"""
    
    def __init__(self, serial_port=None, baudrate=115200):
        """
        初期化
        Args:
            serial_port: ESP32と接続するシリアルポート
            baudrate: ボーレート
        """
        # シリアル通信設定
        # OSに応じてデフォルトポートを設定
        if serial_port is None:
            import platform
            if platform.system() == 'Windows':
                self.serial_port = "COM3"  # Windowsのデフォルト
            elif platform.system() == 'Darwin':  # macOS
                self.serial_port = "/dev/cu.usbserial-1110"
            else:  # Linux
                self.serial_port = "/dev/ttyUSB0"
        else:
            self.serial_port = serial_port
        
        self.baudrate = baudrate
        self.ser = None
        
        # 制御状態
        self.is_flying = False
        self.control_active = False
        self.shutdown_flag = False
        
        # ドローン位置（最新値を保持）
        self.current_position = None
        self.last_valid_position = None
        self.position_lock = threading.Lock()
        
        # PIDコントローラ（拡張版：D項フィルタとI項制御付き）
        # ユーザー指定のゲインを使用: kp=0.139, ki=0.020, kd=0.204
        self.pid_controller = XYPIDController(
            kp_x=0.139, ki_x=0.020, kd_x=0.204,  # X軸（ロール）
            kp_y=-0.139, ki_y=-0.020, kd_y=-0.204,  # Y軸（ピッチ）- 符号反転
            output_limit=(-0.087, 0.087),  # ラジアン制限（約±5度）
            d_filter_alpha=0.6,  # D項フィルタ係数（0.6で適度な平滑化）
            i_decay_rate=0.98,  # I項減衰率
            i_update_threshold=0.3,  # I項更新閾値 [m]
            enable_i_control=True  # I項制御を有効化
        )

        # 位置フィルタ（異常値除去と平滑化）
        self.position_filter = PositionFilter(
            window_size=5,  # 移動平均ウィンドウ
            outlier_threshold=0.1,  # 異常値判定閾値 [m]
            velocity_window=3,  # 速度推定ウィンドウ
            enable_prediction=True  # 位置予測を有効化
        )
        self.rigid_body_id = 1  # 使用するリジッドボディID（StampFly）
        self.last_rigid_body_pose = None
        self.last_data_source = "markers"
        self.confidence_zero_threshold = 0.2  # 信頼度がこの値未満なら指令をゼロ化
        self.last_filter_result = None
        
        # 目標位置（原点）
        self.target_position = (0.0, 0.0)
        
        # NatNetクライアント
        self.natnet_client = None
        
        # 制御スレッド
        self.control_thread = None
        
        # 統計情報
        self.packets_sent = 0
        self.last_send_time = 0
        
        # データロギング関連
        self.log_file = None
        self.csv_writer = None
        self.current_frame_number = 0
        self.current_marker_count = 0
        self.log_start_time = None

        # フィルタリング関連の統計
        self.filter_stats = {
            'outliers_detected': 0,
            'predictions_used': 0,
            'total_frames': 0
        }
        
        # atexit登録（プログラム終了時にファイルを確実にクローズ）
        atexit.register(self.close_log_file)
        
    def close_log_file(self):
        """ログファイルを安全にクローズ"""
        if self.log_file:
            try:
                self.log_file.close()
                print(f"✓ ログファイルを保存しました")
            except:
                pass
            self.log_file = None
            self.csv_writer = None
    
    def start_logging(self):
        """新規ログファイルを作成して記録開始"""
        # タイムスタンプ付きファイル名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "flight_logs"
        
        # ログディレクトリ作成
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        filename = os.path.join(log_dir, f"log_{timestamp}.csv")
        
        try:
            # 既存のファイルをクローズ
            self.close_log_file()
            
            # 新規ファイルオープン（行バッファリングで即座に書き込み）
            self.log_file = open(filename, 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.log_file)
            
            # ヘッダー書き込み（フィルタ関連情報を追加）
            headers = [
                'timestamp', 'elapsed_time',
                'pos_x', 'pos_y', 'pos_z',
                'raw_pos_x', 'raw_pos_y', 'raw_pos_z',  # 生データ
                'error_x', 'error_y',
                'roll_ref_rad', 'pitch_ref_rad',
                'roll_ref_deg', 'pitch_ref_deg',
                'pid_x_p', 'pid_x_i', 'pid_x_d',
                'pid_y_p', 'pid_y_i', 'pid_y_d',
                'frame_number', 'marker_count',
                'send_success', 'control_active',
                'loop_time_ms',
                # フィルタ関連
                'is_outlier', 'used_prediction', 'confidence',
                'consecutive_outliers', 'data_valid',
                'data_source', 'filter_threshold', 'tracking_valid',
                'rb_error', 'rb_marker_count',
                'rb_pos_x', 'rb_pos_y', 'rb_pos_z',
                'rb_qx', 'rb_qy', 'rb_qz', 'rb_qw',
                'rb_roll_deg', 'rb_pitch_deg', 'rb_yaw_deg'
            ]
            self.csv_writer.writerow(headers)
            
            self.log_start_time = time.time()
            print(f"✓ ログファイル作成: {filename}")
            
        except Exception as e:
            print(f"✗ ログファイル作成エラー: {e}")
            self.log_file = None
            self.csv_writer = None
    
    def log_data(self, pos_x, pos_y, pos_z, error_x, error_y,
                  roll_ref, pitch_ref, send_success, loop_time,
                  filter_result=None, is_data_valid=True,
                  confidence=1.0, data_source=None):
        """制御データをCSVに記録（拡張版）"""
        if not self.csv_writer:
            return

        try:
            # PID成分取得
            pid_components = self.pid_controller.get_all_components()

            # 現在時刻と経過時間
            current_time = time.time()
            elapsed = current_time - self.log_start_time if self.log_start_time else 0

            # フィルタ関連情報の取得
            threshold = None
            data_source_val = data_source if data_source is not None else self.last_data_source
            tracking_valid_flag = 1 if is_data_valid else 0
            rb_error = None
            confidence_val = confidence

            if filter_result:
                raw_x, raw_y, raw_z = filter_result['raw_position']
                is_outlier = filter_result['is_outlier']
                used_prediction = filter_result['used_prediction']
                confidence_val = filter_result.get('confidence', confidence_val)
                consecutive_outliers = filter_result['consecutive_outliers']
                threshold = filter_result.get('threshold', None)
                if filter_result.get('tracking_valid') is not None:
                    tracking_valid_flag = 1 if filter_result['tracking_valid'] else 0
                rb_error = filter_result.get('rigid_body_error', None)
                data_source_val = filter_result.get('source', data_source_val)
            else:
                raw_x, raw_y, raw_z = pos_x, pos_y, pos_z
                is_outlier = False
                used_prediction = False
                consecutive_outliers = 0
                threshold = None

            rb_pose = self.last_rigid_body_pose or {}
            rb_pos = rb_pose.get('position_drone')
            rb_quat = rb_pose.get('rotation')
            rb_euler_deg = rb_pose.get('euler_deg')
            rb_tracking = 1 if rb_pose.get('tracking_valid') else 0
            rb_marker_count = rb_pose.get('marker_count')
            rb_error_logged = rb_error if rb_error is not None else rb_pose.get('error')

            # データ行作成
            row = [
                datetime.now().isoformat(),  # timestamp
                f"{elapsed:.4f}",  # elapsed_time
                f"{pos_x:.6f}", f"{pos_y:.6f}", f"{pos_z:.6f}",  # filtered position
                f"{raw_x:.6f}", f"{raw_y:.6f}", f"{raw_z:.6f}",  # raw position
                f"{error_x:.6f}", f"{error_y:.6f}",  # errors
                f"{roll_ref:.6f}", f"{pitch_ref:.6f}",  # rad
                f"{roll_ref * 180 / 3.14159:.3f}", f"{pitch_ref * 180 / 3.14159:.3f}",  # deg
                f"{pid_components['x']['p']:.6f}", f"{pid_components['x']['i']:.6f}", f"{pid_components['x']['d']:.6f}",
                f"{pid_components['y']['p']:.6f}", f"{pid_components['y']['i']:.6f}", f"{pid_components['y']['d']:.6f}",
                self.current_frame_number,
                self.current_marker_count,
                1 if send_success else 0,
                1 if self.control_active else 0,
                f"{loop_time * 1000:.2f}",  # ms単位
                # フィルタ関連
                1 if is_outlier else 0,
                1 if used_prediction else 0,
                f"{confidence_val:.3f}",
                consecutive_outliers,
                1 if is_data_valid else 0,
                data_source_val or "",
                f"{threshold:.4f}" if threshold is not None else "",
                tracking_valid_flag,
                f"{rb_error_logged:.5f}" if rb_error_logged is not None else "",
                rb_marker_count if rb_marker_count is not None else "",
                "" if rb_pos is None else f"{rb_pos[0]:.6f}",
                "" if rb_pos is None else f"{rb_pos[1]:.6f}",
                "" if rb_pos is None else f"{rb_pos[2]:.6f}",
                "" if rb_quat is None else f"{rb_quat[0]:.6f}",
                "" if rb_quat is None else f"{rb_quat[1]:.6f}",
                "" if rb_quat is None else f"{rb_quat[2]:.6f}",
                "" if rb_quat is None else f"{rb_quat[3]:.6f}",
                "" if rb_euler_deg is None else f"{rb_euler_deg[0]:.3f}",
                "" if rb_euler_deg is None else f"{rb_euler_deg[1]:.3f}",
                "" if rb_euler_deg is None else f"{rb_euler_deg[2]:.3f}"
            ]
            
            self.csv_writer.writerow(row)
            
        except Exception as e:
            # ログエラーは制御に影響させない
            if self.packets_sent % 100 == 0:  # エラーを間引いて表示
                print(f"[Log Error] {e}")
    
    def connect_serial(self):
        """シリアルポートに接続"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.01,
                write_timeout=0.01
            )
            
            if self.ser.is_open:
                print(f"✓ シリアルポート {self.serial_port} に接続しました")
                time.sleep(2)  # ESP32の初期化待ち
                return True
                
        except serial.SerialException as e:
            print(f"✗ シリアル接続エラー: {e}")
            return False
            
    def disconnect_serial(self):
        """シリアルポートを切断"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ シリアルポートを切断しました")
            
    def send_command(self, command):
        """
        文字列コマンドをESP32に送信（start/stop）
        """
        if not self.ser or not self.ser.is_open:
            return False
            
        try:
            message = command + '\n'
            self.ser.write(message.encode('utf-8'))
            print(f"→ コマンド送信: {command}")
            return True
        except Exception as e:
            print(f"✗ コマンド送信エラー: {e}")
            return False
            
    def send_angle_command(self, roll_ref, pitch_ref):
        """
        角度指令をESP32に送信
        Args:
            roll_ref: ロール角指令値 [rad]
            pitch_ref: ピッチ角指令値 [rad]
        """
        if not self.ser or not self.ser.is_open:
            return False
            
        try:
            # プロトコル: 'A' + roll(float) + pitch(float) + checksum
            header = b'A'
            data = struct.pack('ff', roll_ref, pitch_ref)
            checksum = sum(data) & 0xFF
            packet = header + data + bytes([checksum])
            
            self.ser.write(packet)
            self.packets_sent += 1
            return True
            
        except Exception as e:
            if self.packets_sent % 100 == 0:  # エラーを間引いて表示
                print(f"✗ 角度送信エラー: {e}")
            return False
            
    def receive_mocap_frame(self, data_dict):
        """
        NatNetからのコールバック（リジッドボディ + マーカーデータを統合）
        """
        mocap_data = data_dict.get("mocap_data", None)
        if mocap_data is None:
            return

        frame_num = data_dict.get("frame_number", 0)
        self.filter_stats['total_frames'] += 1
        now_ts = time.time()

        # リジッドボディ姿勢を取得
        rigid_pose = self.extract_rigid_body_pose(mocap_data)
        self.last_rigid_body_pose = rigid_pose

        # ラベル付きマーカー中心を計算（フォールバック用）
        marker_position = None
        marker_count = 0
        if mocap_data.labeled_marker_data:
            markers = mocap_data.labeled_marker_data.labeled_marker_list
            marker_count = len(markers)
            if marker_count > 0:
                marker_position = self.calculate_center_position(markers)

        # データソースを選択（リジッドボディ優先、無効ならマーカーにフォールバック）
        selected_position = None
        selected_source = "markers"
        tracking_valid = False
        quality_weight = None
        rigid_body_error = None
        effective_marker_count = marker_count

        if rigid_pose and rigid_pose['tracking_valid']:
            selected_position = rigid_pose['position_drone']
            selected_source = "rigid_body"
            tracking_valid = True
            quality_weight = rigid_pose['quality']
            rigid_body_error = rigid_pose['error']
            if rigid_pose['marker_count'] > 0:
                effective_marker_count = rigid_pose['marker_count']
        elif marker_position is not None:
            selected_position = marker_position
            selected_source = "markers"
            tracking_valid = marker_count >= 3
        elif rigid_pose:
            # リジッドボディは存在するがtracking_invalid。最後の姿勢を低信頼で使用
            selected_position = rigid_pose['position_drone']
            selected_source = "rigid_body"
            tracking_valid = False
            quality_weight = rigid_pose['quality']
            rigid_body_error = rigid_pose['error']
            if rigid_pose['marker_count'] > 0:
                effective_marker_count = rigid_pose['marker_count']

        if selected_position is None:
            # データがない場合は記録のみ更新し終了
            with self.position_lock:
                self.last_filter_result = None
                self.last_data_source = "none"
            return

        filter_result = self.position_filter.process_position(
            selected_position,
            marker_count=effective_marker_count,
            current_time=now_ts,
            tracking_valid=tracking_valid,
            quality_weight=quality_weight,
            rigid_body_error=rigid_body_error,
            source=selected_source
        )

        filtered_position = filter_result['filtered_position']

        with self.position_lock:
            self.current_position = filtered_position
            self.last_valid_position = filtered_position
            self.last_filter_result = filter_result
            self.current_frame_number = frame_num
            self.current_marker_count = effective_marker_count
            self.last_data_source = selected_source

        if filter_result['is_outlier']:
            self.filter_stats['outliers_detected'] += 1
        if filter_result['used_prediction']:
            self.filter_stats['predictions_used'] += 1

        if frame_num % 10 == 0:
            source_label = "RB" if selected_source == "rigid_body" else "Markers"
            print(f"[NatNet] Frame:{frame_num} Src:{source_label} Markers:{effective_marker_count} "
                  f"Pos=({filtered_position[0]:+.3f}, {filtered_position[1]:+.3f}, {filtered_position[2]:+.3f}) m "
                  f"Conf={filter_result['confidence']:.2f}")
            if filter_result['is_outlier']:
                raw_pos = filter_result['raw_position']
                print(f"  ⚠ 異常値検出（生データ: {raw_pos[0]:.3f}, {raw_pos[1]:.3f}）")
                        
    def convert_motive_to_drone(self, position):
        """
        Motive座標系(X,Y,Z)をドローン座標系(Z,X,Y)へ変換
        """
        if position is None:
            return None
        motive_x, motive_y, motive_z = position
        drone_x = motive_z  # 横軸 = MotiveのZ
        drone_y = motive_x  # 縦軸 = MotiveのX
        drone_z = motive_y  # 高度 = MotiveのY
        return (drone_x, drone_y, drone_z)

    @staticmethod
    def quaternion_to_euler_xyz(qx, qy, qz, qw):
        """四元数(x,y,z,w)をロール・ピッチ・ヨー(rad)へ変換"""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = copysign(pi / 2.0, sinp)
        else:
            pitch = asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        return (roll, pitch, yaw)

    def extract_rigid_body_pose(self, mocap_data):
        """
        指定IDのリジッドボディ姿勢を取得し、ドローン座標に変換
        """
        rigid_body_data = getattr(mocap_data, 'rigid_body_data', None)
        if rigid_body_data is None:
            return None

        rigid_bodies = getattr(rigid_body_data, 'rigid_body_list', None)
        if not rigid_bodies:
            return None

        for rigid_body in rigid_bodies:
            if getattr(rigid_body, 'id_num', None) != self.rigid_body_id:
                continue

            motive_pos = tuple(rigid_body.pos)
            drone_pos = self.convert_motive_to_drone(motive_pos)
            rotation = tuple(rigid_body.rot)
            tracking_valid = getattr(rigid_body, 'tracking_valid', False)
            error = getattr(rigid_body, 'error', None)
            marker_count = len(getattr(rigid_body, 'rb_marker_list', []))

            euler_rad = self.quaternion_to_euler_xyz(*rotation)
            euler_deg = tuple(degrees(val) for val in euler_rad)

            quality = 1.0
            if error is not None:
                quality = 1.0 / (1.0 + max(0.0, error))
            if not tracking_valid:
                quality *= 0.5
            quality = max(0.05, min(1.0, quality))

            return {
                'timestamp': time.time(),
                'position_motive': motive_pos,
                'position_drone': drone_pos,
                'rotation': rotation,
                'euler_rad': euler_rad,
                'euler_deg': euler_deg,
                'tracking_valid': tracking_valid,
                'error': error,
                'marker_count': marker_count,
                'quality': quality
            }

        return None

    def calculate_center_position(self, markers):
        """
        マーカーリストから中心点を計算
        座標系変換: Motive(X,Y,Z) → ドローン(Z,X,Y)
        """
        if not markers or len(markers) == 0:
            return None
            
        # Motiveの座標系での平均を計算
        sum_x = sum(marker.pos[0] for marker in markers)
        sum_y = sum(marker.pos[1] for marker in markers)
        sum_z = sum(marker.pos[2] for marker in markers)
        
        num_markers = len(markers)
        center_x = sum_x / num_markers
        center_y = sum_y / num_markers
        center_z = sum_z / num_markers
        
        return self.convert_motive_to_drone((center_x, center_y, center_z))
        
    def control_loop(self):
        """
        制御ループ（別スレッドで実行）
        拡張版：フィルタ結果を考慮したPID制御
        """
        print("制御ループ開始（拡張版）")
        control_rate = 100  # Hz
        period = 1.0 / control_rate
        no_data_counter = 0

        while not self.shutdown_flag:
            loop_start = time.time()

            if self.control_active:
                if self.current_position:
                    with self.position_lock:
                        pos_x, pos_y, pos_z = self.current_position
                        filter_result = getattr(self, 'last_filter_result', None)
                        data_source = self.last_data_source

                    error_x = self.target_position[0] - pos_x
                    error_y = self.target_position[1] - pos_y

                    confidence = 0.0
                    is_data_valid = False
                    if filter_result:
                        confidence = filter_result.get('confidence', 1.0)
                        is_data_valid = (
                            confidence > 0.1 and
                            not filter_result['is_outlier'] and
                            filter_result.get('tracking_valid', True)
                        )

                        if (filter_result['consecutive_outliers'] > 3 or
                                confidence < 0.3):
                            self.pid_controller.set_anomaly_state(True)
                        elif filter_result['consecutive_outliers'] == 0 and confidence > 0.5:
                            self.pid_controller.set_anomaly_state(False)

                    # PID制御計算（拡張版）
                    roll_ref, pitch_ref = self.pid_controller.calculate(
                        error_x, error_y, time.time(), is_data_valid
                    )

                    if confidence < self.confidence_zero_threshold:
                        roll_ref = 0.0
                        pitch_ref = 0.0
                    elif confidence < 0.99:
                        roll_ref *= confidence
                        pitch_ref *= confidence

                    # 座標系の確認用: 符号を反転する必要がある可能性
                    # 現在: X誤差→ロール、Y誤差→ピッチ
                    # もし逆の場合は以下のコメントを外す
                    # pitch_ref, roll_ref = roll_ref, pitch_ref
                    
                    # 角度指令送信
                    success = self.send_angle_command(roll_ref, pitch_ref)
                    
                    # データをログに記録（100Hz全て、拡張版）
                    loop_time = time.time() - loop_start
                    self.log_data(pos_x, pos_y, pos_z, error_x, error_y,
                                  roll_ref, pitch_ref, success, loop_time,
                                  filter_result, is_data_valid,
                                  confidence, data_source)
                    
                    # デバッグ出力（5Hzで継続的に表示）
                    if self.packets_sent % 20 == 0:
                        roll_deg = roll_ref * 180 / 3.14159
                        pitch_deg = pitch_ref * 180 / 3.14159
                        print(f"[Control] Src:{data_source} 位置: ({pos_x:+6.3f}, {pos_y:+6.3f}) m | "
                              f"誤差: ({error_x:+6.3f}, {error_y:+6.3f}) m | "
                              f"指令: R={roll_deg:+6.1f}°, P={pitch_deg:+6.1f}° | "
                              f"Conf:{confidence:.2f} 送信: {'OK' if success else 'FAIL'}")
                    
                    no_data_counter = 0
                else:
                    # 位置データがない場合
                    no_data_counter += 1
                    if no_data_counter % 100 == 0:
                        print("[Warning] No position data available!")
                    
                    # 安全のため水平維持
                    success = self.send_angle_command(0.0, 0.0)
                    
                    # データなしでもログに記録（NaNまたは0として）
                    loop_time = time.time() - loop_start
                    self.log_data(0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, success, loop_time,
                                  None, False, confidence=0.0, data_source="none")
                    
            # 制御周期を維持
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)
                
        print("制御ループ終了")
        
    def start_hovering(self):
        """ホバリング開始"""
        if self.is_flying:
            print("既に飛行中です")
            return
            
        print("\n=== ホバリング開始シーケンス ===")
        
        # 先に制御を有効化（ただし離陸前は角度0を維持）
        self.control_active = False  # 最初はFalseで開始
        
        # 1. NatNet接続
        print("1. Optitrack/Motiveに接続中...")
        self.natnet_client = NatNetClient()
        self.natnet_client.set_server_address("127.0.0.1")
        self.natnet_client.set_client_address("127.0.0.1")
        self.natnet_client.set_use_multicast(True)
        
        # コールバック登録（drone_tracker.pyと同じ）
        self.natnet_client.new_frame_with_data_listener = self.receive_mocap_frame
        self.natnet_client.set_print_level(0)  # デバッグ出力OFF
        
        # NatNet開始（drone_tracker.pyと同じ'd'引数）
        if not self.natnet_client.run('d'):
            print("   ✗ 接続に失敗しました")
            return
        
        time.sleep(1)
        
        if not self.natnet_client.connected():
            print("   ✗ Motiveが見つかりません")
            print("   Motiveのストリーミング設定を確認してください:")
            print("     1. Edit → Settings → Streaming を開く")
            print("     2. Broadcast Frame Data にチェック")
            print("     3. Local Interface でネットワークインターフェースを選択")
            self.natnet_client.shutdown()
            return
        
        # データ記述を要求
        self.natnet_client.send_request(
            self.natnet_client.command_socket,
            self.natnet_client.NAT_REQUEST_MODELDEF,
            "",
            ("127.0.0.1", self.natnet_client.command_port)
        )
        
        print("   ✓ Optitrack接続完了")
        
        # 2. 制御スレッド開始
        print("2. 制御ループを開始...")
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        print("   ✓ 制御ループ開始")
        
        # 3. 位置データ確認
        print("3. 位置データの確認...")
        wait_count = 0
        while self.current_position is None and wait_count < 30:  # 最大3秒待機
            time.sleep(0.1)
            wait_count += 1
            if wait_count % 10 == 0:
                print(f"   位置データ待機中... {wait_count/10:.1f}秒")
        
        if self.current_position:
            print(f"   ✓ 初期位置取得: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f}, {self.current_position[2]:.3f}) m")
        else:
            print("   ⚠ 警告: 位置データが取得できていません。マーカーを確認してください。")
        
        # 4. データロギング開始
        print("4. データロギングを開始...")
        self.start_logging()
        
        # 5. 離陸コマンド送信
        print("5. 離陸コマンドを送信...")
        if self.send_command("start"):
            print("   ✓ 離陸コマンド送信完了")
            
            # 6. 即座に制御開始（離陸と同時に制御）
            print("6. 位置制御を開始...")
            self.pid_controller.reset()
            time.sleep(0.5)  # 短い待機
            self.control_active = True  # 制御開始
            self.is_flying = True
            print("   ✓ ホバリング制御開始")
            
            if self.current_position:
                print(f"   目標位置: (0.000, 0.000) m")
                print(f"   現在位置: ({self.current_position[0]:.3f}, {self.current_position[1]:.3f}, {self.current_position[2]:.3f}) m")
            
            print("\n原点ホバリング中... (stopで着陸)")
        else:
            print("   ✗ 離陸コマンド送信失敗")
            self.stop_hovering()
            
    def stop_hovering(self):
        """ホバリング停止"""
        if not self.is_flying:
            print("飛行していません")
            return
            
        print("\n=== 着陸シーケンス ===")
        
        # 1. 制御停止
        print("1. 位置制御を停止...")
        self.control_active = False
        time.sleep(0.5)
        
        # 2. 着陸コマンド
        print("2. 着陸コマンドを送信...")
        if self.send_command("stop"):
            print("   ✓ 着陸コマンド送信完了")
            
        # 3. クリーンアップ
        print("3. クリーンアップ中...")
        self.shutdown_flag = True
        
        if self.control_thread:
            self.control_thread.join(timeout=2)
            
        if self.natnet_client:
            self.natnet_client.shutdown()
            
        # 4. ログファイルをクローズ
        print("4. データロギングを停止...")
        self.close_log_file()
        
        self.is_flying = False
        print("   ✓ 着陸完了")
        
    def run(self):
        """メインループ"""
        print("\n" + "="*50)
        print("Optitrack 原点ホバリング制御システム")
        print("="*50)
        print("コマンド:")
        print("  start - 離陸してホバリング開始")
        print("  stop  - 着陸")
        print("  exit  - プログラム終了")
        print("="*50 + "\n")
        
        # シリアルポート接続
        if not self.connect_serial():
            print("ESP32への接続に失敗しました")
            return
            
        try:
            while True:
                command = input("\nコマンド > ").strip().lower()
                
                if command == "exit":
                    if self.is_flying:
                        print("着陸してから終了します...")
                        self.stop_hovering()
                    print("プログラムを終了します")
                    break
                    
                elif command == "start":
                    self.start_hovering()
                    
                elif command == "stop":
                    self.stop_hovering()
                    
                elif command == "":
                    continue
                    
                else:
                    print(f"✗ 不明なコマンド: {command}")
                    
        except KeyboardInterrupt:
            print("\n\nCtrl+Cが押されました")
            
        finally:
            # 安全な終了処理
            if self.is_flying:
                print("安全のため着陸します...")
                self.stop_hovering()
                time.sleep(2)
                
            self.disconnect_serial()
            print("\nプログラムを終了しました")

def main():
    """メイン関数"""
    controller = HoveringController()
    controller.run()

if __name__ == "__main__":
    main()
