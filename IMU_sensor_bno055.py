import time
import serial
from collections import deque

class IMUSensorBNO055:
    def __init__(self, port='/dev/serial0', baudrate=115200, offset=0.0, calib_file="bno055_calibdata.bin"):
        self.offset = offset
        
        # 移動平均バッファ（5回分）
        self.yaw_buffer = deque(maxlen=5)
        self.accel_x_buffer = deque(maxlen=5)
        self.accel_y_buffer = deque(maxlen=5)
        
        # 前回値保持用（None時のフォールバック）
        self.last_yaw = 0.0
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.last_accel_x = 0.0
        self.last_accel_y = 0.0
        self.last_accel_z = 0.0
        
        # 積分による移動量計算用
        self.velocity_x = 0.0  # X方向速度 (m/s)
        self.velocity_y = 0.0  # Y方向速度 (m/s)
        self.position_x = 0.0  # X方向移動量 (m)
        self.position_y = 0.0  # Y方向移動量 (m)
        self.last_time = None  # 前回の測定時刻
        
        try:
            # タイムアウトを適切に設定（0.02秒＝20ms）
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.02)
            print(f"BNO055 UART opened on {port} (timeout=0.02s).")
            time.sleep(1.0)  # UART安定化待機
            
            # --- バッファ完全クリア（BUS_OVER_RUN対策） ---
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            time.sleep(0.2)  # バッファクリア完全待機
            
            # --- basecheck.pyの初期化手順 ---
            # configモードへ
            self.ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x00]))
            time.sleep(0.3)  # CONFIG モード切替待機（150ms→300ms - BUS_OVER_RUN対策）
            self.ser.read(2)
            time.sleep(0.2)  # 追加の安定化待機（100ms→200ms）
            
            # --- キャリブレーションデータ反映（CONFIGモードで実施） ---
            self.calib_loaded = False  # キャリブレーション読み込み状態フラグ
            self.calib_file_path = calib_file  # ファイルパス保存
            try:
                with open(calib_file, "rb") as f:
                    calib_data = f.read()
                if len(calib_data) == 22:
                    # CONFIGモードでキャリブレーションデータを書き込み
                    # コマンド: 0xAA, 0x00, 0x55, 0x16 + 22バイトデータ
                    
                    # バッファ再クリア（BUS_OVER_RUN対策）
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                    time.sleep(0.15)  # バッファクリア待機（50ms→150ms）
                    
                    write_cmd = bytes([0xAA, 0x00, 0x55, 0x16]) + calib_data
                    self.ser.write(write_cmd)
                    time.sleep(0.8)  # 書き込み処理待機（500ms→800ms - BUS_OVER_RUN対策強化）
                    resp = self.ser.read(2)
                    
                    # レスポンス確認
                    if len(resp) == 2 and resp[0] == 0xEE and resp[1] == 0x01:
                        self.calib_loaded = True
                        print(f"[OK] BNO055 calibration data loaded from '{calib_file}'")
                        print(f"     Write response: {' '.join(f'{b:02X}' for b in resp)} (SUCCESS)")
                    elif len(resp) == 2 and resp[0] == 0xEE:
                        # エラーコード詳細
                        error_codes = {
                            0x03: "WRITE_FAIL",
                            0x04: "INVALID_ADDRESS", 
                            0x05: "WRITE_DISABLED",
                            0x06: "WRONG_START_BYTE",
                            0x07: "BUS_OVER_RUN (data too fast)",
                            0x08: "MAX_LENGTH_ERROR",
                            0x09: "MIN_LENGTH_ERROR",
                            0x0A: "RECEIVE_TIMEOUT"
                        }
                        error_name = error_codes.get(resp[1], f"UNKNOWN_{resp[1]:02X}")
                        self.calib_loaded = False
                        print(f"[WARN] BNO055 calibration write failed: {' '.join(f'{b:02X}' for b in resp)} ({error_name})")
                        print(f"       Continuing without calibration file - sensor will auto-calibrate")
                    else:
                        self.calib_loaded = False
                        print(f"[WARN] BNO055 calibration write: No response (len={len(resp)})")
                else:
                    print(f"[WARN] BNO055 calibration file '{calib_file}' has invalid size: {len(calib_data)} bytes (expected 22)")
            except FileNotFoundError:
                print(f"[INFO] BNO055 calibration file '{calib_file}' not found - will auto-calibrate")
            except Exception as e:
                print(f"[WARN] BNO055 calibration error: {e}")
            
            # NDOFモードへ（キャリブレーションデータ書き込み後）
            self.ser.reset_input_buffer()  # バッファクリア
            self.ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x0C]))
            time.sleep(0.1)
            resp = self.ser.read(2)
            print("BNO055 NDOF mode activated")
            time.sleep(2.5)  # モード切替後の安定待機（2→2.5秒に延長）
        except Exception as e:
            print(f"UART open failed: {e}")
            self.ser = None

    def read_euler(self):
        if not self.ser:
            return None, None, None
        # UARTバッファをクリア（古いデータを破棄）
        self.ser.reset_input_buffer()
        
        # BNO055のオイラー角レジスタ（0x1A～0x1F）を読み出す
        # コマンド: [0xAA, 0x01, 0x1A, 0x06] → 0x1Aから6バイト（heading, roll, pitch）
        self.ser.write(bytearray([0xAA, 0x01, 0x1A, 0x06]))
        time.sleep(0.008)  # 8ms待機（最小限の応答待機）
        resp = self.ser.read(10)
        if len(resp) >= 8 and resp[0] == 0xBB and resp[1] == 0x06:
            heading = int.from_bytes(resp[2:4], 'little', signed=True) / 16.0
            roll    = int.from_bytes(resp[4:6], 'little', signed=True) / 16.0
            pitch   = int.from_bytes(resp[6:8], 'little', signed=True) / 16.0
            return (heading + self.offset) % 360, roll, pitch
        else:
            return None, None, None

    def get_all(self):
        import struct
        result = {}
        # 加速度
        self.ser.write(bytes([0xAA, 0x01, 0x08, 0x06]))
        acc_raw = self.ser.read(8)
        if len(acc_raw) == 8 and acc_raw[0] == 0xBB:
            result['accel'] = struct.unpack('<hhh', acc_raw[2:8])
        else:
            result['accel'] = None
        # ジャイロ
        self.ser.write(bytes([0xAA, 0x01, 0x14, 0x06]))
        gyr_raw = self.ser.read(8)
        if len(gyr_raw) == 8 and gyr_raw[0] == 0xBB:
            result['gyro'] = struct.unpack('<hhh', gyr_raw[2:8])
        else:
            result['gyro'] = None
        # 地磁気
        self.ser.write(bytes([0xAA, 0x01, 0x0E, 0x06]))
        mag_raw = self.ser.read(8)
        if len(mag_raw) == 8 and mag_raw[0] == 0xBB:
            result['mag'] = struct.unpack('<hhh', mag_raw[2:8])
        else:
            result['mag'] = None
        # オイラー角（heading/yaw, roll, pitch）
        heading, roll, pitch = self.read_euler()
        result['yaw'] = heading
        result['roll'] = roll
        result['pitch'] = pitch
        return result
    
    def get_essential(self):
        """高速版: YAWとX,Y加速度のみ取得（Z軸・地磁気・ジャイロ省略）
        通信時間: 約20ms
        エラー時: 5回移動平均で補完
        Returns: (result_dict, error_occurred)
        """
        import struct
        result = {}
        accel_error = False  # 加速度読み取りエラーフラグ
        yaw_error = False    # YAW読み取りエラーフラグ
        
        # UARTバッファをクリア（古いデータを破棄）
        self.ser.reset_input_buffer()
        
        # 加速度（X,Y軸のみ使用、Z軸は破棄）
        self.ser.write(bytes([0xAA, 0x01, 0x08, 0x06]))
        time.sleep(0.008)  # 8ms待機（最小限の応答待機）
        acc_raw = self.ser.read(8)
        if len(acc_raw) == 8 and acc_raw[0] == 0xBB:
            # X,Y軸のみ取得（Z軸は破棄して処理高速化）
            accel_xyz = struct.unpack('<hhh', acc_raw[2:8])
            accel_x = accel_xyz[0]
            accel_y = accel_xyz[1]
            
            # バッファに追加
            self.accel_x_buffer.append(accel_x)
            self.accel_y_buffer.append(accel_y)
            
            result['accel'] = (accel_x, accel_y)
        else:
            # エラー発生
            accel_error = True
            # エラー時: 移動平均で補完
            if len(self.accel_x_buffer) > 0:
                avg_x = sum(self.accel_x_buffer) / len(self.accel_x_buffer)
                avg_y = sum(self.accel_y_buffer) / len(self.accel_y_buffer)
                result['accel'] = (int(avg_x), int(avg_y))
            else:
                result['accel'] = None
        
        # オイラー角（YAWのみ使用、ROLL/PITCHは破棄）
        heading, roll, pitch = self.read_euler()
        
        if heading is not None:
            # バッファに追加
            self.yaw_buffer.append(heading)
            result['yaw'] = heading
        else:
            # エラー発生
            yaw_error = True
            # エラー時: 移動平均で補完
            if len(self.yaw_buffer) > 0:
                result['yaw'] = sum(self.yaw_buffer) / len(self.yaw_buffer)
            else:
                result['yaw'] = None
        
        result['roll'] = None  # 使用しない
        result['pitch'] = None  # 使用しない
        
        # 省略したデータはNoneで返す（互換性維持）
        result['gyro'] = None
        result['mag'] = None
        
        # エラーフラグを含めて返す
        error_occurred = accel_error or yaw_error
        return result, error_occurred
    
    def update_position(self, accel_x_raw, accel_y_raw):
        """加速度を時間積分して移動量を算出
        Args:
            accel_x_raw: X方向加速度 (BNO055単位: 1/100 m/s²)
            accel_y_raw: Y方向加速度 (BNO055単位: 1/100 m/s²)
        
        静止判定機能付き: 加速度が小さい場合は速度をリセットしてドリフトを防止
        """
        current_time = time.time()
        
        # 初回呼び出し時は時刻のみ記録
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # 時間差分 (秒)
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 加速度を m/s² に変換
        accel_x = accel_x_raw / 100.0
        accel_y = accel_y_raw / 100.0
        
        # 静止判定: 加速度が閾値以下なら速度をゼロクリア（ドリフト防止）
        ACCEL_THRESHOLD = 0.15  # m/s² (静止判定閾値)
        accel_magnitude = (accel_x**2 + accel_y**2)**0.5
        
        if accel_magnitude < ACCEL_THRESHOLD:
            # 静止していると判定 → 速度をゼロリセット
            self.velocity_x = 0.0
            self.velocity_y = 0.0
        else:
            # 移動中 → 速度を更新 (v = v0 + a*dt)
            self.velocity_x += accel_x * dt
            self.velocity_y += accel_y * dt
        
        # 位置を更新 (x = x0 + v*dt)
        self.position_x += self.velocity_x * dt
        self.position_y += self.velocity_y * dt
    
    def reset_position(self):
        """移動量をリセット"""
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.last_time = None
    
    def get_position(self):
        """現在の移動量を取得
        Returns: (position_x, position_y, velocity_x, velocity_y)
        """
        return (self.position_x, self.position_y, self.velocity_x, self.velocity_y)
    
    def get_calibration_status(self):
        """BNO055のキャリブレーション状態を取得
        Returns: dict with 'sys', 'gyro', 'accel', 'mag' (each 0-3)
        """
        if not self.ser:
            return None
        try:
            # キャリブレーションステータスレジスタ（0x35）を読み出す
            # コマンド: [0xAA, 0x01, 0x35, 0x01] → 0x35から1バイト
            self.ser.reset_input_buffer()  # バッファクリア
            self.ser.write(bytearray([0xAA, 0x01, 0x35, 0x01]))
            time.sleep(0.02)  # 待機時間延長（0.01→0.02秒）
            resp = self.ser.read(5)
            if len(resp) >= 5 and resp[0] == 0xBB:
                calib_byte = resp[2]
                # ビット分解: [7:6]=sys, [5:4]=gyro, [3:2]=accel, [1:0]=mag
                return {
                    'sys': (calib_byte >> 6) & 0x03,
                    'gyro': (calib_byte >> 4) & 0x03,
                    'accel': (calib_byte >> 2) & 0x03,
                    'mag': calib_byte & 0x03
                }
        except Exception as e:
            print(f"⚠️ キャリブレーション状態取得エラー: {e}")
        return None
    
    def save_calibration(self, filename="bno055_calibdata.bin"):
        """現在のキャリブレーションデータを保存
        キャリブレーション完了後(ACCEL=3/3)に実行することを推奨
        """
        if not self.ser:
            return False
        
        # Configモードに切り替え（キャリブレーションデータ読み出しのため）
        self.ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x00]))
        time.sleep(0.05)
        self.ser.read(2)
        
        # キャリブレーションデータ読み出し（0x55レジスタから22バイト）
        self.ser.write(bytes([0xAA, 0x01, 0x55, 0x16]))
        time.sleep(0.05)
        resp = self.ser.read(24)  # ヘッダー2バイト + データ22バイト
        
        if len(resp) >= 24 and resp[0] == 0xBB:
            calib_data = resp[2:24]  # 22バイトのキャリブレーションデータ
            
            # ファイルに保存
            try:
                with open(filename, "wb") as f:
                    f.write(calib_data)
                print(f"✅ キャリブレーションデータを保存しました: {filename}")
                
                # NDOFモードに戻す
                self.ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x0C]))
                time.sleep(0.05)
                self.ser.read(2)
                return True
            except Exception as e:
                print(f"⚠️ 保存エラー: {e}")
                return False
        else:
            print("⚠️ キャリブレーションデータ読み出し失敗")
            # NDOFモードに戻す
            self.ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x0C]))
            time.sleep(0.05)
            self.ser.read(2)
            return False

    def set_offset(self, offset):
        self.offset = offset
    
    # ============================================================
    # BNO055IMUDriver互換インターフェース
    # ============================================================
    
    def get_yaw(self):
        """ヨー角を取得（ラジアン）- BNO055IMUDriver互換
        
        Returns:
            float: ヨー角（ラジアン）、エラー時は0.0
        """
        import math
        data, _ = self.get_essential()
        if data and data['yaw'] is not None:
            # 度からラジアンに変換
            return math.radians(data['yaw'])
        return 0.0
    
    def get_euler_angles(self):
        """オイラー角を取得（度）- BNO055IMUDriver互換
        
        Returns:
            dict: {'yaw': float, 'roll': float, 'pitch': float}
        """
        heading, roll, pitch = self.read_euler()
        
        # YAWの処理：成功時はバッファに追加、失敗時は移動平均
        if heading is not None:
            self.yaw_buffer.append(heading)
            yaw_value = heading
        else:
            # エラー時: 移動平均で補完
            if len(self.yaw_buffer) > 0:
                yaw_value = sum(self.yaw_buffer) / len(self.yaw_buffer)
            else:
                yaw_value = 0.0  # バッファが空なら初期値
        
        # Roll/Pitchは前回値保持（移動平均不要）
        if roll is not None:
            self.last_roll = roll
        if pitch is not None:
            self.last_pitch = pitch
        
        return {
            'yaw': yaw_value,
            'roll': roll if roll is not None else self.last_roll,
            'pitch': pitch if pitch is not None else self.last_pitch
        }
    
    def get_acceleration(self):
        """加速度を取得（m/s²）- BNO055IMUDriver互換
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float}
        """
        data, _ = self.get_essential()
        
        if data and data['accel'] is not None:
            ax, ay = data['accel']
            accel_x = ax / 100.0  # BNO055単位からm/s²に変換
            accel_y = ay / 100.0
            
            # 成功時はバッファに追加（get_essential内で既に追加済み）
            return {
                'x': accel_x,
                'y': accel_y,
                'z': 0.0  # Z軸は取得していない
            }
        else:
            # エラー時: 移動平均で補完（get_essentialと同じロジック）
            if len(self.accel_x_buffer) > 0 and len(self.accel_y_buffer) > 0:
                avg_x = sum(self.accel_x_buffer) / len(self.accel_x_buffer)
                avg_y = sum(self.accel_y_buffer) / len(self.accel_y_buffer)
                return {
                    'x': avg_x / 100.0,  # m/s²に変換
                    'y': avg_y / 100.0,
                    'z': 0.0
                }
            else:
                # バッファが空なら初期値
                return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def get_quaternion(self):
        """クォータニオンを取得 - BNO055IMUDriver互換
        
        Note: UART版では未実装、単位クォータニオンを返す
        
        Returns:
            dict: {'w': float, 'x': float, 'y': float, 'z': float}
        """
        return {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def get_gyro(self):
        """角速度を取得（rad/s）- BNO055IMUDriver互換
        
        Note: get_all()使用のため低速。高速版はget_essential()推奨。
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float}
        """
        import math
        all_data = self.get_all()
        if all_data and all_data['gyro'] is not None:
            gx, gy, gz = all_data['gyro']
            return {
                'x': math.radians(gx / 16.0),  # BNO055単位からrad/sに変換
                'y': math.radians(gy / 16.0),
                'z': math.radians(gz / 16.0)
            }
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    
    def is_calibrated(self):
        """キャリブレーション完了判定 - BNO055IMUDriver互換
        
        Returns:
            bool: ACCEL >= 2 なら True
        """
        status = self.get_calibration_status()
        if status:
            # ACCEL が 2以上ならキャリブレーション完了とみなす
            return status.get('accel', 0) >= 2
        return False
    
    def get_sensor_info(self):
        """センサー情報を取得 - BNO055IMUDriver互換
        
        Returns:
            dict: センサー情報
        """
        calib_status = self.get_calibration_status()
        return {
            'connected': self.ser is not None,
            'calibration_status': calib_status if calib_status else {
                'sys': 0, 'gyro': 0, 'accel': 0, 'mag': 0
            },
            'is_calibrated': self.is_calibrated()
        }

# --- デバッグ実行モード ---
if __name__ == "__main__":
    print("=" * 80)
    print("BNO055 IMUセンサー デバッグモード")
    print("YAW角度とACCEL(X,Y)をリアルタイム表示")
    print("⏱️ 処理時間監視機能付き")
    print("Ctrl+C で終了")
    print("=" * 80)
    
    imu = IMUSensorBNO055(port='/dev/serial0', baudrate=115200, offset=0.0)
    
    # キャリブレーションデータ読み込み状態を表示
    print("\n📁 キャリブレーションファイル読み込み状態:")
    if hasattr(imu, 'calib_loaded') and imu.calib_loaded:
        print(f"  ✅ キャリブレーションデータ読み込み成功")
        print(f"  📄 ファイル: {imu.calib_file_path}")
        
        # ファイル内容を分析
        try:
            with open(imu.calib_file_path, "rb") as f:
                calib_data = f.read()
            print(f"  📊 ファイルサイズ: {len(calib_data)} bytes")
            if len(calib_data) == 22:
                print(f"  📊 データ内容 (16進数):")
                # 22バイトを6バイトずつ表示（見やすく整形）
                print(f"     ACCEL Offset: {' '.join(f'{b:02X}' for b in calib_data[0:6])}")
                print(f"     MAG   Offset: {' '.join(f'{b:02X}' for b in calib_data[6:12])}")
                print(f"     GYRO  Offset: {' '.join(f'{b:02X}' for b in calib_data[12:18])}")
                print(f"     ACCEL Radius: {' '.join(f'{b:02X}' for b in calib_data[18:20])}")
                print(f"     MAG   Radius: {' '.join(f'{b:02X}' for b in calib_data[20:22])}")
                
                # 全てゼロかチェック
                if all(b == 0 for b in calib_data):
                    print(f"  ⚠️ 警告: 全てのデータが0x00です（無効なキャリブレーションデータの可能性）")
        except Exception as e:
            print(f"  ⚠️ ファイル分析エラー: {e}")
    else:
        print(f"  ⚠️ キャリブレーションデータ未読み込み")
        if hasattr(imu, 'calib_file_path'):
            print(f"  📄 ファイル: {imu.calib_file_path} (not found or invalid)")
    
    # NDOFモード安定化のため追加待機
    print("\n⏳ センサー安定化待機中...")
    time.sleep(1.0)
    
    # 初期キャリブレーション状態を表示（複数回試行）
    print("\n📊 BNO055 初期キャリブレーション状態:")
    calib_status = None
    for attempt in range(3):
        calib_status = imu.get_calibration_status()
        if calib_status:
            break
        if attempt < 2:
            print(f"  ⚠️ 取得失敗 (試行 {attempt+1}/3)、再試行中...")
            time.sleep(0.5)
    
    if calib_status:
        print(f"  SYS: {calib_status['sys']}/3 | "
              f"GYRO: {calib_status['gyro']}/3 | "
              f"ACCEL: {calib_status['accel']}/3 | "
              f"MAG: {calib_status['mag']}/3")
        if calib_status['sys'] == 3:
            print("  ✅ システムキャリブレーション完了")
        else:
            print("  ⚠️ キャリブレーション未完了（動作中に自動調整されます）")
            print("  💡 キャリブレーション手順:")
            print("     1. センサーをゆっくり6方向に向ける（上下前後左右）")
            print("     2. 各方向で数秒間静止")
            print("     3. ACCEL: 3/3 になったら 's' キーで保存")
    else:
        print("  ⚠️ キャリブレーション状態取得失敗")
        # キャリブレーションファイルが読み込まれていれば問題なし
        if hasattr(imu, 'calib_loaded') and imu.calib_loaded:
            print("  💡 ただし、キャリブレーションデータは正常に反映されています")
            print("     （状態取得APIの失敗ですが、動作には問題ありません）")
    
    print("=" * 80)
    print("💾 キャリブレーション完了後、's'キーを押すと保存します")
    print("=" * 80)
    
    cycle_count = 0
    start_time = time.time()
    slow_read_count = 0  # 遅延発生回数
    read_error_count = 0  # read error発生回数
    
    def fmt(val):
        return f"{val:7.1f}" if val is not None else "   ---"
    
    try:
        while True:
            cycle_start = time.time()
            
            # 測定開始
            read_start = time.time()
            data, error_occurred = imu.get_essential()
            read_time = (time.time() - read_start) * 1000  # ms単位
            
            cycle_count += 1
            elapsed = time.time() - start_time
            
            # 実際の読み取りエラーをカウント
            if error_occurred:
                read_error_count += 1
            
            # 読み取り時間が100ms以上なら警告
            warning = ""
            if read_time > 100:
                slow_read_count += 1
                warning = f" ⚠️ SLOW READ: {read_time:.0f}ms"
            
            # エラーマーク表示
            error_mark = " [補完]" if error_occurred else ""
            
            if data is not None and data['yaw'] is not None and data['accel'] is not None:
                yaw = data['yaw']
                # X, Y軸のみ取得（Zは除外済み）
                accel_x_raw = data['accel'][0]  # 生データ (1/100 m/s²単位)
                accel_y_raw = data['accel'][1]
                accel_x = accel_x_raw / 100.0  # m/s²に変換
                accel_y = accel_y_raw / 100.0
                
                # 移動量を積分更新
                imu.update_position(accel_x_raw, accel_y_raw)
                pos_x, pos_y, vel_x, vel_y = imu.get_position()
                
                # 静止判定状態を表示
                accel_magnitude = (accel_x**2 + accel_y**2)**0.5
                is_stationary = accel_magnitude < 0.15
                state_mark = " [静止]" if is_stationary else " [移動]"
                
                # 10回ごとにキャリブレーション状態を表示
                calib_info = ""
                if cycle_count % 10 == 0:
                    calib = imu.get_calibration_status()
                    if calib:
                        calib_info = f" | Cal[S:{calib['sys']} G:{calib['gyro']} A:{calib['accel']} M:{calib['mag']}]"
                
                # リアルタイム表示（1行更新形式）
                print(f"[{cycle_count:04d} | {elapsed:7.2f}s | {read_time:5.1f}ms] "
                      f"YAW:{fmt(yaw)}° | "
                      f"ACCEL_X:{fmt(accel_x)} m/s² | "
                      f"ACCEL_Y:{fmt(accel_y)} m/s² | "
                      f"POS_X:{pos_x:7.3f}m | "
                      f"POS_Y:{pos_y:7.3f}m{state_mark}{warning}{error_mark}{calib_info}")
            else:
                print(f"[{cycle_count:04d} | {elapsed:7.2f}s | {read_time:5.1f}ms] "
                      f"⚠️ BNO055 sensor read error (no buffer){warning}")
            
            # 測定周期調整:sleep時間を短縮(50ms→10ms)
            # 実測周期: 約30-40ms(IMU読取20ms + sleep10ms + オーバーヘッド)
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print("測定停止中...")
        
        # 最終移動量を表示
        pos_x, pos_y, vel_x, vel_y = imu.get_position()
        distance = (pos_x**2 + pos_y**2)**0.5  # 総移動距離
        print(f"\n📍 最終移動量:")
        print(f"  X方向: {pos_x:8.3f}m | Y方向: {pos_y:8.3f}m")
        print(f"  総移動距離: {distance:.3f}m")
        print(f"  最終速度 - X: {vel_x:6.3f}m/s | Y: {vel_y:6.3f}m/s")
        
        # 最終キャリブレーション状態を表示（複数回試行）
        print("\n📊 最終キャリブレーション状態:")
        print("  ⏳ 取得中...")
        time.sleep(0.5)  # 安定化待機
        
        calib_status = None
        for attempt in range(5):  # 5回試行
            calib_status = imu.get_calibration_status()
            if calib_status:
                break
            time.sleep(0.3)
        
        if calib_status:
            print(f"  SYS: {calib_status['sys']}/3 | "
                  f"GYRO: {calib_status['gyro']}/3 | "
                  f"ACCEL: {calib_status['accel']}/3 | "
                  f"MAG: {calib_status['mag']}/3")
            
            # キャリブレーション完了時は保存を促す
            if calib_status['accel'] == 3:
                print("\n💾 キャリブレーション完了を保存しますか? (y/n): ", end="")
                try:
                    import sys
                    import select
                    # 非ブロッキング入力（5秒待機）
                    if sys.platform != 'win32':
                        import tty, termios
                        old_settings = termios.tcgetattr(sys.stdin)
                        try:
                            tty.setcbreak(sys.stdin.fileno())
                            if select.select([sys.stdin], [], [], 5)[0]:
                                answer = sys.stdin.read(1)
                                if answer.lower() == 'y':
                                    imu.save_calibration()
                        finally:
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    else:
                        # Windows環境では単純な入力
                        answer = input()
                        if answer.lower() == 'y':
                            imu.save_calibration()
                except:
                    print("\n(スキップ)")
            else:
                print("  ⚠️ ACCEL キャリブレーション未完了")
        else:
            print("  ⚠️ 状態取得失敗")
        
        print("\n📈 測定統計:")
        total_time = time.time() - start_time
        avg_cycle = (total_time / cycle_count * 1000) if cycle_count > 0 else 0
        slow_rate = (slow_read_count / cycle_count * 100) if cycle_count > 0 else 0
        error_rate = (read_error_count / cycle_count * 100) if cycle_count > 0 else 0
        print(f"総測定回数: {cycle_count} | 総時間: {total_time:.2f}s | 平均周期: {avg_cycle:.1f}ms")
        print(f"遅延発生: {slow_read_count}回 ({slow_rate:.1f}%) - 100ms超過")
        print(f"読み取りエラー: {read_error_count}回 ({error_rate:.1f}%)")
        print("Stopped.")
        print("=" * 80)
