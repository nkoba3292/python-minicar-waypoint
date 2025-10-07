# imu_landmark_calibration.py - コース特徴点ベース IMUキャリブレーション
import json
import math
import time
import numpy as np
from platform_detector import is_raspberry_pi

class IMULandmarkCalibration:
    """コース特徴点を基準としたIMUキャリブレーション"""
    
    def __init__(self, imu_driver=None, config_file="config.json"):
        self.imu_driver = imu_driver
        self.calibration_file = "imu_landmark_calib.json"
        self.offset = 0.0
        
        # 設定読み込み
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            self.imu_config = config.get('sensors', {}).get('imu', {})
        except Exception as e:
            print(f"Config file error: {e}")
            self.imu_config = {}
    
    def get_current_yaw(self):
        """現在のヨー角を取得（ハードウェア/モック自動切り替え）"""
        if self.imu_driver:
            return self.imu_driver.get_yaw()
        else:
            # モック環境
            return 0.0
    
    def calibrate_with_landmarks(self):
        """特徴点ベース2点キャリブレーション実行"""
        print("=== IMU Landmark-Based Calibration ===")
        print("この方法では、コースの確実な特徴点を基準に校正します。")
        print()
        
        # 特徴点情報の入力
        print("【特徴点の選定】")
        print("以下のような明確な構造物を選んでください：")
        print("- ゲート、ポスト、建物の角")
        print("- コーナーのエッジ、壁の交差点")
        print("- 再現性のある明確な基準点")
        print()
        
        # Point 1の測定
        landmark1_desc = input("特徴点1の説明（例: スタートゲート→ゴールポスト）: ")
        input(f"車両を「{landmark1_desc}」方向に正確に向けて、Enterを押してください...")
        
        # 安定化待機
        print("測定中... (3秒間安定化)")
        time.sleep(3)
        
        yaw1 = self.get_current_yaw()
        print(f"特徴点1での測定値: {math.degrees(yaw1):.2f}°")
        
        # 期待角度の入力
        try:
            expected_yaw1 = float(input(f"「{landmark1_desc}」の正しい方位角（度）を入力: "))
            expected_yaw1_rad = math.radians(expected_yaw1)
        except ValueError:
            print("無効な角度です。0°として処理します。")
            expected_yaw1_rad = 0.0
        
        print()
        
        # Point 2の測定
        landmark2_desc = input("特徴点2の説明（例: 第1コーナー→第2コーナー）: ")
        input(f"車両を「{landmark2_desc}」方向に正確に向けて、Enterを押してください...")
        
        print("測定中... (3秒間安定化)")
        time.sleep(3)
        
        yaw2 = self.get_current_yaw()
        print(f"特徴点2での測定値: {math.degrees(yaw2):.2f}°")
        
        # 期待角度の入力
        try:
            expected_yaw2 = float(input(f"「{landmark2_desc}」の正しい方位角（度）を入力: "))
            expected_yaw2_rad = math.radians(expected_yaw2)
        except ValueError:
            print("無効な角度です。90°として処理します。")
            expected_yaw2_rad = math.radians(90)
        
        # 角度差の検証
        measured_diff = abs(yaw2 - yaw1)
        expected_diff = abs(expected_yaw2_rad - expected_yaw1_rad)
        
        # 角度を0-2πの範囲に正規化
        if measured_diff > math.pi:
            measured_diff = 2 * math.pi - measured_diff
        if expected_diff > math.pi:
            expected_diff = 2 * math.pi - expected_diff
        
        print()
        print("=== キャリブレーション結果 ===")
        print(f"測定角度差: {math.degrees(measured_diff):.2f}°")
        print(f"期待角度差: {math.degrees(expected_diff):.2f}°")
        
        # オフセット計算（2点の平均）
        offset1 = expected_yaw1_rad - yaw1
        offset2 = expected_yaw2_rad - yaw2
        
        # 角度の連続性を考慮した平均化
        if abs(offset2 - offset1) > math.pi:
            if offset2 > offset1:
                offset1 += 2 * math.pi
            else:
                offset2 += 2 * math.pi
        
        self.offset = (offset1 + offset2) / 2
        
        # -π to π の範囲に正規化
        while self.offset > math.pi:
            self.offset -= 2 * math.pi
        while self.offset < -math.pi:
            self.offset += 2 * math.pi
        
        print(f"計算されたオフセット: {math.degrees(self.offset):.2f}°")
        
        # 精度評価
        accuracy1 = abs(offset1 - self.offset)
        accuracy2 = abs(offset2 - self.offset)
        max_error = max(accuracy1, accuracy2)
        
        print(f"推定精度: ±{math.degrees(max_error):.2f}°")
        
        if max_error < math.radians(2):
            print("✅ 高精度キャリブレーション成功")
        elif max_error < math.radians(5):
            print("⚠️ 中程度精度（実用レベル）")
        else:
            print("❌ 低精度：再測定を推奨")
        
        # キャリブレーションデータ保存
        calib_data = {
            'offset': self.offset,
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'method': 'landmark_based',
            'landmarks': {
                'point1': {
                    'description': landmark1_desc,
                    'measured_yaw': yaw1,
                    'expected_yaw': expected_yaw1_rad
                },
                'point2': {
                    'description': landmark2_desc,
                    'measured_yaw': yaw2,
                    'expected_yaw': expected_yaw2_rad
                }
            },
            'accuracy': math.degrees(max_error)
        }
        
        self.save_calibration(calib_data)
        return True
    
    def save_calibration(self, calib_data):
        """キャリブレーションデータ保存"""
        try:
            with open(self.calibration_file, 'w') as f:
                json.dump(calib_data, f, indent=2)
            print(f"✓ キャリブレーションデータを {self.calibration_file} に保存しました")
        except Exception as e:
            print(f"保存エラー: {e}")
    
    def load_calibration(self):
        """キャリブレーションデータ読み込み"""
        try:
            with open(self.calibration_file, 'r') as f:
                calib_data = json.load(f)
            
            self.offset = calib_data.get('offset', 0.0)
            
            print(f"✓ 特徴点ベースキャリブレーション読み込み成功")
            print(f"  方法: {calib_data.get('method', 'unknown')}")
            print(f"  日時: {calib_data.get('calibration_date', 'unknown')}")
            print(f"  精度: ±{calib_data.get('accuracy', 0):.2f}°")
            
            if 'landmarks' in calib_data:
                landmarks = calib_data['landmarks']
                print(f"  特徴点1: {landmarks['point1']['description']}")
                print(f"  特徴点2: {landmarks['point2']['description']}")
            
            return True
            
        except FileNotFoundError:
            print(f"⚠️ キャリブレーションファイル {self.calibration_file} が見つかりません")
            return False
        except Exception as e:
            print(f"読み込みエラー: {e}")
            return False
    
    def calibrate_yaw(self, raw_yaw):
        """ヨー角にキャリブレーションを適用"""
        calibrated = raw_yaw + self.offset
        
        # -π to π の範囲に正規化
        while calibrated > math.pi:
            calibrated -= 2 * math.pi
        while calibrated < -math.pi:
            calibrated += 2 * math.pi
            
        return calibrated

# 使用例とテスト
if __name__ == "__main__":
    print("=== IMU Landmark Calibration Test ===")
    
    # BNO055ドライバーの初期化（可能であれば）
    imu_driver = None
    try:
        if is_raspberry_pi():
            from bno055_imu_driver import BNO055IMUDriver
            imu_driver = BNO055IMUDriver()
            print("✓ BNO055 driver loaded")
        else:
            print("⚠️ Mock mode (PC environment)")
    except Exception as e:
        print(f"IMU driver error: {e}")
    
    # 特徴点キャリブレーション実行
    calibrator = IMULandmarkCalibration(imu_driver)
    
    choice = input("実行モード選択: [c]alibrate / [t]est / [l]oad: ").lower()
    
    if choice == 'c':
        calibrator.calibrate_with_landmarks()
    elif choice == 'l':
        if calibrator.load_calibration():
            if imu_driver:
                raw_yaw = imu_driver.get_yaw()
                calibrated_yaw = calibrator.calibrate_yaw(raw_yaw)
                print(f"Raw yaw: {math.degrees(raw_yaw):.2f}°")
                print(f"Calibrated yaw: {math.degrees(calibrated_yaw):.2f}°")
    elif choice == 't':
        # テストモード
        calibrator.load_calibration()
        print("リアルタイムキャリブレーション済み角度表示（Ctrl+Cで終了）")
        try:
            while True:
                if imu_driver:
                    raw_yaw = imu_driver.get_yaw()
                    calibrated_yaw = calibrator.calibrate_yaw(raw_yaw)
                    print(f"Raw: {math.degrees(raw_yaw):6.2f}° → Calibrated: {math.degrees(calibrated_yaw):6.2f}°")
                else:
                    print("Mock mode: no real sensor data")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nテスト終了")
    else:
        print("無効な選択です")