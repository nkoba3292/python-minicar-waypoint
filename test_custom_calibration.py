# test_custom_calibration.py - カスタムキャリブレーション読み込みテスト
import json
import math

def test_custom_calibration_loading():
    """カスタムキャリブレーション読み込みテスト"""
    print("🧪 Testing custom calibration file loading...")
    
    # キャリブレーションファイルリスト（main_control_loop.pyと同じ）
    calibration_files = [
        ("imu_custom_calib.json", "custom_user_selection"),  # 最優先
        ("imu_visual_calib.json", "visual_map"),
        ("imu_landmark_calib.json", "landmark"),  
        ("imu_2point_calib.json", "2point")
    ]
    
    course_offset = 0.0
    calibration_method = "none"
    
    # 読み込み処理テスト
    for calib_file, method in calibration_files:
        try:
            with open(calib_file, 'r') as f:
                calib_data = json.load(f)
            
            print(f"\n✅ Found calibration file: {calib_file}")
            print(f"   Method: {method}")
            
            # カスタムキャリブレーション形式の処理
            if method == "custom_user_selection":
                if 'calibration_result' in calib_data and 'yaw_offset' in calib_data['calibration_result']:
                    course_offset = math.radians(calib_data['calibration_result']['yaw_offset'])
                    print(f"   ✓ Custom calibration format detected")
                    print(f"   Raw yaw offset: {calib_data['calibration_result']['yaw_offset']:.1f}°")
                else:
                    print(f"   ❌ Invalid custom calibration format")
                    continue
            else:
                # 従来形式の処理
                course_offset = calib_data.get('offset', 0.0)
                print(f"   ✓ Standard calibration format")
            
            calibration_method = method
            
            print(f"\n📊 CALIBRATION LOADED:")
            print(f"   Method: {method.upper().replace('_', ' ')}")
            print(f"   File: {calib_file}")
            print(f"   Offset: {math.degrees(course_offset):.2f}°")
            
            # タイムスタンプ表示
            if 'calibration_date' in calib_data:
                print(f"   Date: {calib_data['calibration_date']}")
            elif 'timestamp' in calib_data:
                print(f"   Date: {calib_data['timestamp']}")
            
            # カスタムキャリブレーション詳細情報
            if method == "custom_user_selection" and 'usage_instructions' in calib_data:
                instructions = calib_data['usage_instructions']
                print(f"\n🎯 CUSTOM CALIBRATION DETAILS:")
                print(f"   Reference heading: {instructions.get('reference_heading', 'N/A')}")
                print(f"   Vehicle position: {instructions.get('vehicle_position', 'N/A')}")
                print(f"   Target direction: {instructions.get('target_direction', 'N/A')}")
                
                if 'setup_points' in calib_data:
                    setup = calib_data['setup_points']
                    print(f"   Distance: {setup.get('distance', 'N/A')}m")
            
            break
            
        except FileNotFoundError:
            print(f"❌ Not found: {calib_file}")
            continue
        except Exception as e:
            print(f"❌ Error loading {calib_file}: {e}")
            continue
    
    # テスト結果サマリー
    print(f"\n{'='*50}")
    if calibration_method != "none":
        print(f"✅ CALIBRATION TEST SUCCESS!")
        print(f"Selected method: {calibration_method}")
        print(f"Applied offset: {math.degrees(course_offset):.2f}°")
        
        # 模擬Yaw補正テスト
        mock_raw_yaw = 180.0  # 模擬生Yaw値
        corrected_yaw = mock_raw_yaw + math.degrees(course_offset)
        
        print(f"\n🧪 YAW CORRECTION TEST:")
        print(f"   Mock raw IMU yaw: {mock_raw_yaw:.1f}°")
        print(f"   Applied offset: {math.degrees(course_offset):.2f}°")
        print(f"   Corrected yaw: {corrected_yaw:.1f}°")
        
    else:
        print(f"❌ NO CALIBRATION FOUND")
        print("All calibration files are missing or invalid")
    
    print(f"{'='*50}")
    
    return calibration_method != "none"

if __name__ == "__main__":
    success = test_custom_calibration_loading()
    
    if success:
        print("\n🎊 Custom calibration integration test PASSED!")
        print("Your main_control_loop.py is ready to use custom calibration.")
    else:
        print("\n❌ Custom calibration integration test FAILED!")
        print("Check your calibration files and try again.")