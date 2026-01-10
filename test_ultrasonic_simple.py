#!/usr/bin/env python3
"""
超音波センサー単体テスト（togikai完全準拠）
WiFi切断問題の診断用
"""
import RPi.GPIO as GPIO
import time
import sys

print("="*60)
print("超音波センサー単体テスト（togikai完全準拠版）")
print("="*60)

# GPIO初期化（togikai完全準拠）
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# ピン設定（togikai完全準拠：リストで一括設定）
t_list = [15, 13, 35, 32, 36]
e_list = [26, 24, 37, 31, 38]

print(f"[1] GPIO初期化中...")
try:
    GPIO.setup(t_list, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(e_list, GPIO.IN)
    print("[OK] GPIO初期化完了")
except Exception as e:
    print(f"[ERROR] GPIO初期化失敗: {e}")
    sys.exit(1)

# togikai完全準拠の測定関数
def Mesure(trig, echo, name):
    dis = 0
    n = 1
    for i in range(n):
        sigoff = 0
        sigon = 0
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, GPIO.LOW)
        kijyun = time.time()
        
        while GPIO.input(echo) == GPIO.LOW:
            sigoff = time.time()
            if sigoff - kijyun > 0.02:
                break
        
        while GPIO.input(echo) == GPIO.HIGH:
            sigon = time.time()
            if sigon - sigoff > 0.02:
                break
        
        d = (sigon - sigoff) * 34000 / 2
        if d > 200:
            dis += 200 / n
        else:
            dis += d / n
    
    return dis

# WiFi状態確認関数
def check_wifi():
    import subprocess
    try:
        result = subprocess.run(['iwconfig', 'wlan0'], 
                              capture_output=True, text=True, timeout=2)
        if 'ESSID' in result.stdout:
            # ESSIDから接続名を抽出
            for line in result.stdout.split('\n'):
                if 'ESSID' in line:
                    return f"接続中 ({line.split('ESSID:')[1].split()[0]})"
        return "切断"
    except:
        return "確認失敗"

print(f"\n[2] WiFi初期状態: {check_wifi()}")
print(f"\n[3] 測定開始（10回）...")
print("    測定中にWiFiが切断されるか監視します\n")

try:
    for count in range(10):
        # 全センサー測定
        FRdis = Mesure(15, 26, "FR")
        LHdis = Mesure(13, 24, "LH")
        RHdis = Mesure(32, 31, "RH")
        RLHdis = Mesure(35, 37, "RLH")
        RRHdis = Mesure(36, 38, "RRH")
        
        # WiFi状態確認
        wifi_status = check_wifi()
        
        # 結果表示
        print(f"[{count+1}/10] FR:{FRdis:5.1f} LH:{LHdis:5.1f} RH:{RHdis:5.1f} "
              f"RLH:{RLHdis:5.1f} RRH:{RRHdis:5.1f} | WiFi:{wifi_status}")
        
        # WiFi切断検出
        if "切断" in wifi_status:
            print("\n" + "="*60)
            print("！！！WiFi切断を検出！！！")
            print("="*60)
            break
        
        time.sleep(0.5)
    
    print(f"\n[4] 測定完了")
    print(f"    最終WiFi状態: {check_wifi()}")

except KeyboardInterrupt:
    print("\n中断されました")
finally:
    GPIO.cleanup()
    print("[OK] GPIO cleanup完了")
