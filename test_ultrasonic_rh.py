# test_ultrasonic_rh.py
# 単体テスト: 右90°センサ (RH) の生データを取得する
# TRIG=32, ECHO=31 (togikai サンプルに合わせた値)

import time
import sys

# togikai パス (Raspberry Pi 側の環境にあわせて必要なら変更)
sys.path.append('/home/pi/togikai/togikai_function/')

try:
    import togikai_ultrasonic
    import RPi.GPIO as GPIO
except Exception as e:
    print('モジュール読込エラー:', e)
    togikai_ultrasonic = None

TRIG = 32
ECHO = 31

print('単体テスト: RH (右90°) TRIG/ECHO ->', TRIG, ECHO)
if togikai_ultrasonic is None:
    print('togikai_ultrasonic モジュールが利用できません。Pi 環境で実行してください。')
    sys.exit(1)

for i in range(30):
    try:
        raw = togikai_ultrasonic.Mesure(GPIO, time, TRIG, ECHO)
        print(f'[RAW] {i+1:02d}: {raw}')
    except Exception as ex:
        print(f'[ERR] {i+1:02d}: {ex}')
    time.sleep(0.2)

print('テスト完了')
