#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
白線検出クラス (74HC74 セット動作版)

動作確認済み:
- D入力 = HIGH (VCC接続)
- 白線検出 → RPR220パルス → Q=1にセット
- GPIO18 (CLR) でQ=0にリセット

使用方法:
    detector = WhiteLineDetector()
    
    while True:
        if detector.is_detected():
            print("白線検出!")
            detector.clear()  # 検出後クリア
"""

import RPi.GPIO as GPIO
import time

class WhiteLineDetector:
    """74HC74ベースの白線検出クラス"""
    
    def __init__(self, q_pin=23, clr_pin=18):
        """
        初期化
        
        Args:
            q_pin: 74HC74 Q出力ピン (BCM番号)
            clr_pin: 74HC74 CLR入力ピン (BCM番号)
        """
        self.q_pin = q_pin
        self.clr_pin = clr_pin
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # CLR制御 (初期値HIGH = クリア無効)
        GPIO.setup(self.clr_pin, GPIO.OUT, initial=GPIO.HIGH)
        
        # Q出力読み取り
        GPIO.setup(self.q_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        
        # 初期化時にクリア
        self.clear()
    
    def is_detected(self):
        """
        白線検出状態を取得
        
        Returns:
            bool: True=白線検出, False=白線なし
        """
        return GPIO.input(self.q_pin) == 1
    
    def clear(self):
        """
        検出状態をクリア (Q=0にリセット)
        """
        GPIO.output(self.clr_pin, GPIO.LOW)
        time.sleep(0.001)  # 1ms
        GPIO.output(self.clr_pin, GPIO.HIGH)
        time.sleep(0.001)
    
    def cleanup(self):
        """GPIOクリーンアップ"""
        GPIO.cleanup()


def main():
    """テスト用メイン関数"""
    print("=" * 70)
    print("白線検出クラス テスト")
    print("=" * 70)
    print()
    
    detector = WhiteLineDetector()
    
    try:
        print("白線検出を監視します (0.1秒間隔)")
        print("Ctrl+C で終了")
        print()
        
        detection_count = 0
        prev_state = False
        
        while True:
            detected = detector.is_detected()
            
            if detected and not prev_state:
                # 新規検出
                ts = time.strftime('%H:%M:%S')
                detection_count += 1
                print(f"[{ts}] ★ 白線検出 #{detection_count}")
                
                # 自動クリア
                time.sleep(0.5)  # 検出状態を0.5秒維持
                detector.clear()
                print(f"[{ts}]   クリア完了")
                
                prev_state = False
            elif not detected:
                prev_state = False
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print()
        print()
        print(f"検出回数: {detection_count}回")
        print()
    
    finally:
        detector.cleanup()


if __name__ == "__main__":
    main()
