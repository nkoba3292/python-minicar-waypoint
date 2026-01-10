import RPi.GPIO as GPIO
import time


class UltrasonicSensorsPolling:
    """
    5個の超音波センサをポーリング方式で測定するクラス
    スレッドを使わず、measure_once()を呼び出した時のみ測定
    """
    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        self.gpio_initialized = False
        
        try:
            # GPIO警告を無効化
            GPIO.setwarnings(False)
            
            # モード設定
            if GPIO.getmode() is None:
                GPIO.setmode(GPIO.BOARD)

            # GPIOピン設定（togikai完全準拠）
            self.trig_pins = [15, 13, 35, 32, 36]  # 前方, 左前45, 左90, 右前45, 右90
            self.echo_pins = [26, 24, 37, 31, 38]

            # ピン設定（togikai完全準拠：リストで一括設定）
            GPIO.setup(self.trig_pins, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.echo_pins, GPIO.IN)
            
            self.gpio_initialized = True
            if debug_mode:
                print('[OK] GPIO initialization successful (togikai-compliant)')
            
        except Exception as e:
            print(f'[ERROR] GPIO initialization failed: {e}')
            import traceback
            traceback.print_exc()
            self.gpio_initialized = False
            raise

        # 最後の測定結果保存
        self.distances = [200.0] * 5  # デフォルト200cm
        self.measurement_count = 0

    def _measure_single_sensor(self, trig, echo, channel_name="Unknown", n=1):
        """
        1つのセンサーを測定（togikai完全準拠）
        
        Args:
            trig: トリガーピン番号
            echo: エコーピン番号
            channel_name: センサー名（ログ用）
            n: 平均回数（togikai準拠: デフォルト1回）
        
        Returns:
            float: 測定距離[cm]、エラー時は200.0
        """
        if not self.gpio_initialized:
            return 200.0
        
        dis = 0
        
        for i in range(n):
            sigoff = 0
            sigon = 0
            
            try:
                # トリガー送信（togikai完全準拠）
                GPIO.output(trig, GPIO.HIGH)
                time.sleep(0.00001)  # 10μs
                GPIO.output(trig, GPIO.LOW)
                
                # エコー受信開始待ち（togikai準拠：タイムアウト0.02秒）
                kijyun = time.time()
                while GPIO.input(echo) == GPIO.LOW:
                    sigoff = time.time()
                    if sigoff - kijyun > 0.02:
                        break
                
                # エコー受信終了待ち（togikai準拠：タイムアウト0.02秒）
                while GPIO.input(echo) == GPIO.HIGH:
                    sigon = time.time()
                    if sigon - sigoff > 0.02:
                        break
                
                # 距離計算（togikai完全準拠）
                d = (sigon - sigoff) * 34000 / 2
                if d > 200:
                    dis += 200 / n
                else:
                    dis += d / n
                
            except Exception as e:
                if self.debug_mode:
                    print(f"[ERROR] {channel_name} measurement error: {e}")
                dis += 200 / n
        
        return dis

    def measure_once(self):
        """
        全センサーを1回測定（ブロッキング）
        togikai準拠: センサー間待機なし
        
        Returns:
            list: [前方, 左前45, 左90, 右前45, 右90] の距離[cm]
        """
        if not self.gpio_initialized:
            print('[ERROR] GPIO not initialized')
            return [200.0] * 5
        
        ch_names = ["Front", "Left45", "Left90", "Right45", "Right90"]
        
        for i, (trig, echo, name) in enumerate(zip(self.trig_pins, self.echo_pins, ch_names)):
            self.distances[i] = self._measure_single_sensor(trig, echo, name, n=1)  # togikai準拠: n=1
            # togikai準拠: センサー間待機なし
        
        self.measurement_count += 1
        
        if self.debug_mode and self.measurement_count <= 3:
            print(f"[DEBUG] Measurement {self.measurement_count}: {[f'{d:.1f}' for d in self.distances]}")
        
        return self.distances.copy()

    def get_distances(self):
        """
        最後の測定結果を返す（measure_once()を呼び出さない）
        
        Returns:
            list: 最後の測定結果
        """
        return self.distances.copy()

    def stop(self):
        """
        GPIO クリーンアップ
        """
        try:
            GPIO.cleanup()
            print('[OK] GPIO cleaned up')
        except Exception as e:
            print(f'[WARN] GPIO cleanup warning: {e}')


# --- デバッグ実行モード ---
if __name__ == "__main__":
    print("=" * 80)
    print("超音波センサアレイ - ポーリングモード デバッグ")
    print("測定: 手動呼び出し時のみ | 5チャンネル (前/L45/L90/R45/R90)")
    print("Ctrl+C で終了")
    print("=" * 80)
    
    sensors = UltrasonicSensorsPolling(debug_mode=True)
    cycle_count = 0
    start_time = time.time()
    
    try:
        while True:
            # 測定実行（手動呼び出し）
            d = sensors.measure_once()
            cycle_count += 1
            
            # 表示
            print(f"[{cycle_count:3d}] Fr:{d[0]:6.1f} | L45:{d[1]:6.1f} | L90:{d[2]:6.1f} | R45:{d[3]:6.1f} | R90:{d[4]:6.1f}")
            
            # 次の測定まで待機（0.1秒 = 10Hz）
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print("測定停止中...")
        sensors.stop()
        total_time = time.time() - start_time
        avg_cycle = (total_time / cycle_count * 1000) if cycle_count > 0 else 0
        print(f"総測定回数: {cycle_count} | 総時間: {total_time:.2f}s | 平均周期: {avg_cycle:.1f}ms")
        print("Stopped.")
        print("=" * 80)
