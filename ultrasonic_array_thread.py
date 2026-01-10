import RPi.GPIO as GPIO
import time
import threading
import math


class UltrasonicSensors:
    """
    5個の超音波センサをバックグラウンドで順次測定し、
    常に最新の距離を取得可能にするクラス。
    """
    def __init__(self, debug_mode=False):
        print('[DEBUG] UltrasonicSensors.__init__ start')
        self.debug_mode = debug_mode
        self.gpio_initialized = False
        
        try:
            # GPIO警告を無効化（既存のピン設定があっても警告を出さない）
            GPIO.setwarnings(False)
            
            # 既にモードが設定されていなければBOARDで初期化
            if GPIO.getmode() is None:
                print('[DEBUG] GPIO.setmode(GPIO.BOARD)')
                GPIO.setmode(GPIO.BOARD)
            print('[DEBUG] GPIO mode:', GPIO.getmode())

            # GPIOピン設定
            self.trig_pins = [15, 13, 35, 32, 36]  # 前方, 左前45, 左90, 右前45, 右90
            self.echo_pins = [26, 24, 37, 31, 38]
            print('[DEBUG] trig_pins:', self.trig_pins)
            print('[DEBUG] echo_pins:', self.echo_pins)

            # ピン設定時の安全性確保
            for t in self.trig_pins:
                print(f'[DEBUG] GPIO.setup trig {t}')
                GPIO.setup(t, GPIO.OUT, initial=GPIO.LOW)
                time.sleep(0.01)  # ピン間の待機時間追加（競合回避）
            
            for e in self.echo_pins:
                print(f'[DEBUG] GPIO.setup echo {e}')
                GPIO.setup(e, GPIO.IN)
                time.sleep(0.01)  # ピン間の待機時間追加
            
            self.gpio_initialized = True
            print('[DEBUG] GPIO initialization successful')
            
        except Exception as e:
            print(f'[ERROR] GPIO initialization failed: {e}')
            import traceback
            traceback.print_exc()
            self.gpio_initialized = False
            raise  # 初期化失敗時は例外を再送出

        # 測定結果保存（[前, 左前45, 左90, 右前45, 右90]）
        self.distances = [0.0] * 5
        self.measurement_count = 0  # 測定回数カウンタ（デバッグ用）
        print('[DEBUG] distances initialized')

        # 測定スレッド制御
        self._stop_event = threading.Event()
        print('[DEBUG] threading.Event created')
        self._thread = threading.Thread(target=lambda: self._update_loop(debug_mode), daemon=True)
        print('[DEBUG] Thread object created')
        self._thread.start()
        print('[DEBUG] Thread started')

    def _measure_distance(self, trig, echo, channel_index, max_distance=200.0):
        """
        超音波距離測定（単発）
        togikai_sample完全準拠の実装 + エラー時前回値保持
        max_distance: 最大計測距離 [cm]
        channel_index: チャンネルインデックス（前回値保持用）
        """
        # GPIO初期化されていない場合は即座に失敗値を返す
        if not self.gpio_initialized:
            return 200.0
        
        dis = 0
        n = 1  # togikai準拠：平均化回数（デフォルト1回）
        
        for i in range(n):
            sigoff = 0
            sigon = 0
            
            try:
                GPIO.output(trig, GPIO.HIGH)
                time.sleep(0.00001)
                GPIO.output(trig, GPIO.LOW)
            except Exception as e:
                if self.measurement_count < 5:  # 最初の5回だけログ
                    print(f"[ERROR] GPIO.output failed on trig={trig}: {e}")
                return 200.0  # エラー時は200cmを返す（前回値依存を避ける）
            
            kijyun = time.time()
            timeout_occurred = False
            
            # echo LOW待ち（togikai完全準拠）
            try:
                while GPIO.input(echo) == GPIO.LOW:
                    sigoff = time.time()
                    if sigoff - kijyun > 0.02:
                        timeout_occurred = True
                        if self.measurement_count < 5:
                            print(f"[WARN] Ch{channel_index} timeout waiting for echo LOW->HIGH")
                        break
            except Exception as e:
                if self.measurement_count < 5:
                    print(f"[ERROR] GPIO.input failed on echo={echo}: {e}")
                return 200.0
            
            # echo HIGH待ち（togikai完全準拠）
            if not timeout_occurred:
                while GPIO.input(echo) == GPIO.HIGH:
                    sigon = time.time()
                    if sigon - sigoff > 0.02:
                        timeout_occurred = True
                        if self.measurement_count < 5:
                            print(f"[WARN] Ch{channel_index} timeout waiting for echo HIGH->LOW")
                        break
            
            # タイムアウトまたは信号エラーの場合のみ前回値を返す
            # 初回は200cmをデフォルト値として返す（前回値0の無限ループ回避）
            if sigoff == 0 or sigon == 0:
                if self.distances[channel_index] == 0:
                    return 200.0  # 初回エラー時は200cmを返す
                return self.distances[channel_index]
            
            # 距離計算（togikai完全準拠）
            d = (sigon - sigoff) * 34000 / 2
            if d > 200:
                dis += 200 / n
            else:
                dis += d / n
        
        # デバッグ: 最初の数回は測定結果をログ
        if self.measurement_count < 3:
            print(f"[DEBUG] Ch{channel_index} measured: {dis:.1f}cm (sigoff={sigoff:.6f}, sigon={sigon:.6f}, dt={sigon-sigoff:.6f})")
        
        return dis

    def _update_loop(self, debug_mode=False):
        """
        バックグラウンド測定ループ
        5個のセンサを順次測定
        計測周期: 約100ms (10Hz - 実用的な障害物検知速度)
        エラー時は前回値を保持
        
        注意: WiFi切断が発生する場合は --disable-wifi オプション使用を推奨
        """
        ch_names = ["Front", "L45", "L90", "R45", "R90"]
        
        # 最初のサイクル完了を通知
        first_cycle = True
        
        while not self._stop_event.is_set():
            try:
                for i, (t, e) in enumerate(zip(self.trig_pins, self.echo_pins)):
                    # 測定実行
                    self.distances[i] = self._measure_distance(t, e, i)
                    # センサー間待機（最小限）
                    time.sleep(0.001)  # 1ms

                self.measurement_count += 1
                
                # 最初のサイクル完了時のみログ出力
                if first_cycle:
                    print(f"[DEBUG] First measurement cycle complete: {self.distances}")
                    first_cycle = False

                # 1サイクル完了後の待機（100ms周期を実現）
                time.sleep(0.05)  # 50ms（センサー測定約50ms + 待機50ms = 100ms周期）
                
            except Exception as e:
                print(f"[ERROR] Measurement loop error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.5)  # エラー時は長めに待機

    def get_distances(self):
        """
        最新の5個の距離を返す
        """
        return self.distances.copy()

    def stop(self):
        """
        測定停止とGPIOクリーンアップ
        """
        self._stop_event.set()
        self._thread.join()
        GPIO.cleanup()

# --- デバッグ実行モード ---
if __name__ == "__main__":
    print("=" * 80)
    print("超音波センサアレイ デバッグモード (togikai_sample準拠)")
    print("計測周期: 約100ms | 5チャンネル (前/L45/L90/R45/R90)")
    print("Ctrl+C で終了")
    print("=" * 80)
    
    # センサー初期化（デバッグモード無効でtogikai完全準拠）
    sensors = UltrasonicSensors(debug_mode=False)
    cycle_count = 0
    start_time = time.time()
    
    try:
        while True:
            cycle_start = time.time()
            
            # 最新の距離取得
            d = sensors.get_distances()
            cycle_count += 1
            elapsed = time.time() - start_time
            
            # モニター出力（togikai_sample風）
            print(f"Fr:{d[0]:6.1f} , FrRH:{d[3]:6.1f} , FrLH:{d[1]:6.1f}, RrRH:{d[4]:6.1f} , RrLH:{d[2]:6.1f}")
            
            # togikai準拠：50ms待機
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n" + "=" * 80)
        print("測定停止中...")
        sensors.stop()
        total_time = time.time() - start_time
        avg_cycle = (total_time / cycle_count * 1000) if cycle_count > 0 else 0
        print(f"総測定回数: {cycle_count} | 総時間: {total_time:.2f}s | 平均周期: {avg_cycle:.1f}ms")
        print("Stopped.")
        print("=" * 80)
