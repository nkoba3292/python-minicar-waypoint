import RPi.GPIO as GPIO
import time
import threading
import math

class UltrasonicSensors:
    """
    5個の超音波センサをバックグラウンドで順次測定し、
    常に最新の距離を取得可能にするクラス。
    """
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        # GPIOピン設定
        self.trig_pins = [15, 13, 35, 32, 36]  # 前方, 左前45, 左90, 右前45, 右90
        self.echo_pins = [26, 24, 37, 31, 38]

        for t in self.trig_pins:
            GPIO.setup(t, GPIO.OUT, initial=GPIO.LOW)
        for e in self.echo_pins:
            GPIO.setup(e, GPIO.IN)

        # 測定結果保存（[前, 左前45, 左90, 右前45, 右90]）
        self.distances = [0.0] * 5

        # 測定スレッド制御
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _measure_distance(self, trig, echo, max_distance=200.0):
        """
        超音波距離測定（単発）
        max_distance: 最大計測距離 [cm]
        """
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, GPIO.LOW)

        start_time = time.time()
        sig_off = None
        sig_on = None

        # echo LOW待ち
        while GPIO.input(echo) == GPIO.LOW:
            sig_off = time.time()
            if sig_off - start_time > 0.02:  # タイムアウト20ms
                return max_distance

        # echo HIGH待ち
        while GPIO.input(echo) == GPIO.HIGH:
            sig_on = time.time()
            if sig_on - sig_off > 0.02:  # タイムアウト20ms
                return max_distance

        if sig_off is None or sig_on is None:
            return max_distance

        # 距離計算 [cm]
        distance = (sig_on - sig_off) * 34000 / 2
        return min(distance, max_distance)

    def _update_loop(self):
        """
        バックグラウンド測定ループ
        5個のセンサを順次測定
        """
        while not self._stop_event.is_set():
            for i, (t, e) in enumerate(zip(self.trig_pins, self.echo_pins)):
                self.distances[i] = self._measure_distance(t, e)
                time.sleep(0.01)  # 隣接センサ干渉防止

            # 全センサ更新ごとに少し待機
            time.sleep(0.01)

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

# --- 使用例 ---
if __name__ == "__main__":
    sensors = UltrasonicSensors()
    try:
        while True:
            d = sensors.get_distances()
            print(f"Front: {d[0]:.1f} cm | L45: {d[1]:.1f} | L90: {d[2]:.1f} | R45: {d[3]:.1f} | R90: {d[4]:.1f}")
            time.sleep(0.05)  # 50msごとに表示

    except KeyboardInterrupt:
        sensors.stop()
        print("Stopped.")
