# vehicle_interface_mock.py
import threading
import time
import numpy as np
import random

# ----------------------------
# モックGPIOとPCA9685
# ----------------------------
class MockGPIO:
    BOARD = 'BOARD'
    OUT = 'OUT'
    IN = 'IN'
    LOW = 0
    HIGH = 1

    @staticmethod
    def setmode(mode): pass
    @staticmethod
    def setup(pins, mode, initial=None): pass
    @staticmethod
    def output(pin, val): print(f"[GPIO] set pin {pin} to {val}")
    @staticmethod
    def input(pin): return 0
    @staticmethod
    def cleanup(pins=None): print("[GPIO] cleanup")

class MockPCA9685:
    def set_pwm_freq(self, freq): print(f"[PWM] freq set {freq}")
    def set_pwm(self, channel, on, off): print(f"[PWM] channel {channel} -> {off}")

# ----------------------------
# 超音波センサクラス (モック)
# ----------------------------
class UltrasonicArray:
    def __init__(self, trig_pins, echo_pins, max_distance=200.0, update_interval=0.05, use_mock=True):
        self.distances = np.zeros(5)
        self._stop_flag = False
        self.update_interval = update_interval
        self.use_mock = use_mock
        if not use_mock:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(trig_pins, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(echo_pins, GPIO.IN)
        else:
            self.GPIO = MockGPIO()
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()

    def _measure(self, trig, echo):
        if self.use_mock:
            return random.uniform(10, 150)  # ダミー値
        # 実際の測定は省略
        return 100.0

    def _update_loop(self):
        while not self._stop_flag:
            for i in range(5):
                self.distances[i] = self._measure(0,0)
            time.sleep(self.update_interval)

    def get_distances(self):
        return self.distances.copy()

    def stop(self):
        self._stop_flag = True
        self.thread.join()
        self.GPIO.cleanup()

# ----------------------------
# PWM制御クラス (モック)
# ----------------------------
class VehiclePWM:
    def __init__(self, use_mock=True):
        self.use_mock = use_mock
        if use_mock:
            self.pwm = MockPCA9685()
        else:
            from Adafruit_PCA9685 import PCA9685
            self.pwm = PCA9685()
        self.pwm.set_pwm_freq(50)
        self.PWM_PARAM = ([400, 370, 340], [470, 420, 380])  # 仮値

    def load_parameters(self, path=None):
        return self.PWM_PARAM

    def accel(self, duty):
        print(f"[ACCEL] duty={duty}")

    def steer(self, duty):
        print(f"[STEER] duty={duty}")

# ----------------------------
# IMUクラス (モック)
# ----------------------------
class BNO055_UART:
    def __init__(self, dev=None, baud=115200, use_mock=True):
        self.euler = [0.0,0.0,0.0]
        self._stop_flag = False
        self.use_mock = use_mock
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()

    def _update_loop(self):
        while not self._stop_flag:
            if self.use_mock:
                self.euler = [random.uniform(-180,180),
                              random.uniform(-90,90),
                              random.uniform(-180,180)]
            time.sleep(0.05)

    def get_euler(self):
        return self.euler.copy()

    def stop(self):
        self._stop_flag = True
        self.thread.join()

# ----------------------------
# 使用例
# ----------------------------
if __name__ == "__main__":
    us = UltrasonicArray([0]*5, [0]*5)
    pwm_ctrl = VehiclePWM()
    imu = BNO055_UART()

    try:
        for _ in range(10):
            print("US distances:", us.get_distances())
            print("IMU euler:", imu.get_euler())
            pwm_ctrl.accel(50)
            pwm_ctrl.steer(-30)
            time.sleep(0.1)
    finally:
        us.stop()
        imu.stop()
