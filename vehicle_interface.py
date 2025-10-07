# vehicle_interface.py
import threading
import time
import numpy as np

# -------------------------------
# Mock imports for PCデバッグ
try:
    import RPi.GPIO as GPIO
    import Adafruit_PCA9685
except ImportError:
    class GPIO:
        BOARD = None
        OUT = IN = LOW = HIGH = None
        @staticmethod
        def setmode(a): pass
        @staticmethod
        def setup(a, b, initial=None): pass
        @staticmethod
        def output(a, b): pass
        @staticmethod
        def input(a): return 0
        @staticmethod
        def cleanup(): pass

    class Adafruit_PCA9685:
        class PCA9685:
            def __init__(self, address=None): pass
            def set_pwm_freq(self, freq): pass
            def set_pwm(self, channel, on, off): pass

# -------------------------------
# Ultrasonic Sensor Array
class UltrasonicArray:
    def __init__(self, trig_pins, echo_pins, update_interval=0.05):
        self.trig_pins = trig_pins
        self.echo_pins = echo_pins
        self.n = len(trig_pins)
        self.update_interval = update_interval
        self.distances = [0.0]*self.n
        self.running = False
        GPIO.setmode(GPIO.BOARD)
        for t in trig_pins:
            GPIO.setup(t, GPIO.OUT, initial=GPIO.LOW)
        for e in echo_pins:
            GPIO.setup(e, GPIO.IN)

    def _measure_distance(self, trig, echo):
        # 実機では超音波送受信
        # PCモックではランダム値
        try:
            GPIO.output(trig, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(trig, GPIO.LOW)
            start = time.time()
            while GPIO.input(echo) == GPIO.LOW:
                if time.time() - start > 0.02: break
            t1 = time.time()
            while GPIO.input(echo) == GPIO.HIGH:
                if time.time() - t1 > 0.02: break
            t2 = time.time()
            d = (t2 - t1) * 34000 / 2
            return min(d, 200.0)
        except:
            # モック値
            return np.random.uniform(10, 150)

    def _update_loop(self):
        while self.running:
            for i in range(self.n):
                self.distances[i] = self._measure_distance(self.trig_pins[i], self.echo_pins[i])
            time.sleep(self.update_interval)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        GPIO.cleanup()

    def get_distances(self):
        return self.distances.copy()

# -------------------------------
# IMU Sensor
class IMUSensor:
    def __init__(self, update_interval=0.05):
        self.update_interval = update_interval
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.running = False

    def _update_loop(self):
        while self.running:
            # 実機ではBNO055から読み出し
            # PCモックではランダム値
            self.yaw = np.random.uniform(-180, 180)
            self.pitch = np.random.uniform(-90, 90)
            self.roll = np.random.uniform(-180, 180)
            time.sleep(self.update_interval)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()

    def get_orientation(self):
        return self.yaw, self.pitch, self.roll

# -------------------------------
# Motor Drive
class MotorDrive:
    def __init__(self, pwm_address=0x40):
        self.pwm = Adafruit_PCA9685.PCA9685(address=pwm_address)
        self.pwm.set_pwm_freq(60)
        # PWMパラメータの読み込み（例:キャリブレーションファイル）
        self.PWM_PARAM = self._read_pwm_param()
        self.throttle_pwm = 0
        self.steer_pwm = 0

    def _read_pwm_param(self):
        # 実際はファイル読み込み
        # mock: [steer_right, steer_center, steer_left], [forward, stopped, reverse]
        return ([409, 307, 205],[410, 307, 205])

    def accel(self, duty):
        # duty -100～100
        forward_pwm, stop_pwm, reverse_pwm = self.PWM_PARAM[1]
        if duty > 0:
            pwm_val = int(stop_pwm - (stop_pwm - forward_pwm) * duty / 100)
        elif duty < 0:
            pwm_val = int(stop_pwm + (reverse_pwm - stop_pwm) * abs(duty) / 100)
        else:
            pwm_val = stop_pwm
        self.throttle_pwm = pwm_val
        self.pwm.set_pwm(13, 0, pwm_val)

    def steer(self, duty):
        # duty -100～100
        right_pwm, center_pwm, left_pwm = self.PWM_PARAM[0]
        if duty > 0:
            pwm_val = int(center_pwm - (center_pwm - right_pwm) * duty / 100)
        elif duty < 0:
            pwm_val = int(center_pwm + (left_pwm - center_pwm) * abs(duty) / 100)
        else:
            pwm_val = center_pwm
        self.steer_pwm = pwm_val
        self.pwm.set_pwm(14, 0, pwm_val)

    def stop(self):
        self.accel(0)
        self.steer(0)

# -------------------------------
# Vehicle Interface
class Vehicle:
    def __init__(self):
        # センサピン指定（Triger, Echo）
        self.ultrasonic = UltrasonicArray(
            trig_pins=[15,13,35,32,36],
            echo_pins=[26,24,37,31,38]
        )
        self.imu = IMUSensor()
        self.motor = MotorDrive()

    def start(self):
        self.ultrasonic.start()
        self.imu.start()

    def stop(self):
        self.ultrasonic.stop()
        self.imu.stop()
        self.motor.stop()

    def get_sensors(self):
        distances = self.ultrasonic.get_distances()
        yaw, pitch, roll = self.imu.get_orientation()
        return {
            "ultrasonic": distances,
            "imu": {"yaw": yaw, "pitch": pitch, "roll": roll}
        }

    def set_drive(self, accel_duty, steer_duty):
        self.motor.accel(accel_duty)
        self.motor.steer(steer_duty)

# -------------------------------
# Example usage
if __name__ == "__main__":
    vehicle = Vehicle()
    vehicle.start()
    try:
        for _ in range(20):
            sensors = vehicle.get_sensors()
            print("Distances:", sensors["ultrasonic"])
            print("IMU yaw/pitch/roll:", sensors["imu"])
            vehicle.set_drive(accel_duty=50, steer_duty=10)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        vehicle.stop()
