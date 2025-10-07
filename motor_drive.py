import time

# --- PCでデバッグ用モック ---
try:
    import Adafruit_PCA9685
except ImportError:
    class MockPWM:
        def set_pwm(self, channel, on, off):
            print(f"[MockPWM] Channel {channel} set to {off}")
    Adafruit_PCA9685 = MockPWM()

class VehicleController:
    """
    RCカーや小型車両のスロットル＆ステアリングを制御するクラス
    """
    def __init__(self, pwm=None, param_file='/home/pi/togikai/alignment_parameter.txt'):
        # PCA9685 インスタンス
        self.pwm = pwm or Adafruit_PCA9685.PCA9685()
        self.param_file = param_file
        self.PWM_PARAM = self.read_pwm_param()
        
    def read_pwm_param(self):
        """
        キャリブレーションパラメータをファイルから読み込む
        ファイル構造に依存
        """
        PWM_PARAM = ([0, 0, 0],[0, 0, 0])
        try:
            with open(self.param_file) as f:
                lines = f.readlines()
                # ステアリング [RIGHT, CENTER, LEFT], スロットル [FORWARD, STOPPED, REVERSE]
                PWM_PARAM = (
                    [int(lines[2]), int(lines[4]), int(lines[6])],
                    [int(lines[9]), int(lines[11]), int(lines[13])]
                )
                # 初期セット
                self.pwm.set_pwm(13, 0, PWM_PARAM[1][2])  # スロットルREVERSE
                self.pwm.set_pwm(14, 0, PWM_PARAM[0][2])  # ステアリングLEFT
        except FileNotFoundError:
            print(f"Parameter file {self.param_file} not found. Using defaults.")
        return PWM_PARAM

    def accel(self, Duty):
        """
        スロットル制御
        Duty: -100~100（負は後退、正は前進）
        """
        THROTTLE_FORWARD_PWM = self.PWM_PARAM[1][0]
        THROTTLE_STOPPED_PWM = self.PWM_PARAM[1][1]
        THROTTLE_REVERSE_PWM = self.PWM_PARAM[1][2]

        if Duty > 0:
            throttle_pwm = int(THROTTLE_STOPPED_PWM - (THROTTLE_STOPPED_PWM - THROTTLE_FORWARD_PWM)*Duty/100)
            self.pwm.set_pwm(13, 0, throttle_pwm)
        elif Duty == 0:
            self.pwm.set_pwm(13, 0, THROTTLE_STOPPED_PWM)
            time.sleep(0.01)
        else:  # Duty < 0
            self.pwm.set_pwm(13, 0, THROTTLE_REVERSE_PWM)
            time.sleep(0.01)
            self.pwm.set_pwm(13, 0, THROTTLE_STOPPED_PWM)
            time.sleep(0.01)
            throttle_pwm = int(THROTTLE_STOPPED_PWM + (THROTTLE_REVERSE_PWM - THROTTLE_STOPPED_PWM)*abs(Duty)/100)
            self.pwm.set_pwm(13, 0, throttle_pwm)

    def steer(self, Duty):
        """
        ステアリング制御
        Duty: -100~100（負は左、正は右）
        """
        STEERING_RIGHT_PWM  = self.PWM_PARAM[0][0]
        STEERING_CENTER_PWM = self.PWM_PARAM[0][1]
        STEERING_LEFT_PWM   = self.PWM_PARAM[0][2]

        if Duty >= 0:
            steer_pwm = int(STEERING_CENTER_PWM - (STEERING_CENTER_PWM - STEERING_RIGHT_PWM)*Duty/100)
        else:
            steer_pwm = int(STEERING_CENTER_PWM + (STEERING_LEFT_PWM - STEERING_CENTER_PWM)*abs(Duty)/100)
        self.pwm.set_pwm(14, 0, steer_pwm)

# --- PC上テスト ---
if __name__ == "__main__":
    vc = VehicleController(pwm=Adafruit_PCA9685)  # モック対応
    print("前進 50%")
    vc.accel(50)
    time.sleep(0.5)
    print("左に 30%")
    vc.steer(-30)
    time.sleep(0.5)
    print("停止")
    vc.accel(0)
    print("ステアリングセンター")
    vc.steer(0)
