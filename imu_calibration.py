# imu_calibration.py
import math
import json
import time

class IMUCalibration:
    def __init__(self, imu, odom, save_file="imu_calib.json"):
        """
        imu  : IMUドライバ (get_yaw() が角度[rad]を返す想定)
        odom : オドメトリ (get_position() が (x,y) を返す想定)
        """
        self.imu = imu
        self.odom = odom
        self.save_file = save_file
        self.offset = 0.0

    def calibrate_heading(self, distance=1.0, speed=0.2):
        """
        起動直後に直進してIMU方位をキャリブレーションする
        distance : 直進距離 [m]
        speed    : 走行速度 [m/s] の想定
        """
        print("=== IMU Calibration Start ===")

        # 初期位置と初期IMU角度を記録
        x0, y0 = self.odom.get_position()
        imu_start = self.imu.get_yaw()
        print(f"Start position: ({x0:.3f}, {y0:.3f}), IMU start: {math.degrees(imu_start):.2f}°")

        # 直進コマンド発行（ここはロボットの制御APIに置き換えてください）
        self._drive_forward(speed)

        # 指定距離を走るまで待機
        while True:
            x, y = self.odom.get_position()
            if math.hypot(x - x0, y - y0) >= distance:
                break
            time.sleep(0.1)

        # 停止コマンド発行（ここはロボットの制御APIに置き換えてください）
        self._stop()

        # 最終IMU角度を取得
        imu_end = self.imu.get_yaw()
        print(f"End position: ({x:.3f}, {y:.3f}), IMU end: {math.degrees(imu_end):.2f}°")

        # オフセット計算
        self.offset = imu_end - imu_start
        print(f"Calibration offset: {math.degrees(self.offset):.2f}°")

        # 補正後のIMU角度を確認
        calibrated_yaw = self.get_calibrated_yaw()
        print(f"Calibrated IMU yaw: {math.degrees(calibrated_yaw):.2f}°")

        # キャリブレーション結果をファイルに保存
        self.save_calibration()

        print("=== IMU Calibration End ===")

    def save_calibration(self):
        """
        キャリブレーション結果をファイルに保存
        """
        with open(self.save_file, "w") as f:
            json.dump({"offset": self.offset}, f)
        print(f"Calibration data saved to {self.save_file}")

    def load_calibration(self):
        """
        キャリブレーション結果をファイルから読み込み
        """
        try:
            with open(self.save_file, "r") as f:
                data = json.load(f)
                self.offset = data.get("offset", 0.0)
            print(f"Calibration data loaded from {self.save_file}")
        except Exception as e:
            print(f"Failed to load calibration data: {e}")

    def get_calibrated_yaw(self):
        """
        キャリブレーション後の補正済みyaw値を返す
        """
        raw_yaw = self.imu.get_yaw()
        calibrated_yaw = raw_yaw + self.offset
        return calibrated_yaw

    def _drive_forward(self, speed):
        """
        直進コマンドを発行する（ダミー実装）
        """
        print(f"Driving forward at speed: {speed} m/s")

    def _stop(self):
        """
        停止コマンドを発行する（ダミー実装）
        """
        print("Stopping")

    def calibrate_2points(self, point_names=["point1", "point2"]):
        """
        ２か所に静置してIMUと位置を計測し、補正値を算出
        """
        calib_data = {}
        for name in point_names:
            print(f"Place robot at {name} and press Enter...")
            input()
            x, y = self.odom.get_position()
            yaw = self.imu.get_yaw()
            calib_data[name] = {"x": x, "y": y, "yaw": yaw}
            print(f"{name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f}°")

        # ２点間の座標ベクトルとIMU角度差から補正値算出
        p1 = calib_data[point_names[0]]
        p2 = calib_data[point_names[1]]
        dx = p2["x"] - p1["x"]
        dy = p2["y"] - p1["y"]
        course_angle = math.atan2(dy, dx)
        imu_angle = p2["yaw"] - p1["yaw"]
        offset = course_angle - imu_angle
        self.offset = offset
        print(f"Calibration offset: {math.degrees(offset):.2f}°")

        # 保存
        with open(self.save_file, "w") as f:
            json.dump({"offset": self.offset, "points": calib_data}, f, indent=2)
        print(f"Calibration data saved to {self.save_file}")

# 使用例
# imu, odomは既存のインスタンスを使う
# imu_calib = IMUCalibration(imu, odom)
# imu_calib.calibrate_2points(["start", "goal"])  # 例：スタートとゴールでキャリブレーション
