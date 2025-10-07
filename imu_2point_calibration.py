import json
import math
import time

class IMU2PointCalibration:
    def __init__(self, imu, odom, save_file="imu_2point_calib.json"):
        self.imu = imu
        self.odom = odom
        self.save_file = save_file
        self.calib_data = {}

    def calibrate_at_point(self, point_name):
        print(f"Place robot at {point_name} and press Enter...")
        input()
        x, y = self.odom.get_position()
        yaw = self.imu.get_yaw()
        self.calib_data[point_name] = {"x": x, "y": y, "yaw": yaw}
        print(f"{point_name}: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f}°")

    def compute_offset(self):
        # 2点間の座標ベクトルとIMU角度差から補正値算出
        p1 = self.calib_data["point1"]
        p2 = self.calib_data["point2"]
        dx = p2["x"] - p1["x"]
        dy = p2["y"] - p1["y"]
        course_angle = math.atan2(dy, dx)
        imu_angle = p2["yaw"] - p1["yaw"]
        offset = course_angle - imu_angle
        self.calib_data["offset"] = offset
        print(f"Calibration offset: {math.degrees(offset):.2f}°")
        return offset

    def save(self):
        with open(self.save_file, "w") as f:
            json.dump(self.calib_data, f, indent=2)
        print(f"Calibration data saved to {self.save_file}")

# --- 使用例 ---
if __name__ == "__main__":
    print("IMU 2-Point Calibration Module")
    print("Usage example:")
    print("""
    from imu_2point_calibration import IMU2PointCalibration
    from your_imu_driver import IMU
    from your_odom_system import Odometry
    
    imu = IMU()  # Your IMU instance
    odom = Odometry()  # Your odometry instance
    
    imu_calib = IMU2PointCalibration(imu, odom)
    imu_calib.calibrate_at_point("point1")
    imu_calib.calibrate_at_point("point2") 
    imu_calib.compute_offset()
    imu_calib.save()
    """)

# Example with actual instances (uncomment when drivers are available):
# calibrator = IMU2PointCalibration(my_imu, my_odom)
# calibrator.calibrate_at_point("start")
# calibrator.calibrate_at_point("goal") 
# offset = calibrator.compute_offset()
# calibrator.save()