# IMUセンサ値を一列で表示するサンプル
from IMU_sensor_bno055 import IMUSensorBNO055
import time

imu = IMUSensorBNO055()
while True:
    data = imu.get_all()
    # 1行で主要情報を表示
    print(f"Yaw:{data.get('yaw'):.2f} Roll:{data.get('roll'):.2f} Pitch:{data.get('pitch'):.2f} "
          f"Accel:{data.get('accel')} Gyro:{data.get('gyro')} Mag:{data.get('mag')}")
    time.sleep(0.5)
