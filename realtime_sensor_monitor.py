import time
def fmt(val, digits=0):
    if val is None:
        return "---"
    if digits == 0:
        return str(int(val))
    return f"{val:.{digits}f}"
from IMU_sensor_bno055 import IMUSensorBNO055
try:
    from ultrasonic_array_thread import UltrasonicSensors
    ULTRASONIC_AVAILABLE = True
except ImportError:
    ULTRASONIC_AVAILABLE = False

imu = IMUSensorBNO055(port='/dev/serial0', baudrate=115200, offset=0.0)
ultrasonic = UltrasonicSensors() if ULTRASONIC_AVAILABLE else None

print("=== Realtime Sensor Monitor ===")
print("Press Ctrl+C to stop.")

while True:
    imu_data = imu.get_all()
    print(f"IMU | YAW: {fmt(imu_data.get('yaw'), 0)} | ACC: {imu_data.get('accel', '---')}")
    if ultrasonic:
        us = ultrasonic.get_distances()
        print(f"US  | F: {fmt(us[0], 0)} | L45: {fmt(us[1], 0)} | L90: {fmt(us[2], 0)} | R45: {fmt(us[3], 0)} | R90: {fmt(us[4], 0)}")
    time.sleep(0.05)
