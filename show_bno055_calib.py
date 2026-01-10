import time
import serial

PORT = '/dev/serial0'
BAUD = 115200
CALIB_ADDR = 0x35

def read_vector(ser, addr, length):
    cmd = bytes([0xAA, 0x01, addr, length])
    ser.write(cmd)
    time.sleep(0.01)
    # 1回目の応答は捨てる
    ser.read(2 + length)
    # 2回目の応答を使う
    resp = ser.read(2 + length)
    if len(resp) == 0 or resp[0] != 0xBB:
        print(f"[ERROR] Unexpected response: {list(resp)}")
        return None
    return resp[2:]

def main():
    ser = serial.Serial(
        PORT,
        BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )
    print(f"[INFO] Serial opened: {ser}")
    # NDOFモードへ（必要なら追加）
    ser.write(bytes([0xAA, 0x00, 0x3D, 0x0C]))
    time.sleep(2.0)  # 初期化待機を2秒に延長
    ser.reset_input_buffer()
    while True:
        calib_raw = read_vector(ser, CALIB_ADDR, 1)
        if calib_raw:
            calib_byte = calib_raw[0]
            calib = {
                'system':   (calib_byte >> 6) & 0x03,
                'gyroscope':(calib_byte >> 4) & 0x03,
                'accelerometer':(calib_byte >> 2) & 0x03,
                'magnetometer': calib_byte & 0x03
            }
            print(f"CALIBRATION: raw=0x{calib_byte:02X} sys={calib['system']} gyro={calib['gyroscope']} acc={calib['accelerometer']} mag={calib['magnetometer']}")
        else:
            print("CALIBRATION: read failed")
        time.sleep(1)

if __name__ == '__main__':
    main()