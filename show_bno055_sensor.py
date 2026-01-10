import serial
import struct
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)

def read_reg(addr, length=1):
    ser.write(bytes([0xAA, 0x01, addr, length]))
    resp = ser.read(2 + length)
    if len(resp) != 2 + length or resp[0] != 0xBB:
        print(f"[ERROR] reg 0x{addr:02X}: {list(resp)}")
        return None
    return resp[2:]

def print_calib_stat(byte_val):
    # CALIB_STAT: [SYS<<6 | GYR<<4 | ACC<<2 | MAG]
    sys = (byte_val >> 6) & 0x03
    gyro = (byte_val >> 4) & 0x03
    acc = (byte_val >> 2) & 0x03
    mag = byte_val & 0x03
    print(f"CALIB_STAT: SYS={sys} GYR={gyro} ACC={acc} MAG={mag}")

def print_opr_mode(step):
    for i in range(3):
        opr_mode = read_reg(0x3D)
        print(f"{step} OPR_MODE({i}):", list(opr_mode) if opr_mode else opr_mode)
        time.sleep(0.5)

# STEP1: ソフトリセット
ser.write(bytes([0xAA, 0x00, 0x3F, 0x20]))
time.sleep(2.0)  # 2秒待機
ser.reset_input_buffer()
print_opr_mode("STEP1")

# STEP2: CONFIGモード
ser.write(bytes([0xAA, 0x00, 0x3D, 0x00]))
time.sleep(1.5)  # 1.5秒待機
ser.read(2)
print_opr_mode("STEP2")

# STEP3: NDOFモード
ser.write(bytes([0xAA, 0x00, 0x3D, 0x0C]))
time.sleep(3.0)  # 3秒待機
ser.read(2)
ser.reset_input_buffer()
print_opr_mode("STEP3")

# キャリブレーション状態確認
calib_stat = read_reg(0x35)
if calib_stat:
    print_calib_stat(calib_stat[0])
else:
    print("CALIB_STAT: None")

def read_reg(addr, length=1):
    ser.write(bytes([0xAA, 0x01, addr, length]))
    resp = ser.read(2 + length)
    if len(resp) != 2 + length or resp[0] != 0xBB:
        print(f"[ERROR] reg 0x{addr:02X}: {list(resp)}")
        return None
    return resp[2:]

# 各種レジスタ確認
chip_id = read_reg(0x00)
print("CHIP_ID:", list(chip_id) if chip_id else chip_id)
sys_status = read_reg(0x39)
print("SYS_STATUS:", list(sys_status) if sys_status else sys_status)
sys_err = read_reg(0x3A)
print("SYS_ERR:", list(sys_err) if sys_err else sys_err)
calib_stat = read_reg(0x35)
print("CALIB_STAT:", list(calib_stat) if calib_stat else calib_stat)

def read_vector(addr, length):
    ser.write(bytes([0xAA, 0x01, addr, length]))
    ser.read(2 + length)  # 1回目の応答は捨てる
    resp = ser.read(2 + length)
    if len(resp) != 2 + length or resp[0] != 0xBB:
        print(f"[ERROR] Unexpected response: {list(resp)}")
        return None
    return resp[2:]

for i in range(10):
    acc_raw = read_vector(0x08, 6)
    gyr_raw = read_vector(0x14, 6)
    mag_raw = read_vector(0x0E, 6)
    if acc_raw:
        acc = struct.unpack('<hhh', acc_raw)
        print(f"ACC: {acc}", end='  ')
    if gyr_raw:
        gyr = struct.unpack('<hhh', gyr_raw)
        print(f"GYR: {gyr}", end='  ')
    if mag_raw:
        mag = struct.unpack('<hhh', mag_raw)
        print(f"MAG: {mag}")
    time.sleep(0.5)

ser.close()
