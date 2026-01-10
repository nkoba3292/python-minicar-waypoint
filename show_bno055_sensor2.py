import time
import serial

def init_bno055(ser, saved_offsets=None):
    """
    BNO055を初期化してNDOFモードに設定する。
    saved_offsets があればキャリブレーション値を復元する。
    """

    # --- ソフトリセット ---
    ser.write(bytes([0xAA, 0x00, 0x3F, 0x01, 0x20]))  # SYS_TRIGGER=0x20
    time.sleep(0.7)  # 650ms以上待機

    # --- CONFIGモードに切替 ---
    ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x00]))
    time.sleep(0.05)

    # --- キャリブレーションオフセット復元 ---
    if saved_offsets is not None and len(saved_offsets) == 22:
        ser.write(bytes([0xAA, 0x00, 0x55, 0x16]) + saved_offsets)
        resp = ser.read(2)
        # resp が [0xEE,0x01] でなければ成功
        time.sleep(0.05)

    # --- NDOFモードに切替 ---
    ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x0C]))
    time.sleep(0.05)

    # --- 確認: OPR_MODEを読み出し ---
    ser.write(bytes([0xAA, 0x01, 0x3D, 0x01]))
    resp = ser.read(3)
    if resp[0] == 0xBB and resp[1] == 0x01:
        print(f"OPR_MODE = {hex(resp[2])}")
    else:
        print("モード確認失敗:", resp)
