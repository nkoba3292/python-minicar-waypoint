import serial
import time

ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)
time.sleep(1)
ser.write(bytes([0xAA, 0x01, 0x00, 0x01]))
resp = ser.read(3)
print("chipID;160correct ; " ,list(resp))

time.sleep(1)
ser.write(bytes([0xAA, 0x01, 0x3D, 0x01]))
resp = ser.read(3)
print("OPR_mode 0config,12NDOF,1-11sensormode,16unexpected",list(resp))

time.sleep(1)
ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x00]))
time.sleep(0.05)
resp = ser.read(2)
print(list(resp))


# NDOFモードへ切替
ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x0C]))
time.sleep(1.0)
ser.read(2)
print("NDOFモード切替完了")

def parse_calib_stat(byte_val):
	sys = (byte_val >> 6) & 0x03
	gyro = (byte_val >> 4) & 0x03
	acc = (byte_val >> 2) & 0x03
	mag = byte_val & 0x03
	return f"SYS={sys} GYR={gyro} ACC={acc} MAG={mag}"

import struct
save_done = False
for i in range(20):
	# キャリブレーション値取得
	ser.write(bytes([0xAA, 0x01, 0x35, 0x01]))
	resp = ser.read(3)
	if len(resp) == 3 and resp[0] == 0xBB:
		calib_raw = resp[2]
		sys = (calib_raw >> 6) & 0x03
		gyro = (calib_raw >> 4) & 0x03
		accv = (calib_raw >> 2) & 0x03
		mag = calib_raw & 0x03
		calib = parse_calib_stat(calib_raw)
		calib_disp = f"CALIB: {calib} (raw=0x{calib_raw:02X}, bin={calib_raw:08b})"
		# すべて3なら保存選択
		if not save_done and sys == 3 and gyro == 3 and accv == 3 and mag == 3:
			ans = input("キャリブレーション値を保存しますか？ (y/n): ")
			print(f"入力値: '{ans}'")
			if ans.lower() == 'y':
				ser.write(bytes([0xAA, 0x01, 0x55, 0x16]))
				calib_data = ser.read(24)
				if len(calib_data) == 24 and calib_data[0] == 0xBB:
					with open("bno055_calibdata.bin", "wb") as f:
						f.write(calib_data[2:24])
					print("キャリブレーションデータを保存しました")
					save_done = True
				else:
					print("キャリブレーションデータ取得失敗")
			else:
				print("キャリブレーションデータは保存しませんでした")
				save_done = True
	else:
		calib_disp = "CALIB: ERROR"

	# 加速度
	ser.write(bytes([0xAA, 0x01, 0x08, 0x06]))
	acc_raw = ser.read(8)
	acc = "ERR"
	if len(acc_raw) == 8 and acc_raw[0] == 0xBB:
		acc = struct.unpack('<hhh', acc_raw[2:8])

	# ジャイロ
	ser.write(bytes([0xAA, 0x01, 0x14, 0x06]))
	gyr_raw = ser.read(8)
	gyr = "ERR"
	if len(gyr_raw) == 8 and gyr_raw[0] == 0xBB:
		gyr = struct.unpack('<hhh', gyr_raw[2:8])

	# 地磁気
	ser.write(bytes([0xAA, 0x01, 0x0E, 0x06]))
	mag_raw = ser.read(8)
	magv = "ERR"
	if len(mag_raw) == 8 and mag_raw[0] == 0xBB:
		magv = struct.unpack('<hhh', mag_raw[2:8])

	print(f"{calib_disp}  ACC: {acc}  GYR: {gyr}  MAG: {magv}")
	time.sleep(0.5)

#ser.write(bytes([0xAA, 0x00, 0x3F, 0x01, 0x20]))
#time.sleep(1)

#ser.write(bytes([0xAA, 0x00, 0x3D, 0x01, 0x00]))
#resp = ser.read(2)
#time.sleep(0.05)
#print(resp)




# キャリブレーションデータ表示
try:
	with open("bno055_calibdata.bin", "rb") as f:
		data = f.read()
	print("[キャリブデータ] バイト数:", len(data))
	print("[キャリブデータ] 16進:", data.hex())
	print("[キャリブデータ] 数値リスト:", list(data))
except Exception as e:
	print("キャリブレーションデータ表示失敗:", e)

ser.close()