import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

print("DEBUG: I2C bus init")
i2c = busio.I2C(SCL, SDA)

print("DEBUG: PCA9685 init")
pca = PCA9685(i2c)
pca.frequency = 50

print("DEBUG: Set PWM channel 0 to 0")
pca.channels[0].duty_cycle = 0x0000
time.sleep(1)

print("DEBUG: Set PWM channel 0 to max")
pca.channels[0].duty_cycle = 0xFFFF
time.sleep(1)

print("DEBUG: Done")
pca.deinit()