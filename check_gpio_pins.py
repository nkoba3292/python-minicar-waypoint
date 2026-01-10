#!/usr/bin/env python3
"""
GPIO ピン番号対応表
"""

print("="*70)
print("GPIO Pin Mapping - BOARD vs BCM")
print("="*70)

pins = [
    (15, 22, "TRIG Front"),
    (26, 7, "ECHO Front"),
    (13, 27, "TRIG Left45"),
    (24, 8, "ECHO Left45"),
    (35, 19, "TRIG Left90"),
    (37, 26, "ECHO Left90"),
    (32, 12, "TRIG Right45"),
    (31, 6, "ECHO Right45"),
    (36, 16, "TRIG Right90"),
    (38, 20, "ECHO Right90"),
]

print(f"{'Function':<15} {'BOARD':>6} {'BCM':>6} {'Physical Pin':>15}")
print("-"*70)
for board, bcm, func in pins:
    print(f"{func:<15} {board:>6} {bcm:>6} {'Pin ' + str(board):>15}")

print("="*70)
print("\nCurrent code uses: GPIO.setmode(GPIO.BOARD)")
print("Verify your wiring matches BOARD numbers above!")
print("="*70)
