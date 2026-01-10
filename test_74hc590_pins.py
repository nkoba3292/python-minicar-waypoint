#!/usr/bin/env python3
"""
test_74hc590_pins.py

Test individual pins for 74HC590 connection debugging.
"""

import time
import RPi.GPIO as GPIO

# Pin assignments
D0_PIN = 13   # BOARD 33
D1_PIN = 21   # BOARD 40
D2_PIN = 25   # BOARD 22
CCLR_PIN = 17   # BOARD 11
LATCH_PIN = 24  # BOARD 18

def test_data_pins():
    """Read data pins and show binary value"""
    print("\n=== Testing Data Pins ===")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    data_pins = [D0_PIN, D1_PIN, D2_PIN]
    for pin in data_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    
    print("Reading data pins 10 times (0.5s interval):")
    print("D2 D1 D0 | Value")
    print("---------+------")
    
    for i in range(10):
        d0 = GPIO.input(D0_PIN)
        d1 = GPIO.input(D1_PIN)
        d2 = GPIO.input(D2_PIN)
        value = d0 | (d1 << 1) | (d2 << 2)
        print(f" {d2}  {d1}  {d0} | {value}")
        time.sleep(0.5)
    
    GPIO.cleanup()

def test_clear_pin():
    """Test CCLR pin - toggle it and check if counter resets"""
    print("\n=== Testing CCLR Pin ===")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Setup pins
    data_pins = [D0_PIN, D1_PIN, D2_PIN]
    for pin in data_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    
    GPIO.setup(CCLR_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LATCH_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    def read_counter():
        GPIO.output(LATCH_PIN, GPIO.LOW)
        time.sleep(0.00002)
        GPIO.output(LATCH_PIN, GPIO.HIGH)
        time.sleep(0.00002)
        GPIO.output(LATCH_PIN, GPIO.LOW)
        time.sleep(0.00001)
        
        d0 = GPIO.input(D0_PIN)
        d1 = GPIO.input(D1_PIN)
        d2 = GPIO.input(D2_PIN)
        return d0 | (d1 << 1) | (d2 << 2)
    
    print("Initial counter value:")
    val = read_counter()
    print(f"  Value: {val}")
    
    print("\nTrying CCLR = LOW (active low clear):")
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.0001)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.0001)
    val = read_counter()
    print(f"  After LOW pulse: {val} (should be 0 if active LOW)")
    
    time.sleep(0.5)
    
    print("\nTrying CCLR = HIGH pulse (active high clear):")
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.0001)
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.0001)
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.0001)
    val = read_counter()
    print(f"  After HIGH pulse: {val} (should be 0 if active HIGH)")
    
    GPIO.cleanup()

def test_latch_pin():
    """Test LATCH/STRO pin"""
    print("\n=== Testing LATCH Pin ===")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    data_pins = [D0_PIN, D1_PIN, D2_PIN]
    for pin in data_pins:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    
    GPIO.setup(LATCH_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    print("Testing latch pulse effects:")
    for i in range(5):
        # Pulse latch
        GPIO.output(LATCH_PIN, GPIO.LOW)
        time.sleep(0.00002)
        GPIO.output(LATCH_PIN, GPIO.HIGH)
        time.sleep(0.00002)
        GPIO.output(LATCH_PIN, GPIO.LOW)
        time.sleep(0.00001)
        
        d0 = GPIO.input(D0_PIN)
        d1 = GPIO.input(D1_PIN)
        d2 = GPIO.input(D2_PIN)
        value = d0 | (d1 << 1) | (d2 << 2)
        print(f"  Read {i+1}: {value}")
        time.sleep(0.2)
    
    GPIO.cleanup()

def main():
    print("=== 74HC590 Pin Test ===")
    print(f"CCLR:  BCM {CCLR_PIN} (BOARD 11)")
    print(f"LATCH: BCM {LATCH_PIN} (BOARD 18)")
    print(f"D0:    BCM {D0_PIN} (BOARD 33)")
    print(f"D1:    BCM {D1_PIN} (BOARD 40)")
    print(f"D2:    BCM {D2_PIN} (BOARD 22)")
    
    print("\n--- Test 1: Data Pins ---")
    test_data_pins()
    
    print("\n--- Test 2: Clear Pin ---")
    test_clear_pin()
    
    print("\n--- Test 3: Latch Pin ---")
    test_latch_pin()
    
    print("\n=== Tests Complete ===")
    print("\nDiagnostics:")
    print("1. If data pins always show '5' (101):")
    print("   - Check D0, D1, D2 wiring to 74HC590 Q outputs")
    print("   - Verify 74HC590 has power (VCC/GND)")
    print("   - Check if outputs are tri-stated (OE pin)")
    print("\n2. If CCLR test doesn't clear counter:")
    print("   - Check CCLR wiring to pin 11 (BCM 17)")
    print("   - Try opposite polarity in main script")
    print("   - Verify CCLR is not tied high/low on board")
    print("\n3. If latch doesn't change values:")
    print("   - Check LATCH wiring to pin 18 (BCM 24)")
    print("   - Counter may not be counting (no input pulses)")

if __name__ == '__main__':
    main()
