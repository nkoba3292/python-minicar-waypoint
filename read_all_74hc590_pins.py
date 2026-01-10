#!/usr/bin/env python3
"""
read_all_74hc590_pins.py

Read all accessible pins of 74HC590 to verify connections.
This helps identify which physical GPIO pins correspond to which 74HC590 outputs.
"""

import time
import RPi.GPIO as GPIO

# Current pin assignments (what we think they are)
CURRENT_MAPPING = {
    'D0 (QA-pin15)': 13,  # BCM 13 = BOARD 33
    'D1 (QB-pin1)':  21,  # BCM 21 = BOARD 40
    'D2 (QC-pin2)':  25,  # BCM 25 = BOARD 22
    'CCLR (pin10)':  17,  # BCM 17 = BOARD 11
    'RCLK (pin13)':  24,  # BCM 24 = BOARD 18
}

# All available GPIO pins on your expansion board
# Based on your earlier info: GPIO00,01,04,05,11 pulled out
# Available: BCM 13, 17, 18, 21, 23, 24, 25
ALL_TEST_PINS = [13, 17, 18, 21, 23, 24, 25]

def setup_all_inputs():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in ALL_TEST_PINS:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def read_all_pins():
    """Read all test pins and return dict"""
    values = {}
    for pin in ALL_TEST_PINS:
        values[pin] = GPIO.input(pin)
    return values

def test_sequential_read():
    """Read pins multiple times to see if any change"""
    print("=== Reading All Accessible GPIO Pins ===")
    print("BCM Pin | Value | BOARD Pin | Current Assignment")
    print("--------+-------+-----------+-------------------")
    
    setup_all_inputs()
    
    # Map BCM to BOARD
    bcm_to_board = {
        13: 33,
        17: 11,
        18: 12,
        21: 40,
        23: 16,
        24: 18,
        25: 22,
    }
    
    # Find current assignment for each pin
    pin_assignment = {}
    for name, bcm in CURRENT_MAPPING.items():
        pin_assignment[bcm] = name
    
    # Read once
    values = read_all_pins()
    
    for bcm in sorted(ALL_TEST_PINS):
        val = values[bcm]
        board = bcm_to_board.get(bcm, '?')
        assign = pin_assignment.get(bcm, '-')
        print(f"  {bcm:3d}   |   {val}   |    {board:2}     | {assign}")
    
    print()
    
    # Now read 10 times to see changes
    print("=== Continuous Reading (10 samples, 0.2s interval) ===")
    print("Sample | " + " ".join([f"BCM{p:2d}" for p in sorted(ALL_TEST_PINS)]))
    print("-------+-" + "-".join(["----" for _ in ALL_TEST_PINS]))
    
    for i in range(10):
        values = read_all_pins()
        val_str = " ".join([f"  {values[p]}  " for p in sorted(ALL_TEST_PINS)])
        print(f"  {i+1:2d}   | {val_str}")
        time.sleep(0.2)
    
    print()

def test_with_pattern():
    """Try to identify 74HC590 output pattern"""
    print("=== Analyzing 74HC590 Output Pattern ===")
    print("If D0,D1,D2 are connected correctly, we should see binary counter value")
    print()
    
    setup_all_inputs()
    
    # Assume current mapping is correct
    d0_pin = 13  # BCM 13
    d1_pin = 21  # BCM 21
    d2_pin = 25  # BCM 25
    
    print("Reading D0,D1,D2 pattern (10 samples):")
    print("Sample | D2 D1 D0 | Decimal")
    print("-------+----------+--------")
    
    for i in range(10):
        d0 = GPIO.input(d0_pin)
        d1 = GPIO.input(d1_pin)
        d2 = GPIO.input(d2_pin)
        value = d0 | (d1 << 1) | (d2 << 2)
        print(f"  {i+1:2d}   |  {d2}  {d1}  {d0} |   {value}")
        time.sleep(0.2)
    
    print()
    print("If value is always the same (e.g., 5), possible causes:")
    print("  1. Counter not counting (no clock pulses)")
    print("  2. Output Enable (G, pin 15) not connected to GND")
    print("  3. Register not latching (RCLK issue)")

def main():
    print("=" * 60)
    print("74HC590 Pin Reading Test")
    print("=" * 60)
    print()
    
    test_sequential_read()
    test_with_pattern()
    
    GPIO.cleanup()
    
    print("=" * 60)
    print("INTERPRETATION GUIDE:")
    print("=" * 60)
    print()
    print("1. If all pins show same value (all 0 or all 1):")
    print("   -> Check 74HC590 power supply")
    print()
    print("2. If D0,D1,D2 show constant value (e.g., 101 = 5):")
    print("   -> Pin 15 (G) might not be connected to GND")
    print("   -> Or counter not receiving clock pulses")
    print()
    print("3. If values change over time:")
    print("   -> Good! Counter is working")
    print("   -> Note the pattern to verify correct pin mapping")
    print()
    print("4. If some pins are always 1 or 0 regardless of pull:")
    print("   -> Those pins are driven by 74HC590")
    print("   -> Use this to verify pin mapping")

if __name__ == '__main__':
    main()
