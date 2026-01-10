#!/usr/bin/env python3
"""
gpio_check.py

Check GPIO pin states and help diagnose wiring issues.
Tests basic GPIO read/write functionality.

Usage:
  sudo python3 gpio_check.py [pin_number]
"""

import sys
import time
import RPi.GPIO as GPIO

def check_pin(pin_bcm):
    """Test a single GPIO pin"""
    print(f"\n=== Testing BCM GPIO {pin_bcm} ===")
    
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Test as input with pull-up
        print(f"Setting as INPUT with PULL_UP...")
        GPIO.setup(pin_bcm, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        time.sleep(0.1)
        val = GPIO.input(pin_bcm)
        print(f"  Value: {val} (expect 1)")
        
        # Test as input with pull-down
        print(f"Setting as INPUT with PULL_DOWN...")
        GPIO.setup(pin_bcm, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        time.sleep(0.1)
        val = GPIO.input(pin_bcm)
        print(f"  Value: {val} (expect 0)")
        
        # Test as input without pull (external signal)
        print(f"Reading as INPUT (no pull) - checking external signal...")
        GPIO.setup(pin_bcm, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
        time.sleep(0.1)
        val = GPIO.input(pin_bcm)
        print(f"  Value: {val} (depends on wiring)")
        
        # Sample multiple times
        print(f"Sampling 10 times (0.1s interval)...")
        for i in range(10):
            val = GPIO.input(pin_bcm)
            print(f"  [{i}] {val}", end=' ')
            time.sleep(0.1)
        print()
        
        print(f"BCM GPIO {pin_bcm} test complete")
        
    except Exception as e:
        print(f"ERROR testing BCM GPIO {pin_bcm}: {e}")
    finally:
        GPIO.cleanup()


def main():
    if len(sys.argv) > 1:
        pin = int(sys.argv[1])
        check_pin(pin)
    else:
        # Check common pins used in your setup
        print("Checking common RPR220 sensor pins...")
        print("(Based on your wiring: BCM 17 for wheel sensor)")
        
        pins_to_check = [17, 23, 24, 18]  # wheel, line, latch, reset
        
        for pin in pins_to_check:
            check_pin(pin)
            time.sleep(0.5)
    
    print("\n=== Pin Check Complete ===")
    print("If all pins show reasonable values, wiring is likely OK.")
    print("If a pin is stuck at 0 or 1 regardless of pull setting:")
    print("  - Check for short circuits")
    print("  - Verify sensor power (VCC/GND)")
    print("  - Check sensor output is connected to correct GPIO")
    print("  - Ensure GPIO is not damaged")


if __name__ == '__main__':
    main()
