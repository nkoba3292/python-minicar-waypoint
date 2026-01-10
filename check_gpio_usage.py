#!/usr/bin/env python3
"""
check_gpio_usage.py

Check which processes are using GPIO and which pins are in use.
"""

import os
import sys

def check_gpio_processes():
    """Check for processes using GPIO"""
    print("=== Checking for processes using GPIO ===\n")
    
    # Check for common GPIO-using processes
    commands = [
        ("ps aux | grep python | grep -v grep", "Python processes"),
        ("ps aux | grep gpio | grep -v grep", "GPIO-related processes"),
        ("lsof /dev/gpiomem 2>/dev/null", "Processes using /dev/gpiomem"),
    ]
    
    for cmd, desc in commands:
        print(f"{desc}:")
        os.system(cmd)
        print()

def check_gpio_exports():
    """Check GPIO sysfs exports"""
    print("=== Checking GPIO exports ===\n")
    
    gpio_path = "/sys/class/gpio"
    if os.path.exists(gpio_path):
        exports = os.listdir(gpio_path)
        gpio_pins = [x for x in exports if x.startswith('gpio')]
        
        if gpio_pins:
            print(f"Exported GPIO pins: {gpio_pins}")
            for pin in gpio_pins:
                direction_file = os.path.join(gpio_path, pin, 'direction')
                if os.path.exists(direction_file):
                    with open(direction_file, 'r') as f:
                        direction = f.read().strip()
                    print(f"  {pin}: {direction}")
        else:
            print("No GPIO pins currently exported")
    else:
        print("GPIO sysfs not available")
    print()

def suggest_available_pins():
    """Suggest alternative GPIO pins"""
    print("=== Available GPIO pins (BCM numbering) ===\n")
    print("Based on your expansion board (GPIO00,01,04,05,11 pulled out):")
    print("  Available: BCM 13, 17, 18, 21, 23, 24, 25")
    print("  Avoid: BCM 0, 1 (I2C ID), 2, 3 (I2C)")
    print("\nFor wheel sensor, try BCM 13 if BCM 17 is in use")
    print()

if __name__ == '__main__':
    check_gpio_processes()
    check_gpio_exports()
    suggest_available_pins()
    
    print("=== Recommended actions ===")
    print("1. Kill any conflicting processes: sudo killall python3")
    print("2. Or try a different pin: sudo python3 wheel_count_poll.py --pin 13")
    print("3. Check your wiring matches the pin number")
