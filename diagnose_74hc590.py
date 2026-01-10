#!/usr/bin/env python3
"""
diagnose_74hc590.py

Detailed diagnostic for 74HC590 wiring issues.
Tests all control signals and provides step-by-step guidance.
"""

import time
import RPi.GPIO as GPIO

D0_PIN = 13   # BOARD 33
D1_PIN = 21   # BOARD 40
D2_PIN = 25   # BOARD 22
CCLR_PIN = 17   # BOARD 11
LATCH_PIN = 24  # BOARD 18

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(D0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(D1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(D2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(CCLR_PIN, GPIO.OUT)
    GPIO.setup(LATCH_PIN, GPIO.OUT)

def read_data():
    d0 = GPIO.input(D0_PIN)
    d1 = GPIO.input(D1_PIN)
    d2 = GPIO.input(D2_PIN)
    return d0 | (d1 << 1) | (d2 << 2)

def test_static_wiring():
    print("=== Step 1: Static Output Test ===")
    print("Testing if outputs are stuck or floating...")
    
    # Test with pull-up
    GPIO.setup(D0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(D1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(D2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    time.sleep(0.1)
    
    d0_up = GPIO.input(D0_PIN)
    d1_up = GPIO.input(D1_PIN)
    d2_up = GPIO.input(D2_PIN)
    val_up = d0_up | (d1_up << 1) | (d2_up << 2)
    
    # Test with pull-down
    GPIO.setup(D0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(D1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(D2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    time.sleep(0.1)
    
    d0_dn = GPIO.input(D0_PIN)
    d1_dn = GPIO.input(D1_PIN)
    d2_dn = GPIO.input(D2_PIN)
    val_dn = d0_dn | (d1_dn << 1) | (d2_dn << 2)
    
    # Back to no pull
    GPIO.setup(D0_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(D1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.setup(D2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    time.sleep(0.1)
    
    d0_np = GPIO.input(D0_PIN)
    d1_np = GPIO.input(D1_PIN)
    d2_np = GPIO.input(D2_PIN)
    val_np = d0_np | (d1_np << 1) | (d2_np << 2)
    
    print(f"  With PULL_UP:   D2={d2_up} D1={d1_up} D0={d0_up} -> Value={val_up}")
    print(f"  With PULL_DOWN: D2={d2_dn} D1={d1_dn} D0={d0_dn} -> Value={val_dn}")
    print(f"  No pull:        D2={d2_np} D1={d1_np} D0={d0_np} -> Value={val_np}")
    
    if val_up == val_dn == val_np:
        print("  ✓ Outputs are driven (not floating)")
        print(f"  ✗ BUT: Value is stuck at {val_np}")
        return "stuck"
    else:
        print("  ✗ Outputs are floating or weak-driven")
        return "floating"

def test_control_signals():
    print("\n=== Step 2: Control Signal Test ===")
    
    # Test CCLR both states
    print("Testing CCLR signal...")
    GPIO.output(CCLR_PIN, GPIO.LOW)
    time.sleep(0.01)
    val_cclr_low = read_data()
    
    GPIO.output(CCLR_PIN, GPIO.HIGH)
    time.sleep(0.01)
    val_cclr_high = read_data()
    
    print(f"  CCLR=LOW:  {val_cclr_low}")
    print(f"  CCLR=HIGH: {val_cclr_high}")
    
    if val_cclr_low != val_cclr_high:
        print("  ✓ CCLR signal affects outputs")
    else:
        print("  ✗ CCLR has no effect (check wiring or pin function)")
    
    # Test LATCH both states
    print("\nTesting LATCH signal...")
    GPIO.output(LATCH_PIN, GPIO.LOW)
    time.sleep(0.01)
    val_latch_low = read_data()
    
    GPIO.output(LATCH_PIN, GPIO.HIGH)
    time.sleep(0.01)
    val_latch_high = read_data()
    
    print(f"  LATCH=LOW:  {val_latch_low}")
    print(f"  LATCH=HIGH: {val_latch_high}")
    
    if val_latch_low != val_latch_high:
        print("  ✓ LATCH signal affects outputs")
    else:
        print("  ✗ LATCH has no effect (check wiring)")

def test_74hc590_specific():
    print("\n=== Step 3: 74HC590 Specific Test ===")
    print("74HC590 (TOSHIBA) pinout (16-pin DIP):")
    print("  - Pin 1: QB")
    print("  - Pin 2: QC")
    print("  - Pin 3: QD")
    print("  - Pin 4: QE")
    print("  - Pin 5: QF")
    print("  - Pin 6: QG")
    print("  - Pin 7: QH")
    print("  - Pin 8: GND")
    print("  - Pin 9: QH' (Serial Out)")
    print("  - Pin 10: CCLR (Counter Clear) - active LOW")
    print("  - Pin 11: CCLK (Counter Clock) - receives pulses")
    print("  - Pin 12: (not used)")
    print("  - Pin 13: RCLK (Register Clock/Latch)")
    print("  - Pin 14: G (Output Enable) - active LOW ★重要★")
    print("  - Pin 15: QA")
    print("  - Pin 16: VCC")
    print()
    print("Common issues:")
    print("  1. G (Output Enable, pin 14) is HIGH -> outputs tri-stated")
    print("     Solution: Connect pin 14 to GND")
    print()
    print("  2. No clock pulses to CCLK (pin 11)")
    print("     Solution: Check RPR220 output connection")
    print()
    print("  3. RCLK (pin 13) not pulsed -> outputs not updated")
    print("     Solution: Verify LATCH signal from BCM 24")
    print()
    print("  4. CCLR (pin 10) stuck LOW -> counter always cleared")
    print("     Solution: Verify BCM 17 is normally HIGH")
    print()
    
    print("RECOMMENDATION:")
    print("  Check your 74HC590 circuit:")
    print("  - Pin 14 (G/OE) MUST be connected to GND ★")
    print("  - Pin 10 (CCLR) connected to BCM 17 (BOARD 11)")
    print("  - Pin 13 (RCLK) connected to BCM 24 (BOARD 18)")
    print("  - Pin 11 (CCLK) should receive pulses from RPR220")
    print("  - Pin 15 (QA/D0) to BCM 13 (BOARD 33)")
    print("  - Pin 1 (QB/D1) to BCM 21 (BOARD 40)")
    print("  - Pin 2 (QC/D2) to BCM 25 (BOARD 22)")

def main():
    print("=== 74HC590 Comprehensive Diagnostic ===\n")
    
    setup()
    
    result = test_static_wiring()
    test_control_signals()
    test_74hc590_specific()
    
    print("\n" + "="*50)
    print("DIAGNOSIS SUMMARY:")
    print("="*50)
    
    if result == "stuck":
        print("✗ Outputs are stuck at value 5 (binary 101)")
        print()
        print("Most likely causes (TOSHIBA 74HC590):")
        print("  1. Output Enable (G, pin 15) is HIGH")
        print("     -> Outputs are disabled/tri-stated")
        print("     -> CHECK: Is pin 15 connected to GND?")
        print()
        print("  2. Register Clock (RCLK, pin 13) not working")
        print("     -> Counter value not latched to outputs")
        print("     -> CHECK: Is BCM 24 (BOARD 18) connected to pin 13?")
        print()
        print("  3. Counter not receiving clock pulses")
        print("     -> Value 5 is the power-on state")
        print("     -> CHECK: Is RPR220 connected to pin 11 (CCLK)?")
        print()
        print("  4. Counter Clear (CCLR, pin 10) stuck LOW")
        print("     -> Counter is always cleared")
        print("     -> CHECK: Is BCM 17 normally HIGH?")
        print()
        print("NEXT STEPS:")
        print("  1. Measure voltage at 74HC590 pin 14 (G)")
        print("     Should be: 0V (GND) for outputs to be enabled")
        print("  2. Check pin 13 (RCLK) receives latch pulses from BCM 24")
        print("  3. Check RPR220 output for pulses at pin 11 (CCLK)")
        print("  4. Verify pin 10 (CCLR) is normally at VCC voltage")
    
    GPIO.cleanup()

if __name__ == '__main__':
    main()
