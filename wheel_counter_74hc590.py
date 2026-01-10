#!/usr/bin/env python3
"""
wheel_counter_74hc590.py

Read 8-bit counter (74HC590) connected to Raspberry Pi GPIO.
Designed for wheel speed sensor pulse counting.

Hardware connections (BOARD physical pin numbers):
  D0 (LSB) -> Pin 33 (BCM 13)
  D1       -> Pin 40 (BCM 21)
  D2       -> Pin 22 (BCM 25)
  D3       -> Not connected (読み取りは3ビットのみ)
  D4-D7    -> Not connected
  CCLR     -> Pin 11 (BCM 17) - Counter Clear
  LATCHSTRO-> Pin 18 (BCM 24) - Latch Strobe

Usage:
  sudo python3 wheel_counter_74hc590.py --interval 0.5 --ppr 20
"""

import time
import csv
import argparse
import RPi.GPIO as GPIO

# Pin mapping (BOARD -> BCM)
PIN_MAPPING = {
    33: 13,  # D0 (LSB)
    40: 21,  # D1
    22: 25,  # D2
    11: 17,  # CCLR (Counter Clear)
    18: 24,  # LATCHSTRO (Latch Strobe)
}

# BCM pin assignments (sensor_placement_testと同じ)
D0_PIN = 13   # BOARD 33
D1_PIN = 21   # BOARD 40
D2_PIN = 25   # BOARD 22
CCLR_PIN = 17   # BOARD 11
LATCH_PIN = 24  # BOARD 18

DATA_PINS = [D0_PIN, D1_PIN, D2_PIN]  # LSB first, only 3 bits connected


class Counter74HC590:
    def __init__(self, latch_pin, data_pins, clear_pin, debounce_time=0.03, min_pulse_interval=0.02):
        """
        Initialize 74HC590 counter reader
        
        Args:
            latch_pin: BCM pin for LATCH/STRO (rising edge loads outputs)
            data_pins: List of BCM pins for data bits [D0, D1, D2, ...] LSB first
            clear_pin: BCM pin for CCLR (active LOW clears counter)
            debounce_time: Minimum time (seconds) between valid pulses (default 0.05s = 50ms)
        """
        self.latch_pin = latch_pin
        self.data_pins = data_pins
        self.clear_pin = clear_pin
        self.last_value = 0
        self.total_count = 0
        self.start_time = time.time()  # 初期化時刻を記録
        self.debounce_time = debounce_time  # デバウンス時間
        self.last_pulse_time = 0.0  # 最後の有効パルス時刻
        
        # Setup GPIO - setmode()はメインプログラムで実行済みと想定
        # sensor_placement_testと同じ設定
        GPIO.setwarnings(False)
        
        # Latch: output, idle LOW
        GPIO.setup(self.latch_pin, GPIO.OUT, initial=GPIO.LOW)
        
        # Clear: output, idle HIGH (inactive, clear is active LOW)
        GPIO.setup(self.clear_pin, GPIO.OUT, initial=GPIO.HIGH)
        
        # Data pins: inputs with pull-down (sensor_placement_testと同じ)
        for pin in self.data_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        # Always clear counter on initialization
        self.clear_counter()
        time.sleep(0.1)
        
        # Read initial value
        self.last_value = self.latch_and_read()
        
        print(f"[74HC590] Initialized: latch={latch_pin}, data={data_pins}, clear={clear_pin}")
        print(f"[74HC590] Initial counter value after clear: {self.last_value}")
    
    def latch_and_read(self):
        """
        Latch counter value to output register and read it.
        
        74HC590: STRO (strobe) is active HIGH - rising edge latches
        
        Returns:
            int: Counter value (3-bit: 0-7)
        """
        # Create latch pulse: LOW -> HIGH -> LOW
        GPIO.output(self.latch_pin, GPIO.LOW)
        time.sleep(0.00002)  # 20us
        GPIO.output(self.latch_pin, GPIO.HIGH)
        time.sleep(0.00002)  # 20us latch pulse
        GPIO.output(self.latch_pin, GPIO.LOW)
        time.sleep(0.00001)  # 10us settle
        
        # Read data pins (LSB first)
        value = 0
        for i, pin in enumerate(self.data_pins):
            bit = GPIO.input(pin)
            value |= (bit << i)
        
        return value
    
    def clear_counter(self):
        """
        Clear the counter (CCLR active LOW pulse)
        """
        GPIO.output(self.clear_pin, GPIO.LOW)
        time.sleep(0.00005)  # 50us clear pulse
        GPIO.output(self.clear_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10us
        print("[74HC590] Counter cleared")
    
    def read_incremental(self):
        """
        Read counter and calculate incremental count since last read.
        Handles 3-bit wraparound (0-7).
        
        Returns:
            tuple: (current_value, delta_count, total_count)
        """
        current = self.latch_and_read()
        
        # Calculate delta (handle wraparound for 3-bit counter)
        if current >= self.last_value:
            delta = current - self.last_value
        else:
            # Wrapped around: e.g., 7 -> 0 means +1, not -7
            delta = (8 - self.last_value) + current
        
        self.total_count += delta
        self.last_value = current
        
        return current, delta, self.total_count
    
    def cleanup(self):
        """Cleanup GPIO"""
        GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser(description='Read 74HC590 wheel counter')
    parser.add_argument('--interval', type=float, default=0.5, help='Sampling interval in seconds')
    parser.add_argument('--ppr', type=float, default=20.0, help='Pulses per wheel revolution (for RPM calc)')
    parser.add_argument('--outfile', default='wheel_counter_log.csv', help='Output CSV file')
    parser.add_argument('--clear-on-start', action='store_true', help='Clear counter at startup')
    
    args = parser.parse_args()
    
    print(f"=== 74HC590 Wheel Counter ===")
    print(f"Latch: BCM {LATCH_PIN} (BOARD 18)")
    print(f"Clear: BCM {CCLR_PIN} (BOARD 11)")
    print(f"Data:  BCM {DATA_PINS} (BOARD 33,40,22)")
    print(f"Interval: {args.interval}s, PPR: {args.ppr}")
    print()
    
    counter = Counter74HC590(LATCH_PIN, DATA_PINS, CCLR_PIN)
    
    # Counter is automatically cleared during initialization
    # Use --clear-on-start to force an additional clear
    if args.clear_on_start:
        print("[USER] Forcing additional counter clear...")
        counter.clear_counter()
        time.sleep(0.1)
        counter.last_value = counter.latch_and_read()
        counter.total_count = 0
        print(f"[USER] Counter re-cleared, value: {counter.last_value}")
    
    try:
        with open(args.outfile, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if csvfile.tell() == 0:
                writer.writerow(['timestamp', 'iso', 'counter_value', 'delta', 'total', 'rpm'])
            
            print("Reading counter... (Ctrl+C to stop)")
            print("timestamp, counter, delta, total, rpm")
            
            last_time = time.time()
            
            while True:
                time.sleep(args.interval)
                
                current_time = time.time()
                value, delta, total = counter.read_incremental()
                
                # Calculate RPM from delta
                # rpm = (pulses / ppr) * (60 / interval)
                elapsed = current_time - last_time
                rpm = 0.0
                if args.ppr > 0 and elapsed > 0:
                    rpm = (delta / args.ppr) * (60.0 / elapsed)
                
                iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(current_time))
                
                writer.writerow([f'{current_time:.6f}', iso, value, delta, total, f'{rpm:.2f}'])
                csvfile.flush()
                
                print(f"{iso}, val={value}, Δ={delta}, total={total}, rpm={rpm:.1f}")
                
                last_time = current_time
    
    except KeyboardInterrupt:
        print('\nStopped by user')
    
    finally:
        counter.cleanup()
        print(f"Log saved to {args.outfile}")


if __name__ == '__main__':
    main()
