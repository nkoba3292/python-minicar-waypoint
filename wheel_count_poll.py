#!/usr/bin/env python3
"""
wheel_count_poll.py

Count wheel pulses for RPM testing and/or read a parallel counter (e.g., 74HC590) outputs
for wheel encoder debug. Logs timestamped counts/RPM to CSV.

Usage examples:
  # Pulse-edge counting mode (default)
  sudo python3 wheel_count_poll.py --mode pulse --pin 17 --ppr 20 --interval 1.0

  # Parallel bus read mode (74HC590 parallel outputs)
  sudo python3 wheel_count_poll.py --mode parallel --latch 24 --data 5 6 13 19 26 16 20 21 --reset 18 --interval 0.5

Notes:
- Run on Raspberry Pi with `RPi.GPIO` installed and run as root (sudo).
- Choose GPIO numbering mode with --bcm (default uses BCM). If you wire by physical pin numbers, use --board.
"""

import time
import csv
import argparse
import RPi.GPIO as GPIO
from collections import deque


def setup_gpio(mode_bcm):
    # Set mode first before any cleanup
    if mode_bcm:
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            pass  # Already set
    else:
        try:
            GPIO.setmode(GPIO.BOARD)
        except Exception:
            pass
    
    GPIO.setwarnings(False)


# --- Pulse counting implementation (edge detection) ---
class PulseCounter:
    def __init__(self, pin, edge=GPIO.RISING):
        self.pin = pin
        self.count = 0
        self._start = time.time()
        # RPR220 typically needs pull-up or pull-down depending on output type
        # Try pull-up first (common for open-collector outputs)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        time.sleep(0.1)  # Let pin stabilize
        
        # Remove any existing event detection first
        try:
            GPIO.remove_event_detect(self.pin)
        except Exception:
            pass
        
        # Add event detection with longer bouncetime for mechanical/optical noise
        try:
            GPIO.add_event_detect(self.pin, edge, callback=self._cb, bouncetime=10)
        except RuntimeError as e:
            print(f"ERROR: Failed to add edge detection on pin {self.pin}")
            print(f"  Possible causes:")
            print(f"  1. Pin is already in use by another process")
            print(f"  2. Hardware issue with the pin")
            print(f"  3. Wrong pin number (check BCM vs BOARD mode)")
            raise

    def _cb(self, channel):
        self.count += 1

    def read_and_reset(self):
        now = time.time()
        c = self.count
        self.count = 0
        return now, c

    def close(self):
        try:
            GPIO.remove_event_detect(self.pin)
        except Exception:
            pass


# --- Parallel counter read implementation (read multiple data pins) ---
class ParallelReader:
    def __init__(self, latch_pin, data_pins, reset_pin=None, latch_active=GPIO.HIGH):
        self.latch_pin = latch_pin
        self.data_pins = data_pins
        self.reset_pin = reset_pin
        self.latch_active = latch_active

        GPIO.setup(self.latch_pin, GPIO.OUT, initial=GPIO.LOW if latch_active == GPIO.HIGH else GPIO.HIGH)
        if reset_pin is not None:
            GPIO.setup(self.reset_pin, GPIO.OUT, initial=GPIO.HIGH)
        for p in self.data_pins:
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

    def latch_and_read(self):
        # create a latch pulse (active level -> inactive level)
        inactive = GPIO.LOW if self.latch_active == GPIO.HIGH else GPIO.HIGH
        GPIO.output(self.latch_pin, inactive)
        time.sleep(0.00005)
        GPIO.output(self.latch_pin, self.latch_active)
        time.sleep(0.00005)
        # back to inactive
        GPIO.output(self.latch_pin, inactive)
        time.sleep(0.00002)

        # read bits, LSB is assumed to be first in list
        value = 0
        for i, p in enumerate(self.data_pins):
            bit = GPIO.input(p)
            value |= (bit & 0x1) << i
        return value

    def reset_counter(self, pulse_time_s=0.00005, active_low=True):
        if self.reset_pin is None:
            return
        active = GPIO.LOW if active_low else GPIO.HIGH
        inactive = GPIO.HIGH if active_low else GPIO.LOW
        GPIO.output(self.reset_pin, active)
        time.sleep(pulse_time_s)
        GPIO.output(self.reset_pin, inactive)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['pulse','parallel'], default='pulse')
    parser.add_argument('--bcm', action='store_true', default=True, help='Use BCM GPIO numbering (default)')
    parser.add_argument('--board', action='store_true', help='Use BOARD (physical) numbering')

    # pulse mode args
    parser.add_argument('--pin', type=int, default=17, help='GPIO pin for pulse input (BCM or BOARD depending), default=17')
    parser.add_argument('--edge', choices=['rising','falling','both'], default='rising')
    parser.add_argument('--ppr', type=float, default=20.0, help='Pulses per wheel revolution (for RPM calc)')
    parser.add_argument('--interval', type=float, default=1.0, help='Logging interval in seconds')

    # parallel mode args
    parser.add_argument('--latch', type=int, help='Latch pin (to capture counter outputs)')
    parser.add_argument('--data', type=int, nargs='+', help='Data pins list (LSB first)')
    parser.add_argument('--reset', type=int, help='Reset/Clear pin for counter (optional)')
    parser.add_argument('--outfile', default='wheel_count_log.csv')

    args = parser.parse_args()

    # select numbering mode
    if args.board:
        mode_bcm = False
    else:
        mode_bcm = True  # default to BCM unless --board specified

    setup_gpio(mode_bcm)

    try:
        with open(args.outfile, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            if csvfile.tell() == 0:
                if args.mode == 'pulse':
                    writer.writerow(['timestamp', 'iso', 'count', 'interval', 'rpm'])
                else:
                    writer.writerow(['timestamp', 'iso', 'value'])

            if args.mode == 'pulse':
                # Use default pin if not specified
                pin = args.pin if args.pin is not None else 17
                edge_map = {'rising': GPIO.RISING, 'falling': GPIO.FALLING, 'both': GPIO.BOTH}
                counter = PulseCounter(pin, edge=edge_map[args.edge])

                print(f"Pulse mode: pin={pin} ppr={args.ppr} interval={args.interval}s")
                print(f"NOTE: Running in mock mode on PC - no actual GPIO. Press Ctrl+C to stop.")
                last_ts = time.time()
                try:
                    while True:
                        time.sleep(args.interval)
                        ts, c = counter.read_and_reset()
                        iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(ts))
                        # rpm = (count / pulses_per_rev) * (60 / interval)
                        rpm = 0.0
                        if args.ppr > 0 and args.interval > 0:
                            rpm = (c / args.ppr) * (60.0 / args.interval)
                        writer.writerow([f'{ts:.6f}', iso, c, args.interval, f'{rpm:.2f}'])
                        csvfile.flush()
                        print(f"{iso} count={c} rpm={rpm:.1f}")
                except KeyboardInterrupt:
                    print('\nStopped by user')
                finally:
                    counter.close()

            else:  # parallel mode
                if args.latch is None or args.data is None:
                    raise SystemExit('Parallel mode requires --latch and --data pins')
                reader = ParallelReader(args.latch, args.data, reset_pin=args.reset)
                print(f"Parallel mode: latch={args.latch} data={args.data} reset={args.reset} interval={args.interval}s")
                try:
                    while True:
                        val = reader.latch_and_read()
                        ts = time.time()
                        iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(ts))
                        writer.writerow([f'{ts:.6f}', iso, val])
                        csvfile.flush()
                        print(f"{iso} value=0x{val:02X} ({val})")
                        time.sleep(args.interval)
                except KeyboardInterrupt:
                    print('\nStopped by user')
                finally:
                    # no special close
                    pass

    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
