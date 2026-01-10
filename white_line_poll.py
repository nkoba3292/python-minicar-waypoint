#!/usr/bin/env python3
"""
white_line_poll.py

Poll a latched RPR220 (via 74HC74) and log white-line detections to CSV.

Pins (BCM):
  LATCH_PIN = 24  # Pi output -> 74HC74 CLK (rising-edge samples D->Q)
  LINE_PIN  = 23  # Pi input  <- 74HC74 Q output (active LOW = white line)
  LINE_RESET_PIN = 18  # Pi output -> 74HC74 CLR (active LOW typically)

Usage:
  sudo python3 white_line_poll.py

This script toggles the latch (low->high) to sample the sensor, reads the latched Q,
applies a small majority-vote debounce, logs timestamp and state to `white_line_log.csv`.

Note: Adjust pin numbers and active-level constants to match your hardware wiring.
"""

import time
import csv
import RPi.GPIO as GPIO

# --- Configuration (edit if needed) ---
LATCH_PIN = 24        # BCM24 (physical 18)
LINE_PIN = 23         # BCM23 (physical 16)
LINE_RESET_PIN = 18   # BCM18 (physical 12)

SAMPLE_INTERVAL = 0.05  # polling interval [s] (e.g., 50ms)
DEBOUNCE_SAMPLES = 3    # number of quick samples for majority vote
DEBOUNCE_DELAY = 0.001  # delay between debounce samples [s]
LATCH_PULSE_US = 60e-6  # latch pulse width (seconds)
RESET_PULSE_US = 60e-6  # reset pulse width (seconds)

# Logic levels (adjust if your latch/clear polarity differs)
LATCH_IDLE_LEVEL = GPIO.LOW
LATCH_ACTIVE_LEVEL = GPIO.HIGH  # generate rising edge: LOW -> HIGH
RESET_ACTIVE_LEVEL = GPIO.LOW   # many /CLR are active LOW
RESET_INACTIVE_LEVEL = GPIO.HIGH

LOG_PATH = 'white_line_log.csv'

# --- Setup GPIO ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(LATCH_PIN, GPIO.OUT, initial=LATCH_IDLE_LEVEL)
GPIO.setup(LINE_RESET_PIN, GPIO.OUT, initial=RESET_INACTIVE_LEVEL)
# LINE_PIN is pulled up on your board; do not enable internal pull-up to avoid conflict
GPIO.setup(LINE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)


def pulse_pin(pin, active_level, inactive_level, pulse_s):
    GPIO.output(pin, active_level)
    time.sleep(pulse_s)
    GPIO.output(pin, inactive_level)


def latch_and_read():
    """Latch the inputs into the D-FFs and read the latched Q value(s).
    Returns: 0 if white-line detected (Q low), 1 if no white-line (Q high)
    """
    # Ensure latch idle
    GPIO.output(LATCH_PIN, LATCH_IDLE_LEVEL)
    # small settle
    time.sleep(LATCH_PULSE_US)
    # create rising edge
    GPIO.output(LATCH_PIN, LATCH_ACTIVE_LEVEL)
    time.sleep(LATCH_PULSE_US)
    GPIO.output(LATCH_PIN, LATCH_IDLE_LEVEL)
    # give outputs time to propagate
    time.sleep(0.00005)

    # debounce / majority vote
    votes = []
    for _ in range(DEBOUNCE_SAMPLES):
        val = GPIO.input(LINE_PIN)
        votes.append(val)
        time.sleep(DEBOUNCE_DELAY)
    # majority
    state = 1 if sum(votes) >= (DEBOUNCE_SAMPLES/2.0) else 0
    # state==0 => white-line detected (active low)
    return state


def reset_line_latch():
    # pulse CLR (active low) to clear flipflops if desired
    GPIO.output(LINE_RESET_PIN, RESET_ACTIVE_LEVEL)
    time.sleep(RESET_PULSE_US)
    GPIO.output(LINE_RESET_PIN, RESET_INACTIVE_LEVEL)


def main():
    print('Starting white line poll. Logging to', LOG_PATH)
    with open(LOG_PATH, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # write header if file empty
        if csvfile.tell() == 0:
            writer.writerow(['timestamp', 'iso', 'state', 'note'])

        try:
            while True:
                ts = time.time()
                iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(ts))
                state = latch_and_read()
                note = ''
                writer.writerow([f'{ts:.6f}', iso, state, note])
                csvfile.flush()

                # Optionally clear the latch / flipflop so next pulses count fresh
                # reset_line_latch()

                # print minimal status
                if state == 0:
                    print(f'{iso} - WHITE LINE')
                else:
                    print(f'{iso} - no white')

                time.sleep(SAMPLE_INTERVAL)
        except KeyboardInterrupt:
            print('\nStopping, cleaning up GPIO...')
        finally:
            GPIO.cleanup()


if __name__ == '__main__':
    main()
