#!/usr/bin/env python3
"""Log touch sensor data from badge via CDC serial.

Sends 'status' command repeatedly and logs per-channel deltas with timestamps.
Usage: python3 touch_log.py [/dev/ttyACMxx] [interval_ms]
"""

import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM23"
INTERVAL = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.2  # default 200ms

def main():
    ser = serial.Serial(PORT, 115200, timeout=0.5)
    # time.sleep(0.3)
    ser.reset_input_buffer()

    print(f"Logging touch data from {PORT} every {INTERVAL*1000:.0f}ms  (Ctrl-C to stop)")
    print(f"{'time':>10s}  {'ch0':>6s} {'ch1':>6s} {'ch2':>6s} {'ch3':>6s} {'ch4':>6s} {'ch5':>6s}  note  state")
    print("-" * 80)

    t0 = time.monotonic()
    try:
        while True:
            ser.reset_input_buffer()
            ser.write(b"status\n")
            ser.flush()

            lines = []
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
                if any("brightness" in l for l in lines):
                    break

            touch_line = ""
            state_line = ""
            note_line = ""
            for l in lines:
                if l.startswith("touch:"):
                    touch_line = l
                elif l.startswith("state:"):
                    state_line = l
                elif l.startswith("note:"):
                    note_line = l

            elapsed = time.monotonic() - t0
            if touch_line:
                parts = touch_line.split(":")[1].split()
                vals = []
                for p in parts:
                    detected = p.endswith("*")
                    v = p.rstrip("*")
                    vals.append(f"{v:>5s}{'*' if detected else ' '}")
                state = state_line.split(":")[1].strip() if state_line else "?"
                note = note_line.split(":")[1].strip() if note_line else "?"
                print(f"{elapsed:10.2f}  {' '.join(vals)}  {note:>4s}  {state}")
            else:
                print(f"{elapsed:10.2f}  (no response)")

            # time.sleep(INTERVAL)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
