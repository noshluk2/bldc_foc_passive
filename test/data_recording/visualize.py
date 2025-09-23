#!/usr/bin/env python3
"""
Serial data recorder and visualizer for the data_recording firmware.

- Connect to the board's serial port and log CSV to a file.
- Optionally live-plot angle/velocity/torque.

Usage:
  python visualize.py --port /dev/ttyUSB0 --baud 115200 --out run.csv --plot

Requires:
  pip install pyserial matplotlib
"""
import argparse
import csv
import sys
import time

try:
    import serial  # type: ignore
except Exception as e:  # pragma: no cover
    print("Missing dependency: pyserial. Install with 'pip install pyserial'", file=sys.stderr)
    raise

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="Serial port, e.g., /dev/ttyUSB0 or COM3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default="run.csv", help="Output CSV path")
    p.add_argument("--plot", action="store_true", help="Live plot data")
    return p.parse_args()

def main():
    args = parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    # Drain boot messages
    time.sleep(0.5)
    ser.reset_input_buffer()

    print(f"Logging to {args.out} ... Press Ctrl+C to stop.")

    # optional plotting
    if args.plot:
        try:
            import matplotlib.pyplot as plt  # type: ignore
        except Exception:
            print("Missing dependency: matplotlib. Install with 'pip install matplotlib'", file=sys.stderr)
            args.plot = False
        else:
            plt.ion()
            fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
            lines = [ax[0].plot([], [], label="angle [rad]")[0],
                     ax[1].plot([], [], label="velocity [rad/s]")[0],
                     ax[2].plot([], [], label="torque_cmd")[0]]
            for a in ax:
                a.grid(True)
                a.legend(loc="upper right")
            xs, as_, vs, ts = [], [], [], []

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_ms", "angle_rad", "velocity_rad_s", "torque_cmd", "mode"])  # header

        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                # Filter out non-CSV lines
                if not (line[0].isdigit() and "," in line):
                    # print status lines to console
                    print(line)
                    continue
                parts = line.split(",")
                if len(parts) < 5:
                    continue
                writer.writerow(parts[:5])
                f.flush()

                if args.plot:
                    try:
                        t = float(parts[0]) * 1e-3
                        a = float(parts[1])
                        v = float(parts[2])
                        tc = float(parts[3])
                    except ValueError:
                        continue
                    xs.append(t); as_.append(a); vs.append(v); ts.append(tc)
                    # keep last N points for responsiveness
                    N = 2000
                    xs, as_, vs, ts = xs[-N:], as_[-N:], vs[-N:], ts[-N:]
                    lines[0].set_data(xs, as_)
                    lines[1].set_data(xs, vs)
                    lines[2].set_data(xs, ts)
                    for aax, data in zip((ax[0], ax[1], ax[2]), (as_, vs, ts)):
                        if data:
                            aax.set_xlim(xs[0], xs[-1])
                            dy = max(data) - min(data)
                            pad = 0.1 * dy if dy > 1e-6 else 0.5
                            aax.set_ylim(min(data) - pad, max(data) + pad)
                    plt.pause(0.001)

        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    main()

