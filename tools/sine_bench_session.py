#!/usr/bin/env python3
"""Run calibration plus one or more BENCH1 sine tests in a single UART session.

`tools/sine_bench.py` performs exactly one action per invocation. On boards whose
USB-serial bridge toggles the reset lines when the port is opened, every
invocation reboots the MCU, which returns the firmware to UNCAL and discards the
calibration. This wrapper keeps one port open so calibrate and the tests can be
chained.

Usage:
    python3 tools/sine_bench_session.py --port /dev/ttyACM0 \
        --current 2,15,6 --speed 1,2,5 --outdir bench-results
"""

import argparse
import csv
import os
import sys
import time
from contextlib import closing

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sine_bench as sb  # noqa: E402


def open_board(port_name, settle):
    port = serial.Serial(port=None, baudrate=115200, timeout=0.025, write_timeout=0.15)
    port.port = port_name
    port.dtr = False
    port.rts = False
    port.open()
    port.reset_input_buffer()
    time.sleep(settle)
    port.reset_input_buffer()
    return port


def calibrate(board):
    board.send("bench cal")
    reason, count = board.wait_for_off("cal", 30)
    print(f"calibrate -> {reason} ({count} samples)", flush=True)
    if reason != "ok":
        raise sb.BenchError(f"Calibration failed ({reason}).")
    return True


def run_test(board, mode, frequency, amplitude, cycles, outdir):
    path = os.path.join(outdir, f"{mode}_{frequency:g}Hz_{amplitude:g}.csv")
    with open(path, "x", encoding="utf-8", newline="") as sink:
        writer = csv.writer(sink)
        writer.writerow(("mode", "frequency_Hz", "requested_amplitude", "cycles",
                         "board_end", "expected_samples", "reference_unit",
                         *sb.FIELDS))
        sink.flush()
        board.send(f"bench start {mode} {frequency:.12g} {amplitude:.12g} {cycles}")
        timeout = cycles / frequency + 3
        reason, count = board.wait_for_off(mode, timeout)
        unit = "mA" if mode == "current" else "electrical_rad/s"
        metadata = (mode, frequency, amplitude, cycles, reason, count, unit)
        rows = board.dump(count, writer, metadata)
        sink.flush()
    print(f"{mode} {frequency:g}Hz -> {reason}, {len(rows)} samples, {path}",
          flush=True)
    estimates, warnings = sb.fitted_channels(rows, mode, frequency, cycles, reason)
    for label, gain, phase, r2 in estimates:
        print(f"  {label}: gain {gain:.5g}; phase {phase:+.2f} deg; sine R2 {r2:.3f}")
    for warning in warnings:
        print(f"  {warning}")
    if reason != "ok":
        print(f"  NOTE: fault data retained ({reason})")
    return path


def triple(text):
    parts = text.split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected frequency,amplitude,cycles")
    return float(parts[0]), float(parts[1]), int(parts[2])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--settle", type=float, default=2.5,
                    help="seconds to wait after opening the port (board may reset)")
    ap.add_argument("--outdir", default="bench-results")
    ap.add_argument("--current", type=triple, action="append", default=None,
                    metavar="HZ,AMP,CYCLES")
    ap.add_argument("--speed", type=triple, action="append", default=None,
                    metavar="HZ,AMP,CYCLES")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    saved = []
    try:
        with closing(open_board(args.port, args.settle)) as port:
            board = sb.Board(port)
            state = board.status()
            print(f"BENCH1 state: {state}", flush=True)
            if state not in ("UNCAL", "IDLE", "DONE", "FAULT"):
                raise sb.BenchError(f"Unexpected state {state}.")
            if state == "UNCAL":
                calibrate(board)
            for spec in (args.current or []):
                saved.append(run_test(board, "current",
                                      spec[0], spec[1], spec[2], args.outdir))
            for spec in (args.speed or []):
                saved.append(run_test(board, "speed",
                                      spec[0], spec[1], spec[2], args.outdir))
            board.send("bench stop")
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, sb.BenchError) as exc:
        sys.exit(str(exc))
    print("done: " + ", ".join(saved), flush=True)


if __name__ == "__main__":
    main()
