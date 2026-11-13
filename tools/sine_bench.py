"""Capture one ESP32-P4 BENCH1 test using pyserial and NumPy. Never enables at port open.

python tools/sine_bench.py --port COM5 status
python tools/sine_bench.py --port COM5 stop
python tools/sine_bench.py --port COM5 calibrate
python tools/sine_bench.py --port COM5 current --frequency 2 --amplitude 15 --cycles 6 --output current.csv
python tools/sine_bench.py --port COM5 speed --frequency 1 --amplitude 2 --cycles 5 --output speed.csv
"""
import argparse
from collections import deque
from contextlib import closing
import csv
import math
from pathlib import Path
import re
import sys
import time

import numpy as np


STATES = {"BOOT", "UNCAL", "IDLE", "CAL", "TEST", "DONE", "FAULT", "DUMP"}
FIELDS = ("seq", "t_us", "reference", "iq_mA", "iq_filtered_mA", "omega_raw_e_rad_s",
          "omega_filtered_e_rad_s", "angle_code", "limited")
MAX_RECORDS = 2001
HEARTBEAT_SECONDS = 0.10
ANSI = re.compile(rb"\x1b\[[0-?]*[ -/]*[@-~]")


class BenchError(Exception):
    """Protocol, acquisition or parameter failure."""


class Board:
    def __init__(self, port):
        self.port = port
        self.partial = b""
        self.pending = deque()

    def send(self, command):
        payload = (command + "\r").encode("ascii")
        if self.port.write(payload) != len(payload):
            raise BenchError("Incomplete UART command write.")

    def next_event(self):
        if not self.pending:
            data = self.port.read(min(4096, max(1, self.port.in_waiting)))
            if not data:
                return None
            lines = (self.partial + data).split(b"\n")
            self.partial = lines.pop()
            if len(self.partial) > 16384:
                raise BenchError("Overlong UART line; check port, baud and firmware.")
            self.pending.extend(lines)
        while self.pending:
            line = ANSI.sub(b"", self.pending.popleft())
            prefix = line.find(b"BENCH1 ")
            if prefix >= 0:
                return line[prefix + len(b"BENCH1 "):].strip().decode("ascii").split(maxsplit=1)
        return None

    def status(self):
        self.send("bench status")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            event = self.next_event()
            if event and len(event) == 2 and event[0] == "STATUS" and event[1] in STATES:
                return event[1]
        raise BenchError("No BENCH1 status reply; verify the dedicated bench firmware and UART.")

    def wait_for_off(self, mode, timeout, heartbeat=True):
        deadline = time.monotonic() + timeout
        next_ping = time.monotonic() + HEARTBEAT_SECONDS
        while time.monotonic() < deadline:
            if heartbeat and time.monotonic() >= next_ping:
                self.send("bench ping")
                next_ping = time.monotonic() + HEARTBEAT_SECONDS
            event = self.next_event()
            if not event:
                continue
            kind, *rest = event
            if kind == "ERR":
                raise BenchError("Firmware rejected the command: " + " ".join(rest))
            if kind == "BEGIN":
                if rest != [mode]:
                    raise BenchError("Unexpected active mode.")
                print(f"{mode} active; Ctrl+C requests stop.", flush=True)
            if kind == "END":
                args = " ".join(rest).split()
                if len(args) != 3 or args[0] != mode or not args[2].isdigit():
                    raise BenchError("Unexpected END record.")
                count = int(args[2])
                if count > MAX_RECORDS:
                    raise BenchError("Impossible board record count.")
                return args[1], count
        raise BenchError("Timed out waiting for a drive-off END acknowledgement.")

    def dump(self, count, writer, metadata):
        rows = []
        self.send("bench dump")
        # At 115200 baud the largest transfer can take tens of seconds.
        deadline = time.monotonic() + 5 + count * 0.03
        while time.monotonic() < deadline:
            event = self.next_event()
            if not event:
                continue
            if event[0] == "ERR":
                raise BenchError("Board rejected the data transfer: " + " ".join(event[1:]))
            if event[0] == "EOF":
                if event[1:] != [str(count)] or len(rows) != count:
                    raise BenchError(f"Incomplete capture: {len(rows)} of {count} samples.")
                return rows
            if event[0] != "DATA":
                continue
            if len(event) != 2 or len(rows) >= count:
                raise BenchError("Unexpected DATA record.")
            parts = event[1].split(",")
            if len(parts) != len(FIELDS):
                raise BenchError("Incorrect DATA field count.")
            seq, timestamp = int(parts[0]), int(parts[1])
            signals = [float(x) for x in parts[2:7]]
            code, limited = int(parts[7]), int(parts[8])
            if (seq != len(rows) or timestamp < 0 or (rows and timestamp <= rows[-1][1])
                    or not all(math.isfinite(x) for x in signals) or not 0 <= code < 4096
                    or limited < 0):
                raise BenchError("Invalid sequence, timestamp or sample values.")
            row = (seq, timestamp, *signals, code, limited)
            writer.writerow((*metadata, *row))
            rows.append(row)
        raise BenchError(f"Timed out transferring samples ({len(rows)}/{count}).")


def harmonic(phase, signal):
    # y = a sin(phi) + b cos(phi) + offset; the complex phasor is a+j*b.
    design = np.column_stack((np.sin(phase), np.cos(phase), np.ones(len(phase))))
    coeffs, _, rank, _ = np.linalg.lstsq(design, signal, rcond=None)
    if rank < 3:
        raise BenchError("Insufficient phase coverage for a sine fit.")
    power = np.sum((signal - signal.mean()) ** 2)
    residual = signal - design @ coeffs
    r2 = 1 - float(residual @ residual) / power if power > 0 else 0.0
    return complex(coeffs[0], coeffs[1]), r2


def fitted_channels(rows, mode, frequency, cycles, board_reason):
    """Fit the final two cycles by board timestamps; suppress unqualified estimates."""
    if board_reason != "ok":
        return [], [f"Board ended with {board_reason}; incomplete or faulted trial."]
    if len(rows) < 24:
        return [], ["Insufficient captured samples."]
    data = np.asarray(rows, dtype=float)
    turns = data[:,1] * (1e-6 * frequency)
    last_two = [np.flatnonzero((turns >= n) & (turns < n+1)) for n in (cycles-2, cycles-1)]
    if any(len(ix) < 12 or turns[ix[-1]] - turns[ix[0]] < 0.85
           or np.max(np.diff(turns[ix])) > 0.12 for ix in last_two):
        return [], ["Final two cycles have gaps or insufficient phase coverage."]
    keep = np.concatenate(last_two)
    if np.any(data[:,8]):
        return [], ["Current/voltage limiting occurred in the trial; no linear gain reported."]
    if mode == "speed":
        code_angle = np.unwrap(data[keep,7] * (2*np.pi/4096))
        if np.ptp(code_angle) * (4096/(2*np.pi)) < 2 - 1e-8:
            return [], ["Encoder moved less than two codes peak-to-peak; speed response unresolved."]
    phase = 2*np.pi*turns
    ref_fits = [harmonic(phase[ix], data[ix,2]) for ix in [*last_two, keep]]
    if any(abs(ref) < 1e-9 or r2 < 0.95 for ref, r2 in ref_fits):
        return [], ["Reference waveform has inadequate amplitude or sine fit."]
    estimates, warnings = [], []
    for col in ((3,4) if mode == "current" else (5,6)):
        ratios = []
        for ix, (ref, _) in zip([*last_two, keep], ref_fits):
            out, r2 = harmonic(phase[ix], data[ix,col])
            ratios.append(out/ref)
        label = FIELDS[col]
        if mode == "current" and abs(out) < 2.0:
            warnings.append(f"{label}: fitted amplitude below 2 mA screening threshold.")
        elif r2 < 0.6:
            warnings.append(f"{label}: sine explains only {max(0,r2):.0%} of sample variance.")
        elif (min(abs(x) for x in ratios) < 1e-12
              or abs(abs(ratios[0])-abs(ratios[1])) > 0.25*abs(ratios[2])
              or abs(np.angle(ratios[1]/ratios[0], deg=True)) > 20):
            warnings.append(f"{label}: last two cycles disagree; steady tracking unconfirmed.")
        else:
            estimates.append((label, abs(ratios[-1]), np.angle(ratios[-1], deg=True), r2))
    return estimates, warnings


def options(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="CH343 UART port, e.g. COM5 or /dev/ttyUSB0")
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("status", help="Read the firmware state")
    sub.add_parser("stop", help="Request drive disable on a verified BENCH1 device")
    sub.add_parser("calibrate", help="Explicit powered rotor calibration; expect shaft motion")
    for mode, unit, frequency, amplitude, cycles in (("current","mA peak",2.,15.,6),
                                                      ("speed","electrical rad/s peak",1.,2.,5)):
        p = sub.add_parser(mode, help=f"Execute one bounded {mode}-reference sine test")
        p.add_argument("--frequency", type=float, default=frequency, help="Excitation frequency in Hz")
        p.add_argument("--amplitude", type=float, default=amplitude, help=unit)
        p.add_argument("--cycles", type=int, default=cycles)
        p.add_argument("--output", type=Path, required=True, help="New CSV file (exclusive creation)")
    args = parser.parse_args(argv)
    if args.mode in ("current","speed"):
        limit = 60 if args.mode == "current" else 5
        if not (0.5 <= args.frequency <= 10 and 0 < args.amplitude <= limit
                and 3 <= args.cycles <= 12 and args.cycles / args.frequency <= 8):
            parser.error(f"Require 0.5..10 Hz, 0 < amplitude <= {limit:g}, 3..12 cycles, duration <= 8 s.")
    return args


def run(args, board):
    state = board.status()
    print(f"BENCH1 state: {state}", flush=True)
    if args.mode == "status":
        return
    if args.mode == "stop":
        board.send("bench stop")
        print("Stop sent to BENCH1. For immediate isolation use the hardware power disconnect.")
        return
    cal = args.mode == "calibrate"
    valid_states = ("UNCAL", "IDLE", "DONE", "FAULT") if cal else ("IDLE", "DONE")
    if state not in valid_states:
        raise BenchError("Cannot start in this state. Check the drive; calibrate is an explicit separate action.")
    mode = "cal" if cal else args.mode
    sink = None
    try:
        if cal:
            command, timeout = "bench cal", 30
        else:
            sink = args.output.open("x", encoding="utf-8", newline="")
            writer = csv.writer(sink)
            writer.writerow(("mode", "frequency_Hz", "requested_amplitude", "cycles", "board_end",
                             "expected_samples", "reference_unit", *FIELDS))
            sink.flush()
            command = f"bench start {mode} {args.frequency:.12g} {args.amplitude:.12g} {args.cycles}"
            timeout = args.cycles / args.frequency + 3
        off = False
        interrupted = False
        try:
            board.send(command)
            try:
                reason, count = board.wait_for_off(mode, timeout)
            except KeyboardInterrupt:
                interrupted = True
                board.send("bench stop")
                reason, count = board.wait_for_off(mode, 2, heartbeat=False)
            off = True
        finally:
            if not off:
                try:
                    board.send("bench stop")
                except (OSError, BenchError):
                    pass
                print("No confirmed drive-off END. Stop requested; use the physical power disconnect if needed.",
                      file=sys.stderr, flush=True)
        print(f"Drive-off END: {mode} {reason}; samples: {count}", flush=True)
        if cal:
            if interrupted:
                raise KeyboardInterrupt
            if reason != "ok":
                raise BenchError("Calibration failed. Inspect the fixture and fault before another attempt.")
            return
        unit = "mA" if mode == "current" else "electrical_rad/s"
        metadata = (mode, args.frequency, args.amplitude, args.cycles, reason, count, unit)
        rows = board.dump(count, writer, metadata)
        sink.flush()
        print(f"Saved {len(rows)} samples: {args.output}")
        if interrupted:
            raise KeyboardInterrupt
        estimates, warnings = fitted_channels(rows, mode, args.frequency, args.cycles, reason)
        if estimates:
            print("Finite-window fits to the last two cycles.")
            if mode == "speed":
                print("Raw and filtered speed are estimates from the same AS5600 encoder.")
        for label, gain, phase, r2 in estimates:
            print(f"{label}: gain {gain:.5g}; phase {phase:+.2f} deg; sine R2 {r2:.3f}")
        for warning in warnings:
            print(warning, file=sys.stderr)
        if reason != "ok":
            raise BenchError(f"Fault data retained ({reason}); test did not complete successfully.")
    finally:
        if sink is not None:
            sink.close()


def main():
    args = options()
    try:
        import serial
    except ImportError:
        sys.exit("Install pyserial and numpy in this Python environment first.")
    try:
        with closing(serial.Serial(port=None, baudrate=115200, timeout=0.025, write_timeout=0.15)) as port:
            port.port = args.port
            port.dtr = False
            port.rts = False
            port.open()
            port.reset_input_buffer()
            run(args, Board(port))
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, BenchError) as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    main()
