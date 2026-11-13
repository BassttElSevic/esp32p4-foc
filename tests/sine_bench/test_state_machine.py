"""Compile the real bench.c against a fake gate, timer, console and rotor.

Run from any directory: python tests/sine_bench/test_state_machine.py
CC may point to a C11 GCC/Clang compiler. Hardware and IDF are not accessed.
"""

import os
from pathlib import Path
import subprocess
import tempfile

TEST = Path(__file__).resolve().parent
ROOT = TEST.parents[1]
SDK_HEADERS = ("freertos/FreeRTOS.h", "freertos/task.h", "esp_console.h",
               "esp_timer.h", "driver/ledc.h")


def main():
    with tempfile.TemporaryDirectory() as work:
        build = Path(work)
        for name in SDK_HEADERS:
            header = build / name
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text('#include "fake_idf.h"\n', encoding="ascii")
        binary = build / ("bench_test.exe" if os.name == "nt" else "bench_test")
        subprocess.run([
            os.environ.get("CC", "gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-O1", "-I", str(build), "-I", str(TEST),
            "-I", str(ROOT / "firmware/foc/main"), str(TEST / "state_machine.c"),
            "-lm", "-o", str(binary)
        ], check=True)
        run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=30)
        if run.returncode:
            raise RuntimeError(f"bench assertions failed ({run.returncode})\n"
                               f"{run.stdout[-1200:]}\n{run.stderr}")
        lines = run.stdout.splitlines()
        rows = [line.removeprefix("BENCH1 DATA ").split(",") for line in lines
                if line.startswith("BENCH1 DATA ")]
        assert len(rows) == 2000, "maximum-duration run must retain every row"
        assert [int(row[0]) for row in rows] == list(range(len(rows)))
        assert all(len(row) == 9 for row in rows)
        assert f"BENCH1 EOF {len(rows)}" in lines
        assert "BENCH_TEST_PASSED" in lines
        print("Firmware state machine passed, including 2000-row board dump.")


if __name__ == "__main__":
    main()
