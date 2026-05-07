#!/usr/bin/env python3
"""
decode_log.py — decode datalog_###.bin produced by the binary logger

Usage:
    python decode_log.py datalog_000.bin          # print to stdout
    python decode_log.py datalog_000.bin -o out.csv   # save as CSV
    python decode_log.py datalog.bin --plot        # quick matplotlib preview
"""

import struct
import sys
import argparse
import csv
from pathlib import Path

# ── Constants ──────────────────────────────────────────────────────────────────
MAGIC           = bytes([0xDE, 0xAD, 0xBE, 0xEF])
HEADER_SIZE     = 6          # 4-byte magic + 2-byte record size
RECORD_FMT      = "<IffffffffH"   # little-endian: uint32, 8× float, uint16
RECORD_SIZE_EXP = struct.calcsize(RECORD_FMT)   # should be 38

FIELDS = [
    "timestamp_ms",
    "accelX", "accelY", "accelZ",
    "gyroX",  "gyroY",  "gyroZ",
    "pressureHPa", "altitudeM",
    "crc16",
]

# ── CRC-16/CCITT-FALSE (mirrors the ESP32 implementation) ─────────────────────
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc


# ── Decoder ───────────────────────────────────────────────────────────────────
def decode(path: str) -> list[dict]:
    data = Path(path).read_bytes()

    # Validate magic
    if data[:4] != MAGIC:
        raise ValueError(f"Bad magic: {data[:4].hex()} (expected deadbeef)")

    record_size = struct.unpack_from("<H", data, 4)[0]
    if record_size != RECORD_SIZE_EXP:
        raise ValueError(
            f"Record size mismatch: file says {record_size}, "
            f"decoder expects {RECORD_SIZE_EXP}"
        )

    payload    = data[HEADER_SIZE:]
    n_records  = len(payload) // record_size
    leftover   = len(payload) % record_size

    if leftover:
        print(f"[warn] {leftover} trailing bytes ignored (incomplete record)",
              file=sys.stderr)

    records = []
    crc_errors = 0

    for i in range(n_records):
        chunk  = payload[i * record_size : (i + 1) * record_size]
        values = struct.unpack(RECORD_FMT, chunk)
        row    = dict(zip(FIELDS, values))

        # Verify CRC (covers all bytes except last 2)
        expected_crc = crc16(chunk[:-2])
        if row["crc16"] != expected_crc:
            crc_errors += 1
            print(
                f"[warn] Record {i}: CRC mismatch "
                f"(got 0x{row['crc16']:04X}, expected 0x{expected_crc:04X}) — kept",
                file=sys.stderr,
            )

        records.append(row)

    print(
        f"[info] Decoded {n_records} records from '{path}' "
        f"({crc_errors} CRC errors)",
        file=sys.stderr,
    )
    return records


# ── Output helpers ─────────────────────────────────────────────────────────────
def print_table(records: list[dict]) -> None:
    header = (
        f"{'ms':>10}  {'aX':>8} {'aY':>8} {'aZ':>8}  "
        f"{'gX':>8} {'gY':>8} {'gZ':>8}  "
        f"{'hPa':>8} {'alt':>7}"
    )
    print(header)
    print("─" * len(header))
    for r in records:
        print(
            f"{r['timestamp_ms']:>10}  "
            f"{r['accelX']:>8.4f} {r['accelY']:>8.4f} {r['accelZ']:>8.4f}  "
            f"{r['gyroX']:>8.4f} {r['gyroY']:>8.4f} {r['gyroZ']:>8.4f}  "
            f"{r['pressureHPa']:>8.2f} {r['altitudeM']:>7.2f}"
        )


def write_csv(records: list[dict], out_path: str) -> None:
    fieldnames = [f for f in FIELDS if f != "crc16"]
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in records:
            row = {k: v for k, v in r.items() if k != "crc16"}
            writer.writerow(row)
    print(f"[info] CSV written to '{out_path}'", file=sys.stderr)


def plot(records: list[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — run: pip install matplotlib", file=sys.stderr)
        return

    t  = [r["timestamp_ms"] / 1000.0 for r in records]   # seconds

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    axes[0].plot(t, [r["accelX"] for r in records], label="aX")
    axes[0].plot(t, [r["accelY"] for r in records], label="aY")
    axes[0].plot(t, [r["accelZ"] for r in records], label="aZ")
    axes[0].set_ylabel("Accel (m/s²)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    axes[1].plot(t, [r["gyroX"] for r in records], label="gX")
    axes[1].plot(t, [r["gyroY"] for r in records], label="gY")
    axes[1].plot(t, [r["gyroZ"] for r in records], label="gZ")
    axes[1].set_ylabel("Gyro (°/s)")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    ax3 = axes[2]
    ax3.plot(t, [r["pressureHPa"] for r in records], color="tab:blue",  label="hPa")
    ax3.set_ylabel("Pressure (hPa)", color="tab:blue")
    ax3b = ax3.twinx()
    ax3b.plot(t, [r["altitudeM"] for r in records], color="tab:orange", label="alt")
    ax3b.set_ylabel("Altitude (m)", color="tab:orange")
    ax3.set_xlabel("Time (s)")
    ax3.grid(True)

    fig.suptitle("ESP32 Sensor Log")
    plt.tight_layout()
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Decode ESP32 binary sensor log")
    parser.add_argument("input",          help="Path to datalog.bin")
    parser.add_argument("-o", "--output", help="Save decoded data as CSV")
    parser.add_argument("--plot", action="store_true", help="Plot with matplotlib")
    args = parser.parse_args()

    records = decode(args.input)

    if args.output:
        write_csv(records, args.output)
    elif args.plot:
        plot(records)
    else:
        print_table(records)


if __name__ == "__main__":
    main()