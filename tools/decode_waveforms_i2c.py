#!/usr/bin/env python3
"""
Decode raw WaveForms Logic Analyzer CSV exports for the CaelumFusion shared I2C bus.

The decoder is intentionally independent of the WaveForms GUI so captures can be
checked, archived, and compared deterministically. It expects raw logic samples
for SCL/SDA, reconstructs START/RESTART/STOP, samples SDA on SCL rising edges,
decodes address/data bytes with ACK/NACK, and summarizes expected addresses such
as the LIS3DH 0x18/0x19 probe path.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_EXPECTED_ADDRS = (0x18, 0x19)


@dataclass
class Sample:
    t: float
    scl: int
    sda: int


@dataclass
class ByteDecode:
    t: float
    value: int
    ack: bool
    role: str


@dataclass
class Frame:
    start_t: float
    stop_t: Optional[float] = None
    repeated_start: bool = False
    bytes: List[ByteDecode] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    @property
    def address(self) -> Optional[int]:
        if not self.bytes:
            return None
        return self.bytes[0].value >> 1

    @property
    def rw(self) -> Optional[int]:
        if not self.bytes:
            return None
        return self.bytes[0].value & 1

    @property
    def address_ack(self) -> Optional[bool]:
        if not self.bytes:
            return None
        return self.bytes[0].ack


def parse_level(value: str, threshold: float) -> int:
    text = value.strip().strip('"').strip("'")
    low = text.lower()
    if low in {"0", "l", "lo", "low", "false", "f"}:
        return 0
    if low in {"1", "h", "hi", "high", "true", "t"}:
        return 1
    try:
        return 1 if float(text) >= threshold else 0
    except ValueError as exc:
        raise ValueError(f"cannot parse digital level {value!r}") from exc


def norm_name(name: str) -> str:
    return "".join(ch.lower() for ch in name if ch.isalnum())


def choose_column(header: Sequence[str], requested: Optional[str], candidates: Sequence[str]) -> str:
    if requested:
        if requested in header:
            return requested
        wanted = norm_name(requested)
        for col in header:
            if norm_name(col) == wanted:
                return col
        raise ValueError(f"column {requested!r} not found in CSV header {list(header)!r}")

    normalized = {norm_name(col): col for col in header}
    for candidate in candidates:
        wanted = norm_name(candidate)
        if wanted in normalized:
            return normalized[wanted]

    for col in header:
        n = norm_name(col)
        if any(norm_name(candidate) in n for candidate in candidates):
            return col

    raise ValueError(f"could not auto-detect any of {candidates!r} in CSV header {list(header)!r}")


def parse_time(value: str) -> float:
    text = value.strip().strip('"').strip("'")
    if not text:
        raise ValueError("empty time field")
    lower = text.lower()
    scale = 1.0
    for suffix, factor in (("ns", 1e-9), ("us", 1e-6), ("ms", 1e-3), ("s", 1.0)):
        if lower.endswith(suffix):
            scale = factor
            text = lower[: -len(suffix)].strip()
            break
    return float(text) * scale


def read_waveforms_csv(
    path: Path,
    scl_col: Optional[str],
    sda_col: Optional[str],
    time_col: Optional[str],
    sample_rate: Optional[float],
    threshold: float,
) -> List[Sample]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{path} has no CSV header")

        header = reader.fieldnames
        scl_name = choose_column(header, scl_col, ("i2c_scl", "scl", "dio0", "clock"))
        sda_name = choose_column(header, sda_col, ("i2c_sda", "sda", "dio1", "data"))

        time_name = None
        if time_col:
            time_name = choose_column(header, time_col, (time_col,))
        else:
            for candidate in ("time", "time_s", "t", "seconds"):
                try:
                    time_name = choose_column(header, None, (candidate,))
                    break
                except ValueError:
                    pass

        rows: List[Sample] = []
        for idx, row in enumerate(reader):
            if time_name:
                t = parse_time(row[time_name])
            else:
                if not sample_rate:
                    raise ValueError("CSV has no time column; pass --sample-rate")
                t = idx / sample_rate
            rows.append(
                Sample(
                    t=t,
                    scl=parse_level(row[scl_name], threshold),
                    sda=parse_level(row[sda_name], threshold),
                )
            )

    if len(rows) < 2:
        raise ValueError("need at least two samples")
    return rows


def collapse_runs(samples: Sequence[Sample]) -> List[Sample]:
    events = [samples[0]]
    last_scl = samples[0].scl
    last_sda = samples[0].sda
    for sample in samples[1:]:
        if sample.scl != last_scl or sample.sda != last_sda:
            events.append(sample)
            last_scl = sample.scl
            last_sda = sample.sda
    return events


def glitch_filter(events: Sequence[Sample], min_width_s: float) -> List[Sample]:
    if min_width_s <= 0.0 or len(events) < 3:
        return list(events)

    filtered = list(events)
    changed = True
    while changed:
        changed = False
        out: List[Sample] = [filtered[0]]
        i = 1
        while i < len(filtered) - 1:
            prev = out[-1]
            cur = filtered[i]
            nxt = filtered[i + 1]
            width = nxt.t - cur.t
            if width < min_width_s and prev.scl == nxt.scl and prev.sda == nxt.sda:
                changed = True
                i += 1
                continue
            out.append(cur)
            i += 1
        out.append(filtered[-1])
        filtered = out
    return filtered


def decode_i2c(samples: Sequence[Sample], glitch_s: float = 0.0) -> List[Frame]:
    events = glitch_filter(collapse_runs(samples), glitch_s)
    frames: List[Frame] = []
    current: Optional[Frame] = None
    bits: List[Tuple[float, int]] = []

    def finish_byte() -> None:
        nonlocal bits, current
        if current is None or len(bits) < 9:
            return
        value = 0
        for _, bit in bits[:8]:
            value = ((value << 1) | bit) & 0xFF
        ack = bits[8][1] == 0
        role = "address" if not current.bytes else "data"
        current.bytes.append(ByteDecode(t=bits[0][0], value=value, ack=ack, role=role))
        bits = bits[9:]

    def abort_partial(reason: str) -> None:
        nonlocal bits, current
        if current is not None and bits:
            current.errors.append(f"{reason}: partial {len(bits)}/9 bit byte")
        bits = []

    for prev, cur in zip(events, events[1:]):
        sda_fall = prev.sda == 1 and cur.sda == 0
        sda_rise = prev.sda == 0 and cur.sda == 1
        scl_rise = prev.scl == 0 and cur.scl == 1

        if cur.scl == 1 and sda_fall:
            if current is not None:
                abort_partial("repeated START")
                current.repeated_start = True
            current = Frame(start_t=cur.t)
            frames.append(current)
            bits = []
            continue

        if cur.scl == 1 and sda_rise:
            if current is not None:
                abort_partial("STOP")
                current.stop_t = cur.t
                current = None
            bits = []
            continue

        if current is not None and scl_rise:
            bits.append((cur.t, cur.sda))
            while len(bits) >= 9:
                finish_byte()

    if current is not None:
        abort_partial("capture ended before STOP")
        current.errors.append("capture ended before STOP")
    return frames


def frame_to_dict(frame: Frame, expected_addrs: Iterable[int]) -> Dict[str, object]:
    addr = frame.address
    expected = addr in set(expected_addrs) if addr is not None else False
    return {
        "start_s": frame.start_t,
        "stop_s": frame.stop_t,
        "address": None if addr is None else f"0x{addr:02X}",
        "rw": None if frame.rw is None else ("R" if frame.rw else "W"),
        "address_ack": frame.address_ack,
        "expected_address": expected,
        "byte_count": len(frame.bytes),
        "data": [f"0x{b.value:02X}" for b in frame.bytes[1:]],
        "data_ack": [b.ack for b in frame.bytes[1:]],
        "repeated_start_after": frame.repeated_start,
        "errors": frame.errors,
    }


def print_text(frames: Sequence[Frame], expected_addrs: Sequence[int]) -> None:
    expected_set = set(expected_addrs)
    print("idx,start_us,addr,rw,addr_ack,expected,data,error")
    for idx, frame in enumerate(frames):
        addr = frame.address
        addr_text = "--" if addr is None else f"0x{addr:02X}"
        rw_text = "--" if frame.rw is None else ("R" if frame.rw else "W")
        ack_text = "--" if frame.address_ack is None else ("ACK" if frame.address_ack else "NACK")
        expected = "yes" if addr in expected_set else "no"
        data_text = " ".join(f"0x{b.value:02X}:{'A' if b.ack else 'N'}" for b in frame.bytes[1:])
        err_text = "; ".join(frame.errors)
        print(
            f"{idx},{frame.start_t * 1e6:.3f},{addr_text},{rw_text},"
            f"{ack_text},{expected},{data_text},{err_text}"
        )

    counts: Dict[int, Dict[str, int]] = {}
    for frame in frames:
        if frame.address is None:
            continue
        entry = counts.setdefault(frame.address, {"ack": 0, "nack": 0, "frames": 0})
        entry["frames"] += 1
        if frame.address_ack:
            entry["ack"] += 1
        else:
            entry["nack"] += 1

    print()
    print("summary")
    for addr in sorted(counts):
        entry = counts[addr]
        marker = " expected" if addr in expected_set else ""
        print(
            f"0x{addr:02X}{marker}: frames={entry['frames']} "
            f"ack={entry['ack']} nack={entry['nack']}"
        )


def write_csv(path: Path, frames: Sequence[Frame], expected_addrs: Sequence[int]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=(
                "idx",
                "start_s",
                "stop_s",
                "address",
                "rw",
                "address_ack",
                "expected_address",
                "data",
                "data_ack",
                "errors",
            ),
        )
        writer.writeheader()
        for idx, frame in enumerate(frames):
            row = frame_to_dict(frame, expected_addrs)
            row["idx"] = idx
            row["data"] = " ".join(row["data"])  # type: ignore[index]
            row["data_ack"] = " ".join("ACK" if x else "NACK" for x in row["data_ack"])  # type: ignore[index]
            row["errors"] = "; ".join(row["errors"])  # type: ignore[index]
            writer.writerow(row)


def synth_capture() -> List[Sample]:
    rows: List[Sample] = []
    t = 0.0
    dt = 0.5e-6
    scl = 1
    sda = 1

    def emit(n: int = 1) -> None:
        nonlocal t
        for _ in range(n):
            rows.append(Sample(t=t, scl=scl, sda=sda))
            t += dt

    def set_lines(new_scl: Optional[int] = None, new_sda: Optional[int] = None, n: int = 2) -> None:
        nonlocal scl, sda
        if new_scl is not None:
            scl = new_scl
        if new_sda is not None:
            sda = new_sda
        emit(n)

    def start() -> None:
        set_lines(1, 1)
        set_lines(1, 0)
        set_lines(0, 0)

    def stop() -> None:
        set_lines(0, 0)
        set_lines(1, 0)
        set_lines(1, 1)

    def bit(bit_value: int) -> None:
        set_lines(0, bit_value)
        set_lines(1, bit_value)
        set_lines(0, bit_value)

    def byte(value: int, ack: bool) -> None:
        for shift in range(7, -1, -1):
            bit((value >> shift) & 1)
        bit(0 if ack else 1)

    emit(4)
    start()
    byte((0x18 << 1) | 0, False)
    stop()
    start()
    byte((0x19 << 1) | 0, True)
    byte(0x20, True)
    stop()
    emit(4)
    return rows


def self_test() -> None:
    frames = decode_i2c(synth_capture())
    if len(frames) != 2:
        raise AssertionError(f"expected 2 frames, got {len(frames)}")
    if frames[0].address != 0x18 or frames[0].address_ack is not False:
        raise AssertionError("0x18 NACK frame did not decode correctly")
    if frames[1].address != 0x19 or frames[1].address_ack is not True:
        raise AssertionError("0x19 ACK frame did not decode correctly")
    if not frames[1].bytes[1].ack or frames[1].bytes[1].value != 0x20:
        raise AssertionError("data byte did not decode correctly")
    print("SELF_TEST_PASS decode_waveforms_i2c")


def parse_addr_list(values: Sequence[str]) -> Tuple[int, ...]:
    out: List[int] = []
    for value in values:
        for piece in value.split(","):
            piece = piece.strip()
            if piece:
                out.append(int(piece, 0))
    return tuple(out)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, help="WaveForms raw Logic Analyzer CSV export")
    parser.add_argument("--scl", help="SCL column name, default auto-detects i2c_scl/SCL/DIO0")
    parser.add_argument("--sda", help="SDA column name, default auto-detects i2c_sda/SDA/DIO1")
    parser.add_argument("--time", help="time column name; if absent, pass --sample-rate")
    parser.add_argument("--sample-rate", type=float, help="sample rate in samples/s if the CSV has no time column")
    parser.add_argument("--threshold", type=float, default=0.5, help="numeric threshold for voltage-like CSV values")
    parser.add_argument("--glitch-ns", type=float, default=0.0, help="remove pulses shorter than this width")
    parser.add_argument(
        "--expect-addr",
        action="append",
        default=[],
        help="expected 7-bit address, e.g. 0x18. Can be repeated or comma-separated.",
    )
    parser.add_argument("--json-out", type=Path, help="write decoded frames as JSON")
    parser.add_argument("--csv-out", type=Path, help="write decoded frames as CSV")
    parser.add_argument("--self-test", action="store_true", help="run built-in synthetic capture test")
    args = parser.parse_args(argv)

    if args.self_test:
        self_test()
        return 0

    if not args.csv:
        parser.error("csv is required unless --self-test is used")

    expected_addrs = parse_addr_list(args.expect_addr) or DEFAULT_EXPECTED_ADDRS
    samples = read_waveforms_csv(args.csv, args.scl, args.sda, args.time, args.sample_rate, args.threshold)
    frames = decode_i2c(samples, glitch_s=args.glitch_ns * 1e-9)
    print_text(frames, expected_addrs)

    if args.json_out:
        args.json_out.write_text(
            json.dumps([frame_to_dict(frame, expected_addrs) for frame in frames], indent=2),
            encoding="utf-8",
        )
    if args.csv_out:
        write_csv(args.csv_out, frames, expected_addrs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
