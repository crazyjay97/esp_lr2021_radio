#!/usr/bin/env python3
"""
Capture GC032A UART output and convert it to PPM when possible.

The firmware may dump either:
  - a ready-made binary PPM image, starting with "P6\\n", or
  - the GC032A packet stream:
      FF FF FF 01 <data_id> <width_lo> <width_hi> <height_lo> <height_hi>
      FF FF FF 02 <line_lo> <line_hi>
      FF FF FF 40 <packet_size_lo> <packet_size_hi> <YVYU line data> <crc>
      ...
      FF FF FF 00
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

import serial


FRAME_START = b"\xff\xff\xff\x01"
LINE_START = b"\xff\xff\xff\x02"
LINE_DATA = b"\xff\xff\xff\x40"
FRAME_END = b"\xff\xff\xff\x00"


def clamp(v: int) -> int:
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def yuv_to_rgb(y: int, u: int, v: int) -> tuple[int, int, int]:
    c = max(0, y - 16)
    d = u - 128
    e = v - 128
    r = (298 * c + 409 * e + 128) >> 8
    g = (298 * c - 100 * d - 208 * e + 128) >> 8
    b = (298 * c + 516 * d + 128) >> 8
    return clamp(r), clamp(g), clamp(b)


def yvyu_line_to_rgb(line: bytes, width: int) -> bytes:
    out = bytearray(width * 3)
    src_len = min(len(line), width * 2)
    src_len &= ~3
    x = 0
    for i in range(0, src_len, 4):
        y0 = line[i]
        v = line[i + 1]
        y1 = line[i + 2]
        u = line[i + 3]
        r, g, b = yuv_to_rgb(y0, u, v)
        off = x * 3
        out[off:off + 3] = bytes((r, g, b))
        r, g, b = yuv_to_rgb(y1, u, v)
        out[off + 3:off + 6] = bytes((r, g, b))
        x += 2
    return bytes(out)


def write_ppm(path: Path, width: int, height: int, rgb: bytes) -> None:
    with path.open("wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        f.write(rgb)


def write_hex(path: Path, data: bytes) -> None:
    with path.open("w", encoding="ascii") as f:
        for off in range(0, len(data), 16):
            chunk = data[off:off + 16]
            f.write(" ".join(f"{b:02X}" for b in chunk))
            f.write("\n")


def repack_2bit_lsb(data: bytes, offset_pairs: int) -> bytes:
    if offset_pairs <= 0:
        return data
    out = bytearray()
    acc: list[int] = []
    skipped = 0
    for b in data:
        for pair in (b & 0x03, (b >> 2) & 0x03, (b >> 4) & 0x03, (b >> 6) & 0x03):
            if skipped < offset_pairs:
                skipped += 1
                continue
            acc.append(pair)
            if len(acc) == 4:
                out.append(acc[0] | (acc[1] << 2) | (acc[2] << 4) | (acc[3] << 6))
                acc.clear()
    return bytes(out)


def packet_score(data: bytes) -> tuple[int, int, int]:
    valid = 0
    frame_markers = data.count(FRAME_START) + data.count(FRAME_END)
    pos = 0
    while True:
        line_start = data.find(LINE_START, pos)
        if line_start < 0:
            break
        if line_start + 12 <= len(data) and data[line_start + 6:line_start + 10] == LINE_DATA:
            packet_size = data[line_start + 10] | (data[line_start + 11] << 8)
            if 0 < packet_size <= 8192 and (packet_size % 2) == 0:
                valid += 1
        pos = line_start + 1
    return frame_markers, valid, data.count(LINE_START)


def choose_repack_phase(data: bytes) -> tuple[int, bytes]:
    best_phase = 0
    best_data = data
    best_score = packet_score(data)
    for phase in range(1, 4):
        candidate = repack_2bit_lsb(data, phase)
        score = packet_score(candidate)
        if score > best_score:
            best_phase = phase
            best_data = candidate
            best_score = score
    return best_phase, best_data


def capture_serial(port: str,
                   baud: int,
                   max_bytes: int,
                   timeout_s: float,
                   idle_timeout_s: float,
                   reset_input: bool) -> bytes:
    data = bytearray()
    start = time.monotonic()
    last_rx: float | None = None
    next_report = start + 1.0

    with serial.Serial(port, baudrate=baud, timeout=0.05) as ser:
        if reset_input:
            ser.reset_input_buffer()
        while len(data) < max_bytes:
            now = time.monotonic()
            if now - start >= timeout_s:
                break
            if last_rx is not None and now - last_rx >= idle_timeout_s:
                break

            chunk = ser.read(min(4096, max_bytes - len(data)))
            if chunk:
                data.extend(chunk)
                last_rx = time.monotonic()

            if now >= next_report:
                print(f"captured={len(data)} bytes", file=sys.stderr)
                next_report = now + 1.0

    return bytes(data)


def parse_ppm_header(data: bytes) -> tuple[int, int, int, int] | None:
    start = data.find(b"P6\n")
    if start < 0:
        return None

    header_end = start
    fields: list[bytes] = []
    pos = start + 3
    while len(fields) < 3 and pos < len(data):
        if data[pos:pos + 1] == b"#":
            newline = data.find(b"\n", pos)
            if newline < 0:
                return None
            pos = newline + 1
            continue
        match = re.match(rb"\s*([0-9]+)", data[pos:])
        if not match:
            return None
        fields.append(match.group(1))
        pos += match.end()
        header_end = pos

    if len(fields) != 3:
        return None
    width = int(fields[0])
    height = int(fields[1])
    maxval = int(fields[2])
    if width <= 0 or height <= 0 or maxval != 255:
        return None

    while header_end < len(data) and data[header_end:header_end + 1] in b" \t\r\n":
        header_end += 1
    return start, header_end, width, height


def extract_binary_ppm(data: bytes) -> tuple[int, int, bytes] | None:
    header = parse_ppm_header(data)
    if header is None:
        return None
    start, header_end, width, height = header
    image_len = width * height * 3
    if header_end + image_len > len(data):
        return None
    return width, height, data[start:header_end + image_len]


def extract_incomplete_binary_ppm(data: bytes) -> tuple[int, int, int, int, bytes] | None:
    header = parse_ppm_header(data)
    if header is None:
        return None
    _start, header_end, width, height = header
    image_len = width * height * 3
    payload = data[header_end:header_end + image_len]
    if not payload or len(payload) >= image_len:
        return None
    padded = payload + bytes(image_len - len(payload))
    rebuilt = f"P6\n{width} {height}\n255\n".encode("ascii") + padded
    return width, height, len(payload), image_len, rebuilt


def find_ppm_header_fragment(data: bytes) -> str | None:
    for marker in (b"\n255\n", b"255\n", b" 480\n255\n", b" 240\n255\n"):
        pos = data.find(marker)
        if pos >= 0:
            begin = max(0, pos - 32)
            end = min(len(data), pos + len(marker) + 16)
            return data[begin:end].decode("ascii", errors="replace")
    return None


def extract_partial_ppm(data: bytes, width: int, height: int) -> bytes | None:
    markers = [
        f"{width} {height}\n255\n".encode("ascii"),
        f"{str(width)[1:]} {height}\n255\n".encode("ascii") if width >= 10 else b"",
        f" {height}\n255\n".encode("ascii"),
        b"\n255\n",
    ]
    for marker in markers:
        if not marker:
            continue
        pos = data.find(marker)
        if pos < 0:
            continue
        payload_start = pos + len(marker)
        payload = data[payload_start:payload_start + width * height * 3]
        if not payload:
            continue
        if len(payload) < width * height * 3:
            payload = payload + bytes(width * height * 3 - len(payload))
        return f"P6\n{width} {height}\n255\n".encode("ascii") + payload
    return None


def sync_at(data: bytes, pos: int, tag: int) -> bool:
    return pos + 4 <= len(data) and data[pos:pos + 4] == b"\xff\xff\xff" + bytes((tag,))


def any_sync_at(data: bytes, pos: int) -> bool:
    return (
        sync_at(data, pos, 0x01)
        or sync_at(data, pos, 0x02)
        or sync_at(data, pos, 0x40)
        or sync_at(data, pos, 0x00)
    )


def packet_sync_at(data: bytes, pos: int) -> bool:
    return any_sync_at(data, pos)


def crc8_07(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
    return crc


def checksum8(data: bytes) -> int:
    return sum(data) & 0xff


def xor8(data: bytes) -> int:
    value = 0
    for b in data:
        value ^= b
    return value


def crc_value(data: bytes, mode: str) -> int | None:
    if mode == "crc8-07":
        return crc8_07(data)
    if mode == "sum":
        return checksum8(data)
    if mode == "xor":
        return xor8(data)
    return None


def infer_packet_dimensions(data: bytes,
                            fallback_width: int,
                            fallback_height: int) -> tuple[int, int]:
    sizes: dict[int, int] = {}
    pos = 0
    while True:
        line_start = data.find(LINE_START, pos)
        if line_start < 0:
            break
        if line_start + 12 <= len(data) and data[line_start + 6:line_start + 10] == LINE_DATA:
            packet_size = data[line_start + 10] | (data[line_start + 11] << 8)
            if packet_size > 0 and (packet_size % 2) == 0:
                sizes[packet_size] = sizes.get(packet_size, 0) + 1
        pos = line_start + 1

    if not sizes:
        return fallback_width, fallback_height
    packet_size = max(sizes, key=sizes.get)
    width = packet_size // 2
    if not (0 < width <= 4096):
        width = fallback_width
    return width, fallback_height


def parse_gc032a_packets(data: bytes,
                         fallback_width: int,
                         fallback_height: int,
                         crc_mode: str,
                         require_crc: bool) -> dict[str, object]:
    frame_start = data.find(FRAME_START)
    if frame_start >= 0:
        header = frame_start + len(FRAME_START)
        if header + 5 > len(data):
            raise ValueError("truncated frame header")

        width = data[header + 1] | (data[header + 2] << 8)
        height = data[header + 3] | (data[header + 4] << 8)
        if not (0 < width <= 4096 and 0 < height <= 4096):
            width, height = infer_packet_dimensions(data, fallback_width, fallback_height)
        pos = header + 5
    else:
        if data.find(LINE_START) < 0:
            raise ValueError("frame start FF FF FF 01 not found")
        width, height = fallback_width, fallback_height
        width, height = infer_packet_dimensions(data, width, height)
        pos = 0

    expected_line_bytes = width * 2
    full_rgb = bytearray(width * height * 3)
    strip_lines: list[bytes] = []
    used_lines: list[int] = []
    malformed = 0
    short_lines = 0
    short_packets = 0
    size_mismatch = 0
    crc_present = 0
    crc_ok: dict[str, int] = {"crc8-07": 0, "sum": 0, "xor": 0}
    crc_bad = 0
    crc_missing = 0

    while True:
        line_start = data.find(LINE_START, pos)
        if line_start < 0:
            break
        if line_start + 12 > len(data):
            malformed += 1
            break

        line_no = data[line_start + 4] | (data[line_start + 5] << 8)
        if data[line_start + 6:line_start + 10] != LINE_DATA:
            malformed += 1
            pos = line_start + 1
            continue

        packet_size = data[line_start + 10] | (data[line_start + 11] << 8)
        payload_start = line_start + 12
        payload_end = payload_start + packet_size
        if payload_end > len(data):
            short_packets += 1
            break

        if packet_size != expected_line_bytes:
            size_mismatch += 1

        has_crc = False
        if packet_sync_at(data, payload_end):
            has_crc = False
        elif payload_end < len(data) and packet_sync_at(data, payload_end + 1):
            has_crc = True
        elif payload_end + len(LINE_START) <= len(data):
            malformed += 1
            pos = line_start + 1
            continue

        payload_full = data[payload_start:payload_end]
        if has_crc:
            crc_present += 1
            actual_crc = data[payload_end]
            for mode in crc_ok:
                if crc_value(payload_full, mode) == actual_crc:
                    crc_ok[mode] += 1
        else:
            crc_missing += 1

        crc_valid = True
        if require_crc and not has_crc:
            crc_valid = False
        elif crc_mode != "auto":
            expected_crc = crc_value(payload_full, crc_mode)
            if has_crc and expected_crc is not None and expected_crc != data[payload_end]:
                crc_valid = False
                crc_bad += 1

        if not crc_valid:
            pos = payload_end + (1 if has_crc else 0)
            continue

        payload = payload_full[:min(packet_size, expected_line_bytes)]
        if len(payload) < expected_line_bytes:
            short_lines += 1
        rgb_line = yvyu_line_to_rgb(payload, width)
        strip_lines.append(rgb_line)
        used_lines.append(line_no)

        if line_no < height:
            dst = line_no * width * 3
            full_rgb[dst:dst + width * 3] = rgb_line

        pos = payload_end + (1 if has_crc else 0)

    return {
        "width": width,
        "height": height,
        "frame_start": frame_start,
        "frame_end": data.find(FRAME_END, pos),
        "used_lines": used_lines,
        "short_lines": short_lines,
        "short_packets": short_packets,
        "size_mismatch": size_mismatch,
        "crc_present": crc_present,
        "crc_ok": crc_ok,
        "crc_bad": crc_bad,
        "crc_missing": crc_missing,
        "malformed": malformed,
        "full_rgb": bytes(full_rgb),
        "strip_rgb": b"".join(strip_lines),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture GC032A UART data.")
    parser.add_argument("-p", "--port", help="serial port, e.g. /dev/cu.SLAB_USBtoUART")
    parser.add_argument("--input", help="parse an existing raw capture instead of reading serial")
    parser.add_argument("-b", "--baud", type=int, default=2_000_000)
    parser.add_argument("-o", "--output-prefix", default="uart_capture")
    parser.add_argument("--max-bytes", type=int, default=2 * 1024 * 1024)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--idle-timeout", type=float, default=1.5)
    parser.add_argument("--fallback-width", type=int, default=320)
    parser.add_argument("--fallback-height", type=int, default=240)
    parser.add_argument("--no-hex", action="store_true", help="do not write a text hex dump")
    parser.add_argument(
        "--capture-only",
        action="store_true",
        help="only write raw/hex capture files; skip PPM and packet parsing",
    )
    parser.add_argument(
        "--write-repacked",
        action="store_true",
        help="write the phase-adjusted packet stream as *_repacked.raw.bin and *_repacked.hex.txt",
    )
    parser.add_argument(
        "--reset-input",
        action="store_true",
        help="clear pending serial input right after opening the port",
    )
    parser.add_argument(
        "--extract-partial-ppm",
        action="store_true",
        help="rebuild a partial PPM capture using fallback width/height",
    )
    parser.add_argument(
        "--repack-phase",
        choices=("auto", "0", "1", "2", "3"),
        default="auto",
        help="shift packed 2-bit stream before packet parsing",
    )
    parser.add_argument(
        "--crc-mode",
        choices=("auto", "crc8-07", "sum", "xor"),
        default="auto",
        help="CRC/checksum algorithm to enforce; auto only reports candidate matches",
    )
    parser.add_argument(
        "--require-crc",
        action="store_true",
        help="discard packets that do not have one CRC byte before the next sync",
    )
    args = parser.parse_args()

    if not args.port and not args.input:
        parser.error("one of --port or --input is required")

    prefix = Path(args.output_prefix)
    if args.input:
        data = Path(args.input).read_bytes()
    else:
        data = capture_serial(
            args.port,
            args.baud,
            args.max_bytes,
            args.timeout,
            args.idle_timeout,
            args.reset_input,
        )
    if not data:
        print("no data captured", file=sys.stderr)
        return 1

    raw_path = prefix.with_suffix(".raw.bin")
    if args.input:
        print(f"loaded {len(data)} bytes <- {args.input}")
    else:
        raw_path.write_bytes(data)
        print(f"captured {len(data)} bytes -> {raw_path}")

    if not args.no_hex:
        hex_path = prefix.with_suffix(".hex.txt")
        write_hex(hex_path, data)
        print(f"wrote {hex_path}")

    if args.capture_only:
        return 0

    ppm = extract_binary_ppm(data)
    if ppm is not None:
        width, height, ppm_data = ppm
        ppm_path = prefix.with_suffix(".ppm")
        ppm_path.write_bytes(ppm_data)
        print(f"extracted PPM {width}x{height} -> {ppm_path}")
        return 0

    incomplete_ppm = extract_incomplete_binary_ppm(data)
    if incomplete_ppm is not None:
        width, height, got, expected, ppm_data = incomplete_ppm
        ppm_path = prefix.with_name(prefix.name + "_partial.ppm")
        ppm_path.write_bytes(ppm_data)
        print(f"incomplete PPM {width}x{height}: got {got}/{expected} payload bytes")
        print(f"rebuilt partial PPM -> {ppm_path}")
        return 2

    if args.repack_phase == "auto":
        phase, repacked = choose_repack_phase(data)
    else:
        phase = int(args.repack_phase)
        repacked = repack_2bit_lsb(data, phase)
    if phase:
        print(f"repacked 2-bit stream with phase={phase}")
        data = repacked
    if args.write_repacked:
        repacked_prefix = prefix.with_name(prefix.name + "_repacked")
        repacked_raw_path = repacked_prefix.with_suffix(".raw.bin")
        repacked_raw_path.write_bytes(data)
        print(f"wrote {repacked_raw_path}")
        if not args.no_hex:
            repacked_hex_path = repacked_prefix.with_suffix(".hex.txt")
            write_hex(repacked_hex_path, data)
            print(f"wrote {repacked_hex_path}")

    try:
        parsed = parse_gc032a_packets(
            data,
            args.fallback_width,
            args.fallback_height,
            args.crc_mode,
            args.require_crc,
        )
    except ValueError as exc:
        fragment = find_ppm_header_fragment(data)
        if fragment:
            if args.extract_partial_ppm:
                rebuilt = extract_partial_ppm(data, args.fallback_width, args.fallback_height)
                if rebuilt is not None:
                    ppm_path = prefix.with_name(prefix.name + "_partial.ppm")
                    ppm_path.write_bytes(rebuilt)
                    print(f"rebuilt partial PPM -> {ppm_path}")
            print(f"{exc}", file=sys.stderr)
            print("captured data looks like a partial PPM stream:", file=sys.stderr)
            print(repr(fragment), file=sys.stderr)
            print("Start this script before resetting the board, or rerun with a larger timeout.", file=sys.stderr)
            return 2
        raise
    width = int(parsed["width"])
    height = int(parsed["height"])
    used_lines = list(parsed["used_lines"])

    full_path = prefix.with_name(prefix.name + "_full.ppm")
    strip_path = prefix.with_name(prefix.name + "_strip.ppm")
    write_ppm(full_path, width, height, bytes(parsed["full_rgb"]))
    strip_height = max(1, len(used_lines))
    write_ppm(strip_path, width, strip_height, bytes(parsed["strip_rgb"]) or bytes(width * 3))

    print(f"packet frame_start={parsed['frame_start']} frame_end={parsed['frame_end']}")
    print(f"packet size={width}x{height} lines={len(used_lines)}")
    if used_lines:
        print(f"line_range={min(used_lines)}..{max(used_lines)}")
    print(
        f"malformed={parsed['malformed']} short_lines={parsed['short_lines']} "
        f"short_packets={parsed['short_packets']} size_mismatch={parsed['size_mismatch']}"
    )
    print(
        f"crc_present={parsed['crc_present']} crc_missing={parsed['crc_missing']} "
        f"crc_bad={parsed['crc_bad']} crc_ok={parsed['crc_ok']}"
    )
    print(f"wrote {full_path}")
    print(f"wrote {strip_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
