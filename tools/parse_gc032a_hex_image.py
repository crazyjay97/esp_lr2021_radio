#!/usr/bin/env python3
"""
Parse a GC032A UART hex dump into PPM images.

The expected stream is the camera's packed byte stream shown as text hex, e.g.:
    FF FF FF 01 00 80 02 E0 01
    FF FF FF 02 <line_lo> <line_hi> FF FF FF 40 00 05 <1280 bytes YVYU> <crc>

Outputs:
  - a full-size image with parsed lines placed at their line number
  - a compact strip image containing parsed lines in stream order
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


FRAME_START = b"\xff\xff\xff\x01"
LINE_START = b"\xff\xff\xff\x02"
LINE_DATA = b"\xff\xff\xff\x40"
FRAME_END = b"\xff\xff\xff\x00"


def load_hex_text(path: Path) -> bytes:
    text = path.read_bytes()
    tokens = re.findall(rb"(?<![0-9A-Fa-f])([0-9A-Fa-f]{2})(?![0-9A-Fa-f])", text)
    if not tokens:
        raise ValueError(f"no hex byte tokens found in {path}")
    return bytes(int(tok, 16) for tok in tokens)


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
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        f.write(rgb)


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


def parse_frame(data: bytes,
                fallback_width: int,
                fallback_height: int,
                crc_mode: str,
                require_crc: bool):
    frame_start = data.find(FRAME_START)
    if frame_start < 0:
        raise ValueError("frame start FF FF FF 01 not found")

    header = frame_start + len(FRAME_START)
    if header + 5 > len(data):
        raise ValueError("truncated frame header")

    width = data[header + 1] | (data[header + 2] << 8)
    height = data[header + 3] | (data[header + 4] << 8)
    if not (0 < width <= 4096 and 0 < height <= 4096):
        width, height = fallback_width, fallback_height

    expected_line_bytes = width * 2
    full_rgb = bytearray(width * height * 3)
    strip_lines: list[bytes] = []
    used_lines: list[int] = []
    short_lines = 0
    malformed = 0
    short_packets = 0
    size_mismatch = 0
    crc_present = 0
    crc_ok: dict[str, int] = {"crc8-07": 0, "sum": 0, "xor": 0}
    crc_bad = 0
    crc_missing = 0

    pos = header + 5
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
    parser = argparse.ArgumentParser(description="Parse GC032A UART hex dump to PPM.")
    parser.add_argument("input", nargs="?", default="hex_img.txt")
    parser.add_argument("-o", "--output-prefix", default="hex_img")
    parser.add_argument("--fallback-width", type=int, default=640)
    parser.add_argument("--fallback-height", type=int, default=480)
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

    data = load_hex_text(Path(args.input))
    parsed = parse_frame(
        data,
        args.fallback_width,
        args.fallback_height,
        args.crc_mode,
        args.require_crc,
    )
    width = parsed["width"]
    height = parsed["height"]
    used_lines = parsed["used_lines"]

    full_path = Path(f"{args.output_prefix}_full.ppm")
    strip_path = Path(f"{args.output_prefix}_strip.ppm")
    write_ppm(full_path, width, height, parsed["full_rgb"])
    strip_height = max(1, len(used_lines))
    write_ppm(strip_path, width, strip_height, parsed["strip_rgb"] or bytes(width * 3))

    print(f"bytes={len(data)} frame_start={parsed['frame_start']}")
    print(f"size={width}x{height} lines={len(used_lines)}")
    if used_lines:
        print(f"line_range={min(used_lines)}..{max(used_lines)}")
        print(f"first_lines={used_lines[:12]}")
        print(f"last_lines={used_lines[-12:]}")
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
