#!/usr/bin/env python3
"""Strict GC032A packet stream inspector.

This tool does not hunt for the next line start after every error. It walks the
stream by the packet structure and reports the first place where the structure
breaks. That makes false FF FF FF 02 sequences inside image payload obvious.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


FRAME_START = b"\xff\xff\xff\x01"
LINE_START = b"\xff\xff\xff\x02"
LINE_DATA = b"\xff\xff\xff\x40"
FRAME_END = b"\xff\xff\xff\x00"
SYNC_PREFIX = b"\xff\xff\xff"


def read_auto(path: Path) -> bytes:
    raw = path.read_bytes()
    text = raw.decode("ascii", errors="ignore")
    hex_tokens = re.findall(r"\b[0-9a-fA-F]{2}\b", text)
    if hex_tokens and len(hex_tokens) > len(raw) // 8:
        return bytes(int(t, 16) for t in hex_tokens)
    return raw


def hex_bytes(data: bytes, max_len: int = 32) -> str:
    return " ".join(f"{b:02X}" for b in data[:max_len])


def sync_name(tag: int) -> str:
    return {
        0x00: "frame_end",
        0x01: "frame_start",
        0x02: "line_start",
        0x40: "line_data",
    }.get(tag, f"sync_{tag:02x}")


def find_next_sync(data: bytes, pos: int, limit: int = 2048) -> tuple[int, int] | None:
    end = min(len(data), pos + limit)
    i = data.find(SYNC_PREFIX, pos, end)
    if i < 0 or i + 4 > len(data):
        return None
    return i, data[i + 3]


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


def inspect(data: bytes, fallback_width: int, fallback_height: int, max_lines: int) -> int:
    frame_start = data.find(FRAME_START)
    if frame_start < 0:
        print("frame_start FF FF FF 01 not found")
        return 1
    if frame_start + 9 > len(data):
        print(f"short frame header at offset={frame_start}")
        return 1

    data_id = data[frame_start + 4]
    width = data[frame_start + 5] | (data[frame_start + 6] << 8)
    height = data[frame_start + 7] | (data[frame_start + 8] << 8)
    if width <= 0 or height <= 0 or width > 4096 or height > 4096:
        width = fallback_width
        height = fallback_height

    expected_payload = width * 2
    pos = frame_start + 9
    print(
        f"frame_start={frame_start} data_id=0x{data_id:02X} "
        f"size={width}x{height} expected_payload={expected_payload}"
    )

    valid = 0
    bad_crc = 0
    for expected_line in range(min(height, max_lines)):
        if pos + 6 > len(data):
            print(f"short stream before line {expected_line}, pos={pos}")
            break
        if data[pos:pos + 4] != LINE_START:
            print(f"break at line {expected_line}: expected LINE_START at {pos}")
            print(f"  actual: {hex_bytes(data[pos:pos + 32])}")
            nxt = find_next_sync(data, pos)
            if nxt:
                off, tag = nxt
                print(f"  next sync-like marker: offset={off} tag=0x{tag:02X} {sync_name(tag)}")
            break

        line_no = data[pos + 4] | (data[pos + 5] << 8)
        if line_no != expected_line:
            print(
                f"line number jump at offset={pos}: "
                f"expected={expected_line} got={line_no}"
            )
        data_pos = pos + 6
        if data_pos + 6 > len(data) or data[data_pos:data_pos + 4] != LINE_DATA:
            print(f"break after line_start {line_no}: expected LINE_DATA at {data_pos}")
            print(f"  actual: {hex_bytes(data[data_pos:data_pos + 32])}")
            break

        packet_size = data[data_pos + 4] | (data[data_pos + 5] << 8)
        payload_start = data_pos + 6
        payload_end = payload_start + packet_size
        if payload_end > len(data):
            print(f"short payload at line={line_no} packet_size={packet_size}")
            break
        if packet_size != expected_payload:
            print(
                f"size mismatch line={line_no} offset={data_pos}: "
                f"got={packet_size} expected={expected_payload}"
            )

        payload_full = data[data_pos:data_pos + 6] + data[payload_start:payload_end]
        has_crc = False
        crc = None
        crc_matches: list[str] = []
        if payload_end + 4 <= len(data) and data[payload_end:payload_end + 3] == SYNC_PREFIX:
            next_pos = payload_end
        elif payload_end + 5 <= len(data) and data[payload_end + 1:payload_end + 4] == SYNC_PREFIX:
            has_crc = True
            crc = data[payload_end]
            next_pos = payload_end + 1
            if crc8_07(payload_full) == crc:
                crc_matches.append("crc8-07")
            if (sum(payload_full) & 0xff) == crc:
                crc_matches.append("sum")
            x = 0
            for b in payload_full:
                x ^= b
            if x == crc:
                crc_matches.append("xor")
            if not crc_matches:
                bad_crc += 1
        else:
            print(f"break after payload line={line_no}: expected next sync at {payload_end} or {payload_end + 1}")
            print(f"  actual: {hex_bytes(data[payload_end:payload_end + 32])}")
            nxt = find_next_sync(data, payload_end)
            if nxt:
                off, tag = nxt
                print(f"  next sync-like marker: offset={off} tag=0x{tag:02X} {sync_name(tag)}")
            break

        if valid < 12 or line_no != expected_line or packet_size != expected_payload:
            print(
                f"line={line_no} off={pos} data_off={data_pos} "
                f"size={packet_size} crc={'0x%02X' % crc if has_crc else 'none'} "
                f"crc_match={','.join(crc_matches) or '-'} "
                f"next={next_pos}"
            )

        valid += 1
        pos = next_pos
        if pos + 4 <= len(data) and data[pos:pos + 4] == FRAME_END:
            print(f"frame_end at offset={pos}")
            break

    print(f"valid_lines_walked={valid} bad_crc={bad_crc} final_pos={pos}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--fallback-width", type=int, default=320)
    parser.add_argument("--fallback-height", type=int, default=240)
    parser.add_argument("--max-lines", type=int, default=260)
    args = parser.parse_args()

    data = read_auto(args.input)
    return inspect(data, args.fallback_width, args.fallback_height, args.max_lines)


if __name__ == "__main__":
    raise SystemExit(main())
