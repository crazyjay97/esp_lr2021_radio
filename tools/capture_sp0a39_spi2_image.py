#!/usr/bin/env python3
"""
Capture an SP0A39 image from UART and write a 640x480 PGM image.

Current firmware sends a decoded binary PGM stream directly:
  P5\n640 480\n255\n<307200 grayscale bytes>

The older raw SPI packet format is still supported with --format packet:

Expected sensor packet format:
  FF FF FF 01 <5 frame header bytes>
  FF FF FF 02 <8 line header bytes> <640 grayscale bytes>
  ... 480 lines total ...
  FF FF FF 00
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


FRAME_START = b"\xff\xff\xff\x01"
LINE_START = b"\xff\xff\xff\x02"
FRAME_END = b"\xff\xff\xff\x00"

FRAME_HEADER_BYTES = 9
LINE_HEADER_BYTES = 12
FRAME_END_BYTES = 4
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_BAUD = 2_000_000


def expected_pgm_bytes(width: int, height: int) -> int:
    return len(pgm_header(width, height)) + width * height


def expected_packet_bytes(width: int, height: int) -> int:
    return FRAME_HEADER_BYTES + (width + LINE_HEADER_BYTES) * height + FRAME_END_BYTES


def pgm_header(width: int, height: int) -> bytes:
    return f"P5\n{width} {height}\n255\n".encode("ascii")


def hexdump(data: bytes, limit: int = 32) -> str:
    return " ".join(f"{b:02x}" for b in data[:limit])


def capture_pgm(port: str,
                baud: int,
                width: int,
                height: int,
                timeout_s: float,
                reset_input: bool) -> bytes:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

    header = pgm_header(width, height)
    total_bytes = len(header) + width * height
    start = time.monotonic()
    next_report = start + 1.0
    buf = bytearray()
    sync_at = -1

    with serial.Serial(port, baudrate=baud, timeout=0.05) as ser:
        if reset_input:
            ser.reset_input_buffer()

        while True:
            now = time.monotonic()
            if now - start > timeout_s:
                raise TimeoutError(
                    f"timeout after {timeout_s:.1f}s, captured={len(buf)} bytes, sync_at={sync_at}"
                )

            chunk = ser.read(8192)
            if chunk:
                buf.extend(chunk)
                if sync_at < 0:
                    sync_at = buf.find(header)
                    if sync_at > 4096:
                        del buf[:sync_at]
                        sync_at = 0

                if sync_at >= 0 and len(buf) - sync_at >= total_bytes:
                    return bytes(buf[sync_at:sync_at + total_bytes])

            if now >= next_report:
                if sync_at >= 0:
                    have = len(buf) - sync_at
                    print(f"synced, pgm bytes={have}/{total_bytes}", file=sys.stderr)
                else:
                    print(f"waiting for PGM header, captured={len(buf)} bytes", file=sys.stderr)
                next_report = now + 1.0


def capture_packet(port: str,
                   baud: int,
                   packet_bytes: int,
                   timeout_s: float,
                   reset_input: bool,
                   allow_short: bool) -> bytes:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc

    start = time.monotonic()
    next_report = start + 1.0
    buf = bytearray()
    sync_at = -1

    with serial.Serial(port, baudrate=baud, timeout=0.05) as ser:
        if reset_input:
            ser.reset_input_buffer()

        while True:
            now = time.monotonic()
            if now - start > timeout_s:
                if sync_at >= 0 and allow_short and len(buf) > sync_at:
                    short = bytes(buf[sync_at:])
                    print(
                        f"warning: timeout with short synced packet: {len(short)}/{packet_bytes} bytes",
                        file=sys.stderr,
                    )
                    return short
                raise TimeoutError(
                    f"timeout after {timeout_s:.1f}s, captured={len(buf)} bytes, sync_at={sync_at}"
                )

            chunk = ser.read(8192)
            if chunk:
                buf.extend(chunk)
                if sync_at < 0:
                    sync_at = buf.find(FRAME_START)
                    if sync_at > 4096:
                        del buf[:sync_at]
                        sync_at = 0

                if sync_at >= 0 and len(buf) - sync_at >= packet_bytes:
                    return bytes(buf[sync_at:sync_at + packet_bytes])

            if now >= next_report:
                if sync_at >= 0:
                    have = len(buf) - sync_at
                    print(f"synced, packet bytes={have}/{packet_bytes}", file=sys.stderr)
                else:
                    print(f"waiting for frame header, captured={len(buf)} bytes", file=sys.stderr)
                next_report = now + 1.0


def parse_packet(packet: bytes,
                 width: int,
                 height: int,
                 allow_resync: bool) -> tuple[bytes, dict[str, int | str]]:
    if not packet.startswith(FRAME_START):
        pos = packet.find(FRAME_START)
        if pos < 0:
            raise ValueError("frame header FF FF FF 01 not found")
        packet = packet[pos:]

    image = bytearray(width * height)
    pos = FRAME_HEADER_BYTES
    lines_ok = 0
    lines_resynced = 0
    lines_bad = 0

    for y in range(height):
        if pos + LINE_HEADER_BYTES > len(packet):
            lines_bad += height - y
            break

        if packet[pos:pos + 4] != LINE_START:
            found = -1
            if allow_resync:
                found = packet.find(LINE_START, pos, min(len(packet), pos + 256))
            if found >= 0:
                pos = found
                lines_resynced += 1
            else:
                lines_bad += 1
                pos += LINE_HEADER_BYTES + width
                continue

        payload_at = pos + LINE_HEADER_BYTES
        payload_end = payload_at + width
        if payload_end > len(packet):
            lines_bad += height - y
            break

        dst = y * width
        image[dst:dst + width] = packet[payload_at:payload_end]
        lines_ok += 1
        pos = payload_end

    frame_end = "missing"
    if pos + FRAME_END_BYTES <= len(packet):
        tail = packet[pos:pos + FRAME_END_BYTES]
        frame_end = "ok" if tail == FRAME_END else f"unexpected:{hexdump(tail, 4)}"

    stats: dict[str, int | str] = {
        "lines_ok": lines_ok,
        "lines_resynced": lines_resynced,
        "lines_bad": lines_bad,
        "frame_end": frame_end,
        "payload_bytes": len(image),
    }
    return bytes(image), stats


def write_pgm(path: Path, width: int, height: int, image: bytes) -> None:
    with path.open("wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(image)


def maybe_write_png(path: Path, width: int, height: int, image: bytes) -> Path | None:
    try:
        from PIL import Image
    except ImportError:
        return None

    png_path = path.with_suffix(".png")
    Image.frombytes("L", (width, height), image).save(png_path)
    return png_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port connected to camera UART2")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--out", type=Path, default=Path("sp0a39.pgm"))
    parser.add_argument("--raw", type=Path, help="optional raw SPI packet output")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--format", choices=("pgm", "packet"), default="pgm",
                        help="UART payload format; firmware now defaults to decoded PGM")
    parser.add_argument("--no-resync", action="store_true", help="do not scan forward for shifted line headers")
    parser.add_argument("--strict-length", action="store_true", help="fail instead of writing a partial image on timeout")
    parser.add_argument("--reset-input", action="store_true", help="discard already-buffered serial data first")
    args = parser.parse_args()

    if args.format == "pgm":
        total_len = expected_pgm_bytes(args.width, args.height)
        print(f"capturing {total_len} PGM bytes from {args.port} at {args.baud} baud", file=sys.stderr)
        pgm = capture_pgm(args.port, args.baud, args.width, args.height,
                          args.timeout, args.reset_input)

        if args.raw:
            args.raw.write_bytes(pgm)
            print(f"wrote raw PGM stream: {args.raw} ({len(pgm)} bytes)", file=sys.stderr)

        header = pgm_header(args.width, args.height)
        image = pgm[len(header):]
        args.out.write_bytes(pgm)
        png_path = maybe_write_png(args.out, args.width, args.height, image)

        print(f"pgm header: {header!r}")
        print(f"wrote {args.out} ({len(image)} image bytes)")
        if png_path:
            print(f"wrote {png_path}")
        else:
            print("PNG not written; install Pillow if you want automatic PNG output", file=sys.stderr)
        return 0

    packet_len = expected_packet_bytes(args.width, args.height)
    print(f"capturing {packet_len} packet bytes from {args.port} at {args.baud} baud", file=sys.stderr)
    packet = capture_packet(args.port, args.baud, packet_len, args.timeout,
                            args.reset_input, allow_short=not args.strict_length)

    if args.raw:
        args.raw.write_bytes(packet)
        print(f"wrote raw packet: {args.raw} ({len(packet)} bytes)", file=sys.stderr)

    image, stats = parse_packet(packet, args.width, args.height, allow_resync=not args.no_resync)
    write_pgm(args.out, args.width, args.height, image)
    png_path = maybe_write_png(args.out, args.width, args.height, image)

    print(f"frame header: {hexdump(packet, FRAME_HEADER_BYTES)}")
    print(
        "lines_ok={lines_ok} lines_resynced={lines_resynced} "
        "lines_bad={lines_bad} frame_end={frame_end}".format(**stats)
    )
    print(f"wrote {args.out} ({len(image)} image bytes)")
    if png_path:
        print(f"wrote {png_path}")
    else:
        print("PNG not written; install Pillow if you want automatic PNG output", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
