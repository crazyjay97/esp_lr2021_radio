#!/usr/bin/env python3
"""Read CAMU camera frames from UART and save or preview them.

The ESP32 sender uses this packet header, little-endian:
    magic[4]="CAMU", type u8, flags u8, header_len u16,
    seq u32, payload_len u32, checksum u32
"""

from __future__ import annotations

import argparse
import os
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

try:
    import serial
except ImportError as exc:  # pragma: no cover - user environment dependent
    raise SystemExit("missing dependency: install pyserial with `python3 -m pip install pyserial`") from exc

try:
    from PIL import Image, ImageTk
except ImportError:  # pragma: no cover - user environment dependent
    Image = None
    ImageTk = None


MAGIC = b"CAMU"
HEADER_FMT = "<4sBBHIII"
HEADER_LEN = struct.calcsize(HEADER_FMT)

PKT_STATUS = 1
PKT_FRAME_START = 2
PKT_FRAME_DATA = 3
PKT_FRAME_END = 4

FNV1A_OFFSET = 2166136261
FNV1A_PRIME = 16777619


@dataclass(frozen=True)
class Packet:
    pkt_type: int
    flags: int
    seq: int
    payload: bytes


@dataclass(frozen=True)
class Frame:
    number: int
    width: int
    height: int
    raw: bytes
    rgb: bytes
    raw_len: int


def fnv1a(data: bytes) -> int:
    h = FNV1A_OFFSET
    for byte in data:
        h ^= byte
        h = (h * FNV1A_PRIME) & 0xFFFFFFFF
    return h


def read_exact(stream: BinaryIO, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = stream.read(size - len(chunks))
        if not chunk:
            raise EOFError("serial read timed out or port closed")
        chunks.extend(chunk)
    return bytes(chunks)


def read_packet(stream: BinaryIO, max_payload: int) -> Packet:
    window = bytearray()
    while len(window) < len(MAGIC):
        b = stream.read(1)
        if not b:
            raise EOFError("serial read timed out while waiting for CAMU magic")
        window.extend(b)
    while bytes(window) != MAGIC:
        b = stream.read(1)
        if not b:
            raise EOFError("serial read timed out while seeking CAMU magic")
        window = window[1:] + b

    rest = read_exact(stream, HEADER_LEN - len(MAGIC))
    magic, pkt_type, flags, header_len, seq, payload_len, checksum = struct.unpack(
        HEADER_FMT, MAGIC + rest
    )
    if magic != MAGIC:
        raise RuntimeError("internal CAMU sync error")
    if header_len < HEADER_LEN:
        raise ValueError(f"bad CAMU header_len {header_len}")
    if payload_len > max_payload:
        raise ValueError(f"CAMU payload too large: {payload_len} > {max_payload}")

    if header_len > HEADER_LEN:
        read_exact(stream, header_len - HEADER_LEN)

    payload = read_exact(stream, payload_len)
    actual = fnv1a(payload)
    if actual != checksum:
        raise ValueError(f"checksum mismatch: got 0x{actual:08x}, expected 0x{checksum:08x}")
    return Packet(pkt_type=pkt_type, flags=flags, seq=seq, payload=payload)


def parse_meta(payload: bytes) -> dict[str, str]:
    text = payload.decode("ascii", errors="replace").strip()
    meta: dict[str, str] = {}
    for field in text.split(";"):
        if "=" in field:
            key, value = field.split("=", 1)
            meta[key.strip()] = value.strip()
    return meta


def rgb565le_to_rgb888(raw: bytes, width: int, height: int) -> bytes:
    expected = width * height * 2
    if len(raw) != expected:
        raise ValueError(f"frame has {len(raw)} bytes, expected {expected} for {width}x{height} RGB565")

    out = bytearray(width * height * 3)
    j = 0
    for i in range(0, len(raw), 2):
        px = raw[i] | (raw[i + 1] << 8)
        r5 = (px >> 11) & 0x1F
        g6 = (px >> 5) & 0x3F
        b5 = px & 0x1F
        out[j] = (r5 << 3) | (r5 >> 2)
        out[j + 1] = (g6 << 2) | (g6 >> 4)
        out[j + 2] = (b5 << 3) | (b5 >> 2)
        j += 3
    return bytes(out)


def clamp_u8(value: int) -> int:
    return 0 if value < 0 else 255 if value > 255 else value


def yuv_to_rgb(y: int, u: int, v: int) -> tuple[int, int, int]:
    c = y - 16
    d = u - 128
    e = v - 128
    r = (298 * c + 409 * e + 128) >> 8
    g = (298 * c - 100 * d - 208 * e + 128) >> 8
    b = (298 * c + 516 * d + 128) >> 8
    return clamp_u8(r), clamp_u8(g), clamp_u8(b)


def yvyu_to_rgb888(raw: bytes, width: int, height: int) -> bytes:
    expected = width * height * 2
    if len(raw) != expected:
        raise ValueError(f"frame has {len(raw)} bytes, expected {expected} for {width}x{height} YVYU")
    if width % 2:
        raise ValueError("YVYU frame width must be even")

    out = bytearray(width * height * 3)
    j = 0
    for i in range(0, len(raw), 4):
        y0 = raw[i]
        v = raw[i + 1]
        y1 = raw[i + 2]
        u = raw[i + 3]
        out[j:j + 3] = bytes(yuv_to_rgb(y0, u, v))
        out[j + 3:j + 6] = bytes(yuv_to_rgb(y1, u, v))
        j += 6
    return bytes(out)


def save_ppm(path: Path, frame: Frame) -> None:
    with path.open("wb") as f:
        f.write(f"P6\n{frame.width} {frame.height}\n255\n".encode("ascii"))
        f.write(frame.rgb)


def save_frame(path_pattern: str, frame: Frame) -> Path:
    path = Path(path_pattern.format(frame=frame.number, time=int(time.time())))
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() == ".png":
        if Image is None:
            fallback = path.with_suffix(".ppm")
            save_ppm(fallback, frame)
            return fallback
        image = Image.frombytes("RGB", (frame.width, frame.height), frame.rgb)
        image.save(path)
        return path

    save_ppm(path, frame)
    return path


def save_raw_frame(path_pattern: str, frame: Frame) -> Path:
    path = Path(path_pattern.format(frame=frame.number, time=int(time.time())))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(frame.raw)
    return path


def frames_from_stream(stream: BinaryIO, max_payload: int):
    current_meta: dict[str, str] | None = None
    current_raw = bytearray()

    while True:
        packet = read_packet(stream, max_payload=max_payload)
        if packet.pkt_type == PKT_STATUS:
            print(f"[status] {packet.payload.decode('utf-8', errors='replace')}", file=sys.stderr)
            continue

        if packet.pkt_type == PKT_FRAME_START:
            current_meta = parse_meta(packet.payload)
            current_raw.clear()
            continue

        if packet.pkt_type == PKT_FRAME_DATA:
            if current_meta is not None:
                current_raw.extend(packet.payload)
            continue

        if packet.pkt_type == PKT_FRAME_END:
            if current_meta is None:
                continue
            width = int(current_meta.get("width", "0"))
            height = int(current_meta.get("height", "0"))
            fmt = current_meta.get("format", "RGB565").upper()
            if fmt not in {"RGB565", "YVYU", "YUV422_YVYU"}:
                raise ValueError(f"unsupported frame format {fmt}")
            number = int(current_meta.get("frame", str(packet.seq)))
            raw = bytes(current_raw)
            rgb = rgb565le_to_rgb888(raw, width, height)
            if fmt in {"YVYU", "YUV422_YVYU"}:
                rgb = yvyu_to_rgb888(raw, width, height)
            yield Frame(
                number=number,
                width=width,
                height=height,
                raw=raw,
                rgb=rgb,
                raw_len=len(raw),
            )
            current_meta = None
            current_raw.clear()


def preview_loop(frame_queue: "queue.Queue[Frame]", scale: int) -> None:
    if Image is None or ImageTk is None:
        raise SystemExit("preview needs Pillow: `python3 -m pip install pillow`")

    import tkinter as tk

    root = tk.Tk()
    root.title("CAMU UART Preview")
    label = tk.Label(root)
    label.pack()
    status = tk.StringVar(value="waiting for frame")
    tk.Label(root, textvariable=status, anchor="w").pack(fill="x")

    def pump() -> None:
        latest = None
        try:
            while True:
                latest = frame_queue.get_nowait()
        except queue.Empty:
            pass

        if latest is not None:
            image = Image.frombytes("RGB", (latest.width, latest.height), latest.rgb)
            if scale != 1:
                image = image.resize((latest.width * scale, latest.height * scale), Image.Resampling.NEAREST)
            photo = ImageTk.PhotoImage(image)
            label.configure(image=photo)
            label.image = photo
            status.set(f"frame {latest.number}  {latest.width}x{latest.height}  {latest.raw_len} bytes")

        root.after(15, pump)

    root.after(0, pump)
    root.mainloop()


def capture_worker(args: argparse.Namespace, frame_queue: "queue.Queue[Frame] | None" = None) -> None:
    captured = 0
    saved = 0
    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        print(f"reading CAMU on {args.port} at {args.baud} baud", file=sys.stderr)
        while True:
            try:
                frame_iter = frames_from_stream(ser, max_payload=args.max_payload)
                for frame in frame_iter:
                    captured += 1
                    expected_raw_len = frame.width * frame.height * 2
                    is_full_size = frame.raw_len == expected_raw_len and (
                        frame.width >= args.min_width and frame.height >= args.min_height
                    )
                    if args.skip_partial and not is_full_size:
                        print(
                            f"skipped partial frame {frame.number}: "
                            f"{frame.width}x{frame.height}, raw={frame.raw_len}, expected={expected_raw_len}",
                            file=sys.stderr,
                        )
                        continue
                    if args.output:
                        saved_path = save_frame(args.output, frame)
                        saved += 1
                        print(
                            f"saved {saved_path} ({frame.width}x{frame.height}, "
                            f"raw={frame.raw_len}, frame {frame.number})"
                        )
                    if args.raw_output:
                        raw_path = save_raw_frame(args.raw_output, frame)
                        print(
                            f"saved raw {raw_path} ({frame.width}x{frame.height}, "
                            f"raw={frame.raw_len}, frame {frame.number})"
                        )
                    if frame_queue is not None:
                        try:
                            frame_queue.put_nowait(frame)
                        except queue.Full:
                            pass
                    if args.count and captured >= args.count and not args.preview:
                        return
            except EOFError as exc:
                print(f"[wait] {exc}", file=sys.stderr)
                if args.count and not args.keep_waiting:
                    return


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read CAMU camera frames from UART2 and save/preview images.")
    parser.add_argument("-p", "--port", required=True, help="serial port, for example /dev/tty.usbserial-xxx or COM5")
    parser.add_argument("-b", "--baud", type=int, default=921600, help="UART baud rate, default: 921600")
    parser.add_argument(
        "-o",
        "--output",
        default="captures/camu_{frame:06d}.png",
        help="output path pattern; supports {frame} and {time}; use .ppm to avoid Pillow",
    )
    parser.add_argument(
        "--raw-output",
        default=None,
        help="optional raw YUV/RGB output path pattern; complete 640x480 YVYU is 614400 bytes",
    )
    parser.add_argument("-n", "--count", type=int, default=1, help="number of frames to save without --preview; 0 means forever")
    parser.add_argument("--preview", action="store_true", help="open a live preview window")
    parser.add_argument("--scale", type=int, default=3, help="integer preview scale, default: 3")
    parser.add_argument("--timeout", type=float, default=15.0, help="serial read timeout in seconds")
    parser.add_argument("--keep-waiting", action="store_true", help="keep waiting after serial read timeouts")
    parser.add_argument("--max-payload", type=int, default=1024 * 1024, help="maximum packet payload bytes")
    parser.add_argument("--skip-partial", action="store_true", help="skip tiny diagnostic/partial frames")
    parser.add_argument("--min-width", type=int, default=640, help="minimum width accepted by --skip-partial")
    parser.add_argument("--min-height", type=int, default=480, help="minimum height accepted by --skip-partial")
    parser.add_argument("--no-save", action="store_true", help="preview only, do not write image files")
    args = parser.parse_args(argv)
    if args.no_save:
        args.output = None
    if args.no_save and not args.preview and args.count == 0:
        parser.error("--no-save without --preview needs a positive --count")
    if args.scale < 1:
        parser.error("--scale must be >= 1")
    if args.count < 0:
        parser.error("--count must be >= 0")
    if args.min_width < 1 or args.min_height < 1:
        parser.error("--min-width and --min-height must be >= 1")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.preview:
        q: "queue.Queue[Frame]" = queue.Queue(maxsize=2)
        thread = threading.Thread(target=capture_worker, args=(args, q), daemon=True)
        thread.start()
        preview_loop(q, scale=args.scale)
        return 0

    capture_worker(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
