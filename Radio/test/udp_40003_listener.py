#!/usr/bin/env python3

import argparse
import signal
import socket
import time
from datetime import datetime


DEFAULT_ADDR = "127.0.0.1"
DEFAULT_PORT = 40003
CONTROL_HEADER = b"\xAA\x55"
CONTROL_TAIL = b"\x0D\x0A"


def decode_control_frame(data: bytes) -> str:
    if len(data) != 5:
        return "invalid length"
    if data[:2] != CONTROL_HEADER or data[3:5] != CONTROL_TAIL:
        return "invalid header/tail"
    command = data[2]
    if command not in (1, 2, 3):
        return f"invalid command {command}"
    return f"level {command}"


def main():
    parser = argparse.ArgumentParser(
        description="Listen on UDP 40003 and decode AA 55 CMD 0D 0A control frames."
    )
    parser.add_argument(
        "--addr",
        default=DEFAULT_ADDR,
        help=f"bind address (default: {DEFAULT_ADDR})",
    )
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=DEFAULT_PORT,
        help=f"listen UDP port (default: {DEFAULT_PORT})",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.addr, args.port))
    sock.settimeout(1.0)

    print("=" * 56)
    print("  UDP 40003 Control Listener")
    print("=" * 56)
    print(f"  Listen:  {args.addr}:{args.port}")
    print("  Format:  AA 55 CMD 0D 0A, CMD is one byte: 01/02/03")
    print("  Stop:    Ctrl+C")
    print("=" * 56)

    running = [True]
    packets_recv = 0
    bytes_recv = 0
    start_time = time.time()

    def sig_handler(sig, frame):
        running[0] = False

    try:
        signal.signal(signal.SIGINT, sig_handler)
    except (ValueError, OSError):
        pass

    try:
        while running[0]:
            try:
                data, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue

            packets_recv += 1
            bytes_recv += len(data)
            timestamp = datetime.now().strftime("%H:%M:%S")
            hex_data = data.hex(" ").upper()
            decoded = decode_control_frame(data)
            print(
                f"[{timestamp}] {len(data):3d}B from {peer[0]}:{peer[1]} "
                f"| HEX {hex_data} | control {decoded}",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    elapsed = time.time() - start_time
    rate = packets_recv / elapsed if elapsed > 0 else 0.0
    print()
    print("=" * 56)
    print("  Listener Stats")
    print("=" * 56)
    print(f"  Packets:  {packets_recv}")
    print(f"  Bytes:    {bytes_recv} B")
    print(f"  Runtime:  {elapsed:.1f} s")
    print(f"  Rate:     {rate:.1f} Hz")
    print("=" * 56)


if __name__ == "__main__":
    main()
