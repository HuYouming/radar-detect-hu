import socket
import time
import argparse
import signal
from typing import Union
from datetime import datetime

DEFAULT_ADDR = "127.0.0.1"  # 目标 IP 地址
DEFAULT_PORT = 40003 # 目标 UDP 端口
DEFAULT_RATE = 10.0
CONTROL_HEADER = b"\xAA\x55"
CONTROL_TAIL = b"\x0D\x0A"
VALID_LEVELS = (1, 2, 3)

class InterferenceSender:
    def __init__(self, addr: str = DEFAULT_ADDR, port: int = DEFAULT_PORT):
        self.addr = (addr, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = False
        self._current_value = 1

    @staticmethod
    def build_frame(data: int) -> bytes:
        if data not in VALID_LEVELS:
            raise ValueError(f"data must be one of {VALID_LEVELS}, got {data}")
        return CONTROL_HEADER + bytes([data]) + CONTROL_TAIL

    @staticmethod
    def validate_frame(frame: bytes) -> int:
        if len(frame) != 5:
            raise ValueError(f"frame length must be 5 bytes, got {len(frame)}")
        if frame[:2] != CONTROL_HEADER or frame[3:5] != CONTROL_TAIL:
            raise ValueError(f"invalid frame: {frame.hex(' ').upper()}")
        command = frame[2]
        if command not in VALID_LEVELS:
            raise ValueError(f"command must be one of {VALID_LEVELS}, got {command}")
        return command

    def send_value(self, data: Union[int, bytes, bytearray]) -> int:
        if isinstance(data, int):
            frame = self.build_frame(data)
            value = data
        else:
            frame = bytes(data)
            value = self.validate_frame(frame)
        sent = self.sock.sendto(frame, self.addr)
        self._current_value = value
        return sent

    def get_current_value(self) -> int:
        return self._current_value

    def start_loop(self, interval_ms: int = 100, initial_value: int = 1):
        self._running = True
        self._current_value = initial_value
        interval = interval_ms / 1000.0

        def sig_handler(sig, frame):
            self._running = False

        try:
            signal.signal(signal.SIGINT, sig_handler)
        except (ValueError, OSError):
            pass

        next_tick = time.time()
        packets_sent = 0
        start_time = time.time()

        try:
            while self._running:
                self.send_value(self._current_value)
                packets_sent += 1

                if packets_sent % 100 == 0:
                    elapsed = time.time() - start_time
                    actual_rate = packets_sent / elapsed if elapsed > 0 else 0
                    print(f"  [{datetime.now().strftime('%H:%M:%S')}] "
                          f"已发送 {packets_sent} 包, 当前值 {self._current_value}, ", flush=True)
                next_tick += interval
                sleep_time = next_tick - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = time.time() + interval
        except KeyboardInterrupt:
            pass
        finally:
            elapsed = time.time() - start_time
            actual_rate = packets_sent / elapsed if elapsed > 0 else 0
            print(f"\n[InterferenceSender] 发送结束: {packets_sent} 包, "
                  f"实际频率 {actual_rate:.1f} Hz")

    def stop(self):
        self._running = False

    def close(self):
        self.stop()
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

def main():
    parser = argparse.ArgumentParser(description="UDP 干扰等级控制工具 — 发送 AA 55 CMD 0D 0A 控制帧")
    parser.add_argument("--value", "-v", type=int, choices=VALID_LEVELS, default=1,
        help="要发送的干扰等级/命令字（1/2/3，默认 1）")
    parser.add_argument("--rate", "-r", type=float, default=DEFAULT_RATE,
        help=f"发送频率 Hz（默认 {DEFAULT_RATE}）")
    parser.add_argument("--addr", type=str, default=DEFAULT_ADDR,
        help=f"目标地址（默认 {DEFAULT_ADDR}）")
    parser.add_argument("--port", "-p", type=int, default=DEFAULT_PORT,
        help=f"目标端口（默认 {DEFAULT_PORT}）")
    parser.add_argument("--once", action="store_true",
        help="只发送一次就退出（默认循环发送）")
    args = parser.parse_args()

    sender = InterferenceSender(addr=args.addr, port=args.port)

    frame = sender.build_frame(args.value)
    print(f"  数值:  {args.value}")
    print(f"  帧格式: AA 55 CMD 0D 0A")
    print(f"  HEX:  {frame.hex(' ').upper()}")
    print("=" * 56)

    if args.once:
        sender.send_value(args.value)
        print(f"已发送: {args.value}")
        sender.close()
    else:
        interval_ms = int(1000.0 / args.rate)
        sender.start_loop(interval_ms=interval_ms, initial_value=args.value)
        sender.close()

if __name__ == "__main__":
    main()
