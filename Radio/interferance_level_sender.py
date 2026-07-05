import socket
import time
import argparse
import signal
import sys
import struct
from datetime import datetime

# ============================================================
# 固定通道配置
# ============================================================
DEFAULT_ADDR = "0.0.0.0"  # 目标 IP 地址
DEFAULT_PORT = 40003
DEFAULT_RATE = 10.0  # Hz

class InterferenceSender:
    """
    固定通道 UDP 发送器，专门用于发送 int 类型数字。
    """

    def __init__(self, addr: str = DEFAULT_ADDR, port: int = DEFAULT_PORT):
        """
        初始化发送器。

        Args:
            addr: 目标 IP 地址
            port: 目标 UDP 端口
        """
        self.addr = (addr, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._running = False
        self._current_value = 0

    def send_value(self, value: int) -> int:
        """
        发送一个 int 类型的数字。

        Args:
            value: 要发送的整数值（会被打包为 4 字节有符号整数）

        Returns:
            发送的字节数
        """
        # 使用 struct 将 int 打包为 4 字节（大端序，兼容 C/C++ 接收端）
        data = struct.pack(">i", value)
        sent = self.sock.sendto(data, self.addr)
        self._current_value = value
        return sent

    def get_current_value(self) -> int:
        """获取当前发送的值。"""
        return self._current_value

    def start_loop(self, interval_ms: int = 100, initial_value: int = 0):
        """
        以固定间隔循环发送当前值（阻塞式）。
        可通过 stop() 方法或 Ctrl+C 停止。

        Args:
            interval_ms: 发送间隔，单位毫秒（默认 100ms = 10Hz）
            initial_value: 初始发送值
        """
        self._running = True
        self._current_value = initial_value
        interval = interval_ms / 1000.0

        def sig_handler(sig, frame):
            self._running = False

        try:
            signal.signal(signal.SIGINT, sig_handler)
        except (ValueError, OSError):
            pass

        print(f"[InterferenceSender] 循环发送启动: {self.addr[0]}:{self.addr[1]}, "
              f"间隔 {interval_ms}ms, 初始值 {initial_value}")

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
                          f"已发送 {packets_sent} 包, 当前值 {self._current_value}, "
                          f"实际频率 {actual_rate:.1f} Hz", flush=True)

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
        """停止循环发送。"""
        self._running = False

    def close(self):
        """关闭 socket。"""
        self.stop()
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="UDP int 发送工具 — 固定通道发送 4 字节整数"
    )
    parser.add_argument(
        "--value", "-v", type=int, default=0,
        help="要发送的 int 数值（默认 0）"
    )
    parser.add_argument(
        "--rate", "-r", type=float, default=DEFAULT_RATE,
        help=f"发送频率 Hz（默认 {DEFAULT_RATE}）"
    )
    parser.add_argument(
        "--addr", type=str, default=DEFAULT_ADDR,
        help=f"目标地址（默认 {DEFAULT_ADDR}）"
    )
    parser.add_argument(
        "--port", "-p", type=int, default=DEFAULT_PORT,
        help=f"目标端口（默认 {DEFAULT_PORT}）"
    )
    parser.add_argument(
        "--once", action="store_true",
        help="只发送一次就退出（默认循环发送）"
    )
    args = parser.parse_args()

    sender = InterferenceSender(addr=args.addr, port=args.port)

    print("=" * 56)
    print("  UDP Int Sender")
    print("=" * 56)
    print(f"  目标:         {args.addr}:{args.port}")
    print(f"  数值:         {args.value}")
    print(f"  打包格式:     4 字节大端有符号整数 (struct '>i')")
    print(f"  HEX:          {struct.pack('>i', args.value).hex(' ').upper()}")
    print("=" * 56)

    if args.once:
        sender.send_value(args.value)
        print(f"  已发送: {args.value}")
        sender.close()
    else:
        interval_ms = int(1000.0 / args.rate)
        print(f"  频率: {args.rate:.1f} Hz (间隔 {interval_ms}ms)")
        print("  按 Ctrl+C 停止")
        print()
        sender.start_loop(interval_ms=interval_ms, initial_value=args.value)
        sender.close()


if __name__ == "__main__":
    main()
