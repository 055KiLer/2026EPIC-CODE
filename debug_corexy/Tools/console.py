#!/usr/bin/env python3
"""
调试台架的控制台驱动（我这边用，台架上也能用）

用途：不开串口助手，直接从命令行把命令敲进板子、把回包抓回来。
     加"-l"还能在别的程序烧录/复位板子的同时一直监听，把上电自检那段抓下来。

用法：
  python Tools/console.py                       # 只监听 3 秒（抓上电自检）
  python Tools/console.py "scan 20" "ack 3"     # 依次发命令并抓回包
  python Tools/console.py -t 2 -p COM10 "help"  # 指定超时/端口
  python Tools/console.py --listen 8            # 监听 8 秒（复位后抓自检）

前提：串口助手**必须先关闭**（同一个 COM 口只能被一个程序打开）。
"""
import argparse
import sys
import time

import serial

# 板子上每次命令都会打 [READY] 收尾，用它判断"这条命令的输出收完了"
READY = b"[READY]"


def open_port(port: str, baud: int, timeout: float) -> serial.Serial:
    ser = serial.Serial(port, baud, timeout=timeout)
    ser.reset_input_buffer()
    return ser


def drain(ser: serial.Serial, seconds: float, until_ready: bool = False) -> str:
    """收空口子：收 seconds 秒，或（until_ready）收到 [READY] 就停"""
    deadline = time.time() + seconds
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            if until_ready and READY in buf:
                break
    return buf.decode("utf-8", "replace")


def main() -> int:
    ap = argparse.ArgumentParser(description="板子控制台驱动")
    ap.add_argument("cmds", nargs="*", help="要发送的命令（可多条）")
    ap.add_argument("-p", "--port", default="COM10")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-t", "--timeout", type=float, default=0.2, help="串口读超时")
    ap.add_argument("-l", "--listen", type=float, default=0.0, help="只监听这么多秒（抓自检）")
    ap.add_argument("--gap", type=float, default=0.05, help="两条命令之间的间隔")
    args = ap.parse_args()

    try:
        ser = open_port(args.port, args.baud, args.timeout)
    except Exception as exc:  # noqa: BLE001
        print(f"打不开 {args.port}: {exc}")
        print("→ 串口助手可能还占着这个口，点它的「关闭」按钮后再试")
        return 2

    try:
        if args.listen > 0.0 or not args.cmds:
            secs = args.listen if args.listen > 0.0 else 3.0
            print(f"--- 监听 {secs:g} 秒（复位/上电时的自检会在这里出现）---")
            text = drain(ser, secs)
            print(text if text else "(没收到任何字节)")
            return 0

        for cmd in args.cmds:
            print(f"--- 发送: {cmd} ---")
            ser.reset_input_buffer()
            ser.write(cmd.encode() + b"\r\n")
            text = drain(ser, 30.0, until_ready=True)
            print(text if text else "(无回包)")
            time.sleep(args.gap)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())