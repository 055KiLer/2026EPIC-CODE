#!/usr/bin/env python3
"""
TJC 串口屏自检（PC 直连屏，不经 STM32）

用途：一条一条核对屏工程那 6 个"成功数量"控件的**名字与行序**。它是评分字段，
      "名字对不上"的表现是那一列永远不刷新、也不报错 —— 只能靠人眼逐行核。

用法（需要 pyserial：pip install pyserial）
  cd code && python Tools/screen_check.py COM7           # 第 1 行写 1、第 2 行写 2 … 第 6 行写 6
  cd code && python Tools/screen_check.py COM7 zero      # 6 行全部写回 0（测完恢复）
  cd code && python Tools/screen_check.py COM7 88        # 6 行全写 88（看整列是不是都能动）
  cd code && python Tools/screen_check.py COM7 -b 9600   # 屏工程若还是老波特率

接线：USB-TTL 的 TX → 屏的 RX、GND ↔ GND（屏 TX → USB-TTL 的 RX，只有要看回包时才接）。
      ⚠️ 屏的 RX 不能同时被 USB-TTL 和主板 TX 驱 —— 测之前把主板那根 TX 拔掉。
波特率：缺省 115200 8N1（与产品 USART6 一致）。

判据
  * 屏上"成功数量"列出现 1,2,3,4,5,6 ⇒ 控件名 n1~n6 与行序都对，可以交给固件写。
  * 某一行没动 ⇒ 那个控件名不存在或拼错：改屏工程，或改产品 sort_config.h 的
    CFG_TJC_VAR_CNT_BASE / CFG_TJC_CNT_FIRST 两处。
  * 数字出现在别的行 ⇒ 屏上行序与类别号不一致（重排屏工程的行，或按屏的行序改盒号表）。
  * 回包 `01 FF FF FF` = 指令成功；`02`/`1A` 之类 = 控件名无效或指令错（以屏的手册为准）。
  * 一个字节回包都没有 ⇒ 先把波特率试成屏工程里设的那个（老工程常是 9600）。
"""
import argparse
import sys
import time

import serial

ROWS = 6                 # 屏上行数 = 类别数 = CFG_NUM_CATEGORIES
BASE = "n"               # 控件名基名 = CFG_TJC_VAR_CNT_BASE
FIRST = 1                # 起始行号 = CFG_TJC_CNT_FIRST
TERM = b"\xff\xff\xff"   # TJC 指令尾


def var_of(row: int) -> str:
    return f"{BASE}{FIRST + row - 1}"


def send(ser: serial.Serial, cmd: str) -> bytes:
    ser.write(cmd.encode("ascii") + TERM)
    ser.flush()
    time.sleep(0.08)
    return ser.read(64)


def main() -> int:
    ap = argparse.ArgumentParser(description="TJC 串口屏自检（PC 直连）")
    ap.add_argument("port", help="串口号，如 COM7")
    ap.add_argument("value", nargs="?", default="row",
                    help="row（缺省，逐行写行号）/ zero / 一个数字（6 行都写它）")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.3)
    except serial.SerialException as exc:
        print(f"打不开 {args.port}：{exc}\n（串口助手必须先关掉；换个口名试试）")
        return 2
    ser.reset_input_buffer()
    print(f"{args.port} @{args.baud} 已打开，开始逐行写…\n")

    bad = 0
    for row in range(1, ROWS + 1):
        if args.value == "row":
            val = row
        elif args.value == "zero":
            val = 0
        else:
            val = int(args.value)
        cmd = f"{var_of(row)}.val={val}"
        reply = send(ser, cmd)
        hex_reply = reply.hex(" ") if reply else "<无回包>"
        print(f"  第 {row} 行  {cmd:<14} 回包 {hex_reply}")
        if reply and reply[0] != 0x01:
            bad += 1
    ser.close()

    print()
    if args.value == "row":
        print("看屏：'成功数量'列应自上而下出现 1,2,3,4,5,6（哪一行没动 = 那个控件名不对）。")
        print("核完把 6 行写回 0：python Tools/screen_check.py COM7 zero")
    if bad:
        print(f"⚠️ 有 {bad} 条回包不是 01（指令成功）——见文件头'判据'一节。")
    return 0


if __name__ == "__main__":
    sys.exit(main())