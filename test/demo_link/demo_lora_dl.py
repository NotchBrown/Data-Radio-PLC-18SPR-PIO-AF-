#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_lora_dl.py - LoRa 演示: 只 DL (CI=0x40: 16路数字输入低8位 DI0~7) @ 50帧/s

用法:
  python demo_lora_dl.py COM_A COM_B [--freq 470] [--sf 7] [--bw 500] [--cr 5]
      [--period 20] [--ci 0x40] [--runtime 10]

流程: 写双板一致配置(LoRa) + 0x29 应用 + 0x1E 保存 -> 拨模式3 -> 双向通联监测
默认: SF7/BW500/CR5, 周期 15ms -> ~66帧/s (只 DL, 帧更小, release 构建)
可变参数全保留, 可用 --period 调更新率。
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import run_lora  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="LoRa 只DL数据演示 (50帧/s)")
    ap.add_argument("com_a")
    ap.add_argument("com_b")
    ap.add_argument("--freq", type=float, default=470.0)
    ap.add_argument("--sf", type=int, default=7)
    ap.add_argument("--bw", type=int, default=500)
    ap.add_argument("--cr", type=int, default=5)
    ap.add_argument("--syncword", type=int, default=0x12)
    ap.add_argument("--power", type=int, default=13)
    ap.add_argument("--preamble", type=int, default=8)
    ap.add_argument("--ci", type=lambda s: int(s, 0), default=0x40)
    ap.add_argument("--period", type=int, default=15, help="任务0 周期 ms (~66帧/s)")
    ap.add_argument("--runtime", type=int, default=10, help="通联监测时长 s")
    return run_lora(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
