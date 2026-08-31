#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_fsk_full.py - FSK 演示: 全量数据 (CI=0xFE: 16DI+16DO+4路10bit模拟量) @ 200帧/s

用法:
  python demo_fsk_full.py COM_A COM_B [--freq 470] [--bitrate 100000] [--fdev 50000]
      [--rxbw 200000] [--syncword 0xC1] [--period 5] [--ci 0xFE] [--runtime 10]

流程: 写双板一致配置 + 0x1E 保存 -> 0x2F=1 切 FSK -> 直写 FSK 物理参数+包格式
      -> 拨模式3 -> 双向通联监测
默认: BitRate=100k/Fdev=50k/RxBw=200k, 周期 5ms -> 200帧/s (release 构建)
可变参数全保留, 可用 --period 调更新率。
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import run_fsk  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="FSK 全量数据演示 (200帧/s)")
    ap.add_argument("com_a")
    ap.add_argument("com_b")
    ap.add_argument("--freq", type=float, default=470.0)
    ap.add_argument("--bitrate", type=int, default=100000, help="FSK BitRate bps")
    ap.add_argument("--fdev", type=int, default=50000, help="FSK 频偏 Hz")
    ap.add_argument("--rxbw", type=int, default=200000, help="FSK 接收带宽 Hz")
    ap.add_argument("--power", type=int, default=15, help="发射功率 dBm (FSK 拉满=15)")
    ap.add_argument("--preamble", type=int, default=8)
    ap.add_argument("--syncword", type=int, default=0xC1, help="FSK 同步字节")
    ap.add_argument("--packet1", type=lambda s: int(s, 0), default=0x90,
                    help="PACKETCONFIG1 (0x90=可变包+CRC)")
    ap.add_argument("--packet2", type=lambda s: int(s, 0), default=0x40, help="PACKETCONFIG2")
    ap.add_argument("--ci", type=lambda s: int(s, 0), default=0xFE)
    ap.add_argument("--period", type=int, default=5, help="任务0 周期 ms (200帧/s)")
    ap.add_argument("--runtime", type=int, default=10, help="通联监测时长 s")
    return run_fsk(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
