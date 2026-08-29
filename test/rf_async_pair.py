#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rf_async_pair.py - 单板配置"异步收发(半双工)"模式 (只有一根 USB-B 线时用)

适用: 一次只连一块板到 UART3(CH340N, 115200/8N1), 板子在模式2(RUN=OFF,DEBUG=ON)
协议: doc/upperpc.md (帧=定位头+地址+数据2B小端+CRC-8+尾)

用法 (跑两遍, 中间换板):
    插板A: python rf_async_pair.py --board A [--port COMx]
    换板B: python rf_async_pair.py --board B [--port COMx]

对当前板写入 (并存 EEPROM 持久化):
    0x16 本机地址    0x17 对端地址
    0x18 收发模式 = 3 (异步收发: 收到一帧应答一帧)
    0x80 任务0 内容指示 = 全通道 (DH|DL|A3|A2|A1|A0)
    0x81 任务0 ENA = 1
    0x82/0x83 任务0 周期 (单位128us; 异步收发下仅记录)
    0x1E 保存配置 (阻塞约数百ms, 需等待回复)

板A/板B 预设: A: self=1 peer=2 | B: self=2 peer=1
    可用 --self/--peer 覆盖
内容指示: 默认 0xFC = 8bit截断 全通道; 0xFE = 10bit 全通道
"""

import sys
import argparse
import time

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[错误] 缺少 pyserial, 请先安装: pip install pyserial")
    sys.exit(1)

# ---------------- 协议常量 (与 test_uart3.py 一致) ----------------
HEAD_WRITE = 0x37   # 写入帧定位头
HEAD_READ  = 0x36   # 读取帧定位头
BAUD       = 115200
TIMEOUT    = 2.0    # 秒 (保存配置阻塞, 需较长超时)
FRAME_LEN  = 6

# ---------------- 地址表 ----------------
ADDR_SELF     = 0x16   # 本机 RF 地址
ADDR_PEER     = 0x17   # 对端 RF 地址
ADDR_MODE     = 0x18   # 收发模式 (3=异步收发)
ADDR_TX0_C    = 0x80   # 任务0: 内容指示
ADDR_TX0_E    = 0x81   # 任务0: ENA
ADDR_TX0_PL   = 0x82   # 任务0: 周期低16bit(128us)
ADDR_TX0_PH   = 0x83   # 任务0: 周期高16bit
ADDR_SAVE     = 0x1E   # 保存配置 (写 0x0001)
ADDR_ROLE     = 0x19   # 主从位 (0=从机 1=主机)

MODE_ASYNC_TXRX = 3    # 异步收发(半双工)

# 全通道内容指示: DH|DL|A3|A2|A1|A0, 字长0 压缩0 (8bit截断)
CONTENT_ALL_8BIT  = 0xFC
CONTENT_ALL_10BIT = 0xFE

# 板预设: (self, peer, role)  板A=主机(主动轮询), 板B=从机(收到应答)
BOARD_DEF = {
    "A": (0x01, 0x02, 1),
    "B": (0x02, 0x01, 0),
}


def crc8(data: bytes) -> int:
    """CRC-8: poly 0x07, init 0x00"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_frame(head: int, addr: int, data16: int) -> bytes:
    body = bytes([head & 0xFF, addr & 0xFF, data16 & 0xFF, (data16 >> 8) & 0xFF])
    return body + bytes([crc8(body), (~head) & 0xFF])


def parse_frame(resp: bytes, head: int):
    if len(resp) < FRAME_LEN:
        return None
    if resp[0] != head or resp[FRAME_LEN - 1] != ((~head) & 0xFF):
        return None
    if crc8(resp[0:4]) != resp[4]:
        return None
    return resp[2] | (resp[3] << 8)


def read_addr(ser, addr: int):
    ser.reset_input_buffer()
    ser.write(build_frame(HEAD_READ, addr, 0))
    return parse_frame(ser.read(FRAME_LEN), HEAD_READ)


def write_addr(ser, addr: int, val: int):
    ser.reset_input_buffer()
    ser.write(build_frame(HEAD_WRITE, addr, val & 0xFFFF))
    return parse_frame(ser.read(FRAME_LEN), HEAD_WRITE)


# ---------------- 串口选择 ----------------
def choose_port(prompt: str):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[错误] 未检测到任何串口, 请检查 CH340N USB 连接")
        sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    while True:
        try:
            idx = int(input("%s 输入序号: " % prompt))
            if 0 <= idx < len(ports):
                return ports[idx].device
        except (ValueError, EOFError):
            pass
        print("  序号无效, 请重试")


# ---------------- 配置当前板 ----------------
def configure_board(ser, name, self_addr, peer_addr, role, content, period_l, period_h):
    print("\n===== 配置板[%s] (%s) =====" % (name, ser.portstr))
    print("     角色: %s" % ("主机(主动轮询)" if role else "从机(收到应答)"))
    steps = [
        ("本机地址 0x16", ADDR_SELF, self_addr),
        ("对端地址 0x17", ADDR_PEER, peer_addr),
        ("收发模式 0x18=3(异步收发)", ADDR_MODE, MODE_ASYNC_TXRX),
        ("主从位 0x19", ADDR_ROLE, role),
        ("任务0 内容指示 0x80", ADDR_TX0_C, content),
        ("任务0 ENA 0x81", ADDR_TX0_E, 0x01),
        ("任务0 周期低 0x82", ADDR_TX0_PL, period_l),
        ("任务0 周期高 0x83", ADDR_TX0_PH, period_h),
    ]
    ok = True
    for label, addr, val in steps:
        r = write_addr(ser, addr, val)
        stat = "OK " if r is not None and (r & 0xFF) == (val & 0xFF) else "FAIL"
        print("  写入 %-28s = 0x%04X  -> 回复 0x%04X  [%s]"
              % (label, val & 0xFFFF, r if r is not None else 0xFFFF, stat))
        if r is None or (r & 0xFF) != (val & 0xFF):
            ok = False

    # 保存到 EEPROM (阻塞数百ms, 需等待回复)
    print("  保存配置 0x1E = 0x0001 (EEPROM, 阻塞中...)")
    r = write_addr(ser, ADDR_SAVE, 0x0001)
    stat = "OK " if r == 0x0001 else "FAIL"
    print("  保存回复 0x%04X  [%s]" % (r if r is not None else 0xFFFF, stat))
    if r != 0x0001:
        ok = False

    # 读回验证
    print("  -- 读回验证 --")
    for label, addr in [("本机地址", ADDR_SELF), ("对端地址", ADDR_PEER),
                        ("收发模式", ADDR_MODE), ("主从位", ADDR_ROLE),
                        ("任务0内容", ADDR_TX0_C), ("配置有效性", ADDR_SAVE)]:
        v = read_addr(ser, addr)
        print("    读 %-10s (0x%02X) = 0x%04X" % (label, addr, v if v is not None else 0xFFFF))
        if v is None:
            ok = False
    print("  [%s]" % ("成功" if ok else "有失败项"))
    return ok


def period_to_lh(period_ms: int):
    """周期 ms -> (低16bit, 高16bit), 单位 = TIM4 6kHz 节拍(166.7us): 1ms = 6 tick"""
    p = max(1, int(period_ms * 6))
    return p & 0xFFFF, (p >> 16) & 0xFFFF


def main():
    ap = argparse.ArgumentParser(description="单板配置异步收发模式(半双工), 跑两遍换板")
    ap.add_argument("--board", choices=["A", "B"], default=None,
                    help="板A(self=1 peer=2) 或 板B(self=2 peer=1)")
    ap.add_argument("--port", help="COM口, 如 COM4 (缺省交互选择)")
    ap.add_argument("--self", type=lambda s: int(s, 0), default=None, help="本机地址覆盖")
    ap.add_argument("--peer", type=lambda s: int(s, 0), default=None, help="对端地址覆盖")
    ap.add_argument("--role", type=lambda s: int(s, 0), default=None,
                    help="主从位覆盖 0=从机 1=主机 (默认 A=1主机 B=0从机)")
    ap.add_argument("--content", type=lambda s: int(s, 0), default=CONTENT_ALL_8BIT,
                    help="任务0内容指示, 默认0xFC(8bit全通道); 0xFE=10bit全通道")
    ap.add_argument("--period-ms", type=int, default=100, help="任务0周期ms, 默认100")
    args = ap.parse_args()

    # 确定地址/角色
    if args.board:
        self_a, peer_a, role_a = BOARD_DEF[args.board]
    elif args.self is not None and args.peer is not None:
        self_a, peer_a, role_a = args.self, args.peer, 0
    else:
        ap.error("请用 --board A/B 或同时 --self/--peer 指定地址")
    self_a = args.self if args.self is not None else self_a
    peer_a = args.peer if args.peer is not None else peer_a
    role   = args.role if args.role is not None else role_a

    name = args.board or "?"
    port = args.port or choose_port("板%s 串口" % name)
    content = args.content & 0xFF
    pl, ph = period_to_lh(args.period_ms)

    print("参数: 板%s self=0x%02X peer=0x%02X role=%d(%s) | content=0x%02X (%s) | period=%dms"
          % (name, self_a, peer_a, role, "主机" if role else "从机", content,
             "+".join([x for b, x in ((0x80, "DH"), (0x40, "DL"), (0x20, "A3"),
                                      (0x10, "A2"), (0x08, "A1"), (0x04, "A0"))
                       if b & content]) or "无",
             args.period_ms))

    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    try:
        ok = configure_board(ser, name, self_a, peer_a, role, content, pl, ph)
    finally:
        ser.close()

    print("\n===== 板%s 配置完成 %s =====" % (name, "成功" if ok else "存在失败项"))
    if ok:
        print("下一步:")
        if args.board == "A":
            print("  拔掉板A, 换上板B, 运行: python rf_async_pair.py --board B")
        elif args.board == "B":
            print("  两块板都已配置完成, 拨到 mode3 (RUN=ON, DEBUG=OFF) 即可互发应答")
        else:
            print("  若还有另一块板, 换上后运行: python rf_async_pair.py --board B"
                  "(或 --board A)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
