#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
common.py - demo_link 系列脚本的公共工具
  - UART3 帧构建/解析 (Device)
  - LoRa 空速、FSK 寄存器换算
用法: 由 demo_lora_full/dl 与 demo_fsk_full/dl 脚本 import
"""

import sys
import time

import serial

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HEAD_WRITE = 0x37
HEAD_READ = 0x36
BAUD = 115200
TIMEOUT = 2.0          # 覆盖 EEPROM 写阻塞 (~1s)


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_frame(head, addr, data16):
    body = bytes([head & 0xFF, addr & 0xFF, data16 & 0xFF, (data16 >> 8) & 0xFF])
    return body + bytes([crc8(body), (~head) & 0xFF])


def parse_freq(mhz):
    return int(mhz * 1e6)


def lora_airrate(sf, bw_khz, cr):
    """LoRa 空速 bps (近似)"""
    bw = bw_khz * 1000.0
    return sf * bw / (2 ** sf) * (4.0 / cr)


def fsk_bitrate_regs(bps):
    """BitRate 寄存器 (16bit) = 32MHz / bps"""
    v = int(32000000 / bps)
    return (v >> 8) & 0xFF, v & 0xFF


def fsk_fdev_regs(hz):
    """Fdev 寄存器 (16bit) = hz / 61.035"""
    v = int(hz / 61.035)
    return (v >> 8) & 0xFF, v & 0xFF


def fsk_bw_reg(hz):
    """RxBw 寄存器值查表 (参考 FskBandwidths)"""
    tbl = [(2600,0x17),(3100,0x0F),(3900,0x07),(5200,0x16),(6300,0x0E),(7800,0x06),
           (10400,0x15),(12500,0x0D),(15600,0x05),(20800,0x14),(25000,0x0C),(31300,0x04),
           (41700,0x13),(50000,0x0B),(62500,0x03),(83333,0x12),(100000,0x0A),(125000,0x02),
           (166700,0x11),(200000,0x09),(250000,0x01)]
    for bw, reg in tbl:
        if hz <= bw:
            return reg
    return 0x00


class Device:
    def __init__(self, ser, name):
        self.ser = ser
        self.name = name

    def _read_reply(self, head):
        end = time.time() + TIMEOUT
        acc = bytearray()
        while time.time() < end:
            if self.ser.in_waiting:
                acc += self.ser.read(self.ser.in_waiting)
                while len(acc) >= 1:
                    if acc[0] != HEAD_READ and acc[0] != HEAD_WRITE:  # 跳 debug 文本
                        del acc[0]
                        continue
                    if len(acc) < 6:
                        break
                    if acc[0] == head and acc[5] == ((~head) & 0xFF) \
                            and crc8(bytes(acc[0:4])) == acc[4]:
                        return acc[2] | (acc[3] << 8)
                    del acc[0]
            else:
                time.sleep(0.01)
        return None

    def read(self, addr):
        self.ser.reset_input_buffer()
        self.ser.write(build_frame(HEAD_READ, addr, 0))
        r = self._read_reply(HEAD_READ)
        time.sleep(0.03)
        print("      R[0x%02X] -> %s" % (addr, "TIMEOUT" if r is None else "0x%04X" % r))
        return r

    def write(self, addr, val):
        self.ser.reset_input_buffer()
        self.ser.write(build_frame(HEAD_WRITE, addr, val & 0xFFFF))
        r = self._read_reply(HEAD_WRITE)
        time.sleep(0.05)
        print("      W[0x%02X]=0x%04X -> %s" % (addr, val & 0xFFFF,
              "TIMEOUT" if r is None else "0x%04X" % r))
        return r

    def check(self, addr, val, desc):
        r = self.write(addr, val)
        ok = (r == val)
        print("  [%s] %s 0x%02X %-20s <- 0x%04X (回读 %s)" % (
            "OK" if ok else "FAIL", self.name, addr, desc, val,
            "无" if r is None else "0x%04X" % r))
        return ok


def _wait_mode3():
    print(">>> 请把两块板拨到【模式3 (RUN=1 DEBUG=0)】, 拨好后按回车继续...")
    try:
        input()
    except EOFError:
        time.sleep(3)


def _monitor(args, A, B, label):
    A.write(0x21, 0)
    B.write(0x21, 0)
    time.sleep(0.2)
    print("监测 %ds, 主机A 按 %dms 周期发帧, 从机B 回传..." % (args.runtime, args.period))
    time.sleep(args.runtime)

    a_rx = A.read(0x21)
    b_rx = B.read(0x21)
    a_link = A.read(0x2B)
    b_link = B.read(0x2B)
    a_crc = A.read(0x22)
    b_crc = B.read(0x22)

    print("\n----- 结果 -----")
    print("  主机A: RX计数=%s link=%s CRC错=%s" % (a_rx, a_link, a_crc))
    print("  从机B: RX计数=%s link=%s CRC错=%s" % (b_rx, b_link, b_crc))
    exp = args.runtime * 1000.0 / args.period
    print("  期望帧数(单侧)≈%d 帧 @ %dms" % (int(exp), args.period))

    b_ok = bool(b_rx and (b_rx > 0))
    a_ok = bool(a_rx and (a_rx > 0))
    print("  => 从机B收到=%s 主机A收到回传=%s" % ("是" if b_ok else "否", "是" if a_ok else "否"))
    if b_ok and a_ok:
        print("  => 双向通联 OK: %s" % label)
        return True
    print("  => 未完全通联! 检查参数/地址/主从/模式3/天线")
    return False


def _save(A, B):
    for dev in (A, B):
        r = dev.write(0x1E, 1)
        time.sleep(1.0)
        post = dev.read(0x1E)
        print("  %s 保存: 回=%s 后有效=%s" % (dev.name,
              "OK" if r == 1 else "FAIL", "无" if post is None else "0x%04X" % post))


def run_lora(args):
    hz = parse_freq(args.freq)
    if not (137000000 <= hz <= 525000000):
        print("频率越界: 137~525 MHz")
        return 1

    A = Device(serial.Serial(args.com_a, BAUD, timeout=0.2), "A(主)")
    B = Device(serial.Serial(args.com_b, BAUD, timeout=0.2), "B(从)")

    rate = lora_airrate(args.sf, args.bw, args.cr)
    print("== LoRa: freq=%dMHz SF=%d BW=%dkHz CR=4/%d sync=0x%02X CI=0x%02X ==" % (
        args.freq, args.sf, args.bw, args.cr, args.syncword, args.ci))
    print("   空速≈%.1fkbps, 周期=%dms -> %d帧/s" % (
        rate / 1000.0, args.period, int(1000.0 / args.period)))

    all_ok = True
    tick = args.period * 6  # TIM4 6kHz: 6tick=1ms

    print("\n===== 1. 写入配置 =====")
    for dev, selfa, peera, role in ((A, 1, 2, 1), (B, 2, 1, 0)):
        print("---- %s (主从=%d) ----" % (dev.name, role))
        for a, v, d in [
            (0x16, selfa, "本机地址"), (0x17, peera, "对端地址"), (0x19, role, "主从位"),
            (0x30, hz & 0xFFFF, "频率低16"), (0x31, (hz >> 16) & 0xFFFF, "频率高16"),
            (0x32, args.sf, "SF"), (0x33, args.bw, "BW"), (0x34, args.cr, "CR"),
            (0x35, args.power, "功率"), (0x36, args.preamble, "前导"), (0x37, args.syncword, "同步字"),
            (0x38, 0x23, "LNA"),
            (0x40, args.ci, "任务0 CI2(从站回传)"), (0x80, args.ci, "任务0 CI1(本机发)"),
            (0x81, 1, "任务0 ENA"),
            (0x82, tick & 0xFFFF, "任务0 周期L"), (0x83, (tick >> 16) & 0xFFFF, "任务0 周期H"),
        ]:
            if not dev.check(a, v, d):
                all_ok = False

    print("\n===== 2. 应用(0x29) + 保存(0x1E) =====")
    for dev in (A, B):
        dev.write(0x29, 1)
    _save(A, B)

    if not all_ok:
        print("\n>>> 有配置写入失败! 请检查串口/接线后重试")
        return 1

    print("\n===== 3. 双向通联监测 =====")
    _wait_mode3()
    ok = _monitor(args, A, B, "LoRa 数据已双向传送")
    print("\n===== 完成: %s =====" % ("全部 OK" if (all_ok and ok) else "存在失败"))
    return 0 if (all_ok and ok) else 1


def run_fsk(args):
    hz = parse_freq(args.freq)
    if not (137000000 <= hz <= 525000000):
        print("频率越界: 137~525 MHz")
        return 1

    A = Device(serial.Serial(args.com_a, BAUD, timeout=0.2), "A(主)")
    B = Device(serial.Serial(args.com_b, BAUD, timeout=0.2), "B(从)")

    br_msb, br_lsb = fsk_bitrate_regs(args.bitrate)
    fd_msb, fd_lsb = fsk_fdev_regs(args.fdev)
    bw_reg = fsk_bw_reg(args.rxbw)

    print("== FSK: freq=%dMHz BitRate=%dbps Fdev=%dHz RxBw=%dHz sync=0x%02X CI=0x%02X ==" % (
        args.freq, args.bitrate, args.fdev, args.rxbw, args.syncword, args.ci))
    print("   BitRate寄存=0x%02X%02X Fdev寄存=0x%02X%02X RxBw寄存=0x%02X" % (
        br_msb, br_lsb, fd_msb, fd_lsb, bw_reg))
    print("   周期=%dms -> %d帧/s" % (args.period, int(1000.0 / args.period)))

    all_ok = True
    tick = args.period * 6

    print("\n===== 1. 写入高层配置 + 任务0 =====")
    for dev, selfa, peera, role in ((A, 1, 2, 1), (B, 2, 1, 0)):
        print("---- %s (主从=%d) ----" % (dev.name, role))
        for a, v, d in [
            (0x16, selfa, "本机地址"), (0x17, peera, "对端地址"), (0x19, role, "主从位"),
            (0x30, hz & 0xFFFF, "频率低16"), (0x31, (hz >> 16) & 0xFFFF, "频率高16"),
            (0x35, args.power, "功率"), (0x36, args.preamble, "前导"), (0x37, args.syncword, "同步字"),
            (0x40, args.ci, "任务0 CI2(从站回传)"), (0x80, args.ci, "任务0 CI1(本机发)"),
            (0x81, 1, "任务0 ENA"),
            (0x82, tick & 0xFFFF, "任务0 周期L"), (0x83, (tick >> 16) & 0xFFFF, "任务0 周期H"),
        ]:
            if not dev.check(a, v, d):
                all_ok = False

    print("\n===== 2. 保存 (0x1E) =====")
    _save(A, B)

    print("\n===== 3. 切 FSK (0x2F=1) =====")
    for dev in (A, B):
        dev.check(0x2F, 1, "射频调制=FSK")

    print("\n===== 4. 直写 FSK 物理参数 =====")
    for dev in (A, B):
        print("---- %s ----" % dev.name)
        for a, v, d in [
            (0x62, br_msb, "BitRateMSB(0x02)"), (0x63, br_lsb, "BitRateLSB(0x03)"),
            (0x64, fd_msb, "FdevMSB(0x04)"), (0x65, fd_lsb, "FdevLSB(0x05)"),
            (0x69, 0x80 | (args.power & 0x0F), "PA(0x09)"), (0x6C, 0x23, "LNA(0x0C)"),
            (0x6D, 0x1E, "RxConfig(0x0D)"), (0x72, bw_reg, "RxBw(0x12)"),
            (0x73, bw_reg, "AfcBw(0x13)"),
        ]:
            if not dev.check(a, v, d):
                all_ok = False

    print("\n===== 5. 直写 FSK 包格式 =====")
    for dev in (A, B):
        print("---- %s ----" % dev.name)
        for a, v, d in [
            (0x39, args.packet1, "PACKETCONFIG1(0x30)"), (0x3A, args.packet2, "PACKETCONFIG2(0x31)"),
            (0x3B, 0xFF, "PAYLOADLENGTH(0x32)"), (0x3E, args.syncword, "SYNCVALUE1(0x28)"),
        ]:
            if not dev.check(a, v, d):
                all_ok = False

    if not all_ok:
        print("\n>>> 有配置写入失败! 请检查串口/接线后重试")
        return 1

    print("\n===== 6. 双向通联监测 =====")
    _wait_mode3()
    ok = _monitor(args, A, B, "FSK 数据已双向传送")
    print("\n===== 完成: %s =====" % ("全部 OK" if (all_ok and ok) else "存在失败"))
    return 0 if (all_ok and ok) else 1

