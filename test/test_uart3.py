#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_uart3.py - UART3 上位机协议自动化测试

协议 (见 doc/upperpc.md):
    帧 = 帧定位头(0x37写 / 0x36读) + 地址 + 数据码(2B小端) + CRC-8 + 帧定位尾(定位头按位非)
    CRC-8: poly 0x07, init 0x00, 覆盖 定位头~数据码
    写入: 回复设定后的数值; 读取: 主机发数据 0, 从机回该地址数值

运行:
    python test_uart3.py     (需 pyserial: pip install pyserial)
流程:
    1. 列出可用串口 -> 选择 UART3 所在 COM
    2. 115200/8N1 通信测试 (读ID/RTC/DI/DO, 写DO/RTC, 非法帧)
    3. 生成 markdown 测试报告 (report_uart3_<时间戳>.md)
"""

import serial
import serial.tools.list_ports
import sys
import os
import time
import datetime

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

# ---------------- 协议常量 ----------------
HEAD_WRITE = 0x37   # 写入帧定位头
HEAD_READ  = 0x36   # 读取帧定位头
BAUD       = 115200
TIMEOUT    = 0.6    # 秒, 等回复超时
FRAME_LEN  = 6      # 定位头+地址+数据2+CRC+尾

# ---------------- 协议函数 ----------------
def crc8(data: bytes) -> int:
    """CRC-8: poly 0x07, init 0x00, 覆盖 定位头~数据码"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def build_frame(head: int, addr: int, data16: int) -> bytes:
    """构造帧: head addr lo hi crc tail"""
    body = bytes([head & 0xFF, addr & 0xFF, data16 & 0xFF, (data16 >> 8) & 0xFF])
    return body + bytes([crc8(body), (~head) & 0xFF])

def parse_frame(resp: bytes, head: int):
    """解析回复帧, 成功返回 data16(int), 失败返回 None"""
    if len(resp) < FRAME_LEN:
        return None
    if resp[0] != head or resp[FRAME_LEN - 1] != ((~head) & 0xFF):
        return None
    if crc8(resp[0:4]) != resp[4]:
        return None
    return resp[2] | (resp[3] << 8)

def read_addr(ser, addr: int):
    """读地址: 返回 data16 或 None"""
    ser.reset_input_buffer()
    ser.write(build_frame(HEAD_READ, addr, 0))
    return parse_frame(ser.read(FRAME_LEN), HEAD_READ)

def write_addr(ser, addr: int, val: int):
    """写地址: 返回回复(设定后值) 或 None"""
    ser.reset_input_buffer()
    ser.write(build_frame(HEAD_WRITE, addr, val & 0xFFFF))
    return parse_frame(ser.read(FRAME_LEN), HEAD_WRITE)

# ---------------- 串口选择 ----------------
def choose_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[错误] 未检测到任何串口, 请检查 CH340N USB 连接")
        sys.exit(1)
    print("检测到以下串口:")
    for i, p in enumerate(ports):
        print("  [%d] %s  -  %s" % (i, p.device, p.description or "(无描述)"))
    while True:
        try:
            s = input("请选择 UART3 所在串口编号: ").strip()
            idx = int(s)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except (ValueError, EOFError):
            pass
        print("输入无效, 请重新输入")

# ---------------- 测试框架 ----------------
def t_case(results, name, ok, detail=""):
    results.append({"name": name, "ok": ok, "detail": detail})
    print("  [%s] %-24s %s" % ("PASS" if ok else "FAIL", name, detail))

def run_tests(ser):
    results = []
    print("\n========== 通信测试 ==========")

    # 1. 读 MCU ID (0x00~0x02, 每地址2字节小端)
    ids, ok = [], True
    for a in (0x00, 0x01, 0x02):
        v = read_addr(ser, a)
        if v is None:
            ok = False
            break
        ids.append(v)
    detail = "ID=%s" % ("-".join("%04X" % x for x in ids)) if ids else "无回复"
    t_case(results, "读 MCU ID (0x00~0x02)", ok and ids != [0, 0, 0], detail)

    # 2. 读 RTC 各字段 (范围校验)
    for name, addr, lo, hi in (
            ("读 RTC 毫秒(0x03)", 0x03, 0, 999),
            ("读 RTC 秒(0x04)", 0x04, 0, 59),
            ("读 RTC 分(0x05)", 0x05, 0, 59),
            ("读 RTC 时(0x06)", 0x06, 0, 23),
            ("读 RTC 日(0x07)", 0x07, 1, 31),
            ("读 RTC 月(0x08)", 0x08, 1, 12),
            ("读 RTC 年(0x09)", 0x09, 0, 255)):
        v = read_addr(ser, addr)
        t_case(results, name, v is not None and lo <= v <= hi, "=%s" % v if v is not None else "无回复")

    # 3. 读 DI / DO
    for name, addr in (("读 DI7~0(0x0A)", 0x0A), ("读 DI15~8(0x0B)", 0x0B),
                       ("读 DO7~0(0x0C)", 0x0C), ("读 DO15~8(0x0D)", 0x0D)):
        v = read_addr(ser, addr)
        t_case(results, name, v is not None,
               "=0x%02X" % v if v is not None else "无回复")

    # 4. 写 DO7~0 = 0xFF (全高=全关, 不改变输出状态), 验证回读=设定值
    v = write_addr(ser, 0x0C, 0xFF)
    t_case(results, "写 DO7~0=0xFF (0x0C)", v == 0xFF, "回读=%s" % v if v is not None else "无回复")

    # 5. 写 RTC 毫秒 = 500, 验证回读=设定后值
    v = write_addr(ser, 0x03, 500)
    t_case(results, "写 RTC 毫秒=500 (0x03)", v == 500, "回读=%s" % v if v is not None else "无回复")

    # 6. 非法 CRC 帧 -> 从机应丢弃, 无回复
    bad = bytearray(build_frame(HEAD_READ, 0x03, 0))
    bad[4] ^= 0xFF   # 改坏 CRC
    ser.reset_input_buffer()
    ser.write(bytes(bad))
    r = ser.read(FRAME_LEN)
    t_case(results, "非法 CRC 帧应无回复", len(r) == 0, "收到 %d 字节" % len(r))

    # 7. 非法尾帧 -> 从机应丢弃, 无回复
    bad2 = bytearray(build_frame(HEAD_READ, 0x03, 0))
    bad2[5] ^= 0xFF  # 改坏尾
    ser.reset_input_buffer()
    ser.write(bytes(bad2))
    r = ser.read(FRAME_LEN)
    t_case(results, "非法尾帧应无回复", len(r) == 0, "收到 %d 字节" % len(r))

    return results

# ---------------- 报告 ----------------
def gen_report(results, port):
    now = datetime.datetime.now()
    # 报告生成到脚本同目录 (test/)
    here = os.path.dirname(os.path.abspath(__file__))
    fn = os.path.join(here, "report_uart3_%s.md" % now.strftime("%Y%m%d_%H%M%S"))
    passed = sum(1 for r in results if r["ok"])
    total = len(results)
    lines = [
        "# UART3 自动化测试报告",
        "",
        "- 测试时间: %s" % now.strftime("%Y-%m-%d %H:%M:%S"),
        "- 串口: %s @ %d 8N1" % (port, BAUD),
        "- 结论: **%d / %d 通过 (%s)**" % (passed, total, "PASS" if passed == total else "FAIL"),
        "",
        "| # | 用例 | 结果 | 详情 |",
        "|---|------|------|------|",
    ]
    for i, r in enumerate(results, 1):
        lines.append("| %d | %s | %s | %s |" % (i, r["name"], "✅" if r["ok"] else "❌", r["detail"]))
    lines.append("")
    with open(fn, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\n报告已生成: %s" % fn)

def main():
    port = choose_port()
    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT,
                            bytesize=8, parity='N', stopbits=1)
    except Exception as e:
        print("[错误] 打开串口失败: %s" % e)
        sys.exit(1)
    print("已打开 %s @ %d 8N1, 开始测试..." % (port, BAUD))
    time.sleep(0.2)
    results = run_tests(ser)
    ser.close()
    gen_report(results, port)

if __name__ == "__main__":
    main()
