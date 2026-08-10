#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dac_test.py - DAC (AD5314, AO0~3) 逐通道误差测试

适用: 模式2 (RUN=0, DEBUG=1), UART3 串口 115200/8N1
流程:
  1. 选串口连接
  2. 逐个通道测试 (一次一个通道, 每通道均匀 8 个点 0~10V)
  3. 每点: 脚本写入 DAC 码 -> 你用万用表/示波器量该通道电压(0~10V) -> 输入实测值
     (直接回车 = 接受目标值)
  4. 全部测完导出报告: test/report_dac_<时间戳>.md + .csv

说明:
  - 板子外部电路把 DAC 0~3.3V 放大到 0~10V
  - DAC 码 = 目标电压 / 10 * 1023
  - 误差 = 实测 - 目标 (mV / %)

运行: python dac_test.py   (需 pyserial)
"""

import sys
import os
import time
import datetime

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[错误] 缺少 pyserial, 请先安装: pip install pyserial")
    sys.exit(1)

# ---------------- 协议 (同 monitor_uart3.py) ----------------
HEAD_WRITE = 0x37
HEAD_READ  = 0x36
BAUD       = 115200
TIMEOUT    = 0.5
FRAME_LEN  = 6

AO_ADDR    = [0x12, 0x13, 0x14, 0x15]   # AO0~AO3 写地址

POINTS     = 8                          # 每通道均匀测试点数
FULL_V     = 10.0                       # 外部放大后的满量程电压
FULL_CODE  = 1023                       # DAC 10bit 满量程


def crc8(data: bytes) -> int:
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


def write_addr(ser, addr: int, val: int):
    """写地址, 返回回复(设定后值) 或 None"""
    ser.reset_input_buffer()
    ser.write(build_frame(HEAD_WRITE, addr, val & 0xFFFF))
    return parse_frame(ser.read(FRAME_LEN), HEAD_WRITE)


def choose_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[错误] 未检测到串口, 请检查 CH340N USB 连接")
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


def ask_channel():
    """询问要测试的通道, 返回通道号或 None(结束)"""
    while True:
        s = input("\n选择要测试的通道 AO0~AO3 (输入 0~3; q=结束并导出): ").strip()
        if s.lower() in ("q", "quit", "exit"):
            return None
        try:
            ch = int(s)
            if 0 <= ch <= 3:
                return ch
        except ValueError:
            pass
        print("无效输入, 请输入 0~3")


def measure_one(ser, ch, target_v):
    """写目标电压并让用户实测, 返回 (code, measured_v)"""
    code = round(target_v / FULL_V * FULL_CODE)
    if code > FULL_CODE:
        code = FULL_CODE
    # 写入 DAC
    r = write_addr(ser, AO_ADDR[ch], code)
    if r is None:
        print("  [警告] 写入无回复, 重试...")
        time.sleep(0.3)
        r = write_addr(ser, AO_ADDR[ch], code)
    # 提示测量
    print("  -> 已写入 AO%d: 目标 %.3f V (DAC码 %d/%d, 回读=%s)"
          % (ch, target_v, code, FULL_CODE, r))
    while True:
        s = input("     请测量 AO%d 实际电压(0~10V), 回车=接受目标值: " % ch).strip()
        if s == "":
            return code, target_v
        try:
            mv = float(s)
            if 0.0 <= mv <= 12.0:      # 允许稍超量程
                return code, mv
            print("     超出合理范围(0~12V), 请重输")
        except ValueError:
            print("     无效输入, 请重输")


def test_channel(ser, ch, results):
    print("\n========== 测试 AO%d ==========" % ch)
    print("请将万用表/示波器接到 AO%d (0~10V 量程)" % ch)
    input("准备好后按回车开始...")
    ch_res = []
    for i in range(POINTS):
        target_v = i * FULL_V / (POINTS - 1)      # 0, 1.43, ..., 10 V 均匀 8 点
        print("\n[点 %d/%d] 目标 %.3f V" % (i + 1, POINTS, target_v))
        code, meas = measure_one(ser, ch, target_v)
        ch_res.append({"point": i + 1, "target": target_v, "code": code,
                       "measured": meas})
        time.sleep(0.05)
    results[ch] = ch_res
    print("\nAO%d 测试完成" % ch)


def err_info(row):
    """返回 (err_mv, err_pct)"""
    err = (row["measured"] - row["target"]) * 1000.0          # mV
    pct = (row["measured"] - row["target"]) / row["target"] * 100.0 if row["target"] else 0.0
    return err, pct


def export(results, path_base):
    lines = []
    lines.append("# DAC 通道误差测试报告")
    lines.append("")
    lines.append("- 日期: %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("- 量程: 0~10V (外部放大 0~3.3V), 每通道 %d 个均匀点" % POINTS)
    lines.append("- 设备: STM8S208MB + AD5314 (模式2 串口配置)")
    lines.append("")
    for ch in sorted(results):
        rows = results[ch]
        lines.append("## AO%d" % ch)
        lines.append("")
        lines.append("| 点 | 目标电压(V) | DAC码 | 实测电压(V) | 误差(mV) | 误差(%) |")
        lines.append("|----|------------|-------|-------------|----------|---------|")
        errs = []
        for row in rows:
            err, pct = err_info(row)
            errs.append(abs(err))
            lines.append("| %d | %.3f | %d | %.3f | %+.1f | %+.3f |"
                         % (row["point"], row["target"], row["code"],
                            row["measured"], err, pct))
        avg = sum(errs) / len(errs) if errs else 0
        mx = max(errs) if errs else 0
        lines.append("")
        lines.append("- 平均绝对误差: %.1f mV, 最大绝对误差: %.1f mV" % (avg, mx))
        lines.append("")
    # 写文件
    with open(path_base + ".md", "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    # CSV
    with open(path_base + ".csv", "w", encoding="utf-8-sig", newline="") as f:
        import csv
        w = csv.writer(f)
        w.writerow(["channel", "point", "target_V", "dac_code", "measured_V",
                    "err_mV", "err_pct"])
        for ch in sorted(results):
            for row in results[ch]:
                err, pct = err_info(row)
                w.writerow([ch, row["point"], "%.3f" % row["target"], row["code"],
                            "%.3f" % row["measured"], "%.1f" % err, "%.3f" % pct])
    print("\n报告已导出: %s.md / %s.csv" % (path_base, path_base))


def main():
    port = choose_port()
    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    except Exception as e:
        print("[错误] 打开串口失败: %s" % e)
        sys.exit(1)
    print("已连接 %s, 请确认板子处于模式2 (RUN=0, DEBUG=1)" % port)

    results = {}
    try:
        while True:
            ch = ask_channel()
            if ch is None:
                break
            test_channel(ser, ch, results)
    finally:
        ser.close()

    if results:
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "report_dac_%s" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        export(results, base)
    else:
        print("未测试任何通道")


if __name__ == "__main__":
    main()
