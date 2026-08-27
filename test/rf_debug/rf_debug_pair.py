#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rf_debug_pair.py - 双板 RF 调试 (配合 test/rf_debug 调试固件)

同时连接两块已烧录 rf_debug 固件的板子(UART3/115200), 交互发命令 + 联动收发测试。

用法:
    python rf_debug_pair.py --port-a COM4 --port-b COM5
    python rf_debug_pair.py                       # 交互选择两个 COM

交互命令:
    a <cmd>          发命令到板A (如 a ver / a reg 1D)
    b <cmd>          发命令到板B
    both <cmd>       同时发到两板 (如 both init / both freq 470000000)
    link <hex>       联动: 板A 发帧 + 板B 接收 (hex 如 "00 01 02")
    link2 <hex>      联动反向: 板B 发帧 + 板A 接收
    help             帮助
    q                退出

典型流程:
    both init
    a rssi
    link 00 01 02 03    -> 看板B 是否收到 + RSSI/SNR
"""

import sys
import time
import argparse
import threading
import re

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[错误] 缺少 pyserial, 请先安装: pip install pyserial")
    sys.exit(1)

BAUD = 115200
PROMPT = b">\r\n"      # 固件命令结束提示符


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


def read_until(ser, marker, timeout):
    """读串口直到出现 marker 或超时, 返回收到的字节"""
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf += chunk
            if marker in buf:
                break
        else:
            time.sleep(0.02)
    return buf


def send_cmd(ser, cmd, timeout=6):
    """发一条命令并读回复(到提示符)"""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    return read_until(ser, PROMPT, timeout)


def show(name, reply):
    text = reply.decode("utf-8", errors="replace").replace("\r\n", "\n").strip()
    print("---- [%s] ----" % name)
    print(text if text else "(无回复)")
    print()


def do_rssi_test(ser_a, ser_b, hexs="00 01 02 03", ms=4000):
    """并行诊断: 板B 进 scan(监听RSSI), 同时板A 连发帧。
    看板B 的 RSSI 是否随板A 发射而跳变:
      跳变(如 -115->-40) = 信号到达, 问题在解调
      不变(-115左右)    = 信号没到, 发射/天线/硬件问题
    """
    result = {}
    print("RSSI 诊断: 板A 连发 [%s], 板B 监听 %dms ..." % (hexs, ms))

    def rx_scan():
        ser_b.reset_input_buffer()
        ser_b.write(("scan %d\r\n" % ms).encode("ascii"))
        result["b"] = read_until(ser_b, PROMPT, ms / 1000.0 + 2)

    t = threading.Thread(target=rx_scan)
    t.start()
    time.sleep(0.5)                      # 给板B 进 scan 一点时间
    ser_a.reset_input_buffer()
    # 发真实数据帧(非载波): 连发多帧确保板B 采样能碰到, 并在 scan 里看解调标志
    ser_a.write(("txn 10 %s\r\n" % hexs).encode("ascii"))
    result["a"] = read_until(ser_a, PROMPT, ms / 1000.0 + 2)
    t.join()

    show("板A(tx)", result["a"])
    show("板B(rssi scan)", result["b"])


def do_link(ser_a, ser_b, hexs, reverse=False):
    """联动带ACK: 发送板 tx 单帧并等ACK, 接收板靠 loop 自动接收并自动回ACK。
    可靠方法: 不用 rx 阻塞法(已证不可靠); 接收板平时就 RXCONT(loop 自动收),
    脚本读接收板串口的 [收]/[回ACK] 和发送板的 [ACK OK] 判断链路。"""
    tx_ser, rx_ser = (ser_b, ser_a) if reverse else (ser_a, ser_b)
    tx_name, rx_name = ("B", "A") if reverse else ("A", "B")

    print("联动带ACK: 板%s 发 [%s] -> 板%s 自动收+回ACK..." % (tx_name, hexs, rx_name))
    rx_ser.reset_input_buffer()      # 清接收板缓冲(平时一直RXCONT自动收)
    tx_ser.reset_input_buffer()
    tx_ser.write(("tx %s\r\n" % hexs).encode("ascii"))

    r_tx = read_until(tx_ser, PROMPT, 3)      # 板tx: [TxDone]... [ACK OK]/[无ACK]
    r_rx = read_until(rx_ser, PROMPT, 3)      # 板rx: [收]... [回ACK]
    show("板%s(tx)" % tx_name, r_tx)
    show("板%s(rx)" % rx_name, r_rx)

    ok = b"[ACK OK]" in r_tx      # A 收到 ACK = 双向通(B 只有收到帧才会回 ACK)
    print("=> %s" % ("\u2713 \u94fe\u8def\u6253\u901a(\u542bACK)!" if ok else "\u2717 \u672a\u5b8c\u5168\u6253\u901a"))
    if not ok:
        # 失败诊断: 读两板 OPMODE(reg 01) + 实时 RSSI
        print("[\u8bca\u65ad] \u4e24\u677f\u72b6\u6001:")
        for ser, name in ((tx_ser, "\u677f%s(tx)" % tx_name), (rx_ser, "\u677f%s(rx)" % rx_name)):
            ser.reset_input_buffer()
            ser.write(b"reg 01\r\n")
            d1 = read_until(ser, PROMPT, 2)
            ser.reset_input_buffer()
            ser.write(b"rssi\r\n")
            d2 = read_until(ser, PROMPT, 2)
            show(name, d1 + d2)


def do_txc_test(ser_a, ser_b, ms=4000):
    """并行: 板B 阻塞接收, 板A 连续发真实LoRa帧(诊断解调)。
    板B 在后台线程进 rx, 板A 同时 txc 连续发帧, 不会互相阻塞。"""
    result = {}
    print("TX 解调诊断: 板A 连续发帧 %dms -> 板B 接收 ..." % ms)

    def rx_wait():
        ser_b.reset_input_buffer()
        ser_b.write(("rx %d\r\n" % ms).encode("ascii"))
        result["b"] = read_until(ser_b, PROMPT, ms / 1000.0 + 2)

    t = threading.Thread(target=rx_wait)
    t.start()
    time.sleep(0.5)                      # 给板B 进 rx 一点时间
    ser_a.reset_input_buffer()
    ser_a.write(("txc %d\r\n" % (ms - 500)).encode("ascii"))
    result["a"] = read_until(ser_a, PROMPT, ms / 1000.0 + 2)
    t.join()

    show("板A(tx)", result["a"])
    show("板B(rx)", result["b"])


def do_txcscan(ser_a, ser_b, ms=4000):
    """并行: 板B scan(带irq解调标志) + 板A 连续发真实帧。
    看板B 能否检测到任一 LoRa 解调标志:
      0x40=RxDone 0x10=ValidHeader 0x20=CRC错 0x80=RxTimeout
    全无 = 完全没同步(频率/调制大不匹配)  有0x10/0x20无0x40 = 头同步但payload失败"""
    result = {}
    print("解调标志诊断: 板A 连续发帧 %dms -> 板B scan(看irq标志) ..." % ms)

    def rx_scan():
        ser_b.reset_input_buffer()
        ser_b.write(("scan %d\r\n" % ms).encode("ascii"))
        result["b"] = read_until(ser_b, PROMPT, ms / 1000.0 + 2)

    t = threading.Thread(target=rx_scan)
    t.start()
    time.sleep(0.5)
    ser_a.reset_input_buffer()
    ser_a.write(("txc %d\r\n" % (ms - 500)).encode("ascii"))
    result["a"] = read_until(ser_a, PROMPT, ms / 1000.0 + 2)
    t.join()

    show("板A(tx)", result["a"])
    show("板B(scan)", result["b"])


def do_rfdiag(ser_a, ser_b, ms=4000):
    """射频诊断: A txc 连发 ms 帧, B 后台循环读 RSSI。
    A 发射时 B RSSI 升高 => 信号到了(解调问题); 不变 => 没发射/频率大偏"""
    result = {}

    def rssi_loop():
        vals = []
        end = time.time() + ms / 1000.0 + 0.5
        while time.time() < end:
            ser_b.reset_input_buffer()
            ser_b.write(b"rssi\r\n")
            r = read_until(ser_b, PROMPT, 0.6)
            m = re.search(rb"RSSI=(-?\d+)dBm", r)
            if m:
                vals.append(int(m.group(1)))
        result["b"] = vals

    t = threading.Thread(target=rssi_loop)
    t.start()
    time.sleep(0.4)
    ser_a.reset_input_buffer()
    ser_a.write(("txc %d\r\n" % ms).encode("ascii"))
    result["a"] = read_until(ser_a, PROMPT, ms / 1000.0 + 2)
    t.join()

    vals = result.get("b", [])
    if vals:
        print("\u677fB RSSI (A\u53d1\u5c04\u671f\u95f4): min=%d max=%d avg=%d"
              % (min(vals), max(vals), sum(vals) // len(vals)))
        if max(vals) > -80:
            print("=> A \u4fe1\u53f7\u5df2\u5230 B (RSSI=%d), \u662f\u89e3\u8c03\u95ee\u9898(\u67e5\u63a5\u6536\u914d\u7f6e)" % max(vals))
        else:
            print("=> A \u4fe1\u53f7\u672a\u5230 B (\u5e95\u566a~%d), \u68c0\u67e5 A \u53d1\u5c04/PA/\u9891\u7387" % max(vals))
    else:
        print("=> \u672a\u91c7\u5230 RSSI \u6570\u636e")
    show("\u677fA(tx)", result["a"])


def do_fetest(ser_a, ser_b, offset_khz=50):
    """FE 校正验证: A 设 470.000MHz 发 txl 标准帧, B 设 470.000+offset kHz 收帧读 FE。
    打印 FE 实测值, 用校正公式 tuned = B当前 - FE 校正后再次读 FE 验证(应≈0)。
    同时打印反向公式结果, 用于确定 FE 符号约定。"""
    base = 470000000
    b_freq = base + offset_khz * 1000
    result = {}

    def txA():
        ser_a.reset_input_buffer()
        ser_a.write(b"freq 470000000\r\n")
        read_until(ser_a, PROMPT, 2)
        ser_a.write(b"txl 80\r\n")
        result["a"] = read_until(ser_a, PROMPT, 9)

    t = threading.Thread(target=txA)
    t.start()
    time.sleep(0.5)

    print("FE 校正验证: 板A@470.000MHz 发 txl, 板B 偏 +%dkHz 收帧读 FE..." % offset_khz)
    ser_b.reset_input_buffer()
    ser_b.write(("freq %d\r\n" % b_freq).encode("ascii"))
    read_until(ser_b, PROMPT, 2)
    ser_b.reset_input_buffer()
    ser_b.write(b"fe\r\n")
    r1 = read_until(ser_b, PROMPT, 5)
    show("\u677fB(fe @ +%dkHz \u504f\u9891)" % offset_khz, r1)

    m = re.search(rb"FrequencyError=(-?\d+) Hz", r1)
    if m:
        fe = int(m.group(1))
        print("FE \u5b9e\u6d4b = %d Hz" % fe)
        tuned1 = b_freq - fe
        tuned2 = b_freq + fe
        print("\u6821\u6b63\u516c\u5f0fA(tuned=B-f) = %d Hz\n\u6821\u6b63\u516c\u5f0fB(tuned=B+f) = %d Hz" % (tuned1, tuned2))
        # \u7528\u516c\u5f0fA \u6821\u6b63\u540e\u9a8c\u8bc1
        ser_b.reset_input_buffer()
        ser_b.write(("freq %d\r\n" % tuned1).encode("ascii"))
        read_until(ser_b, PROMPT, 2)
        ser_b.reset_input_buffer()
        ser_b.write(b"fe\r\n")
        r2 = read_until(ser_b, PROMPT, 5)
        show("\u677fB(fe @ \u6821\u6b63\u540e A)", r2)
        m2 = re.search(rb"FrequencyError=(-?\d+) Hz", r2)
        if m2:
            fe2 = int(m2.group(1))
            print("\u6821\u6b63\u540e FE = %d Hz (\u63a5\u8fd10=\u516c\u5f0fA\u6b63\u786e, \u4e0d\u63a5\u8fd10=\u8bd5\u516c\u5f0fB)" % fe2)
    else:
        print("\u672a\u8bfb\u5230 FE (\u677fB \u6ca1\u6536\u5230\u5e27)")
    t.join()
    show("\u677fA(txl)", result.get("a", b""))


def do_freqsweep(ser_a, ser_b):
    """频率扫描: 板A 依次在候选频率发射真实帧, 板B 固定在470MHz接收。
    扫描多个频率点, 打印每个点的 RSSI, 找接收信号最强的频率(=实际匹配点),
    从而测出两板实际频率偏差。"""
    # 粗扫 + 细扫: 从 469.6 到 470.4, 步长 25kHz
    freqs = [469600000, 469625000, 469650000, 469675000, 469700000,
             469725000, 469750000, 469775000, 469800000, 469825000,
             469850000, 469875000, 469900000, 469925000, 469950000,
             469975000, 470000000, 470025000, 470050000, 470075000,
             470100000, 470125000, 470150000, 470175000, 470200000,
             470225000, 470250000, 470275000, 470300000, 470325000,
             470350000, 470375000, 470400000]
    print("频率扫描(精细): 板A 依次发射, 板B 固定 470MHz 接收 ...")
    best = None
    best_rssi = -200

    for f in freqs:
        result = {}
        def rx_wait():
            ser_b.reset_input_buffer()
            ser_b.write(b"rx 1200\r\n")
            result["b"] = read_until(ser_b, PROMPT, 2)
        t = threading.Thread(target=rx_wait)
        t.start()
        time.sleep(0.35)
        ser_a.reset_input_buffer()
        ser_a.write(("freq %d\r\n" % f).encode("ascii"))
        time.sleep(0.2)
        ser_a.write(b"txc 900\r\n")
        result["a"] = read_until(ser_a, PROMPT, 2)
        t.join()

        text = result.get("b", b"").decode("utf-8", errors="replace")
        rssi = None
        if "RSSI=" in text:
            for part in text.split():
                if part.startswith("RSSI="):
                    try:
                        rssi = int(part[5:].rstrip("dBm"))
                    except ValueError:
                        pass
        if rssi is not None:
            mark = ""
            if rssi > best_rssi:
                best_rssi = rssi
                best = f
                mark = "  <-- 最强"
            print("  %8.3f MHz: 收到 RSSI=%ddBm%s" % (f / 1e6, rssi, mark))
        else:
            print("  %8.3f MHz: 未收到" % (f / 1e6))

    if best is not None:
        dev = (best - 470000000) // 1000
        print("  => 最佳匹配: 板A=%.3f MHz (偏差 %+d kHz)" % (best / 1e6, dev))
        print("  => 板A 实际频率比设定值偏 %+d kHz" % (-dev))
    else:
        print("  => 所有频率均未收到, 不是简单频率偏差")


def do_autotune(ser_a, ser_b, base="470000000", rng="400000"):
    """并行: 板B 自动扫频精调(tune base rng), 板A 明确设基准频率后用标准帧(txl)连发。
    板B 后台跑 tune(扫±range找对端频率并 FE 精调), 板A 同时 txl 循环标准帧。
    注意时隙: txl 100帧×~91ms ≈9.1s; tune 扫 ±400k/100k 步进×500ms ≈5s, 完全覆盖。
    板A 先 freq base(实际发射 ≈ base+225kHz 晶振偏), 板B 扫到即 FE 校正。
    关键: 板A 必须用 txl(标准帧) 而非 txc(连续模式出非标准帧, 接收端解不出 RxDone)。
    固件 tune 已改为"收到任一帧(CRC对/错)都读 FE", 更鲁棒。
    验证点: 校正后板B 频率应 ≈ 板A 实际发射频率(≈470.225MHz 当 base=470)。
    若校正值明显相反方向, 说明 FE 符号反了(代码 tuned=f-fe 需改 f+fe)。"""
    result = {}
    print("自动扫频精调: 板A freq=%sHz 连发 + 板B tune(±%sHz) ..." % (base, rng))

    def tune_wait():
        ser_b.reset_input_buffer()
        ser_b.write(("tune %s %s\r\n" % (base, rng)).encode("ascii"))
        # tune 扫描: 25kHz 步进 (± range), 每点 ~600ms rx 阻塞
        # 点数 = 2*range/25000, 总时长 ≈ 点数 * 0.6s
        steps = int(rng) * 2 // 25000 + 2
        result["b"] = read_until(ser_b, PROMPT, steps * 0.7 + 3)

    t = threading.Thread(target=tune_wait)
    t.start()
    time.sleep(0.3)
    ser_a.reset_input_buffer()
    ser_a.write(("freq %s\r\n" % base).encode("ascii"))   # 板A 设基准频率
    _ = read_until(ser_a, PROMPT, 2)      # 等板A 确认改频, 再发 txl (状态确定, 输出不被吞)
    # 关键: 用 txl(标准单帧循环≈9.1s) 而不是 txc!
    # crcscan 注释已记录: txc(TX连续模式)产生非标准帧, 接收端解不出 RxDone -> 永远扫不到。
    # 已验证可靠配置 = 板A txl + 板B RXCONT 轮询 (crcscan 用此法找到 0x40 打通链路)。
    ser_a.write(b"txl 100\r\n")
    result["a"] = read_until(ser_a, PROMPT, 12)
    t.join(timeout=25)
    if t.is_alive():
        print("[提示] tune 仍在后台运行, 等待完成...")
        t.join()

    show("板A(tx)", result.get("a", b""))
    show("板B(tune)", result.get("b", b""))


def do_txlscan(ser_a, ser_b, ms=4000):
    """并行: 板B scan(带irq解调标志) + 板A 循环标准单帧(txl)。
    txl 每帧含前导码+header, 接收端应能正常解调 -> 应看到 ValidHeader/RxDone。
    与 txc(TX连续模式, 只发一次前导码)不同。"""
    result = {}
    print("标准帧解调诊断: 板A 循环标准帧 %dms -> 板B scan(看irq标志) ..." % ms)

    def rx_scan():
        ser_b.reset_input_buffer()
        ser_b.write(("scan %d\r\n" % ms).encode("ascii"))
        result["b"] = read_until(ser_b, PROMPT, ms / 1000.0 + 2)

    t = threading.Thread(target=rx_scan)
    t.start()
    time.sleep(0.4)
    ser_a.reset_input_buffer()
    ser_a.write(b"txl 60\r\n")   # 板A 循环60帧标准帧(帧间隔50ms, 约3s)
    result["a"] = read_until(ser_a, PROMPT, ms / 1000.0 + 2)
    t.join()

    show("板A(tx)", result["a"])
    show("板B(scan)", result["b"])


def do_crcscan(ser_a, ser_b):
    """CRC 微调扫描(修正时序版): 板A 标准帧(txl) + 板B scan 看 irq。
    之前全"未收到"的根因(非信号问题, 是测试时隙):
      1) scan 1500ms + txl 15帧: 每帧~91ms, 发射窗口 t=0.55~1.92s,
         而 scan 窗口只到 1.5s, 后半段发射在窗口外, 边缘损耗 -> 假未收到。
         修正: 改用与 txlscan 同款长时序 scan 5000 + txl 40(发射完全落在窗口内)。
      2) 据 freqsweep: 板A 晶振偏高 ~+225~300kHz, 板B@470 的匹配点在
         板A 设定 ~469.7MHz, 旧列表从 469.8 起漏掉匹配区。修正: 补 469.6~469.8 细扫。
      3) 首轮固定 470/470 做对照, 先验证测试方法可靠(应见 0x60)再扫频, 不盲目试错。
    帧同步/位同步条件(txl 每帧含前导码+header)与 txlscan 完全一致, 不改变。
    irq: 0x40=RxDone无CRC(链路打通) 0x60=RxDone+CRC错 0x10=ValidHeader"""
    import re
    # 细扫匹配区(据 freqsweep 板A 需设定 ~469.7) + 470 附近 + 高段
    freqs = [469600000, 469700000, 469725000, 469750000, 469775000,
             469800000, 469900000, 470000000, 470100000, 470200000,
             470300000]
    seq = [470000000] + freqs   # 首轮固定 470/470 对照
    print("CRC 微调扫描(修正时序): 板A txl + 板B scan ...")
    print("  首轮对照: 固定 470/470MHz, 验证测试方法(应见 0x60 或 0x40)")
    found = False
    for idx, f in enumerate(seq):
        tag = "\u5bf9\u7167" if idx == 0 else "\u626b\u9891"
        result = {}
        def rx_scan():
            ser_b.reset_input_buffer()
            ser_b.write(b"scan 5000\r\n")
            result["b"] = read_until(ser_b, PROMPT, 7)
        t = threading.Thread(target=rx_scan)
        t.start()
        time.sleep(0.3)
        ser_a.reset_input_buffer()
        ser_a.write(("freq %d\r\n" % f).encode("ascii"))
        time.sleep(0.4)                  # 等板A 改频+稳定
        ser_a.write(b"txl 40\r\n")       # 40帧*~91ms ≈ 3.6s, 完全落在 scan 5000 内
        result["a"] = read_until(ser_a, PROMPT, 7)
        t.join()

        text = result.get("b", b"").decode("utf-8", errors="replace")
        hits = set(int(v, 16) for v in re.findall(r"irq=0x([0-9A-Fa-f]{2})", text))
        if 0x40 in hits:
            crc_ok = (0x60 not in hits)
            if crc_ok:
                print("  [%s] %8.3f MHz: \u2713 RxDone \u65e0CRC \u94fe\u8def\u6253\u901a\uff01" % (tag, f / 1e6))
                show("\u677fA(tx)", result["a"])
                show("\u677fB(scan)", result["b"])
                found = True
            else:
                print("  [%s] %8.3f MHz: RxDone \u4f46\u671f\u95f4\u4e5f\u6709 CRC \u9519 (0x40+0x60 \u6df7\u5408)" % (tag, f / 1e6))
            if found:
                break
        elif 0x60 in hits:
            print("  [%s] %8.3f MHz: \u6536\u5230\u4f46 CRC \u9519 (0x60)" % (tag, f / 1e6))
        elif 0x10 in hits:
            print("  [%s] %8.3f MHz: \u53ea\u540c\u6b65\u5230\u5934 (0x10)" % (tag, f / 1e6))
        else:
            print("  [%s] %8.3f MHz: \u672a\u6536\u5230" % (tag, f / 1e6))
    if not found:
        print("  => \u672a\u627e\u5230\u7eaf\u5782 CRC \u6b63\u786e\u9891\u7387")
        print("     \u82e5\u5168\u90e8\u90fd\u662f 0x60(\u6536\u5230\u4f46CRC\u9519) -> \u9891\u7387\u504f\u5dee\u662f\u6839\u56e0,")
        print("     \u63a5\u7740\u8dd1 autotune \u8ba9\u677fB \u7528 FrequencyError \u81ea\u52a8\u5bf9\u51c6")
        print("     \u82e5\u8fde\u5bf9\u7167\u90fd\u672a\u6536\u5230 -> \u65f6\u5e8f\u4ecd\u6709\u95ee\u9898,\u68c0\u67e5\u4e32\u53e3/scan")


def help_text():
    print("""交互命令:
  a <cmd>         发命令到板A (如: a ver / a reg 1D / a tx 00 01)
  b <cmd>         发命令到板B
  both <cmd>      同时发到两板 (如: both init / both freq 470000000)
  link <hex>      联动: 板A 发帧 + 板B 接收 (hex 如 "00 01 02")
  link2 <hex>     联动反向: 板B 发帧 + 板A 接收
  rssitest [hex]  并行诊断: 板B监听RSSI + 板A连发(默认 00 01 02 03)
  txctest         并行诊断: 板A连续发真实LoRa帧 + 板B接收(解调)
  txcscan         并行: 板A连续发帧 + 板B scan看解调标志
  rfdiag          射频诊断: A连发 + B循环读RSSI(分辨信号到达/解调)
  fetest [khz]     FE验证: A发帧 + B偏频读FE + 校正验证(默认偏50kHz)
  txlscan         并行: 板A循环标准帧 + 板B scan看解调标志
  crcscan         并行: 板A标准帧扫频 + 板B判断CRC(找链路打通频率)
  freqsweep       频率扫描: 板A扫频发射 + 板B固定470MHz收, 测频率偏差
  autotune [base] [r]  并行: 板B自动扫频精调 + 板A连发帧
  help            帮助
  q               退出
固件命令(help 内详): ver/reg/wreg/init/freq/sf/bw/cr/pwr/tx/rx/rssi
""")


def main():
    ap = argparse.ArgumentParser(description="双板 RF 调试 (test/rf_debug 固件)")
    ap.add_argument("--port-a", help="板A COM口, 如 COM4")
    ap.add_argument("--port-b", help="板B COM口, 如 COM5")
    args = ap.parse_args()

    port_a = args.port_a or choose_port("板A")
    port_b = args.port_b or choose_port("板B")
    if port_a == port_b:
        print("[错误] 板A/板B 不能是同一个 COM 口")
        sys.exit(1)

    ser_a = serial.Serial(port_a, BAUD, timeout=0.1)
    ser_b = serial.Serial(port_b, BAUD, timeout=0.1)
    print("\n已连接 板A=%s 板B=%s" % (port_a, port_b))
    help_text()

    try:
        while True:
            try:
                line = input("dbg> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            parts = line.split(None, 1)
            cmd = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else ""

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "help":
                help_text()
            elif cmd == "a":
                show("板A", send_cmd(ser_a, arg))
            elif cmd == "b":
                show("板B", send_cmd(ser_b, arg))
            elif cmd == "both":
                show("板A", send_cmd(ser_a, arg))
                show("板B", send_cmd(ser_b, arg))
            elif cmd == "rssitest":
                do_rssi_test(ser_a, ser_b, arg or "00 01 02 03")
            elif cmd == "txctest":
                do_txc_test(ser_a, ser_b)
            elif cmd == "txcscan":
                do_txcscan(ser_a, ser_b)
            elif cmd == "rfdiag":
                do_rfdiag(ser_a, ser_b)
            elif cmd == "fetest":
                parts = arg.split()
                off = int(parts[0]) if len(parts) > 0 and parts[0].isdigit() else 50
                do_fetest(ser_a, ser_b, off)
            elif cmd == "txlscan":
                do_txlscan(ser_a, ser_b)
            elif cmd == "crcscan":
                do_crcscan(ser_a, ser_b)
            elif cmd == "freqsweep":
                do_freqsweep(ser_a, ser_b)
            elif cmd == "autotune":
                parts = arg.split()
                base = parts[0] if len(parts) > 0 else "470000000"
                rng = parts[1] if len(parts) > 1 else "400000"
                do_autotune(ser_a, ser_b, base, rng)
            elif cmd in ("link", "link2"):
                do_link(ser_a, ser_b, arg, reverse=(cmd == "link2"))
            else:
                print("未知命令, 输入 help")
    finally:
        ser_a.close()
        ser_b.close()
        print("已断开")


if __name__ == "__main__":
    main()
