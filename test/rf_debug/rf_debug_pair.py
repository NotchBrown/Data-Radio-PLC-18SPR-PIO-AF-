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
    """联动: 一板发帧, 另一板接收 (5s 窗口)"""
    tx_ser, rx_ser = (ser_b, ser_a) if reverse else (ser_a, ser_b)
    tx_name, rx_name = ("B", "A") if reverse else ("A", "B")

    print("联动测试: 板%s 发帧 [%s] -> 板%s 接收(5s 窗口)..." % (tx_name, hexs, rx_name))
    rx_ser.reset_input_buffer()
    tx_ser.reset_input_buffer()
    rx_ser.write(("rx 5000\r\n").encode("ascii"))   # 接收板先进入接收
    time.sleep(0.3)                                  # 给接收板进 RX 一点时间
    tx_ser.write(("tx %s\r\n" % hexs).encode("ascii"))

    r_tx = read_until(tx_ser, PROMPT, 2)
    r_rx = read_until(rx_ser, PROMPT, 6)
    show("板%s(tx)" % tx_name, r_tx)
    show("板%s(rx)" % rx_name, r_rx)


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
    """并行: 板B 自动扫频精调(tune base rng), 板A 持续发真实帧(txc)。
    板B 在后台线程跑 tune(扫±range找对端频率并校正), 板A 同时 txc 连续发帧。
    校正成功后板B 会打印"校正成功! 本机频率设为 X Hz"。"""
    result = {}
    print("自动扫频精调: 板B tune(±%sHz) + 板A txc 连发 ..." % rng)

    def tune_wait():
        ser_b.reset_input_buffer()
        ser_b.write(("tune %s %s\r\n" % (base, rng)).encode("ascii"))
        # tune 扫描约 (2*range/step)*300ms, 给足时间
        steps = int(rng) * 2 // 100000 + 2
        result["b"] = read_until(ser_b, PROMPT, steps * 0.4 + 3)

    t = threading.Thread(target=tune_wait)
    t.start()
    time.sleep(0.4)
    ser_a.reset_input_buffer()
    ser_a.write(b"txc 60000\r\n")   # 板A 长时间连发 (tune 全扫期间保持发帧)
    # 板A txc 最长10s, 与 tune 同步推进; 用较短的轮次便于看到进度
    result["a"] = read_until(ser_a, PROMPT, 12)
    t.join(timeout=20)
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
    """CRC 微调扫描: 板A 在候选频率用标准帧(txl)发, 板B 用 scan 看 irq 标志。
    irq=0x40 = RxDone 无 CRC = 链路打通!   irq=0x60 = RxDone+CRC错
    之前用 rx 阻塞收一帧方法不可靠, 改用 scan(和 txlscan 同款可靠时序)。"""
    freqs = [469800000, 469900000, 469950000, 470000000,
             470050000, 470100000, 470200000]
    print("CRC 微调扫描(scan法): 板A 标准帧 txl + 板B scan看irq ...")
    found = False
    for f in freqs:
        result = {}
        def rx_scan():
            ser_b.reset_input_buffer()
            ser_b.write(b"scan 1500\r\n")
            result["b"] = read_until(ser_b, PROMPT, 2.5)
        t = threading.Thread(target=rx_scan)
        t.start()
        time.sleep(0.35)
        ser_a.reset_input_buffer()
        ser_a.write(("freq %d\r\n" % f).encode("ascii"))
        time.sleep(0.2)
        ser_a.write(b"txl 15\r\n")
        result["a"] = read_until(ser_a, PROMPT, 2.5)
        t.join()

        text = result.get("b", b"").decode("utf-8", errors="replace")
        if "[irq=0x40]" in text:
            print("  %8.3f MHz: \u2713 RxDone \u65e0CRC \u94fe\u8def\u6253\u901a\uff01" % (f / 1e6))
            show("板A(tx)", result["a"])
            show("板B(scan)", result["b"])
            found = True
            break
        elif "[irq=0x60]" in text:
            print("  %8.3f MHz: RxDone\u4f46CRC\u9519" % (f / 1e6))
        else:
            print("  %8.3f MHz: \u672a\u6536\u5230" % (f / 1e6))
    if not found:
        print("  => \u672a\u627e\u5230 CRC \u6b63\u786e\u9891\u7387, \u8bd5\u6269\u5927\u8303\u56f4\u6216\u68c0\u67e5\u8c03\u5236")


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
