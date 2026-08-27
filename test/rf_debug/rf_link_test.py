#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rf_link_test.py - 双向通信测试

在找到的工作频率上测试：
1. 单向 link (A→B)
2. 反向 link2 (B→A)
3. 连续通信稳定性
4. 数据完整性
"""

import sys, time, serial, threading
from collections import deque

BAUD = 115200
PROMPT = b">\r\n"

def read_until(ser, marker, timeout):
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if marker in buf:
                break
        else:
            time.sleep(0.02)
    return buf

def send_cmd(ser, cmd, timeout=2):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    return read_until(ser, PROMPT, timeout)

def test_link(ser_a, ser_b, freq, direction="forward"):
    """测试单向 link"""
    if direction == "forward":
        print(f"\n[link A→B @ {freq/1e6:.3f}MHz]")
        tx_ser, rx_ser = ser_a, ser_b
        tx_name, rx_name = "A", "B"
    else:
        print(f"\n[link2 B→A @ {freq/1e6:.3f}MHz]")
        tx_ser, rx_ser = ser_b, ser_a
        tx_name, rx_name = "B", "A"

    # 设频率
    send_cmd(tx_ser, f"freq {freq}")
    send_cmd(rx_ser, f"freq {freq}")
    time.sleep(0.2)

    # RX 先进入接收
    rx_ser.reset_input_buffer()
    rx_ser.write(b"rx 5000\r\n")
    time.sleep(0.5)  # 等RX稳定

    # TX 连发（而不是单帧）
    tx_ser.reset_input_buffer()
    tx_ser.write(b"txc 3000\r\n")  # 连发3秒

    # 读结果
    r_tx = read_until(tx_ser, PROMPT, 2).decode("utf-8", errors="ignore")
    r_rx = read_until(rx_ser, PROMPT, 6).decode("utf-8", errors="ignore")

    print(f"[{tx_name}(tx)] {r_tx.split(chr(10))[0]}")

    # 分析 RX 结果
    if "收到" in r_rx:
        crc_ok = "CRC错" not in r_rx
        data_ok = "AA 55 01 02" in r_rx
        rssi = "?"
        data_bytes = ""

        # 提取RSSI和数据
        for line in r_rx.split("\n"):
            if "RSSI=" in line:
                for part in line.split():
                    if part.startswith("RSSI="):
                        rssi = part.split("=")[1]
                # 提取数据
                if ":" in line:
                    parts = line.split(":")
                    if len(parts) > 1:
                        data_bytes = parts[1].split("|")[0].strip()

        status = "✓" if (crc_ok and data_ok) else "▲"
        print(f"[{rx_name}(rx)] {status} {rssi} CRC={'✓' if crc_ok else '✗'} Data={'✓' if data_ok else '✗'}")
        if data_bytes:
            print(f"       收到: {data_bytes[:60]}")
        return crc_ok and data_ok
    else:
        print(f"[{rx_name}(rx)] ✗ 无包")
        return False

def test_txl_stability(ser_a, ser_b, freq, count=20):
    """测试连续发帧稳定性"""
    print(f"\n[txl stability @ {freq/1e6:.3f}MHz, {count} frames]")

    send_cmd(ser_a, f"freq {freq}")
    send_cmd(ser_b, f"freq {freq}")
    time.sleep(0.2)

    # 板B 用 scan 看解调标志
    ser_b.reset_input_buffer()
    ser_b.write(b"scan 5000\r\n")
    time.sleep(0.3)

    # 板A 连发
    ser_a.reset_input_buffer()
    ser_a.write(f"txl {count}\r\n".encode())

    # 读结果
    _ = read_until(ser_a, PROMPT, 5)
    r_b = read_until(ser_b, PROMPT, 6).decode("utf-8", errors="ignore")

    # 统计 irq 标志
    irq_0x40 = r_b.count("irq=0x40")  # RxDone
    irq_0x60 = r_b.count("irq=0x60")  # RxDone + CRC错
    irq_0x10 = r_b.count("irq=0x10")  # ValidHeader only

    print(f"  RxDone(0x40): {irq_0x40} frames")
    print(f"  RxDone+CRC错(0x60): {irq_0x60} frames")
    print(f"  ValidHeader(0x10): {irq_0x10} frames")

    return irq_0x40

def main():
    if len(sys.argv) < 4:
        print("用法: python rf_link_test.py COM7 COM10 <freq_hz>")
        print("例如: python rf_link_test.py COM7 COM10 469500000")
        sys.exit(1)

    port_a, port_b, freq_str = sys.argv[1], sys.argv[2], sys.argv[3]
    freq = int(freq_str)

    print(f"连接 {port_a} (A) 和 {port_b} (B)...")
    ser_a = serial.Serial(port_a, BAUD, timeout=0.1)
    ser_b = serial.Serial(port_b, BAUD, timeout=0.1)
    print("✓\n")

    # 初始化
    print("初始化...")
    send_cmd(ser_a, "init")
    send_cmd(ser_b, "init")
    time.sleep(0.5)

    print(f"\n工作频率: {freq/1e6:.3f}MHz\n")

    # 测试1: 单向 A→B
    ok1 = test_link(ser_a, ser_b, freq, "forward")

    # 测试2: 反向 B→A
    ok2 = test_link(ser_a, ser_b, freq, "reverse")

    # 测试3: 连续发帧稳定性
    ok3 = test_txl_stability(ser_a, ser_b, freq, 20)

    # 总结
    print("\n" + "="*50)
    print(f"A→B 单向: {'✓' if ok1 else '✗'}")
    print(f"B→A 反向: {'✓' if ok2 else '✗'}")
    print(f"连续帧(0x40): {ok3}/20")

    if ok1 and ok2 and ok3 > 10:
        print("\n✓ 通信正常！可以开始应用")
    else:
        print("\n⚠ 通信有问题，需要调查")

    ser_a.close()
    ser_b.close()

if __name__ == "__main__":
    main()
