#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sync_test.py - 精确时序控制的同步发收测试

同时启动 A 的 txc 和 B 的 rx，看是否能通信
"""

import sys, time, serial, threading
from queue import Queue

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

def main():
    if len(sys.argv) < 4:
        print("用法: python sync_test.py COM7 COM10 <freq_hz>")
        print("例如: python sync_test.py COM7 COM10 469500000")
        sys.exit(1)

    port_a, port_b, freq_str = sys.argv[1], sys.argv[2], sys.argv[3]
    freq = int(freq_str)

    print(f"连接 {port_a} (A-TX) 和 {port_b} (B-RX)...")
    ser_a = serial.Serial(port_a, BAUD, timeout=0.1)
    ser_b = serial.Serial(port_b, BAUD, timeout=0.1)
    print("✓\n")

    # 初始化
    print("初始化...")
    send_cmd(ser_a, "init")
    send_cmd(ser_b, "init")
    time.sleep(0.5)

    # 设置频率
    print(f"设频率 {freq/1e6:.3f}MHz...")
    send_cmd(ser_a, f"freq {freq}")
    send_cmd(ser_b, f"freq {freq}")
    time.sleep(0.3)

    # 结果队列
    result_a = Queue()
    result_b = Queue()

    def rx_thread():
        """后台：B 进入 rx"""
        print("[B] 进入 rx 10000ms...")
        ser_b.reset_input_buffer()
        ser_b.write(b"rx 10000\r\n")
        r = read_until(ser_b, PROMPT, 12)
        result_b.put(r.decode("utf-8", errors="ignore"))
        print("[B] rx 完成")

    def tx_thread():
        """后台：A 连发"""
        time.sleep(0.2)  # 等B进RX
        print("[A] 开始 txc 5000ms...")
        ser_a.reset_input_buffer()
        ser_a.write(b"txc 5000\r\n")
        r = read_until(ser_a, PROMPT, 7)
        result_a.put(r.decode("utf-8", errors="ignore"))
        print("[A] txc 完成")

    # 启动两个线程
    t_b = threading.Thread(target=rx_thread, daemon=True)
    t_a = threading.Thread(target=tx_thread, daemon=True)

    print("\n[同时启动 rx 和 txc]\n")
    t_b.start()
    time.sleep(0.05)  # 很短的延迟确保B先进RX
    t_a.start()

    # 等待完成
    t_b.join(timeout=15)
    t_a.join(timeout=15)

    # 获取结果
    try:
        r_a = result_a.get(timeout=1)
        r_b = result_b.get(timeout=1)
    except:
        print("✗ 超时")
        return

    print("\n" + "="*50)
    print("\n[A 发送结果]")
    print(r_a)

    print("\n[B 接收结果]")
    print(r_b)

    # 分析
    if "收到" in r_b:
        crc_ok = "CRC错" not in r_b
        data_ok = "AA 55 01 02" in r_b
        print(f"\n✓ 收到信号")
        print(f"  CRC: {'✓' if crc_ok else '✗'}")
        print(f"  数据: {'✓ AA 55 01 02' if data_ok else '✗ 其他数据'}")
    else:
        print(f"\n✗ 无包")

    ser_a.close()
    ser_b.close()

if __name__ == "__main__":
    main()
