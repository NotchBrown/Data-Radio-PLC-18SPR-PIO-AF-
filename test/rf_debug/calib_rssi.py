#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calib_rssi.py: 用信号源(N9310A/2023B) 470.000MHz CW 自动扫频定标板A/板B 晶振偏差。

原理:
  信号源输出 470.000MHz CW(极准) -> 板A/板B 各自扫频读实时 RSSI(0x1B)
  RSSI 峰值点 = 该板"设定该频率时实收 470.000" 的频率
  => 晶振偏差 Δ = 470.000 - 峰值点(重心法加权)

特性:
  纯 pyserial, 无 matplotlib/GUI 依赖(只标准库 + pyserial)
  自动扫 N 遍(默认3), 结果存 CSV + 终端汇总, 扫完自动退出

用法:
  python calib_rssi.py COM7 COM10 [start_kHz] [end_kHz] [step_kHz] [rounds]
  默认 469400 470600 5 3

输出:
  calib_result.csv : name,round,freq_hz,rssi_dbm (每板每点一行)

信号源设置:
  频率 470.000MHz, 调制 OFF(CW), 功率 -30~0dBm(N9310A 最大+13dBm),
  RF ON, 天线靠近两板
"""

import sys, time, serial, threading, re, statistics, csv

BAUD = 115200
PROMPT = b">\r\n"
OUT_CSV = "calib_result.csv"


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


def read_rssi(ser):
    """发 rssi 命令, 解析 RSSI=-XXdBm (失败返回 -200)"""
    r = send_cmd(ser, "rssi", timeout=2).decode("utf-8", errors="ignore")
    m = re.search(r"RSSI=(-?\d+)dBm", r)
    return int(m.group(1)) if m else -200


def sweep_worker(port, name, freqs, rows, lock, round_no, stop_evt):
    """循环扫频 worker: 每点 (name, round, freq, rssi) 追加到共享 rows"""
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except Exception as e:
        print(f"[{name}] 打开 {port} 失败: {e}", flush=True)
        stop_evt.set()
        return
    send_cmd(ser, "init")
    time.sleep(0.3)
    while not stop_evt.is_set():
        with lock:
            rnd = round_no[name] + 1
        for f in freqs:
            if stop_evt.is_set():
                break
            send_cmd(ser, f"freq {f}", timeout=1)
            time.sleep(0.03)
            rssi = read_rssi(ser)
            with lock:
                rows.append((name, rnd, f, rssi))
            time.sleep(0.01)
        with lock:
            round_no[name] = rnd
    ser.close()


def analyze(pts):
    """pts: [(freq, rssi)] -> (peak, centroid, rssi) or None(无信号)"""
    if not pts:
        return None
    best = max(pts, key=lambda p: p[1])
    if best[1] < -120:                       # 底噪水平, 判无信号
        return None
    min_rssi = min(r for _, r in pts)
    num = den = 0.0
    for f, r in pts:
        if abs(f - best[0]) <= 100000 and r - min_rssi > 0:
            num += f * (r - min_rssi)
            den += (r - min_rssi)
    return (best[0], int(num / den) if den > 0 else best[0], best[1])


def main():
    if len(sys.argv) < 3:
        print("用法: python calib_rssi.py COM7 COM10 [start_kHz] [end_kHz] [step_kHz] [rounds]")
        sys.exit(1)
    port_a, port_b = sys.argv[1], sys.argv[2]
    start = int(sys.argv[3]) if len(sys.argv) > 3 else 469400
    end = int(sys.argv[4]) if len(sys.argv) > 4 else 470600
    step = int(sys.argv[5]) if len(sys.argv) > 5 else 5
    rounds = int(sys.argv[6]) if len(sys.argv) > 6 else 3

    freqs = list(range(start * 1000, end * 1000 + 1, step * 1000))
    print(f"扫频 {start/1000:.3f}~{end/1000:.3f}MHz 步进{step}kHz ({len(freqs)}点/遍) x {rounds}遍")
    print("信号源: 470.000MHz CW RF ON, 天线靠近两板!")

    rows = []
    round_no = {"板A": 0, "板B": 0}
    lock = threading.Lock()
    stop_evt = threading.Event()

    ta = threading.Thread(target=sweep_worker,
                          args=(port_a, "板A", freqs, rows, lock, round_no, stop_evt),
                          daemon=True)
    tb = threading.Thread(target=sweep_worker,
                          args=(port_b, "板B", freqs, rows, lock, round_no, stop_evt),
                          daemon=True)
    ta.start()
    tb.start()

    try:
        while True:
            with lock:
                ra, rb = round_no["板A"], round_no["板B"]
            if min(ra, rb) >= rounds:
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\nCtrl+C 停止")
    finally:
        stop_evt.set()

    # 写 CSV
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["name", "round", "freq_hz", "rssi_dbm"])
        w.writerows(rows)
    print(f"\nCSV 已存: {OUT_CSV} ({len(rows)} 行)")

    # 汇总
    print("\n" + "=" * 56)
    for name in ("板A", "板B"):
        hist = []
        for rnd in range(1, rounds + 1):
            pts = [(f, r) for (n, rr, f, r) in rows if n == name and rr == rnd]
            r = analyze(pts)
            if r is None:
                print(f"[{name} round {rnd}] 未检测到信号", flush=True)
                continue
            peak, c, rssi = r
            dev = 470000000 - c              # Hz
            hist.append((peak, c, rssi, dev))
            print(f"[{name} round {rnd}] peak={peak/1e6:.3f}MHz "
                  f"centroid={c/1e6:.3f}MHz dev={dev/1000:+.1f}kHz rssi={rssi}dBm", flush=True)
        if hist:
            dev_med = statistics.median(d[3] for d in hist)
            cent_med = statistics.median(d[1] for d in hist)
            print(f"[RESULT] {name}: dev={dev_med/1000:+.1f}kHz "
                  f"(centroid={cent_med/1e6:.3f}MHz, {len(hist)}遍)")
        else:
            print(f"[RESULT] {name}: 无有效数据 (信号源开了吗? 天线近吗? 功率够吗?)")
    print("=> 设定 <centroid> Hz 时实际收/发 470.000MHz; dev=470.000-centroid (负=偏高, 正=偏低)")


if __name__ == "__main__":
    main()
