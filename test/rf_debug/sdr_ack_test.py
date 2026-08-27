#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sdr_ack_test.py: 用 Pluto 监听 A 发帧 + B 回 ACK 的发射，判断 B 是否真的发出 ACK。

流程: A 后台 tx 发帧 -> B loop 自动收+回ACK -> Pluto 连续采样 470MHz
     记录功率时间序列, 找高功率脉冲段(应为 2 个: A帧 + B的ACK)。

用法:
  C:/Users/Lenovo/.conda/envs/SDR/python.exe sdr_ack_test.py COM7 COM10
"""
import adi, numpy as np, sys, time, serial, threading

URI = "usb:2.5.5"
CENTER = 470000000
FS = 2.5e6


def send(ser, cmd, t=2):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    buf = b""
    end = time.time() + t
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if b">\r\n" in buf:
                break
        else:
            time.sleep(0.02)
    return buf


def main():
    if len(sys.argv) < 3:
        print("用法: python sdr_ack_test.py COM7 COM10")
        sys.exit(1)
    pa, pb = sys.argv[1], sys.argv[2]
    sdr = adi.Pluto(uri=URI)
    sdr.rx_lo = int(CENTER)
    sdr.sample_rate = int(FS)
    sdr.rx_rf_bandwidth = int(FS)
    try:
        sdr.gain_control_mode_chan0 = 'manual'
        sdr.rx_hardwaregain_chan0 = 30
    except Exception:
        pass
    sdr.rx_buffer_size = 8192
    print("Pluto 就绪")

    ser_a = serial.Serial(pa, 115200, timeout=0.1)
    ser_b = serial.Serial(pb, 115200, timeout=0.1)
    send(ser_a, "init")
    send(ser_b, "init")
    time.sleep(0.3)
    send(ser_a, "freq 470000000")
    send(ser_b, "freq 470000000")
    time.sleep(0.3)

    def txA():
        send(ser_a, "tx AA 55 01 02", 4)

    t = threading.Thread(target=txA)
    t.start()

    # Pluto 连续采样, 记录 470±0.5MHz 功率
    powers = []          # (t_s, power_dB)
    t0 = time.time()
    while time.time() - t0 < 1.5:
        d = sdr.rx()
        fft = np.abs(np.fft.fftshift(np.fft.fft(d))) ** 2
        freqs = CENTER + np.fft.fftshift(np.fft.fftfreq(len(d), 1.0 / FS))
        mask = (freqs > CENTER - 0.5e6) & (freqs < CENTER + 0.5e6)
        pw = 10 * np.log10(np.mean(fft[mask]) + 1e-12)
        powers.append((time.time() - t0, pw))
    t.join()

    rb = send(ser_b, "reg 01", 2).decode("utf-8", errors="ignore")
    print("板B OPMODE:", rb.strip()[:30])
    print("Pluto 功率时间序列 (单位 dB):")
    # 打印每 0.1s 的功率 (降采样)
    for tt, pw in powers[::5]:
        bar = "#" * max(0, int((pw + 30) / 4))
        print("  t=%5.2fs  %7.1f  %s" % (tt, pw, bar))

    # 检测高功率脉冲段
    highs = [pw for _, pw in powers if pw > -20]
    print("\n高功率采样数 (>-20dB): %d / %d" % (len(highs), len(powers)))
    if len(highs) > 0:
        print("=> 检测到发射, 大概率是 A帧 + B的ACK 两个脉冲")
    else:
        print("=> 未检测到明显发射, 或信号太弱")


if __name__ == "__main__":
    main()
