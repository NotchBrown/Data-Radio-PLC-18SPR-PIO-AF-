#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""sdr_measure.py: 用 Pluto SDR 测板A/板B 实际发射频率。

原理:
  板A/B 发 LoRa 载波(carr, TX连续模式), Pluto RX @470MHz 采样,
  FFT 功率谱找信号包络峰值/重心 = 该板实际发射频率。

用法:
  C:/Users/Lenovo/.conda/envs/SDR/python.exe sdr_measure.py COM7 COM10

前置:
  Pluto 插 USB(uri usb:2.5.5), 板A/B 串口
"""
import adi, numpy as np, sys, time, serial, threading

URI = "usb:2.5.5"
CENTER = 470000000
FS = 2.5e6


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


def send(ser, cmd, t=2):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    return read_until(ser, b">\r\n", t)


def meas(sdr, seconds=1.0):
    """采样并平均功率谱, 返回 (峰值频率, 信号重心频率, 峰值dB)"""
    sdr.rx_lo = int(CENTER)
    sdr.sample_rate = int(FS)
    sdr.rx_rf_bandwidth = int(FS)
    try:
        sdr.gain_control_mode_chan0 = 'manual'
        sdr.rx_hardwaregain_chan0 = 20
    except Exception:
        pass
    sdr.rx_buffer_size = 8192
    freqs = CENTER + np.fft.fftshift(np.fft.fftfreq(8192, 1.0 / FS))
    psd = np.zeros(8192)
    it = 0
    endt = time.time() + seconds
    while time.time() < endt:
        d = sdr.rx()
        psd += np.abs(np.fft.fftshift(np.fft.fft(d))) ** 2
        it += 1
    if it == 0:
        return 0, 0, -200
    psd /= it
    mask = (freqs > CENTER - 0.6e6) & (freqs < CENTER + 0.6e6)
    fw = freqs[mask]
    pw = psd[mask]
    peak_i = int(np.argmax(pw))
    # 重心: 高于峰值-20dB 的区间
    th = np.max(pw) * 0.01
    sel = pw > th
    if sel.any():
        cent = np.sum(fw[sel] * pw[sel]) / np.sum(pw[sel])
    else:
        cent = fw[peak_i]
    return fw[peak_i], cent, 10 * np.log10(np.max(pw))


def main():
    if len(sys.argv) < 3:
        print("用法: python sdr_measure.py COM7 COM10")
        sys.exit(1)
    pa, pb = sys.argv[1], sys.argv[2]
    sdr = adi.Pluto(uri=URI)
    print("Pluto 连接成功\n")
    for port, name in ((pa, "板A"), (pb, "板B")):
        ser = serial.Serial(port, 115200, timeout=0.1)
        send(ser, "init")
        time.sleep(0.3)
        send(ser, "freq 470000000")
        time.sleep(0.3)

        def tx():
            send(ser, "carr 4000", 6)

        t = threading.Thread(target=tx)
        t.start()
        time.sleep(0.8)
        fpk, fcen, pkdb = meas(sdr, 1.0)
        t.join()
        ser.close()
        print(f"{name}: 峰值 {fpk/1e6:.4f}MHz  重心 {fcen/1e6:.4f}MHz  功率 {pkdb:.1f}dB")
        if pkdb > -90:
            print(f"   => 实际发射 ≈ {fcen/1e6:.4f}MHz  (偏差 {(fcen-CENTER)/1000:+.1f}kHz)\n")
        else:
            print(f"   => 未检测到发射 (功率 {pkdb:.1f}dB)\n")


if __name__ == "__main__":
    main()
