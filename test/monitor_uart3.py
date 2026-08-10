#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_uart3.py - UART3 实时状态监视 (tkinter GUI)

适用模式: RUN=0, DEBUG=1 (模式2 读写配置)
    拨码: RUN_CTRL(PH2)=OFF, DEBUG_CTRL(PH3)=ON

功能:
    - 选择 COM 口连接 UART3 (115200/8N1)
    - 后台线程定时轮询, GUI 实时刷新:
        RTC 时钟 / 日期 / DI 16通道 / DO 16通道 / AI0~3 / AO0~3
    - 刷新间隔可调 (默认 200ms)

说明:
    DI/DO 均为低有效: bit=0 (低电平) = 导通 -> 指示灯点亮(绿); bit=1 = 断开(灰)
    AI/AO 为 10bit (0~1023), 板上映射 0~3.3V

运行 (需 pyserial):
    python monitor_uart3.py
"""

import sys
import threading
import queue
import datetime

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[错误] 缺少 pyserial, 请先安装: pip install pyserial")
    sys.exit(1)

import tkinter as tk
from tkinter import ttk, messagebox

# ---------------- 协议常量/函数 (与 test_uart3.py 一致) ----------------
HEAD_WRITE = 0x37
HEAD_READ  = 0x36
BAUD       = 115200
TIMEOUT    = 0.3
FRAME_LEN  = 6


def crc8(data: bytes) -> int:
    """CRC-8: poly 0x07, init 0x00"""
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


def read_addr(ser, addr: int):
    ser.reset_input_buffer()
    ser.write(build_frame(HEAD_READ, addr, 0))
    return parse_frame(ser.read(FRAME_LEN), HEAD_READ)


# ---------------- 状态读取 (后台线程) ----------------
ADDR_RTC   = [0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09]  # ms s min hour day mon year
ADDR_DI    = [0x0A, 0x0B]    # DI_L, DI_H
ADDR_DO    = [0x0C, 0x0D]    # DO_L, DO_H
ADDR_AI    = [0x0E, 0x0F, 0x10, 0x11]  # AI0~3
ADDR_AO    = [0x12, 0x13, 0x14, 0x15]  # AO0~3


def read_all(ser):
    """读取全部状态, 返回 dict 或 None (任一失败)"""
    out = {}
    for name, addrs in (("rtc", ADDR_RTC), ("di", ADDR_DI), ("do", ADDR_DO),
                        ("ai", ADDR_AI), ("ao", ADDR_AO)):
        vals = []
        for a in addrs:
            v = read_addr(ser, a)
            if v is None:
                return None
            vals.append(v)
        out[name] = vals
    return out


def reader_loop(ser, stop_evt, out_q, interval):
    """后台轮询线程"""
    while not stop_evt.is_set():
        try:
            st = read_all(ser)
            if st is None:
                out_q.put(("err", "超时/无回复"))
            else:
                out_q.put(("ok", st))
        except Exception as e:
            out_q.put(("err", str(e)))
            break
        stop_evt.wait(interval / 1000.0)


# ---------------- 指示灯控件 ----------------
class Led(tk.Canvas):
    SIZE = 22

    def __init__(self, parent, text="", size=SIZE):
        super().__init__(parent, width=size, height=size + 14, highlightthickness=0,
                         bg=parent.cget("bg"))
        self.text = text
        self.size = size
        self._name = self.create_oval(2, 2, size - 2, size - 2, fill="#888888",
                                      outline="#666666")
        self._lab = self.create_text(size // 2, size + 8, text=text, font=("Consolas", 8))
        self.set(1)   # 默认断开(灰)

    def set(self, active):
        """active=1 导通(绿), 0 断开(灰)"""
        self.itemconfig(self._name, fill="#22cc44" if active else "#888888")

    def set_label(self, text):
        """设置下方文字 (Canvas 用 itemconfig)"""
        self.itemconfig(self._lab, text=text)


# ---------------- 主界面 ----------------
class MonitorApp:
    def __init__(self, root):
        self.root = root
        root.title("UART3 实时监视  (RUN=0, DEBUG=1 模式2)")
        root.geometry("980x720")
        root.configure(bg="#f0f0f0")

        self.ser = None
        self.stop_evt = threading.Event()
        self.thread = None
        self.out_q = queue.Queue()
        self.connected = False
        self.err_count = 0

        self._build_top()
        self._build_dash()
        self._build_status()

        self.root.after(60, self._poll_queue)

    # ---------- 顶部: 串口/连接/刷新间隔 ----------
    def _build_top(self):
        top = tk.Frame(self.root, bg="#f0f0f0")
        top.pack(fill=tk.X, padx=10, pady=(10, 4))

        tk.Label(top, text="串口:", bg="#f0f0f0").pack(side=tk.LEFT)
        self.port_cb = ttk.Combobox(top, width=18, state="readonly")
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        self.btn = ttk.Button(top, text="连接", command=self.toggle)
        self.btn.pack(side=tk.LEFT, padx=8)

        tk.Label(top, text="刷新间隔(ms):", bg="#f0f0f0").pack(side=tk.LEFT, padx=(20, 2))
        self.iv_entry = ttk.Entry(top, width=6)
        self.iv_entry.insert(0, "200")
        self.iv_entry.pack(side=tk.LEFT)

        self.conn_lab = tk.Label(top, text="未连接", bg="#f0f0f0", fg="#cc0000",
                                 font=("Microsoft YaHei", 10, "bold"))
        self.conn_lab.pack(side=tk.RIGHT)

        self.refresh_ports()

    # ---------- 仪表区 ----------
    def _build_dash(self):
        dash = tk.Frame(self.root, bg="#f0f0f0")
        dash.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

        # RTC
        rtc = tk.LabelFrame(dash, text="RTC 实时时钟", font=("Microsoft YaHei", 10, "bold"))
        rtc.pack(fill=tk.X, pady=2)
        self.lbl_time = tk.Label(rtc, text="--:--:--.---", font=("Consolas", 26, "bold"),
                                 fg="#0044cc", bg="#ffffff", relief=tk.GROOVE)
        self.lbl_time.pack(side=tk.LEFT, padx=10, pady=6)
        self.lbl_date = tk.Label(rtc, text="----/--/--", font=("Consolas", 14), fg="#333333")
        self.lbl_date.pack(side=tk.LEFT, padx=16)

        # DI / DO (各 2 行 x 8)
        self.di_leds = self._build_led_block(dash, "DI 输入 (bit=0 导通亮绿)")
        self.do_leds = self._build_led_block(dash, "DO 输出 (bit=0 导通亮绿)")
        for led, name in self.di_leds + self.do_leds:
            led.set_label(name)

        # AI / AO
        self.ai_bars, self.ai_lbls, self.ai_vlbls = self._build_val_block(dash, "AI 输入")
        self.ao_bars, self.ao_lbls, self.ao_vlbls = self._build_val_block(dash, "AO 输出")

    def _build_led_block(self, parent, title):
        box = tk.LabelFrame(parent, text=title, font=("Microsoft YaHei", 10, "bold"))
        box.pack(fill=tk.X, pady=2)
        grid = tk.Frame(box, bg="#f0f0f0")
        grid.pack(padx=6, pady=4)
        leds = []
        names = []
        # 两行: 低8位(0~7) 和 高8位(8~15)
        for row in range(2):
            for col in range(8):
                n = row * 8 + col
                led = Led(grid)
                led.grid(row=row, column=col, padx=5, pady=2)
                leds.append(led)
                names.append("D%d" % n)
        return list(zip(leds, names))

    def _build_val_block(self, parent, title):
        box = tk.LabelFrame(parent, text=title, font=("Microsoft YaHei", 10, "bold"))
        box.pack(fill=tk.X, pady=2)
        inner = tk.Frame(box, bg="#f0f0f0")
        inner.pack(fill=tk.X, padx=8, pady=4)
        bars, lbls, vlbls = [], [], []
        for i in range(4):
            tk.Label(inner, text="Ch%d" % i, width=4, anchor="e", bg="#f0f0f0").grid(
                row=i, column=0, padx=2, pady=3, sticky="e")
            var = tk.DoubleVar()
            bar = ttk.Progressbar(inner, maximum=1023, length=300, variable=var)
            bar.grid(row=i, column=1, padx=4, pady=3, sticky="we")
            lbl = tk.Label(inner, text="1023 / 3.30V", width=16, anchor="w",
                           font=("Consolas", 9), bg="#f0f0f0")
            lbl.grid(row=i, column=2, padx=4, pady=3, sticky="w")
            bars.append(var)
            lbls.append(lbl)
            vlbls.append(var)
        inner.columnconfigure(1, weight=1)
        return bars, lbls, vlbls

    def _build_status(self):
        bar = tk.Frame(self.root, bg="#e8e8e8", relief=tk.SUNKEN, bd=1)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        self.status = tk.Label(bar, text="就绪", anchor="w", bg="#e8e8e8")
        self.status.pack(fill=tk.X, padx=8, pady=2)

    # ---------- 串口 ----------
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and self.port_cb.get() == "":
            self.port_cb.current(0)

    def toggle(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            return
        self.connected = True
        self.btn.config(text="断开")
        self.conn_lab.config(text="已连接 %s" % port, fg="#008800")
        self.stop_evt.clear()
        self.out_q.queue.clear()
        self.thread = threading.Thread(target=reader_loop,
                                       args=(self.ser, self.stop_evt, self.out_q,
                                             self._interval()), daemon=True)
        self.thread.start()
        self.status.config(text="已连接, 开始监视...")

    def disconnect(self):
        self.stop_evt.set()
        if self.thread:
            self.thread.join(timeout=1)
            self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connected = False
        self.btn.config(text="连接")
        self.conn_lab.config(text="未连接", fg="#cc0000")
        self.status.config(text="已断开")

    def _interval(self):
        try:
            v = int(self.iv_entry.get())
            return max(50, min(5000, v))
        except ValueError:
            return 200

    # ---------- 刷新 ----------
    def _poll_queue(self):
        try:
            while True:
                kind, data = self.out_q.get_nowait()
                if kind == "err":
                    self.err_count += 1
                    self.status.config(text="[错误 x%d] %s" % (self.err_count, data))
                else:
                    self.err_count = 0
                    self._render(data)
                    self.status.config(text="刷新正常")
        except queue.Empty:
            pass
        self.root.after(60, self._poll_queue)

    def _render(self, st):
        # RTC: [ms, s, min, hour, day, mon, year]
        ms, s, mi, h, d, mo, y = st["rtc"]
        self.lbl_time.config(text="%02d:%02d:%02d.%03d" % (h, mi, s, ms & 0x3FF))
        self.lbl_date.config(text="20%02d/%02d/%02d" % (y, mo, d))

        # DI / DO: [L, H], bit=0 导通
        for i, (led, _) in enumerate(self.di_leds):
            v = st["di"][0 if i < 8 else 1]
            led.set(1 if ((v >> (i % 8)) & 1) == 0 else 0)
        for i, (led, _) in enumerate(self.do_leds):
            v = st["do"][0 if i < 8 else 1]
            led.set(1 if ((v >> (i % 8)) & 1) == 0 else 0)

        # AI / AO: 10bit + 电压
        for i in range(4):
            v = st["ai"][i]
            self.ai_bars[i].set(v)
            self.ai_lbls[i].config(text="%4d / %.2fV" % (v, v * 3.3 / 1023.0))
            v = st["ao"][i]
            self.ao_bars[i].set(v)
            self.ao_lbls[i].config(text="%4d / %.2fV" % (v, v * 3.3 / 1023.0))

    def on_close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = MonitorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
