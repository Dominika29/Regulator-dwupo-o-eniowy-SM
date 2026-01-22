import sys
import re
import time
import csv
import threading
from dataclasses import dataclass
from datetime import datetime

import serial
from serial.tools import list_ports

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from typing import Optional


import os

DATA_DIR = "dane"
os.makedirs(DATA_DIR, exist_ok=True)

LINE_RE = re.compile(
    r"T=(?P<T>-?\d+(?:\.\d+)?)\s+SP=(?P<SP>-?\d+(?:\.\d+)?)\s+OUT=(?P<OUT>-?\d+(?:\.\d+)?)"
    r"(?:\s+KP=(?P<KP>-?\d+(?:\.\d+)?)\s+KI=(?P<KI>-?\d+(?:\.\d+)?)\s+KD=(?P<KD>-?\d+(?:\.\d+)?))?"
)


# Opcjonalnie (jeśli wysyłasz też KP/KI/KD w tej samej linii)
K_RE = re.compile(r"KP=(?P<KP>-?\d+(\.\d+)?)\s+KI=(?P<KI>-?\d+(\.\d+)?)\s+KD=(?P<KD>-?\d+(\.\d+)?)")


@dataclass
class Sample:
    t: float
    T: float
    SP: float
    OUT: float
    KP: Optional[float] = None
    KI: Optional[float] = None
    KD: Optional[float] = None
    phase: str = ""



def list_serial_ports():
    return [p.device for p in list_ports.comports()]


def auto_pick_stlink_port():
    ports = list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "stlink" in desc or "st-link" in desc or "stmicroelectronics" in desc:
            return p.device
        if "vid:pid=0483" in hwid:  # ST
            return p.device
    return None


class SerialWorker:
    def __init__(self, on_line_cb):
        self.on_line_cb = on_line_cb
        self.ser = None
        self.thread = None
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud):
        self.disconnect()
        self.stop_evt.clear()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        self.thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_evt.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send_line(self, s: str):
        if not s.endswith("\n"):
            s += "\n"
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(s.encode("ascii", errors="ignore"))

    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                try:
                    txt = line.decode("utf-8", errors="ignore").strip()
                except Exception:
                    txt = ""
                if txt:
                    self.on_line_cb(txt)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("STM32 PID – GUI + UART + Logger + Plot")

        # state
        self.worker = SerialWorker(self._on_uart_line_thread)
        self.samples = []  # list[Sample]
        self.last_k = (None, None, None)

        self.csv_file = None
        self.csv_writer = None
        self.log_path = None
        self.t0 = None
        self.ui_queue = []
        self.ui_lock = threading.Lock()

        # UI
        self._build_ui()
        self._refresh_ports()

        # --- test identyfikacyjny (0% -> 100% -> 0%) ---
        self.test_active = False
        self.test_t0 = None
        self.test_hz = 2.0

        self.test_pre_s = 180.0  # czas na 0%
        self.test_heat_s = 1800.0  # czas na 100%
        self.test_cool_s = 600.0  # czas na 0% (chłodzenie)

        self.test_next_get = 0.0
        self.test_phase = ""

        self.after(50, self._poll_ui_queue)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- UI ----------
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value="")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="Odśwież", command=self._refresh_ports).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Auto (STLink)", command=self._auto_select).pack(side=tk.LEFT, padx=4)

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(16, 2))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_entry = ttk.Entry(top, textvariable=self.baud_var, width=8)
        self.baud_entry.pack(side=tk.LEFT, padx=4)

        self.status_var = tk.StringVar(value="Rozłączony")
        ttk.Button(top, text="Połącz", command=self.on_connect).pack(side=tk.LEFT, padx=10)
        ttk.Button(top, text="Rozłącz", command=self.on_disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.status_var, foreground="blue").pack(side=tk.LEFT, padx=12)

        mid = ttk.Frame(self)
        mid.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        # Telemetria
        tel = ttk.LabelFrame(mid, text="Telemetria (live)")
        tel.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))

        self.t_var = tk.StringVar(value="0.00")
        self.sp_var = tk.StringVar(value="0.00")
        self.out_var = tk.StringVar(value="0.0")

        row = ttk.Frame(tel)
        row.pack(fill=tk.X, padx=8, pady=8)

        self._big_label(row, "T [°C]", self.t_var).pack(side=tk.LEFT, padx=10)
        self._big_label(row, "SP [°C]", self.sp_var).pack(side=tk.LEFT, padx=10)
        self._big_label(row, "OUT [%]", self.out_var).pack(side=tk.LEFT, padx=10)

        # Sterowanie
        ctrl = ttk.LabelFrame(mid, text="Sterowanie")
        ctrl.pack(side=tk.LEFT, fill=tk.X, expand=True)

        grid = ttk.Frame(ctrl)
        grid.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(grid, text="SET [°C]:").grid(row=0, column=0, sticky="w")
        self.set_var = tk.StringVar(value="23.0")
        ttk.Entry(grid, textvariable=self.set_var, width=8).grid(row=0, column=1, padx=6)
        ttk.Button(grid, text="Wyślij SET", command=self.send_set).grid(row=0, column=2, padx=6)

        ttk.Label(grid, text="KP:").grid(row=1, column=0, sticky="w")
        self.kp_var = tk.StringVar(value="10.0")
        ttk.Entry(grid, textvariable=self.kp_var, width=8).grid(row=1, column=1, padx=6, sticky="w")

        ttk.Label(grid, text="KI:").grid(row=1, column=2, sticky="w")
        self.ki_var = tk.StringVar(value="0.50")
        ttk.Entry(grid, textvariable=self.ki_var, width=8).grid(row=1, column=3, padx=6, sticky="w")

        ttk.Label(grid, text="KD:").grid(row=1, column=4, sticky="w")
        self.kd_var = tk.StringVar(value="0.00")
        ttk.Entry(grid, textvariable=self.kd_var, width=8).grid(row=1, column=5, padx=6, sticky="w")

        ttk.Button(grid, text="Wyślij PID", command=self.send_pid).grid(row=1, column=6, padx=10)

        ttk.Separator(grid, orient="horizontal").grid(row=2, column=0, columnspan=7, sticky="ew", pady=6)

        ttk.Label(grid, text="OUT [%]:").grid(row=3, column=0, sticky="w")
        self.out_manual_var = tk.StringVar(value="40")
        ttk.Entry(grid, textvariable=self.out_manual_var, width=8).grid(row=3, column=1, padx=6, sticky="w")
        ttk.Button(grid, text="Wyślij OUT", command=self.send_out).grid(row=3, column=2, padx=6)

        ttk.Button(grid, text="MODE PID", command=self.send_mode_pid).grid(row=3, column=3, padx=6)
        ttk.Button(grid, text="MODE OPEN", command=self.send_mode_open).grid(row=3, column=4, padx=6)

        ttk.Button(grid, text="HELP", command=lambda: self.send_raw("HELP")).grid(row=4, column=0, pady=6)
        ttk.Button(grid, text="GET", command=lambda: self.send_raw("GET")).grid(row=4, column=1, pady=6)
        ttk.Button(grid, text="Clear plot", command=self.clear_plot).grid(row=4, column=2, pady=6)

        ttk.Separator(grid, orient="horizontal").grid(row=5, column=0, columnspan=7, sticky="ew", pady=6)

        ttk.Label(grid, text="TEST pre@0% [s]:").grid(row=6, column=0, sticky="w")
        self.test_pre_var = tk.StringVar(value="180")
        ttk.Entry(grid, textvariable=self.test_pre_var, width=8).grid(row=6, column=1, padx=6, sticky="w")

        ttk.Label(grid, text="heat@100% [s]:").grid(row=6, column=2, sticky="w")
        self.test_heat_var = tk.StringVar(value="1800")
        ttk.Entry(grid, textvariable=self.test_heat_var, width=8).grid(row=6, column=3, padx=6, sticky="w")

        ttk.Label(grid, text="cool@0% [s]:").grid(row=6, column=4, sticky="w")
        self.test_cool_var = tk.StringVar(value="600")
        ttk.Entry(grid, textvariable=self.test_cool_var, width=8).grid(row=6, column=5, padx=6, sticky="w")

        ttk.Label(grid, text="Hz:").grid(row=7, column=0, sticky="w")
        self.test_hz_var = tk.StringVar(value="2")
        ttk.Entry(grid, textvariable=self.test_hz_var, width=8).grid(row=7, column=1, padx=6, sticky="w")

        ttk.Button(grid, text="Start test 0-100-0", command=self.start_test).grid(row=7, column=4, padx=6)
        ttk.Button(grid, text="Stop test", command=self.stop_test).grid(row=7, column=5, padx=6)

        # Plot
        plot_frame = ttk.LabelFrame(self, text="Wykres (T i SP)")
        plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=6)

        self.fig = Figure(figsize=(7, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("t [s]")
        self.ax.set_ylabel("°C")
        self.line_T, = self.ax.plot([], [], label="T")
        self.line_SP, = self.ax.plot([], [], label="SP")
        self.ax.legend(loc="best")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Log window
        log_frame = ttk.LabelFrame(self, text="UART log")
        log_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=False, padx=8, pady=(0, 8))

        self.log_text = tk.Text(log_frame, height=8)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _big_label(self, parent, title, var):
        f = ttk.Frame(parent)
        ttk.Label(f, text=title).pack()
        ttk.Label(f, textvariable=var, font=("Segoe UI", 16, "bold")).pack()
        return f

    # ---------- Ports ----------
    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        if ports and (self.port_var.get() not in ports):
            self.port_var.set(ports[0])

    def _auto_select(self):
        p = auto_pick_stlink_port()
        if p:
            self.port_var.set(p)
        else:
            messagebox.showinfo("Auto", "Nie znaleziono STLink VCP. Wybierz port ręcznie.")

    # ---------- Connect/Disconnect ----------
    def on_connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Błąd", "Wybierz port COM.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Błąd", "Błędny baud.")
            return

        try:
            self.worker.connect(port, baud)
        except Exception as e:
            messagebox.showerror("Błąd", f"Nie mogę otworzyć portu: {e}")
            return

        self.t0 = time.time()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(DATA_DIR, f"pid_session_{ts}.csv")
        self._open_csv(self.log_path)
        self.status_var.set(f"Połączony: {port} | log: {self.log_path}")
        self.worker.send_line("GET")

    def on_disconnect(self):
        self.worker.disconnect()
        self.status_var.set("Rozłączony")
        self._close_csv()

    def on_close(self):
        self.on_disconnect()
        self.destroy()

    # ---------- CSV ----------
    def _open_csv(self, path):
        self._close_csv()
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["t_s", "phase", "T_C", "SP_C", "OUT_pct", "KP", "KI", "KD", "raw_line"])
        self.csv_file.flush()

    def _close_csv(self):
        try:
            if self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass
        self.csv_file = None
        self.csv_writer = None

    def _append_csv(self, sample: Sample, raw_line: str):
        if not self.csv_writer:
            return
        self.csv_writer.writerow([
            f"{sample.t:.3f}",
            sample.phase,
            f"{sample.T:.3f}",
            f"{sample.SP:.3f}",
            f"{sample.OUT:.3f}",
            "" if sample.KP is None else f"{sample.KP:.6f}",
            "" if sample.KI is None else f"{sample.KI:.6f}",
            "" if sample.KD is None else f"{sample.KD:.6f}",
            raw_line
        ])

        self.csv_file.flush()

    # ---------- Sending ----------
    def send_raw(self, s):
        if not self.worker.is_connected():
            return
        # DEBUG: pokaż co wysyłamy
        self.log_text.insert(tk.END, f">>> {s}\n")
        self.log_text.see(tk.END)

        self.worker.send_line(s)

    def send_set(self):
        try:
            v = float(self.set_var.get().strip())
        except ValueError:
            messagebox.showerror("Błąd", "SET musi być liczbą.")
            return
        self.send_raw(f"SET {v:.2f}")

    def _send_sequence(self, lines, delay_ms=60):
        """Wysyła listę komend z opóźnieniem, bez blokowania GUI."""
        if not lines:
            return
        self.send_raw(lines[0])
        if len(lines) > 1:
            self.after(delay_ms, lambda: self._send_sequence(lines[1:], delay_ms))

    def send_pid(self):
        try:
            kp = float(self.kp_var.get().strip())
            ki = float(self.ki_var.get().strip())
            kd = float(self.kd_var.get().strip())
        except ValueError:
            messagebox.showerror("Błąd", "KP/KI/KD muszą być liczbami.")
            return

        self._send_sequence([
            f"KP {kp:.4f}",
            f"KI {ki:.4f}",
            f"KD {kd:.4f}",
        ], delay_ms=80)

    def send_out(self):
        try:
            outp = float(self.out_manual_var.get().strip())
        except ValueError:
            messagebox.showerror("Błąd", "OUT musi być liczbą.")
            return
        self.send_raw(f"OUT {outp:.1f}")

    def send_mode_pid(self):
        self.send_raw("MODE PID")

    def send_mode_open(self):
        self.send_raw("MODE OPEN")

    # ---------- RX handling ----------
    def _on_uart_line_thread(self, line: str):
        # to UI queue
        with self.ui_lock:
            self.ui_queue.append(line)

    def _poll_ui_queue(self):
        lines = []
        with self.ui_lock:
            if self.ui_queue:
                lines = self.ui_queue[:]
                self.ui_queue.clear()

        for line in lines:
            self._process_uart_line(line)

        self._test_tick()

        self.after(30, self._poll_ui_queue)

    def _process_uart_line(self, line: str):
        # raw log
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)

        m = LINE_RE.search(line)
        if m:
            T = float(m.group("T"))
            SP = float(m.group("SP"))
            OUT = float(m.group("OUT"))
            t = time.time() - (self.t0 or time.time())

            KP = float(m.group("KP")) if m.group("KP") is not None else None
            KI = float(m.group("KI")) if m.group("KI") is not None else None
            KD = float(m.group("KD")) if m.group("KD") is not None else None

            if KP is not None and KI is not None and KD is not None:
                self.last_k = (KP, KI, KD)

            phase = getattr(self, "test_phase", "")

            s = Sample(t=t, T=T, SP=SP, OUT=OUT, KP=KP, KI=KI, KD=KD, phase=phase)
            self.samples.append(s)

            self.t_var.set(f"{T:.2f}")
            self.sp_var.set(f"{SP:.2f}")
            self.out_var.set(f"{OUT:.1f}")

            self._append_csv(s, line)
            self._update_plot()

        mk = K_RE.search(line)
        if mk:
            KP = float(mk.group("KP"))
            KI = float(mk.group("KI"))
            KD = float(mk.group("KD"))
            self.last_k = (KP, KI, KD)

    def _update_plot(self):
        N = 2000
        data = self.samples[-N:]
        xs = [s.t for s in data]
        ysT = [s.T for s in data]
        ysSP = [s.SP for s in data]

        self.line_T.set_data(xs, ysT)
        self.line_SP.set_data(xs, ysSP)

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def clear_plot(self):
        self.samples.clear()
        self.line_T.set_data([], [])
        self.line_SP.set_data([], [])
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def start_test(self):
        if not self.worker.is_connected():
            messagebox.showerror("Błąd", "Najpierw połącz UART.")
            return

        try:
            self.test_pre_s = float(self.test_pre_var.get().strip())
            self.test_heat_s = float(self.test_heat_var.get().strip())
            self.test_cool_s = float(self.test_cool_var.get().strip())
            self.test_hz = float(self.test_hz_var.get().strip())
        except ValueError:
            messagebox.showerror("Błąd", "Parametry testu muszą być liczbami.")
            return

        if self.test_hz <= 0:
            messagebox.showerror("Błąd", "Hz musi być > 0.")
            return
        if self.test_pre_s < 0 or self.test_heat_s <= 0 or self.test_cool_s < 0:
            messagebox.showerror("Błąd", "Czasy: pre>=0, heat>0, cool>=0.")
            return

        # nowy plik logu
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"pid_test_{ts}_0_100_0_pre{int(self.test_pre_s)}_heat{int(self.test_heat_s)}_cool{int(self.test_cool_s)}.csv"
        self.log_path = os.path.join(DATA_DIR, fname)
        self._open_csv(self.log_path)

        # metadane
        self.csv_writer.writerow(["#", "test_type", "0-100-0", "", "", "", "", "", ""])
        self.csv_writer.writerow(["#", "pre_s", self.test_pre_s, "", "", "", "", "", ""])
        self.csv_writer.writerow(["#", "heat_s", self.test_heat_s, "", "", "", "", "", ""])
        self.csv_writer.writerow(["#", "cool_s", self.test_cool_s, "", "", "", "", "", ""])
        self.csv_writer.writerow(["#", "hz", self.test_hz, "", "", "", "", "", ""])
        self.csv_file.flush()

        self.clear_plot()

        # start
        self.test_active = True
        self.test_t0 = time.time()
        self.t0 = self.test_t0
        self.test_next_get = self.test_t0
        self.test_phase = "PRE0"

        self._send_sequence(["MODE OPEN", "OUT 0", "GET"], delay_ms=120)

        self.status_var.set(f"TEST active | log: {self.log_path}")

    def stop_test(self):
        if self.test_active:
            self.test_active = False
            self.test_phase = "STOPPED"
            self._send_sequence(["OUT 0", "GET"], delay_ms=80)
            self.status_var.set(f"TEST stopped | saved: {self.log_path}")
            self._close_csv()

    def _test_tick(self):
        if not getattr(self, "test_active", False):
            return

        now = time.time()
        elapsed = now - self.test_t0

        # fazy czasowe
        t_pre_end = self.test_pre_s
        t_heat_end = self.test_pre_s + self.test_heat_s
        t_cool_end = self.test_pre_s + self.test_heat_s + self.test_cool_s

        new_phase = self.test_phase

        if elapsed < t_pre_end:
            new_phase = "PRE0"
        elif elapsed < t_heat_end:
            new_phase = "HEAT100"
        elif elapsed < t_cool_end:
            new_phase = "COOL0"
        else:
            new_phase = "DONE"

        if new_phase != self.test_phase:
            self.test_phase = new_phase

            if new_phase == "HEAT100":
                if self.csv_writer:
                    self.csv_writer.writerow(["#", "phase", "HEAT100", "", "", "", "", "", ""])
                    self.csv_file.flush()
                self.send_raw("OUT 100")
            elif new_phase == "COOL0":
                if self.csv_writer:
                    self.csv_writer.writerow(["#", "phase", "COOL0", "", "", "", "", "", ""])
                    self.csv_file.flush()
                self.send_raw("OUT 0")
            elif new_phase == "DONE":
                self.send_raw("OUT 0")
                self.send_raw("GET")
                self.test_active = False
                self.status_var.set(f"TEST done | saved: {self.log_path}")
                self._close_csv()
                return

        # zbieranie danych
        period = 1.0 / self.test_hz
        if now >= self.test_next_get:
            self.send_raw("GET")
            self.test_next_get += period
            if self.test_next_get < now - period:
                self.test_next_get = now + period

    def start_run(self):
        if not self.worker.is_connected():
            messagebox.showerror("Błąd", "Najpierw połącz UART.")
            return

        try:
            self.run_out0 = float(self.run_out0_var.get().strip())
            self.run_out1 = float(self.run_out1_var.get().strip())
            self.run_step_s = float(self.run_step_var.get().strip())
            self.run_total_s = float(self.run_total_var.get().strip())
            self.run_hz = float(self.run_hz_var.get().strip())
        except ValueError:
            messagebox.showerror("Błąd", "RUN parametry muszą być liczbami.")
            return

        if self.run_hz <= 0:
            messagebox.showerror("Błąd", "Hz musi być > 0.")
            return
        if self.run_total_s <= 0:
            messagebox.showerror("Błąd", "Total musi być > 0.")
            return
        if self.run_step_s < 0 or self.run_step_s > self.run_total_s:
            messagebox.showerror("Błąd", "Step musi być w zakresie 0..Total.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"pid_run_{ts}_out{self.run_out0:.0f}_to_{self.run_out1:.0f}.csv"
        self.log_path = os.path.join(DATA_DIR, fname)
        self._open_csv(self.log_path)

        self.csv_writer.writerow(["#", "run_out0", self.run_out0, "", ""])
        self.csv_writer.writerow(["#", "run_out1", self.run_out1, "", ""])
        self.csv_writer.writerow(["#", "run_step_s", self.run_step_s, "", ""])
        self.csv_writer.writerow(["#", "run_total_s", self.run_total_s, "", ""])
        self.csv_writer.writerow(["#", "run_hz", self.run_hz, "", ""])
        self.csv_file.flush()

        self.clear_plot()

        self.run_active = True
        self.run_t0 = time.time()
        self.run_next_get = self.run_t0 + (1.0 / self.run_hz)

        self.run_step_done = False
        self.t0 = self.run_t0

        self.send_raw("MODE OPEN")
        time.sleep(0.15)

        self.send_raw(f"OUT {self.run_out0:.1f}")
        time.sleep(0.10)

        self.send_raw("GET")

        self.status_var.set(f"RUN active | log: {self.log_path}")

    def stop_run(self):
            if self.run_active:
                self.run_active = False
                self.send_raw("OUT 0")
                self.status_var.set(f"RUN stopped | log saved: {self.log_path}")
                self._close_csv()

    def _run_tick(self):
        if not getattr(self, "run_active", False):
            return
        now = time.time()
        elapsed = now - self.run_t0

        if (not self.run_step_done) and (elapsed >= self.run_step_s):
            self.send_raw(f"OUT {self.run_out1:.1f}")
            self.run_step_done = True

        period = 1.0 / self.run_hz
        if now >= self.run_next_get:
            self.send_raw("GET")
            self.run_next_get += period
            if self.run_next_get < now - period:
                self.run_next_get = now + period

        if elapsed >= self.run_total_s:
            self.send_raw("OUT 0")
            self.run_active = False
            self._close_csv()
            self.status_var.set(f"RUN done | saved: {self.log_path}")



if __name__ == "__main__":
    app = App()
    app.mainloop()
