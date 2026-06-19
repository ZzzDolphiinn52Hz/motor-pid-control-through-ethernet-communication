import queue
import socket
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk


CMD_READ_ADC = 0x01
CMD_WRITE_PWM = 0x02
CMD_LED = 0x03
CMD_STATUS = 0x04
CMD_IDN = 0x05
CMD_PING = 0x06
CMD_READ_ENCODER = 0x07
CMD_SET_TARGET_RPM = 0x08
CMD_SET_KP = 0x09
CMD_SET_KI = 0x0A
CMD_PI_ENABLE = 0x0B
CMD_READ_PWM = 0x0C

STATUS_SEL_FLAGS = 0x00
STATUS_SEL_TARGET_RPM = 0x01
STATUS_SEL_DEADLINE_MISSES = 0x02
STATUS_SEL_MAX_RT_EXEC_US = 0x03
STATUS_SEL_RX_FRAMES = 0x04
STATUS_SEL_CRC_ERRORS = 0x05

STATUS_OK = 0x00

CONTROL_FLAG_PI_ENABLED = 0x0001

PORT_DEFAULT = 5000
FRAME_LEN = 12
SOCKET_TIMEOUT_S = 0.25
POLL_PERIOD_MS = 100
POLL_PERIOD_S = POLL_PERIOD_MS / 1000.0
GUI_EVENT_PERIOD_MS = 16
CHART_HISTORY_POINTS = 600
CHART_MAX_RPM_FLOOR = 300
CHART_PWM_MAX = 255


def calc_crc(frame):
    crc = 0
    for i in range(10):
        crc ^= frame[i]
    return crc & 0xFF


def build_frame(cmd, value=0, seq=0, opt1=0, opt2=0):
    value &= 0xFFFF
    frame = bytearray(FRAME_LEN)
    frame[0:4] = b"@SEN"
    frame[4] = cmd & 0xFF
    frame[5] = (value >> 8) & 0xFF
    frame[6] = value & 0xFF
    frame[7] = opt1 & 0xFF
    frame[8] = opt2 & 0xFF
    frame[9] = seq & 0xFF
    frame[10] = calc_crc(frame) 
    frame[11] = ord("#")
    return bytes(frame)


def recv_exact(sock, size):
    buffer = bytearray(size)
    view = memoryview(buffer)
    received = 0
    while received < size:
        count = sock.recv_into(view[received:], size - received)
        if count == 0:
            raise ConnectionError("ESP32 closed the connection")
        received += count
    return bytes(buffer)


def parse_response(frame, expected_seq=None):
    if len(frame) != FRAME_LEN:
        raise ValueError("Response frame is not 12 bytes")
    if frame[0:4] != b"@REC" or frame[11] != ord("#"):
        raise ValueError("Invalid response frame header")
    if calc_crc(frame) != frame[10]:
        raise ValueError("Invalid response CRC")
    if expected_seq is not None and frame[9] != expected_seq:
        raise ValueError(f"Sequence mismatch: got {frame[9]}, expected {expected_seq}")

    value = struct.unpack(">H", frame[5:7])[0]
    return {
        "cmd": frame[4],
        "value": value,
        "status": frame[7],
        "error": frame[8],
        "seq": frame[9],
    }


@dataclass
class CommandRequest:
    cmd: int
    value: int = 0
    label: str = ""
    callback: object = None
    opt1: int = 0
    opt2: int = 0


class RealtimeClient:
    def __init__(self, event_queue):
        self.event_queue = event_queue
        self.command_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = None
        self.sock = None
        self.seq = 0
        self.connected = False
        self.endpoint = None

    def connect(self, host, port):
        self.disconnect()
        self.stop_event.clear()
        self.endpoint = (host, port)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.stop_event.set()
        self._close_socket()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.8)
        self.thread = None
        self.connected = False

    def send(self, cmd, value=0, label="", callback=None, opt1=0, opt2=0):
        self.command_queue.put(CommandRequest(cmd, value, label, callback, opt1, opt2))

    def _run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect(self.endpoint)
            sock.settimeout(SOCKET_TIMEOUT_S)
            self.sock = sock
            self.connected = True
            self.event_queue.put(("connected", self.endpoint))

            while not self.stop_event.is_set():
                try:
                    request = self.command_queue.get(timeout=0.05)
                except queue.Empty:
                    continue

                started = time.perf_counter()
                response = self._transaction(request.cmd, request.value, request.opt1, request.opt2)
                latency_ms = (time.perf_counter() - started) * 1000.0
                self.event_queue.put(("response", request, response, latency_ms))

        except Exception as exc:
            self.event_queue.put(("error", str(exc)))
        finally:
            self.connected = False
            self._close_socket()
            self.event_queue.put(("disconnected", None))

    def _transaction(self, cmd, value, opt1=0, opt2=0):
        if self.sock is None:
            raise ConnectionError("Not connected")
        self.seq = (self.seq + 1) & 0xFF
        self.sock.sendall(build_frame(cmd, value, self.seq, opt1, opt2))
        return parse_response(recv_exact(self.sock, FRAME_LEN), self.seq)

    def _close_socket(self):
        sock = self.sock
        self.sock = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass


class MotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Ethernet Motor Supervisor")
        self.root.geometry("1080x820")
        self.root.resizable(False, False)

        self.events = queue.Queue()
        self.client = RealtimeClient(self.events)
        self.auto_poll = False
        self.next_poll_deadline = time.perf_counter() + POLL_PERIOD_S

        self.ip_var = tk.StringVar(value="192.168.2.177")
        self.port_var = tk.StringVar(value=str(PORT_DEFAULT))
        self.status_var = tk.StringVar(value="Disconnected")
        self.rpm_var = tk.StringVar(value="0")
        self.pwm_feedback_var = tk.StringVar(value="0")
        self.latency_var = tk.StringVar(value="-")
        self.mode_var = tk.StringVar(value="Manual PWM")
        self.target_feedback_var = tk.StringVar(value="0")
        self.deadline_miss_var = tk.StringVar(value="0")
        self.max_exec_var = tk.StringVar(value="0")
        self.rx_frames_var = tk.StringVar(value="0")
        self.pwm_var = tk.IntVar(value=0)
        self.target_rpm_var = tk.IntVar(value=100)
        self.kp_var = tk.StringVar(value="0.50")
        self.ki_var = tk.StringVar(value="0.10")
        self.current_rpm = 0
        self.current_pwm = 0
        self.current_target = 0
        self.chart_samples = []

        self._configure_style()
        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(GUI_EVENT_PERIOD_MS, self._drain_events)
        self.root.after(POLL_PERIOD_MS, self._poll_telemetry)

    def _configure_style(self):
        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass

        self.root.configure(bg="#f3f5f7")
        self.style.configure(".", font=("Segoe UI", 10))
        self.style.configure("TFrame", background="#f3f5f7")
        self.style.configure("Panel.TLabelframe", background="#f3f5f7", padding=10)
        self.style.configure("Panel.TLabelframe.Label", font=("Segoe UI", 10, "bold"))
        self.style.configure("Title.TLabel", background="#f3f5f7", font=("Segoe UI", 15, "bold"))
        self.style.configure("Hint.TLabel", background="#f3f5f7", foreground="#5f6873")
        self.style.configure("Metric.TLabel", background="#f3f5f7", foreground="#5f6873")
        self.style.configure("MetricValue.TLabel", background="#f3f5f7", font=("Segoe UI", 20, "bold"))
        self.style.configure("BigValue.TLabel", background="#f3f5f7", font=("Segoe UI", 30, "bold"))
        self.style.configure("Legend.TLabel", background="#f3f5f7", foreground="#374151", font=("Segoe UI", 9, "bold"))
        self.style.configure("LegendRPM.TLabel", background="#f3f5f7", foreground="#2563eb", font=("Segoe UI", 9, "bold"))
        self.style.configure("LegendTarget.TLabel", background="#f3f5f7", foreground="#16a34a", font=("Segoe UI", 9, "bold"))
        self.style.configure("LegendPWM.TLabel", background="#f3f5f7", foreground="#dc2626", font=("Segoe UI", 9, "bold"))
        self.style.configure("Good.TLabel", background="#f3f5f7", foreground="#1f7a3f", font=("Segoe UI", 10, "bold"))
        self.style.configure("Bad.TLabel", background="#f3f5f7", foreground="#a83232", font=("Segoe UI", 10, "bold"))
        self.style.configure("Warn.TLabel", background="#f3f5f7", foreground="#8a5b00", font=("Segoe UI", 10, "bold"))
        self.style.configure("TButton", padding=(10, 5))
        self.style.configure("Accent.TButton", padding=(10, 5), font=("Segoe UI", 10, "bold"))

    def _build_widgets(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        header = ttk.Frame(main)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(header, text="ESP32 Motor Control", style="Title.TLabel").pack(side="left")
        self.status_label = ttk.Label(header, textvariable=self.status_var, style="Bad.TLabel")
        self.status_label.pack(side="right", padx=(8, 0))

        conn = ttk.LabelFrame(main, text="Connection", style="Panel.TLabelframe")
        conn.pack(fill="x", pady=(0, 8))
        conn.columnconfigure(1, weight=1)
        ttk.Label(conn, text="ESP32 IP", style="Metric.TLabel").grid(row=0, column=0, padx=(0, 8), pady=4, sticky="w")
        ttk.Entry(conn, textvariable=self.ip_var, width=20).grid(row=0, column=1, padx=(0, 12), pady=4, sticky="ew")
        ttk.Label(conn, text="Port", style="Metric.TLabel").grid(row=0, column=2, padx=(0, 8), pady=4, sticky="w")
        ttk.Entry(conn, textvariable=self.port_var, width=8).grid(row=0, column=3, padx=(0, 12), pady=4)
        self.btn_connect = ttk.Button(conn, text="Connect", command=self.connect_esp32, style="Accent.TButton")
        self.btn_connect.grid(row=0, column=4, padx=(0, 6), pady=4)
        self.btn_disconnect = ttk.Button(conn, text="Disconnect", command=self.disconnect_esp32)
        self.btn_disconnect.grid(row=0, column=5, pady=4)

        controls = ttk.Frame(main)
        controls.pack(fill="x", pady=(0, 8))
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)

        manual = ttk.LabelFrame(controls, text="Manual PWM", style="Panel.TLabelframe")
        manual.grid(row=0, column=0, padx=(0, 6), sticky="nsew")
        manual.columnconfigure(1, weight=1)
        ttk.Label(manual, text="PWM output", style="Metric.TLabel").grid(row=0, column=0, padx=(0, 8), pady=6, sticky="w")
        self.pwm_scale = ttk.Scale(manual, from_=0, to=255, orient="horizontal", command=self._on_pwm_scale)
        self.pwm_scale.grid(row=0, column=1, padx=(0, 8), pady=6, sticky="ew")
        ttk.Entry(manual, textvariable=self.pwm_var, width=7, justify="right").grid(row=0, column=2, pady=6)
        ttk.Button(manual, text="Apply", command=self.send_pwm).grid(row=1, column=1, padx=(0, 8), pady=(8, 0), sticky="ew")
        ttk.Button(manual, text="Stop", command=self.stop_motor, style="Accent.TButton").grid(
            row=1, column=2, pady=(8, 0), sticky="ew"
        )

        pi = ttk.LabelFrame(controls, text="Firmware PI Loop", style="Panel.TLabelframe")
        pi.grid(row=0, column=1, padx=(6, 0), sticky="nsew")
        for col in range(6):
            pi.columnconfigure(col, weight=1)
        ttk.Label(pi, text="Target RPM", style="Metric.TLabel").grid(row=0, column=0, columnspan=2, padx=(0, 8), pady=6, sticky="w")
        ttk.Entry(pi, textvariable=self.target_rpm_var, width=8, justify="right").grid(row=0, column=2, padx=(0, 10), pady=6)
        ttk.Label(pi, text="Kp", style="Metric.TLabel").grid(row=0, column=3, padx=(0, 6), pady=6, sticky="e")
        ttk.Entry(pi, textvariable=self.kp_var, width=7, justify="right").grid(row=0, column=4, padx=(0, 10), pady=6)
        ttk.Label(pi, text="Ki", style="Metric.TLabel").grid(row=0, column=5, padx=(0, 6), pady=6, sticky="e")
        ttk.Entry(pi, textvariable=self.ki_var, width=7, justify="right").grid(row=0, column=6, pady=6)
        ttk.Button(pi, text="Send PI", command=self.send_pi_params).grid(row=1, column=0, columnspan=2, padx=(0, 6), pady=(8, 0), sticky="ew")
        ttk.Button(pi, text="Enable PI", command=self.enable_pi, style="Accent.TButton").grid(
            row=1, column=2, columnspan=3, padx=6, pady=(8, 0), sticky="ew"
        )
        ttk.Button(pi, text="Disable PI", command=self.disable_pi).grid(row=1, column=5, columnspan=2, pady=(8, 0), sticky="ew")

        telemetry = ttk.LabelFrame(main, text="Realtime Monitoring", style="Panel.TLabelframe")
        telemetry.pack(fill="x", pady=(0, 8))
        for col in range(6):
            telemetry.columnconfigure(col, weight=1)

        self._metric(telemetry, 0, 0, "RPM", self.rpm_var, "BigValue.TLabel")
        self._metric(telemetry, 0, 2, "PWM output", self.pwm_feedback_var, "MetricValue.TLabel")
        self._metric(telemetry, 0, 4, "Target RPM", self.target_feedback_var, "MetricValue.TLabel")
        self._metric(telemetry, 1, 0, "TCP latency ms", self.latency_var)
        self._metric(telemetry, 1, 2, "Mode", self.mode_var)
        self._metric(telemetry, 1, 4, "Budget miss", self.deadline_miss_var)
        self._metric(telemetry, 2, 0, "Max ISR us", self.max_exec_var)
        self._metric(telemetry, 2, 2, "RX frames", self.rx_frames_var)
       

        ttk.Button(telemetry, text="Read Now", command=self.read_telemetry).grid(row=6, column=0, columnspan=3, padx=(0, 6), pady=(10, 0), sticky="ew")
        self.btn_auto = ttk.Button(telemetry, text="Start Monitor", command=self.toggle_auto_poll, style="Accent.TButton")
        self.btn_auto.grid(row=6, column=3, columnspan=3, pady=(10, 0), sticky="ew")

        chart_frame = ttk.LabelFrame(main, text="Trend", style="Panel.TLabelframe")
        chart_frame.pack(fill="x", pady=(0, 8))
        chart_header = ttk.Frame(chart_frame)
        chart_header.pack(fill="x", pady=(0, 6))
        ttk.Label(chart_header, text="RPM", style="LegendRPM.TLabel").pack(side="left", padx=(0, 14))
        ttk.Label(chart_header, text="Target", style="LegendTarget.TLabel").pack(side="left", padx=(0, 14))
        ttk.Label(chart_header, text="PWM output", style="LegendPWM.TLabel").pack(side="left", padx=(0, 14))
        ttk.Button(chart_header, text="Clear Chart", command=self.clear_chart).pack(side="right")
        self.chart_canvas = tk.Canvas(
            chart_frame,
            height=230,
            bg="#ffffff",
            highlightthickness=1,
            highlightbackground="#d1d5db",
            relief="flat",
        )
        self.chart_canvas.pack(fill="x")
        self.chart_canvas.bind("<Configure>", lambda _event: self.draw_chart())

        log_frame = ttk.LabelFrame(main, text="Log", style="Panel.TLabelframe")
        log_frame.pack(fill="both", expand=True)
        self.log_text = tk.Text(
            log_frame,
            height=5,
            bg="#111827",
            fg="#e5e7eb",
            insertbackground="#e5e7eb",
            relief="flat",
            font=("Consolas", 9),
        )
        self.log_text.pack(fill="both", expand=True)

    def _metric(self, parent, row, col, title, variable, value_style="MetricValue.TLabel"):
        ttk.Label(parent, text=title, style="Metric.TLabel").grid(row=row * 2, column=col, columnspan=2, padx=6, pady=(4, 0), sticky="w")
        ttk.Label(parent, textvariable=variable, style=value_style).grid(row=row * 2 + 1, column=col, columnspan=2, padx=6, pady=(0, 8), sticky="w")

    def clear_chart(self):
        self.chart_samples.clear()
        self.draw_chart()

    def append_chart_sample(self):
        self.chart_samples.append((time.perf_counter(), self.current_rpm, self.current_target, self.current_pwm))
        if len(self.chart_samples) > CHART_HISTORY_POINTS:
            del self.chart_samples[: len(self.chart_samples) - CHART_HISTORY_POINTS]
        self.draw_chart()

    def draw_chart(self):
        if not hasattr(self, "chart_canvas"):
            return

        canvas = self.chart_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 20)
        height = max(canvas.winfo_height(), 20)
        left = 48
        right = width - 16
        top = 12
        bottom = height - 28
        plot_w = max(right - left, 1)
        plot_h = max(bottom - top, 1)

        canvas.create_rectangle(left, top, right, bottom, outline="#e5e7eb", fill="#ffffff")
        for i in range(1, 5):
            y = top + (plot_h * i / 5)
            canvas.create_line(left, y, right, y, fill="#eef2f7")
        for i in range(1, 6):
            x = left + (plot_w * i / 6)
            canvas.create_line(x, top, x, bottom, fill="#f3f4f6")

        rpm_max = CHART_MAX_RPM_FLOOR
        for _stamp, rpm, target, _pwm in self.chart_samples:
            rpm_max = max(rpm_max, rpm, target)
        rpm_max = int(((rpm_max + 99) // 100) * 100)

        canvas.create_text(8, top + 2, text=str(rpm_max), anchor="nw", fill="#6b7280", font=("Segoe UI", 8))
        canvas.create_text(8, bottom - 12, text="0", anchor="nw", fill="#6b7280", font=("Segoe UI", 8))
        canvas.create_text(left, bottom + 8, text=f"last {len(self.chart_samples)} samples", anchor="nw", fill="#6b7280", font=("Segoe UI", 8))

        if len(self.chart_samples) < 2:
            canvas.create_text((left + right) / 2, (top + bottom) / 2, text="Start Monitor to plot live data", fill="#9ca3af")
            return

        def points(series_index, max_value):
            count = len(self.chart_samples)
            pts = []
            for index, sample in enumerate(self.chart_samples):
                value = sample[series_index]
                x = left + (plot_w * index / max(count - 1, 1))
                y = bottom - (plot_h * min(max(value, 0), max_value) / max_value)
                pts.extend((x, y))
            return pts

        canvas.create_line(*points(1, rpm_max), fill="#2563eb", width=2, smooth=True)
        canvas.create_line(*points(2, rpm_max), fill="#16a34a", width=2, smooth=True)
        canvas.create_line(*points(3, CHART_PWM_MAX), fill="#dc2626", width=2, smooth=True)

    def log(self, msg):
        stamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"{stamp}  {msg}\n")
        self.log_text.see("end")

    def connect_esp32(self):
        try:
            self.client.connect(self.ip_var.get().strip(), int(self.port_var.get().strip()))
            self.status_var.set("Connecting...")
            self.status_label.configure(style="Warn.TLabel")
        except Exception as exc:
            messagebox.showerror("Connection error", str(exc))

    def disconnect_esp32(self):
        self.auto_poll = False
        self.btn_auto.config(text="Start Monitor")
        self.client.disconnect()
        self.mode_var.set("Manual PWM")
        self.status_label.configure(style="Bad.TLabel")

    def send_pwm(self):
        pwm = max(0, min(255, int(self.pwm_var.get())))
        self.pwm_var.set(pwm)
        self.pwm_scale.set(pwm)
        self.client.send(CMD_WRITE_PWM, pwm, "PWM")

    def stop_motor(self):
        self.pwm_var.set(0)
        self.pwm_scale.set(0)
        self.client.send(CMD_WRITE_PWM, 0, "Stop")
        self.client.send(CMD_PI_ENABLE, 0, "Disable PI")

    def send_pi_params(self):
        try:
            target = max(0, min(65535, int(self.target_rpm_var.get())))
            kp_x100 = max(0, min(65535, int(float(self.kp_var.get()) * 100.0)))
            ki_x100 = max(0, min(65535, int(float(self.ki_var.get()) * 100.0)))
        except ValueError as exc:
            messagebox.showerror("PI parameter error", str(exc))
            return False

        self.client.send(CMD_SET_TARGET_RPM, target, "Target RPM")
        self.client.send(CMD_SET_KP, kp_x100, "Kp")
        self.client.send(CMD_SET_KI, ki_x100, "Ki")
        return True

    def enable_pi(self):
        if self.send_pi_params():
            self.client.send(CMD_PI_ENABLE, 1, "Enable PI")

    def disable_pi(self):
        self.client.send(CMD_PI_ENABLE, 0, "Disable PI")

    def read_telemetry(self):
        self.client.send(CMD_READ_ENCODER, 0, "Read RPM")
        self.client.send(CMD_READ_PWM, 0, "Read PWM")
        self.client.send(CMD_STATUS, 0, "Read flags", opt1=STATUS_SEL_FLAGS)
        self.client.send(CMD_STATUS, 0, "Read target", opt1=STATUS_SEL_TARGET_RPM)
        self.client.send(CMD_STATUS, 0, "Read misses", opt1=STATUS_SEL_DEADLINE_MISSES)
        self.client.send(CMD_STATUS, 0, "Read max ISR", opt1=STATUS_SEL_MAX_RT_EXEC_US)
        self.client.send(CMD_STATUS, 0, "Read RX frames", opt1=STATUS_SEL_RX_FRAMES)
        

    def toggle_auto_poll(self):
        self.auto_poll = not self.auto_poll
        if self.auto_poll:
            self.next_poll_deadline = time.perf_counter()
        self.btn_auto.config(text="Stop Monitor" if self.auto_poll else "Start Monitor")
        self.log("Monitoring enabled" if self.auto_poll else "Monitoring disabled")

    def _on_pwm_scale(self, value):
        self.pwm_var.set(int(float(value)))

    def _poll_telemetry(self):
        now = time.perf_counter()
        if self.auto_poll and self.client.connected and now >= self.next_poll_deadline:
            self.read_telemetry()
            while self.next_poll_deadline <= now:
                self.next_poll_deadline += POLL_PERIOD_S
        elif not self.auto_poll:
            self.next_poll_deadline = now + POLL_PERIOD_S

        delay_s = max(0.001, self.next_poll_deadline - time.perf_counter())
        self.root.after(max(1, int(delay_s * 1000)), self._poll_telemetry)

    def _drain_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)
        self.root.after(GUI_EVENT_PERIOD_MS, self._drain_events)

    def _handle_event(self, event):
        kind = event[0]
        if kind == "connected":
            host, port = event[1]
            self.status_var.set(f"Connected to {host}:{port}")
            self.status_label.configure(style="Good.TLabel")
            self.log(f"Connected to ESP32 {host}:{port}")
        elif kind == "disconnected":
            self.status_var.set("Disconnected")
            self.status_label.configure(style="Bad.TLabel")
        elif kind == "error":
            self.log(f"I/O error: {event[1]}")
        elif kind == "response":
            _, request, response, latency_ms = event
            self.latency_var.set(f"{latency_ms:.1f}")
            if response["status"] != STATUS_OK:
                self.log(f"{request.label or 'Command'} failed, error={response['error']}, value={response['value']}")
                return
            self._apply_response(request, response)

    def _apply_response(self, request, response):
        if request.cmd == CMD_READ_ENCODER:
            self.current_rpm = response["value"]
            self.rpm_var.set(str(self.current_rpm))
            self.append_chart_sample()
        elif request.cmd == CMD_READ_PWM:
            self.current_pwm = response["value"]
            self.pwm_feedback_var.set(str(self.current_pwm))
        elif request.cmd == CMD_WRITE_PWM:
            self.mode_var.set("Manual PWM")
            self.current_pwm = response["value"]
            self.pwm_feedback_var.set(str(self.current_pwm))
            self.log(f"PWM applied: {response['value']}")
        elif request.cmd == CMD_PI_ENABLE:
            self.mode_var.set("Firmware PI" if response["value"] else "Manual PWM")
            self.log("PI enabled" if response["value"] else "PI disabled")
        elif request.cmd == CMD_STATUS:
            value = response["value"]
            if request.opt1 == STATUS_SEL_FLAGS:
                pi_enabled = bool(value & CONTROL_FLAG_PI_ENABLED)
                self.mode_var.set("Firmware PI" if pi_enabled else "Manual PWM")
            elif request.opt1 == STATUS_SEL_TARGET_RPM:
                self.current_target = value
                self.target_feedback_var.set(str(self.current_target))
            elif request.opt1 == STATUS_SEL_DEADLINE_MISSES:
                self.deadline_miss_var.set(str(value))
            elif request.opt1 == STATUS_SEL_MAX_RT_EXEC_US:
                self.max_exec_var.set(str(value))
            elif request.opt1 == STATUS_SEL_RX_FRAMES:
                self.rx_frames_var.set(str(value))
        elif request.label:
            self.log(f"{request.label} accepted")

    def on_close(self):
        self.disconnect_esp32()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()
