import socket
import time
import math
import threading
import traceback
import sys

try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext
except Exception as e:
    with open("crash_log.txt", "w") as f:
        f.write("Failed to import tkinter:\n")
        traceback.print_exc(file=f)
    sys.exit(1)

# ─────────────────────────────── CONFIG ────────────────────────────────
SIMULATION_MODE  = False             # Set to True for testing without hardware
PI_IP            = 'stringart.local'     
PI_PORT          = 5000             

MOTOR_DISTANCE   = 42.0          
STEPS_PER_INCH   = 2032.0        
PULLEY_RADIUS    = 0.3183        

CAL_L_MEASURED   = 12.0
CAL_R_MEASURED   = 11.5625
# ────────────────────────────────────────────────────────────────────────

def get_steps_for_coord(x: float, y: float, is_left: bool, cur_steps_l: int, cur_steps_r: int) -> int:
    mx = 0.0 if is_left else MOTOR_DISTANCE
    dx = x - mx
    dy = y
    d2 = dx * dx + dy * dy
    d  = math.sqrt(d2)
    r  = PULLEY_RADIUS
    cur_steps = cur_steps_l if is_left else cur_steps_r
    if d < (r + 0.1): return cur_steps
    l_straight = math.sqrt(max(0.0, d2 - r * r))
    theta = math.atan2(dy, dx)
    phi   = math.acos(max(-1.0, min(1.0, r / d)))
    if is_left: wrap_angle = math.pi / 2.0 - (theta + phi)
    else:        wrap_angle = theta - phi - math.pi / 2.0
    return int((l_straight + r * abs(wrap_angle)) * STEPS_PER_INCH)

def compute_home_position() -> tuple[float, float]:
    sl = CAL_L_MEASURED * STEPS_PER_INCH
    sr = CAL_R_MEASURED * STEPS_PER_INCH
    L  = sl / STEPS_PER_INCH
    R  = sr / STEPS_PER_INCH
    hx = (L * L - R * R + MOTOR_DISTANCE * MOTOR_DISTANCE) / (2.0 * MOTOR_DISTANCE)
    hy = math.sqrt(max(0.0, L * L - hx * hx))
    return 21.0, 15.0

class PositionTracker:
    def __init__(self, home_x: float, home_y: float):
        self.x = home_x
        self.y = home_y
        self.steps_l = int(CAL_L_MEASURED * STEPS_PER_INCH)
        self.steps_r = int(CAL_R_MEASURED * STEPS_PER_INCH)

    def apply_relative_move(self, dx: float, dy: float):
        tx, ty = self.x + dx, self.y + dy
        self.steps_l = get_steps_for_coord(tx, ty, True,  self.steps_l, self.steps_r)
        self.steps_r = get_steps_for_coord(tx, ty, False, self.steps_l, self.steps_r)
        self.x, self.y = tx, ty

class RobotComms:
    def __init__(self, log_callback):
        self.sock = None
        self.sock_file = None
        self.log = log_callback
        self.lock = threading.Lock() # Ensures movement and polling queries don't interleave

    def connect(self):
        if SIMULATION_MODE:
            self.log("[SIM] Network socket bypassed.")
            return True
        try:
            self.log(f"[NET] Connecting to {PI_IP}:{PI_PORT} ...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((PI_IP, PI_PORT))
            self.sock_file = self.sock.makefile('r', encoding='utf-8')
            self.log("[NET] Connected to Pi.")
            return True
        except Exception as e:
            self.log(f"[ERROR] Connection failed: {e}")
            return False

    def send_command(self, cmd: str, expected_ack: str, timeout: float = 60.0) -> bool:
        if SIMULATION_MODE:
            self.log(f"[SIM OUT] {cmd.strip()}")
            time.sleep(0.1)
            return True
        with self.lock:
            try:
                self.sock.sendall((cmd.strip() + '\n').encode('utf-8'))
                self.sock.settimeout(timeout)
                while True:
                    line = self.sock_file.readline()
                    if not line: return False
                    line = line.strip()
                    self.log(f"[RX] {line}")
                    if line == expected_ack or (line.startswith("ACK:") and line[4:] == expected_ack):
                        return True
                    if line == "DONE" or line == "CNC_READY":
                        return True
            except Exception as e:
                self.log(f"[NET ERROR] {e}")
                return False

    def poll_status(self) -> dict | None:
        """Sends 'S' to query switch pin states safely without moving motors."""
        if SIMULATION_MODE:
            return {'Z': 0, 'L': 0, 'R': 0}
        
        # Non-blocking acquire so polling yields gracefully while motors are executing moves
        if not self.lock.acquire(blocking=False):
            return None
            
        try:
            self.sock.sendall(b"S\n")
            self.sock.settimeout(2.0)
            status = {}
            while True:
                line = self.sock_file.readline()
                if not line: break
                line = line.strip()
                if "STATUS:" in line:
                    parts = line.split()
                    for p in parts:
                        if "Z_PIN10=" in p: status['Z'] = int(p.split('=')[1])
                        elif "L_PIN9=" in p: status['L'] = int(p.split('=')[1])
                        elif "R_PIN13=" in p: status['R'] = int(p.split('=')[1])
                if line.endswith("DONE") or line.endswith("ACK:DONE"):
                    break
            return status if status else None
        except Exception:
            return None
        finally:
            self.lock.release()

    def send_move(self, z_flag: int, dx: float, dy: float) -> bool:
        return self.send_command(f"G {z_flag} {dx:.4f} {dy:.4f}", expected_ack='DONE')

    def home(self) -> bool:
        return self.send_command('H', expected_ack='CNC_READY', timeout=300)

class PronterfaceGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("String Art Robot - Pronterface Debugger")
        
        window_width = 680
        window_height = 540
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        center_x = int((screen_width - window_width) / 2)
        center_y = int((screen_height - window_height) / 2)
        self.geometry(f"{window_width}x{window_height}+{center_x}+{center_y}")
        
        self.comms = RobotComms(self.log)
        hx, hy = compute_home_position()
        self.tracker = PositionTracker(hx, hy)
        
        self.speed = tk.DoubleVar(value=0.5)
        self.is_moving = False
        
        self.setup_ui()
        self.bind_shortcuts()
        
        self.after(200, lambda: threading.Thread(target=self.init_connection, daemon=True).start())
        
        # Start background polling loop for virtual LEDs (polls every 300ms)
        self.after(1000, self.start_status_polling)

    def setup_ui(self):
        self.pos_var = tk.StringVar(value="X: 0.00 | Y: 0.00")
        self.steps_var = tk.StringVar(value="L: 0 | R: 0")
        
        frame_status = ttk.Frame(self)
        frame_status.pack(pady=5)
        ttk.Label(frame_status, textvariable=self.pos_var, font=("Courier", 16, "bold")).pack()
        ttk.Label(frame_status, textvariable=self.steps_var, font=("Courier", 10)).pack()

        # ── VIRTUAL LED DASHBOARD ──
        frame_leds = ttk.LabelFrame(self, text=" Hardware Limit Switch Monitor ")
        frame_leds.pack(pady=5, padx=10, fill=tk.X)
        
        f_indicators = ttk.Frame(frame_leds)
        f_indicators.pack(pady=5)

        # Z-Stop LED Canvas & Label
        self.canvas_z = tk.Canvas(f_indicators, width=20, height=20, highlightthickness=0)
        self.canvas_z.grid(row=0, column=0, padx=(10, 2))
        self.led_z = self.canvas_z.create_oval(2, 2, 18, 18, fill="gray")
        self.lbl_z = ttk.Label(f_indicators, text="Z-LIMIT: UNKNOWN", font=("Courier", 10, "bold"))
        self.lbl_z.grid(row=0, column=1, padx=(0, 20))

        # Left-Limit LED Canvas & Label
        self.canvas_l = tk.Canvas(f_indicators, width=20, height=20, highlightthickness=0)
        self.canvas_l.grid(row=0, column=2, padx=(10, 2))
        self.led_l = self.canvas_l.create_oval(2, 2, 18, 18, fill="gray")
        self.lbl_l = ttk.Label(f_indicators, text="LEFT: UNKNOWN", font=("Courier", 10, "bold"))
        self.lbl_l.grid(row=0, column=3, padx=(0, 20))

        # Right-Limit LED Canvas & Label
        self.canvas_r = tk.Canvas(f_indicators, width=20, height=20, highlightthickness=0)
        self.canvas_r.grid(row=0, column=4, padx=(10, 2))
        self.led_r = self.canvas_r.create_oval(2, 2, 18, 18, fill="gray")
        self.lbl_r = ttk.Label(f_indicators, text="RIGHT: UNKNOWN", font=("Courier", 10, "bold"))
        self.lbl_r.grid(row=0, column=5, padx=(0, 10))

        # ── CONTROLS FRAME ──
        frame_ctrl = ttk.Frame(self)
        frame_ctrl.pack(pady=5)

        # D-Pad Column
        f_dpad = ttk.Frame(frame_ctrl)
        f_dpad.grid(row=0, column=0, padx=20)
        
        ttk.Button(f_dpad, text="▲ UP", command=lambda: self.jog(0, -1)).grid(row=0, column=1)
        ttk.Button(f_dpad, text="▼ DOWN", command=lambda: self.jog(0, 1)).grid(row=2, column=1)
        ttk.Button(f_dpad, text="◀ LEFT", command=lambda: self.jog(-1, 0)).grid(row=1, column=0)
        ttk.Button(f_dpad, text="RIGHT ▶", command=lambda: self.jog(1, 0)).grid(row=1, column=2)
        ttk.Button(f_dpad, text="HOME", command=self.do_home).grid(row=1, column=1)

        # Settings & Pen Column
        f_settings = ttk.Frame(frame_ctrl)
        f_settings.grid(row=0, column=1, padx=20)
        
        ttk.Label(f_settings, text="Z-Axis (Pen Controls)", font=("Arial", 10, "bold")).pack(anchor="w")
        
        f_pen_btns = ttk.Frame(f_settings)
        f_pen_btns.pack(anchor="w", pady=5)
        ttk.Button(f_pen_btns, text="Pen UP (Z=1)", command=lambda: self.send_pen_command(1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(f_pen_btns, text="Pen DOWN (Z=0)", command=lambda: self.send_pen_command(0)).pack(side=tk.LEFT, padx=2)

        ttk.Label(f_settings, text="\nSpeed (Inches)", font=("Arial", 10, "bold")).pack(anchor="w")
        self.rb_micro = ttk.Radiobutton(f_settings, text="Micro (0.05\") [Key 1]", variable=self.speed, value=0.05)
        self.rb_micro.pack(anchor="w")
        self.rb_med = ttk.Radiobutton(f_settings, text="Medium (0.5\") [Key 2]", variable=self.speed, value=0.5)
        self.rb_med.pack(anchor="w")
        self.rb_sprint = ttk.Radiobutton(f_settings, text="Sprint (2.5\") [Key 3]", variable=self.speed, value=2.5)
        self.rb_sprint.pack(anchor="w")

        self.console = scrolledtext.ScrolledText(self, height=7, state='disabled', bg='black', fg='lime', font=("Courier", 9))
        self.console.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)

    def start_status_polling(self):
        """Poll limit switch status periodically in a background thread."""
        def _poll():
            st = self.comms.poll_status()
            if st is not None:
                self.after(0, lambda: self.update_led_display(st))
            self.after(300, self.start_status_polling)

        threading.Thread(target=_poll, daemon=True).start()

    def update_led_display(self, st: dict):
        """Update LED colors and text labels based on active high/low switch states."""
        # Z-Limit (HIGH = Pressed / Triggered)
        if st.get('Z') == 1:
            self.canvas_z.itemconfig(self.led_z, fill="#ff3333") # Red
            self.lbl_z.config(text="Z-LIMIT: TRIGGERED", foreground="#cc0000")
        else:
            self.canvas_z.itemconfig(self.led_z, fill="#00cc44") # Green
            self.lbl_z.config(text="Z-LIMIT: CLEAR", foreground="#008822")

        # Left-Limit (HIGH = Pressed)
        if st.get('L') == 1:
            self.canvas_l.itemconfig(self.led_l, fill="#ff3333")
            self.lbl_l.config(text="LEFT: TRIGGERED", foreground="#cc0000")
        else:
            self.canvas_l.itemconfig(self.led_l, fill="#00cc44")
            self.lbl_l.config(text="LEFT: CLEAR", foreground="#008822")

        # Right-Limit (HIGH = Pressed)
        if st.get('R') == 1:
            self.canvas_r.itemconfig(self.led_r, fill="#ff3333")
            self.lbl_r.config(text="RIGHT: TRIGGERED", foreground="#cc0000")
        else:
            self.canvas_r.itemconfig(self.led_r, fill="#00cc44")
            self.lbl_r.config(text="RIGHT: CLEAR", foreground="#008822")

    def bind_shortcuts(self):
        self.bind("<Up>", lambda event: self.jog(0, -1))
        self.bind("<Down>", lambda event: self.jog(0, 1))
        self.bind("<Left>", lambda event: self.jog(-1, 0))
        self.bind("<Right>", lambda event: self.jog(1, 0))

        self.bind("1", lambda event: self.change_speed(0.05, "Micro (0.05\")"))
        self.bind("2", lambda event: self.change_speed(0.5, "Medium (0.5\")"))
        self.bind("3", lambda event: self.change_speed(2.5, "Sprint (2.5\")"))

    def change_speed(self, val, label):
        self.speed.set(val)
        self.log(f"[SYS] Speed set to {label}")

    def log(self, msg):
        def _safe_append():
            self.console.config(state='normal')
            self.console.insert(tk.END, msg + "\n")
            self.console.see(tk.END)
            self.console.config(state='disabled')
        self.after(0, _safe_append)

    def update_pos_labels(self):
        self.pos_var.set(f"X: {self.tracker.x:.2f} | Y: {self.tracker.y:.2f}")
        self.steps_var.set(f"Steps L: {self.tracker.steps_l} | Steps R: {self.tracker.steps_r}")

    def init_connection(self):
        if self.comms.connect():
            self.do_home()

    def do_home(self, event=None):
        if self.is_moving: return
        self.is_moving = True
        self.log(">>> Requesting HOME...")
        def task():
            if self.comms.home():
                hx, hy = compute_home_position()
                self.tracker = PositionTracker(hx, hy)
                self.after(0, self.update_pos_labels)
                self.log(">>> Homed successfully.")
            self.is_moving = False
        threading.Thread(target=task, daemon=True).start()

    def send_pen_command(self, z_val):
        if self.is_moving: return
        self.is_moving = True
        def task():
            self.log(f">>> Sending isolated Pen command: Z={z_val}")
            if self.comms.send_move(z_val, 0.0, 0.0):
                self.log(f">>> Pen state updated to Z={z_val}")
            self.is_moving = False
        threading.Thread(target=task, daemon=True).start()

    def jog(self, dir_x, dir_y):
        if self.is_moving: return
        self.is_moving = True
        z = 1 
        dx = dir_x * self.speed.get()
        dy = dir_y * self.speed.get()
        
        def task():
            if self.comms.send_move(z, dx, dy):
                self.tracker.apply_relative_move(dx, dy)
                self.after(0, self.update_pos_labels)
            self.is_moving = False
        
        threading.Thread(target=task, daemon=True).start()

if __name__ == "__main__":
    app = PronterfaceGUI()
    app.mainloop()
