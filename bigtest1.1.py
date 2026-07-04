"""
unified_wizard.py
=================
Marries magicgrab_1_1.py (interactive workspace calibration GUI) with
safewizard1_1.py (G-code parsing + plunge-point execution).

Flow:
  1.  Auto-home robot  (send 'H', wait for CNC_READY)
  2.  Rectangle calibration  (arrow-key jog to mark TL/TR/BR corners)   OR load preset
  3.  Load & preview G-code mapped into that rectangle
  4.  Execute on hardware

Position tracking philosophy
-----------------------------
The Arduino tracks its own internal step counters (stepsL / stepsR) and
absolute position (curX / curY).  This script mirrors that state exactly
in Python so both sides are always in sync:

    self.phys_x, self.phys_y   – current absolute gondola position (inches)
    self.steps_l, self.steps_r – mirror of Arduino step counters

Every motion sent to the robot is a *relative* delta (the Arduino expects
"G <zflag> <dx> <dy>").  G-code paths are in absolute coordinates, so we
convert to relative deltas here before sending.
"""

import serial
import time
import math
import re
import sys
import json
import os

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button

# ─────────────────────────────── CONFIG ────────────────────────────────
SIMULATION_MODE  = False          # Set False for real hardware
PORT             = 'COM7'
BAUD             = 115200

MOTOR_DISTANCE   = 42.0          # inches between motor shafts
STEPS_PER_INCH   = 2032.0        # from Arduino
PULLEY_RADIUS    = 0.3183        # inches
RESOLUTION       = 0.05          # segment length for kinematics

# Known string lengths after homing calibration (matches Arduino CAL_ values)
CAL_L_MEASURED   = 12.0
CAL_R_MEASURED   = 11.5625

INPUT_FILE = r"C:\Users\Evan\Desktop\String Art\llama_output.ncg"
# ────────────────────────────────────────────────────────────────────────

# ─────────────────────────────── PRESET MANAGER ─────────────────────────
PRESET_FILE = os.path.join(os.path.dirname(__file__), "presets.json")

def load_presets():
    """Load presets dictionary from JSON file."""
    if not os.path.exists(PRESET_FILE):
        return {}
    try:
        with open(PRESET_FILE, 'r') as f:
            return json.load(f)
    except (json.JSONDecodeError, IOError):
        return {}

def save_presets(presets):
    """Save presets dictionary to JSON file."""
    with open(PRESET_FILE, 'w') as f:
        json.dump(presets, f, indent=2)

def delete_preset(name):
    presets = load_presets()
    if name in presets:
        del presets[name]
        save_presets(presets)
        print(f"Preset '{name}' deleted.")
    else:
        print(f"Preset '{name}' not found.")

def list_presets():
    presets = load_presets()
    if not presets:
        print("No saved presets.")
        return []
    print("\nAvailable presets:")
    for i, name in enumerate(presets.keys(), 1):
        print(f"  {i}. {name}")
    return list(presets.keys())

def get_preset_corners(name):
    presets = load_presets()
    if name in presets:
        return presets[name].get("corners", None)
    return None

def save_preset(name, corners):
    """Save a rectangle preset (corners: [TL, TR, BR, BL])."""
    presets = load_presets()
    presets[name] = {"corners": corners}
    save_presets(presets)
    print(f"Preset '{name}' saved.")

# ────────────────────────────────────────────────────────────────────────

# ══════════════════════════════════════════════════════════════════════
#   KINEMATICS  (mirrors Arduino getStepsForCoord)
# ══════════════════════════════════════════════════════════════════════
def get_steps_for_coord(x: float, y: float, is_left: bool,
                        cur_steps_l: int, cur_steps_r: int) -> int:
    """Return the absolute step count the motor should be at to reach (x,y)."""
    mx = 0.0 if is_left else MOTOR_DISTANCE
    dx = x - mx
    dy = y
    d2 = dx * dx + dy * dy
    d  = math.sqrt(d2)
    r  = PULLEY_RADIUS

    cur_steps = cur_steps_l if is_left else cur_steps_r
    if d < (r + 0.1):
        return cur_steps

    l_straight = math.sqrt(max(0.0, d2 - r * r))
    theta = math.atan2(dy, dx)
    phi   = math.acos(max(-1.0, min(1.0, r / d)))

    if is_left:
        wrap_angle = math.pi / 2.0 - (theta + phi)
    else:
        wrap_angle = theta - phi - math.pi / 2.0

    return int((l_straight + r * abs(wrap_angle)) * STEPS_PER_INCH)


def compute_home_position() -> tuple[float, float]:
    """
    Reproduce the Arduino post-calibration position calculation.
    After homing, stepsL and stepsR are set from CAL measurements.
    """
    sl = CAL_L_MEASURED * STEPS_PER_INCH
    sr = CAL_R_MEASURED * STEPS_PER_INCH
    L  = sl / STEPS_PER_INCH
    R  = sr / STEPS_PER_INCH
    hx = (L * L - R * R + MOTOR_DISTANCE * MOTOR_DISTANCE) / (2.0 * MOTOR_DISTANCE)
    hy = math.sqrt(max(0.0, L * L - hx * hx))
    # Arduino then moves to (21, 15) — centre of the 42-inch span
    return 21.0, 15.0


# ══════════════════════════════════════════════════════════════════════
#   SERIAL LAYER
# ══════════════════════════════════════════════════════════════════════
class RobotComms:
    def __init__(self):
        self.ser = None

    def connect(self):
        if SIMULATION_MODE:
            print("[SIM] Serial not opened.")
            return
        self.ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(3)
        self.ser.reset_input_buffer()

    def send_raw(self, data: bytes):
        if SIMULATION_MODE:
            return
        self.ser.write(data)

    def wait_for(self, keyword: str, timeout: float = 60.0) -> bool:
        if SIMULATION_MODE:
            return True
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  [ROBOT] {line}")
            if keyword in line:
                return True
        return False

    def home(self) -> bool:
        print("Sending home command 'H' …")
        self.send_raw(b'H\n')
        ok = self.wait_for('CNC_READY', timeout=120)
        if ok:
            print("Robot homed and ready.")
        else:
            print("WARNING: Timed out waiting for CNC_READY.")
        return ok

    def send_move(self, z_flag: int, dx: float, dy: float) -> bool:
        """
        Send a relative move command: G <z_flag> <dx> <dy>
        z_flag 0 = pen down, 1 = pen up.
        Waits for DONE acknowledgement.
        """
        cmd = f"G {z_flag} {dx:.4f} {dy:.4f}\n"
        if SIMULATION_MODE:
            return True
        self.send_raw(cmd.encode())
        return self.wait_for('DONE', timeout=30)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ══════════════════════════════════════════════════════════════════════
#   POSITION TRACKER  — always knows where the gondola is
# ══════════════════════════════════════════════════════════════════════
class PositionTracker:
    """
    Mirrors the Arduino's curX/curY and stepsL/stepsR exactly.
    Call apply_relative_move() after every physical motion.
    """
    def __init__(self, home_x: float, home_y: float):
        self.x = home_x
        self.y = home_y
        # Seed step counters from calibration measurements
        self.steps_l = int(CAL_L_MEASURED * STEPS_PER_INCH)
        self.steps_r = int(CAL_R_MEASURED * STEPS_PER_INCH)

    def apply_relative_move(self, dx: float, dy: float):
        """Update position and step counters after a relative move."""
        tx = self.x + dx
        ty = self.y + dy
        self.steps_l = get_steps_for_coord(tx, ty, True,  self.steps_l, self.steps_r)
        self.steps_r = get_steps_for_coord(tx, ty, False, self.steps_l, self.steps_r)
        self.x = tx
        self.y = ty

    def relative_to(self, abs_x: float, abs_y: float) -> tuple[float, float]:
        """Return (dx, dy) from current position to an absolute target."""
        return abs_x - self.x, abs_y - self.y

    def __repr__(self):
        return (f"Pos({self.x:.3f}, {self.y:.3f}) "
                f"stL={self.steps_l} stR={self.steps_r}")


# ══════════════════════════════════════════════════════════════════════
#   G-CODE PARSER
# ══════════════════════════════════════════════════════════════════════
def load_gcode_absolute(filename: str) -> list[list[float]]:
    """Return a list of [x, y] absolute G-code waypoints."""
    pts = []
    try:
        with open(filename, 'r') as f:
            last_x, last_y = 0.0, 0.0
            for line in f:
                line = line.split(';')[0].strip()
                if not line:
                    continue
                xm = re.search(r'X([-+]?\d*\.\d+|\d+)', line)
                ym = re.search(r'Y([-+]?\d*\.\d+|\d+)', line)
                if xm or ym:
                    cx = float(xm.group(1)) if xm else last_x
                    cy = float(ym.group(1)) if ym else last_y
                    pts.append([cx, cy])
                    last_x, last_y = cx, cy
    except FileNotFoundError:
        print(f"[WARN] G-code file not found: {filename}")
        # Fallback demo square
        pts = [[0,0],[5,0],[5,5],[0,5],[0,0]]
    return pts


def fit_gcode_to_workspace(gcode_pts: list[list[float]],
                           workspace: dict) -> list[list[float]]:
    """
    Map G-code absolute coordinates into the calibrated physical workspace.
    workspace = {
        'origin': (phys_x, phys_y),   # TL corner in robot coordinates
        'width':  float,               # inches
        'height': float,               # inches
    }
    Returns a list of [phys_x, phys_y] absolute robot coordinates.
    """
    xs = [p[0] for p in gcode_pts]
    ys = [p[1] for p in gcode_pts]
    gw = max(xs) - min(xs) or 1.0
    gh = max(ys) - min(ys) or 1.0
    cx_g = (max(xs) + min(xs)) / 2.0
    cy_g = (max(ys) + min(ys)) / 2.0

    scale = min(workspace['width'] / gw, workspace['height'] / gh) * 0.90
    ox, oy = workspace['origin']
    cx_w = ox + workspace['width']  / 2.0
    cy_w = oy + workspace['height'] / 2.0

    mapped = []
    for px, py in gcode_pts:
        mx = cx_w + (px - cx_g) * scale
        my = cy_w + (py - cy_g) * scale
        mapped.append([mx, my])
    return mapped


# ══════════════════════════════════════════════════════════════════════
#   PLUNGE-POINT COMMAND BUILDER
# ══════════════════════════════════════════════════════════════════════
def build_command_sequence(mapped_pts: list[list[float]],
                           tracker: PositionTracker) -> list[tuple]:
    """
    Convert absolute mapped waypoints into a sequence of
    (z_flag, dx, dy) relative robot commands.

    Each waypoint gets:  MOVE (pen up) → PLUNGE → RETRACT

    The tracker is NOT mutated here — we use a shadow copy so the
    real tracker only advances when commands are actually confirmed sent.
    """
    commands = []
    sx, sy = tracker.x, tracker.y   # shadow position

    for ax, ay in mapped_pts:
        rdx = ax - sx
        rdy = ay - sy
        if abs(rdx) < 0.0001 and abs(rdy) < 0.0001:
            continue
        commands.append((1, rdx, rdy))   # move pen-up
        commands.append((0, 0.0, 0.0))   # plunge
        commands.append((1, 0.0, 0.0))   # retract
        sx, sy = ax, ay

    # Return to starting position (pen up)
    commands.append((1, tracker.x - sx, tracker.y - sy))
    return commands


# ══════════════════════════════════════════════════════════════════════
#   CALIBRATION + PREVIEW GUI
# ══════════════════════════════════════════════════════════════════════
class CalibrationWizard:
    """
    Interactive jog UI taken from magicgrab.
    States:
      0 – jog to Top-Left,  press Enter
      1 – jog to Top-Right, press Enter (Y locked to TL)
      2 – jog to Bot-Right, press Enter (X locked to TR)
      3 – preview / adjust artwork overlay
      4 – confirmed → close window
    """
    SPEEDS = {'1': 0.05, '2': 0.5, '3': 2.5}
    SPEED_LABELS = {'1': 'MICRO', '2': 'MEDIUM', '3': 'SPRINT'}

    def __init__(self, comms: RobotComms, tracker: PositionTracker,
                 gcode_pts: list[list[float]], preset_corners=None):
        self.comms   = comms
        self.tracker = tracker
        self.gcode_pts = gcode_pts

        if preset_corners is not None and len(preset_corners) == 4:
            # Load preset rectangle – skip jogging, go straight to preview
            self.corners = preset_corners[:]   # [TL, TR, BR, BL]
            self.state   = 3
            self._needs_auto_fit = True
        else:
            self.corners   = [None, None, None, None]  # TL TR BR BL
            self.state     = 0
            self._needs_auto_fit = False

        self.gear      = '2'

        # Artwork transform (applied on top of workspace mapping)
        self.rotation  = 0
        self.scale_x   = 1.0
        self.scale_y   = 1.0
        self.off_x     = 0.0
        self.off_y     = 0.0

        # Drag-handle state
        self.active_handle = None
        self.last_mouse    = (0, 0)

        # Normalise raw G-code to centred unit space
        self._normalise_gcode()

    # ── G-code normalisation ──────────────────────────────────────────
    def _normalise_gcode(self):
        pts = self.gcode_pts
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        cx = (max(xs) + min(xs)) / 2.0
        cy = (max(ys) + min(ys)) / 2.0
        self.base_w = max(max(xs) - min(xs), 0.1)
        self.base_h = max(max(ys) - min(ys), 0.1)
        self.raw_pts = [[p[0] - cx, p[1] - cy] for p in pts]

    # ── Artwork transform ─────────────────────────────────────────────
    def get_transformed_pts(self) -> list[list[float]]:
        rad = math.radians(self.rotation)
        out = []
        ox, oy = self.corners[0] if self.corners[0] else (0,0)
        for px, py in self.raw_pts:
            rx = px * math.cos(rad) - py * math.sin(rad)
            ry = px * math.sin(rad) + py * math.cos(rad)
            out.append([ox + self.off_x + rx * self.scale_x,
                        oy + self.off_y + ry * self.scale_y])
        return out

    # ── Auto-fit artwork into workspace ──────────────────────────────
    def btn_center(self, _event):
        if self.state < 3 or self.corners[0] is None:
            return
        bw = self.corners[1][0] - self.corners[0][0]
        bh = self.corners[2][1] - self.corners[1][1]
        self.off_x  = bw / 2.0
        self.off_y  = bh / 2.0
        s = min(bw / self.base_w, bh / self.base_h) * 0.85
        self.scale_x = self.scale_y = s
        self.update_plot()

    # ── Plot update ───────────────────────────────────────────────────
    def update_plot(self):
        self.ax.clear()

        if self.state < 3:
            # Full machine view
            self.ax.plot([0, MOTOR_DISTANCE], [0, 0],
                         'ks', ms=10, label="Motors")
            self.ax.set_xlim(-5, MOTOR_DISTANCE + 5)
            self.ax.set_ylim(-5, 45)
            if self.state == 1 and self.corners[0]:
                self.ax.axhline(self.corners[0][1], color='cyan',
                                ls=':', alpha=0.6, label='Y-lock')
            if self.state == 2 and self.corners[1]:
                self.ax.axvline(self.corners[1][0], color='magenta',
                                ls=':', alpha=0.6, label='X-lock')
        else:
            # Zoomed dashboard view (state 3)
            bw = self.corners[1][0] - self.corners[0][0]
            bh = self.corners[2][1] - self.corners[1][1]
            self.ax.set_xlim(self.corners[0][0] - 2, self.corners[1][0] + 2)
            self.ax.set_ylim(self.corners[0][1] - 2, self.corners[2][1] + 2)
            self.ax.add_patch(patches.Rectangle(
                self.corners[0], bw, bh,
                lw=2, ec='b', fc='none', ls='--', label='Workspace'))

            t_pts = self.get_transformed_pts()
            if t_pts:
                tx, ty = zip(*t_pts)
                self.ax.plot(tx, ty, 'g-', alpha=0.6, label='Artwork')

                # Drag handles (N/S/E/W)
                mx = (max(tx) + min(tx)) / 2.0
                my = (max(ty) + min(ty)) / 2.0
                self.ax.plot(
                    [mx, mx, max(tx), min(tx)],
                    [min(ty), max(ty), my, my],
                    'yo', ms=12, mec='k', zorder=10, label='Handles')

        # Corner markers
        for i, pt in enumerate(self.corners):
            if pt:
                col = ['cyan', 'magenta', 'green', 'yellow'][i]
                lbl = ['TL', 'TR', 'BR', 'BL'][i]
                self.ax.plot(pt[0], pt[1], 'x', color=col, ms=10, mew=2)
                self.ax.text(pt[0] + 0.4, pt[1], lbl, color=col, weight='bold')

        # Current gondola position
        self.ax.plot(self.tracker.x, self.tracker.y, 'ro', ms=8, zorder=20,
                     label='Gondola')

        # HUD
        task_names = ['SET TL', 'SET TR', 'SET BR', 'PREVIEW/ADJUST']
        hud = (f"GEAR: {self.SPEED_LABELS[self.gear]}\n"
               f"TASK: {task_names[min(self.state, 3)]}\n"
               f"POS:  ({self.tracker.x:.2f}, {self.tracker.y:.2f})\n"
               f"stL={self.tracker.steps_l}  stR={self.tracker.steps_r}")
        self.ax.text(1.03, 0.85, hud, transform=self.ax.transAxes,
                     bbox=dict(facecolor='wheat', alpha=0.85), fontsize=8,
                     va='top', fontfamily='monospace')

        self.ax.invert_yaxis()
        self.ax.set_aspect('equal')
        self.ax.legend(loc='lower left', fontsize=7)
        self.fig.canvas.draw_idle()

    # ── Keyboard handler ──────────────────────────────────────────────
    def on_key(self, event):
        if event.key in self.SPEEDS:
            self.gear = event.key
            self.update_plot()
            return

        if self.state < 3:
            s  = self.SPEEDS[self.gear]
            dx = dy = 0.0

            # Axis locks: TR shares Y with TL; BR shares X with TR
            if event.key == 'up'    and self.state in [0, 2]: dy = -s
            elif event.key == 'down'  and self.state in [0, 2]: dy =  s
            elif event.key == 'left'  and self.state in [0, 1]: dx = -s
            elif event.key == 'right' and self.state in [0, 1]: dx =  s

            if dx or dy:
                self.comms.send_move(1, dx, dy)          # pen up, relative move
                self.tracker.apply_relative_move(dx, dy)
                self.update_plot()

            if event.key == 'enter':
                self.corners[self.state] = (self.tracker.x, self.tracker.y)

                if self.state == 0:
                    self.state = 1
                    # Enforce Y-lock: snap TR starting point to same Y as TL
                    dy_snap = self.corners[0][1] - self.tracker.y
                    if abs(dy_snap) > 0.0001:
                        self.comms.send_move(1, 0.0, dy_snap)
                        self.tracker.apply_relative_move(0.0, dy_snap)

                elif self.state == 1:
                    self.state = 2
                    # Enforce X-lock: snap BR starting point to same X as TR
                    dx_snap = self.corners[1][0] - self.tracker.x
                    if abs(dx_snap) > 0.0001:
                        self.comms.send_move(1, dx_snap, 0.0)
                        self.tracker.apply_relative_move(dx_snap, 0.0)

                elif self.state == 2:
                    self.state = 3
                    h = self.corners[2][1] - self.corners[1][1]
                    self.corners[3] = (self.corners[0][0],
                                       self.corners[0][1] + h)
                    self.btn_center(None)

                self.update_plot()

    # ── Mouse drag handlers ───────────────────────────────────────────
    def on_click(self, event):
        if self.state != 3 or event.inaxes != self.ax:
            return
        self.last_mouse = (event.xdata, event.ydata)
        t_pts = self.get_transformed_pts()
        if not t_pts:
            return
        xs, ys = [p[0] for p in t_pts], [p[1] for p in t_pts]
        mx, my = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
        handles = {
            'N': (mx, min(ys)), 'S': (mx, max(ys)),
            'E': (max(xs), my), 'W': (min(xs), my)
        }
        for k, pos in handles.items():
            if math.hypot(event.xdata - pos[0], event.ydata - pos[1]) < 1.0:
                self.active_handle = k
                return
        self.active_handle = 'center'

    def on_motion(self, event):
        if not self.active_handle or event.inaxes != self.ax:
            return
        dx = event.xdata - self.last_mouse[0]
        dy = event.ydata - self.last_mouse[1]
        if self.active_handle == 'center':
            self.off_x += dx
            self.off_y += dy
        elif self.active_handle in ['E', 'W']:
            self.scale_x += (dx / self.base_w) * (1 if self.active_handle == 'E' else -1)
        elif self.active_handle in ['N', 'S']:
            self.scale_y += (dy / self.base_h) * (1 if self.active_handle == 'S' else -1)
        self.last_mouse = (event.xdata, event.ydata)
        self.update_plot()

    # ── Build workspace dict from confirmed corners ───────────────────
    def get_workspace(self) -> dict:
        tl = self.corners[0]
        tr = self.corners[1]
        br = self.corners[2]
        return {
            'origin': tl,
            'width':  tr[0] - tl[0],
            'height': br[1] - tr[1],
            'corners': self.corners,
        }

    # ── GUI setup + run ───────────────────────────────────────────────
    def run(self):
        self.fig, self.ax = plt.subplots(figsize=(13, 8))
        plt.subplots_adjust(left=0.22)

        ax_rot = plt.axes([0.02, 0.65, 0.14, 0.05])
        self.b_rot = Button(ax_rot, 'Rotate 90°')
        self.b_rot.on_clicked(
            lambda e: (setattr(self, 'rotation', (self.rotation + 90) % 360),
                       self.update_plot()))

        ax_cnt = plt.axes([0.02, 0.55, 0.14, 0.05])
        self.b_cnt = Button(ax_cnt, 'Auto-Fit')
        self.b_cnt.on_clicked(self.btn_center)

        ax_go = plt.axes([0.02, 0.10, 0.14, 0.09], facecolor='#90ee90')
        self.b_go = Button(ax_go, 'CONFIRM\n& EXECUTE')
        self.b_go.on_clicked(lambda e: (setattr(self, 'state', 4), plt.close()))

        self.fig.canvas.mpl_connect('button_press_event',   self.on_click)
        self.fig.canvas.mpl_connect('button_release_event',
                                    lambda e: setattr(self, 'active_handle', None))
        self.fig.canvas.mpl_connect('motion_notify_event',  self.on_motion)
        self.fig.canvas.mpl_connect('key_press_event',      self.on_key)

        self.update_plot()

        if self._needs_auto_fit:
            self.btn_center(None)
            self.update_plot()

        plt.suptitle(
            "UNIFIED WIZARD  |  Arrow keys: jog   1/2/3: gear   Enter: confirm corner",
            fontsize=9)
        plt.show()

        return self.state == 4   # True if user confirmed


# ══════════════════════════════════════════════════════════════════════
#   EXECUTION ENGINE
# ══════════════════════════════════════════════════════════════════════
def execute_commands(commands: list[tuple],
                     tracker: PositionTracker,
                     comms: RobotComms):
    """
    Send every command to the robot, updating the Python tracker after
    each confirmed DONE so we never drift out of sync.
    """
    total = len(commands)
    print(f"\nExecuting {total} commands …")
    for i, (z_flag, dx, dy) in enumerate(commands):
        action = "PLUNGE" if z_flag == 0 else f"MOVE ({dx:+.3f}, {dy:+.3f})"
        print(f"  [{i+1:4d}/{total}] z={z_flag} {action}   "
              f"pos≈({tracker.x:.2f},{tracker.y:.2f})")
        ok = comms.send_move(z_flag, dx, dy)
        if not ok:
            print("  ERROR: DONE not received — aborting.")
            break
        tracker.apply_relative_move(dx, dy)

    print(f"Execution complete.  Final pos: {tracker}")


# ══════════════════════════════════════════════════════════════════════
#   MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    # ── 1. Connect ────────────────────────────────────────────────────
    comms = RobotComms()
    comms.connect()

    # ── 2. Auto-home ──────────────────────────────────────────────────
    if not comms.home():
        print("Homing failed — exiting.")
        sys.exit(1)

    # Seed Python-side position tracker with post-home state
    hx, hy = compute_home_position()
    tracker = PositionTracker(hx, hy)
    print(f"Home position: {tracker}")

    # ── 3. Load G-code ────────────────────────────────────────────────
    print(f"Loading G-code from: {INPUT_FILE}")
    gcode_pts = load_gcode_absolute(INPUT_FILE)
    print(f"  {len(gcode_pts)} waypoints loaded.")

    # ── 3.5 Preset selection (before calibration) ─────────────────────
    preset_corners = None
    choice = input("\nLoad preset or new? (l/n): ").strip().lower()
    if choice == 'l':
        presets = load_presets()
        if not presets:
            print("No saved presets. Starting new calibration.")
        else:
            names = list(presets.keys())
            print("\nAvailable presets:")
            for i, name in enumerate(names, 1):
                print(f"  {i}. {name}")
            print("  d. Delete a preset")
            sel = input("Enter number or name (or 'd' to delete): ").strip()
            if sel.lower() == 'd':
                del_name = input("Enter preset name to delete: ").strip()
                if del_name in presets:
                    delete_preset(del_name)
                    print("Deleted. Starting new calibration.")
                else:
                    print("Preset not found. Starting new calibration.")
            else:
                try:
                    idx = int(sel) - 1
                    if 0 <= idx < len(names):
                        name = names[idx]
                    else:
                        name = sel
                except ValueError:
                    name = sel
                corners = get_preset_corners(name)
                if corners:
                    preset_corners = corners
                    print(f"Loaded preset '{name}'.")
                else:
                    print(f"Preset '{name}' not found. Starting new calibration.")

    # ── 4. Calibration GUI (or load preset) ───────────────────────────
    print("\nOpening calibration GUI …")
    wizard = CalibrationWizard(comms, tracker, gcode_pts, preset_corners)
    confirmed = wizard.run()

    if not confirmed:
        print("Calibration cancelled.")
        comms.close()
        sys.exit(0)

    workspace = wizard.get_workspace()
    print(f"\nWorkspace confirmed:")
    print(f"  TL={workspace['corners'][0]}  TR={workspace['corners'][1]}")
    print(f"  BR={workspace['corners'][2]}  BL={workspace['corners'][3]}")
    print(f"  Size: {workspace['width']:.2f}\" × {workspace['height']:.2f}\"")

    # ── 4.5 Ask to save preset (if not loaded from one) ───────────────
    if preset_corners is None:
        save_ans = input("\nSave this rectangle as preset? (y/n): ").strip().lower()
        if save_ans == 'y':
            name = input("Preset name: ").strip()
            if name:
                save_preset(name, workspace['corners'])
            else:
                print("No name entered – not saved.")

    # ── 5. Map artwork into workspace ─────────────────────────────────
    # Use the fine-tuned transform from the wizard if available
    if wizard.state >= 3:
        mapped_pts = wizard.get_transformed_pts()
    else:
        mapped_pts = fit_gcode_to_workspace(gcode_pts, workspace)

    # ── 6. Build relative command sequence ───────────────────────────
    commands = build_command_sequence(mapped_pts, tracker)
    print(f"\n{len(commands)} robot commands built.")

    # ── 7. Preview ────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.plot([0, MOTOR_DISTANCE], [0, 0], 'ks', ms=10)

    cx, cy = tracker.x, tracker.y
    plunge_x, plunge_y = [], []
    for z_flag, dx, dy in commands:
        nx, ny = cx + dx, cy + dy
        if z_flag == 1 and (abs(dx) > 0.0001 or abs(dy) > 0.0001):
            ax.plot([cx, nx], [cy, ny], 'g--', alpha=0.3)
        if z_flag == 0:
            plunge_x.append(nx)
            plunge_y.append(ny)
        cx, cy = nx, ny

    ax.scatter(plunge_x, plunge_y, c='red', s=20, zorder=5, label=f"Plunges ({len(plunge_x)})")
    ax.plot(tracker.x, tracker.y, 'bo', ms=10, label='Home')

    # Draw workspace rectangle
    ws = workspace
    tl = ws['corners'][0]
    ax.add_patch(patches.Rectangle(tl, ws['width'], ws['height'],
                                   lw=2, ec='blue', fc='none', ls='--', label='Workspace'))

    ax.invert_yaxis()
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title(f"Execution Preview — {len(plunge_x)} plunge points")
    plt.tight_layout()
    plt.show()

    # ── 8. Final confirmation ─────────────────────────────────────────
    ans = input("\nSend to robot? (y/n): ").strip().lower()
    if ans != 'y':
        print("Aborted.")
        comms.close()
        sys.exit(0)

    # ── 9. Execute ────────────────────────────────────────────────────
    execute_commands(commands, tracker, comms)
    print("\nJob complete.")
    comms.close()


if __name__ == "__main__":
    main()