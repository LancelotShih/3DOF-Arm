"""
IK GUI for 3DOF arm — coordinate-based control.
Solves IK for preset (x, y, z) positions and sends servo angles over UART.
Command format: S<ch>:<angle>\n  (e.g. "S1:90\n")

Requires: pip install pyserial
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import IK_control as ik

# ---------------------------------------------------------------------------
# Claw limits (degrees)
# ---------------------------------------------------------------------------
CLAW_MIN_DEG     = 10
CLAW_MAX_DEG     = 75
CLAW_DEFAULT_DEG = 75   # open

# ---------------------------------------------------------------------------
# 3×3 grid of preset arm positions: (row_label, col_label, x_mm, y_mm, z_mm)
# Arrange as 3 rows (far → near) × 3 columns (left → right).
# Tune x/y/z values to match your physical workspace.
# ---------------------------------------------------------------------------
GRID_PRESETS = [
    # row 0 — far
    ("Far",  "Left",   130, -60, 80),
    ("Far",  "Center", 130,   0, 80),
    ("Far",  "Right",  130,  60, 80),
    # row 1 — mid
    ("Mid",  "Left",   100, -60, 80),
    ("Mid",  "Center", 100,   0, 80),
    ("Mid",  "Right",  100,  60, 80),
    # row 2 — near
    ("Near", "Left",    70, -60, 80),
    ("Near", "Center",  70,   0, 80),
    ("Near", "Right",   70,  60, 80),
]

SEND_STAGGER_MS = 500   # ms between consecutive servo commands

# Default angles to send when reset is pressed — (ch, angle, lo, hi)
RESET_COMMANDS = [
    (1,  90,  0, 180),   # Base    — default 90°
    (2,  90,  0, 120),   # Height  — default 90°
    (3, 102, 55, 180),   # Depth   — default 102°
    (4,  75, 10,  75),   # Claw    — default open (75°)
]


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# ---------------------------------------------------------------------------
# Core function: solve IK and queue staggered UART sends
# ---------------------------------------------------------------------------
def move_to_coordinate(x: float, y: float, z: float,
                        status_cb=None, ik_cb=None, stagger_fn=None):
    """
    Solve IK for (x, y, z) in mm and send S1/S2/S3 commands over UART.

    Parameters
    ----------
    ser        : open serial.Serial instance
    x, y, z    : target position in mm
    status_cb  : optional callable(str) for status messages
    ik_cb      : optional callable(str) for IK result display
    stagger_fn : callable(commands) that sends a list of (ch, angle, lo, hi)
                 with inter-command delays (GUI provides this)
    """
    try:
        result = ik.solve_ik(x, y, z)
    except ValueError as e:
        if status_cb:
            status_cb(f"IK error: {e}")
        if ik_cb:
            ik_cb("")
        return

    commands = [
        (1, result.base_deg,     0,   180),
        (2, result.shoulder_deg, 40,  120),
        (3, result.elbow_deg,    55,  180),
    ]

    if ik_cb:
        msg = (f"base={result.base_deg}°  "
               f"shoulder={result.shoulder_deg}°  "
               f"elbow={result.elbow_deg}°")
        if result.clamped:
            msg += "  |  ⚠ " + ";  ".join(result.clamp_notes)
        ik_cb(msg)

    if status_cb:
        status_cb(f"Moving to ({x}, {y}, {z}) mm…")

    if stagger_fn:
        stagger_fn(commands)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class IKGUI:
    BAUD_RATE = 115200

    def __init__(self, root):
        self.root = root
        self.root.title("3DOF Arm — IK Control")
        self.root.resizable(False, False)
        self.ser = None
        self._grid_btns = []

        self._build_connection_frame()
        self._build_grid_frame()
        self._build_claw_frame()
        self._build_status_bar()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_connection_frame(self):
        frame = ttk.LabelFrame(self.root, text="Connection", padding=8)
        frame.grid(row=0, column=0, padx=10, pady=(10, 4), sticky="ew")

        ttk.Label(frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var,
                                       width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 8))

        ttk.Button(frame, text="Refresh",
                   command=self._refresh_ports).grid(row=0, column=2, padx=(0, 8))
        self.connect_btn = ttk.Button(frame, text="Connect",
                                      command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=3)

        self._refresh_ports()

    def _build_grid_frame(self):
        frame = ttk.LabelFrame(self.root,
                               text="Preset Positions  (x, y, z  in mm)", padding=8)
        frame.grid(row=1, column=0, padx=10, pady=4, sticky="ew")

        for i, (row_lbl, col_lbl, x, y, z) in enumerate(GRID_PRESETS):
            grid_row, grid_col = divmod(i, 3)
            btn = ttk.Button(
                frame,
                text=f"{row_lbl} {col_lbl}\n({x}, {y}, {z})",
                width=16,
                state="disabled",
                command=lambda _x=x, _y=y, _z=z: self._on_grid_press(_x, _y, _z),
            )
            btn.grid(row=grid_row, column=grid_col, padx=4, pady=4)
            self._grid_btns.append(btn)

    def _build_claw_frame(self):
        frame = ttk.LabelFrame(self.root, text="Claw Control", padding=8)
        frame.grid(row=2, column=0, padx=10, pady=4, sticky="ew")

        self.claw_var = tk.IntVar(value=CLAW_DEFAULT_DEG)

        ttk.Label(frame, text=f"Angle ({CLAW_MIN_DEG}–{CLAW_MAX_DEG}°):").grid(
            row=0, column=0, sticky="w")

        slider = ttk.Scale(
            frame,
            from_=CLAW_MIN_DEG, to=CLAW_MAX_DEG,
            orient="horizontal",
            variable=self.claw_var,
            length=220,
            command=lambda val: self.claw_var.set(
                max(CLAW_MIN_DEG, min(CLAW_MAX_DEG, int(float(val))))),
        )
        slider.grid(row=0, column=1, padx=(6, 6))

        entry = ttk.Entry(frame, textvariable=self.claw_var, width=5)
        entry.grid(row=0, column=2, padx=(0, 8))
        entry.bind("<Return>",   lambda e: self._clamp_claw())
        entry.bind("<FocusOut>", lambda e: self._clamp_claw())

        self.claw_send_btn = ttk.Button(frame, text="Send Claw",
                                        state="disabled",
                                        command=self._send_claw)
        self.claw_send_btn.grid(row=0, column=3)

        self.reset_btn = ttk.Button(frame, text="Reset to Defaults",
                                    state="disabled",
                                    command=self._reset_to_defaults)
        self.reset_btn.grid(row=1, column=0, columnspan=4, pady=(8, 2))

    def _build_status_bar(self):
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(self.root, textvariable=self.status_var,
                  relief="sunken", anchor="w").grid(
            row=3, column=0, padx=10, pady=(4, 2), sticky="ew")

        self.ik_var = tk.StringVar(value="")
        ttk.Label(self.root, textvariable=self.ik_var,
                  relief="sunken", anchor="w", foreground="gray").grid(
            row=4, column=0, padx=10, pady=(0, 4), sticky="ew")

        self.rx_var = tk.StringVar(value="STM32: —")
        ttk.Label(self.root, textvariable=self.rx_var,
                  relief="sunken", anchor="w").grid(
            row=5, column=0, padx=10, pady=(0, 10), sticky="ew")

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _refresh_ports(self):
        ports = list_ports()
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "No port selected.")
            return
        try:
            self.ser = serial.Serial(port, self.BAUD_RATE, timeout=0.1)
            self.connect_btn.config(text="Disconnect")
            for btn in self._grid_btns:
                btn.config(state="normal")
            self.claw_send_btn.config(state="normal")
            self.reset_btn.config(state="normal")
            self.status_var.set(f"Connected: {port} @ {self.BAUD_RATE} baud")
            self._poll_rx()
        except serial.SerialException as e:
            messagebox.showerror("Connection failed", str(e))

    def _disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connect_btn.config(text="Connect")
        for btn in self._grid_btns:
            btn.config(state="disabled")
        self.claw_send_btn.config(state="disabled")
        self.reset_btn.config(state="disabled")
        self.status_var.set("Disconnected")

    def _reset_to_defaults(self):
        self.claw_var.set(RESET_COMMANDS[3][1])   # sync claw slider to default
        self._send_staggered(list(RESET_COMMANDS), 0)
        self.ik_var.set("")

    def _poll_rx(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if line:
                self.rx_var.set(f"STM32: {line}")
        except serial.SerialException:
            pass
        self.root.after(50, self._poll_rx)

    # ------------------------------------------------------------------
    # Movement and sending
    # ------------------------------------------------------------------

    def _on_grid_press(self, x, y, z):
        move_to_coordinate(
            x=x, y=y, z=z,
            status_cb=lambda msg: self.status_var.set(msg),
            ik_cb=lambda msg: self.ik_var.set(msg),
            stagger_fn=lambda cmds: self._send_staggered(cmds, 0),
        )

    def _send_staggered(self, commands, index):
        if index >= len(commands):
            return
        ch, angle, lo, hi = commands[index]
        self._send_command(ch, angle, lo, hi)
        self.root.after(SEND_STAGGER_MS,
                        lambda: self._send_staggered(commands, index + 1))

    def _send_command(self, channel, angle, lo, hi):
        if not self.ser or not self.ser.is_open:
            self.status_var.set("Not connected.")
            return
        angle = max(lo, min(hi, int(round(angle))))
        cmd = f"S{channel}:{angle}\n"
        try:
            self.ser.write(cmd.encode("ascii"))
            self.status_var.set(f"Sent: {cmd.strip()}")
        except serial.SerialException as e:
            self.status_var.set(f"Send error: {e}")

    def _clamp_claw(self):
        try:
            val = max(CLAW_MIN_DEG, min(CLAW_MAX_DEG, int(self.claw_var.get())))
            self.claw_var.set(val)
        except (ValueError, tk.TclError):
            self.claw_var.set(CLAW_DEFAULT_DEG)

    def _send_claw(self):
        self._clamp_claw()
        self._send_command(4, self.claw_var.get(), CLAW_MIN_DEG, CLAW_MAX_DEG)


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    root = tk.Tk()
    app = IKGUI(root)
    root.mainloop()
