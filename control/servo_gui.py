"""
Servo control GUI for 3DOF arm.
Sends ASCII commands over UART in the format: S<ch>:<angle>\n
Example: "S1:90\n" sets channel 1 to 90 degrees.

Requires: pip install pyserial
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import IK_control as ik_solver


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# Per-channel config: (ch_num, label, default_angle, min_angle, max_angle)
CHANNELS = [
    (1, "Base",   90,  0,   180),
    (2, "Height", 90,  0,  120),
    (3, "Depth",  102, 55,  180),
]

CLAW_OPEN_DEG  = 75
CLAW_CLOSE_DEG = 18


class ServoGUI:
    BAUD_RATE = 115200

    def __init__(self, root):
        self.root = root
        self.root.title("3DOF Arm Servo Control")
        self.root.resizable(False, False)
        self.ser = None
        self.channel_vars = {}  # ch_num -> IntVar
        self._claw_open = True  # True = open (75°), False = closed (30°)

        self._build_connection_frame()
        self._build_servo_frame()
        self._build_ik_frame()
        self._build_status_bar()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_connection_frame(self):
        frame = ttk.LabelFrame(self.root, text="Connection", padding=8)
        frame.grid(row=0, column=0, padx=10, pady=(10, 4), sticky="ew")

        ttk.Label(frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 8))

        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 8))

        self.connect_btn = ttk.Button(frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=3)

        self._refresh_ports()

    def _build_servo_frame(self):
        frame = ttk.LabelFrame(self.root, text="Servo Channels", padding=8)
        frame.grid(row=1, column=0, padx=10, pady=4, sticky="ew")

        self.send_btns = []

        for row_idx, (ch_num, label, default, ch_min, ch_max) in enumerate(CHANNELS):
            angle_var = tk.IntVar(value=default)
            self.channel_vars[ch_num] = angle_var

            ttk.Label(frame, text=f"Ch {ch_num} – {label}:", width=14, anchor="w").grid(
                row=row_idx, column=0, sticky="w", pady=3
            )

            slider = ttk.Scale(
                frame, from_=ch_min, to=ch_max, orient="horizontal",
                variable=angle_var, length=260,
                command=lambda val, v=angle_var, lo=ch_min, hi=ch_max: v.set(max(lo, min(hi, int(float(val)))))
            )
            slider.grid(row=row_idx, column=1, padx=(8, 6))

            entry = ttk.Entry(frame, textvariable=angle_var, width=5)
            entry.grid(row=row_idx, column=2, padx=(0, 8))
            entry.bind("<Return>",   lambda e, v=angle_var, lo=ch_min, hi=ch_max: self._clamp_entry(v, lo, hi))
            entry.bind("<FocusOut>", lambda e, v=angle_var, lo=ch_min, hi=ch_max: self._clamp_entry(v, lo, hi))

            btn = ttk.Button(
                frame, text="Send",
                command=lambda n=ch_num, v=angle_var, lo=ch_min, hi=ch_max: self._send_command(n, v.get(), lo, hi),
                state="disabled"
            )
            btn.grid(row=row_idx, column=3)
            self.send_btns.append(btn)

        # Claw toggle row
        claw_row = len(CHANNELS)
        ttk.Label(frame, text="Ch 4 – Claw:", width=14, anchor="w").grid(
            row=claw_row, column=0, sticky="w", pady=3
        )
        self.claw_btn = ttk.Button(
            frame, text="Open", command=self._toggle_claw, state="disabled"
        )
        self.claw_btn.grid(row=claw_row, column=1, sticky="w", padx=(8, 0))
        self.send_btns.append(self.claw_btn)

        self.reset_btn = ttk.Button(
            frame, text="Reset to Defaults", command=self._reset_to_defaults, state="disabled"
        )
        self.reset_btn.grid(row=claw_row + 1, column=0, columnspan=4, pady=(8, 2))

    def _build_ik_frame(self):
        frame = ttk.LabelFrame(self.root, text="IK Control (x, y, z in mm)", padding=8)
        frame.grid(row=2, column=0, padx=10, pady=4, sticky="ew")

        self.ik_vars = {}
        for col, axis in enumerate(["x", "y", "z"]):
            ttk.Label(frame, text=f"{axis}:").grid(row=0, column=col * 2, sticky="e", padx=(8 if col else 0, 2))
            var = tk.StringVar(value="100" if axis == "x" else "0" if axis == "y" else "80")
            self.ik_vars[axis] = var
            ttk.Entry(frame, textvariable=var, width=7).grid(row=0, column=col * 2 + 1, padx=(0, 6))

        self.elbow_up_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="Elbow up", variable=self.elbow_up_var).grid(row=0, column=6, padx=(4, 8))

        self.ik_btn = ttk.Button(frame, text="Solve & Send", command=self._solve_and_send, state="disabled")
        self.ik_btn.grid(row=0, column=7, padx=(0, 4))

        self.ik_result_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.ik_result_var, foreground="gray").grid(
            row=1, column=0, columnspan=8, sticky="w", pady=(4, 0)
        )

    def _solve_and_send(self):
        try:
            x = float(self.ik_vars["x"].get())
            y = float(self.ik_vars["y"].get())
            z = float(self.ik_vars["z"].get())
        except ValueError:
            self.ik_result_var.set("⚠ x, y, z must be numbers.")
            return

        try:
            result = ik_solver.solve_ik(x, y, z, elbow_up=self.elbow_up_var.get())
        except ValueError as e:
            self.ik_result_var.set(f"✗ {e}")
            return

        # Update sliders so the GUI reflects the computed angles
        self.channel_vars[1].set(int(round(result.base_deg)))
        self.channel_vars[2].set(int(round(result.shoulder_deg)))
        self.channel_vars[3].set(int(round(result.elbow_deg)))

        # Send over the same serial connection the GUI already owns
        angles = [
            (1, result.base_deg,     0,  180),
            (2, result.shoulder_deg, 40, 120),
            (3, result.elbow_deg,    55, 150),
        ]
        self._send_staggered(angles, 0)

        msg = f"base={result.base_deg}°  shoulder={result.shoulder_deg}°  elbow={result.elbow_deg}°"
        if result.clamped:
            notes = "  |  ⚠ " + ";  ".join(result.clamp_notes)
            msg += notes
        self.ik_result_var.set(msg)

    def _build_status_bar(self):
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=3, column=0, padx=10, pady=(4, 2), sticky="ew"
        )

        self.rx_var = tk.StringVar(value="STM32: —")
        ttk.Label(self.root, textvariable=self.rx_var, relief="sunken", anchor="w").grid(
            row=4, column=0, padx=10, pady=(0, 10), sticky="ew"
        )

    # ------------------------------------------------------------------
    # Event handlers
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
            for btn in self.send_btns:
                btn.config(state="normal")
            self.reset_btn.config(state="normal")
            self.ik_btn.config(state="normal")
            self.status_var.set(f"Connected: {port} @ {self.BAUD_RATE} baud")
            self._poll_rx()
        except serial.SerialException as e:
            messagebox.showerror("Connection failed", str(e))

    def _disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connect_btn.config(text="Connect")
        for btn in self.send_btns:
            btn.config(state="disabled")
        self.reset_btn.config(state="disabled")
        self.ik_btn.config(state="disabled")
        self.status_var.set("Disconnected")

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

    def _reset_to_defaults(self):
        for ch_num, _label, default, _lo, _hi in CHANNELS:
            self.channel_vars[ch_num].set(default)
        self._send_reset_step(list(CHANNELS), 0)

    def _toggle_claw(self):
        self._claw_open = not self._claw_open
        angle = CLAW_OPEN_DEG if self._claw_open else CLAW_CLOSE_DEG
        self.claw_btn.config(text="Open" if self._claw_open else "Close")
        self._send_command(4, angle, CLAW_CLOSE_DEG, CLAW_OPEN_DEG)

    def _send_staggered(self, commands, index):
        if index >= len(commands):
            return
        ch, angle, lo, hi = commands[index]
        self._send_command(ch, angle, lo, hi)
        self.root.after(500, lambda: self._send_staggered(commands, index + 1))

    def _send_reset_step(self, channels, index):
        if index >= len(channels):
            return
        ch_num, _label, default, ch_min, ch_max = channels[index]
        self._send_command(ch_num, default, ch_min, ch_max)
        self.root.after(500, lambda: self._send_reset_step(channels, index + 1))

    def _clamp_entry(self, var, lo, hi):
        try:
            val = max(lo, min(hi, int(var.get())))
            var.set(val)
        except (ValueError, tk.TclError):
            var.set(lo)

    def _send_command(self, channel, angle, lo, hi):
        if not self.ser or not self.ser.is_open:
            self.status_var.set("Not connected.")
            return

        angle = max(lo, min(hi, int(angle)))
        command = f"S{channel}:{angle}\n"
        try:
            self.ser.write(command.encode("ascii"))
            self.status_var.set(f"Sent: {command.strip()}")
        except serial.SerialException as e:
            self.status_var.set(f"Send error: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = ServoGUI(root)
    root.mainloop()