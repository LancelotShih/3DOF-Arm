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


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


class ServoGUI:
    CHANNELS = [1, 2, 3, 4]
    BAUD_RATE = 115200

    def __init__(self, root):
        self.root = root
        self.root.title("3DOF Arm Servo Control")
        self.root.resizable(False, False)
        self.ser = None

        self._build_connection_frame()
        self._build_servo_frame()
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
        frame = ttk.LabelFrame(self.root, text="Servo Command", padding=8)
        frame.grid(row=1, column=0, padx=10, pady=4, sticky="ew")

        ttk.Label(frame, text="Channel:").grid(row=0, column=0, sticky="w")
        self.channel_var = tk.IntVar(value=1)
        channel_combo = ttk.Combobox(
            frame, textvariable=self.channel_var,
            values=self.CHANNELS, width=4, state="readonly"
        )
        channel_combo.grid(row=0, column=1, padx=(4, 16), sticky="w")

        ttk.Label(frame, text="Angle (0–180°):").grid(row=0, column=2, sticky="w")
        self.angle_var = tk.IntVar(value=90)

        self.slider = ttk.Scale(
            frame, from_=0, to=180, orient="horizontal",
            variable=self.angle_var, length=220,
            command=self._on_slider
        )
        self.slider.grid(row=0, column=3, padx=(4, 8))

        self.angle_entry = ttk.Entry(frame, textvariable=self.angle_var, width=5)
        self.angle_entry.grid(row=0, column=4, padx=(0, 8))
        self.angle_entry.bind("<Return>", self._on_entry)
        self.angle_entry.bind("<FocusOut>", self._on_entry)

        self.send_btn = ttk.Button(frame, text="Send", command=self._send_command, state="disabled")
        self.send_btn.grid(row=0, column=5)

    def _build_status_bar(self):
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=2, column=0, padx=10, pady=(4, 2), sticky="ew"
        )

        self.rx_var = tk.StringVar(value="STM32: —")
        ttk.Label(self.root, textvariable=self.rx_var, relief="sunken", anchor="w").grid(
            row=3, column=0, padx=10, pady=(0, 10), sticky="ew"
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
            self.send_btn.config(state="normal")
            self.status_var.set(f"Connected: {port} @ {self.BAUD_RATE} baud")
            self._poll_rx()
        except serial.SerialException as e:
            messagebox.showerror("Connection failed", str(e))

    def _disconnect(self):
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connect_btn.config(text="Connect")
        self.send_btn.config(state="disabled")
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

    def _on_slider(self, _event=None):
        self.angle_var.set(int(float(self.angle_var.get())))

    def _on_entry(self, _event=None):
        try:
            val = int(self.angle_entry.get())
            val = max(0, min(180, val))
            self.angle_var.set(val)
        except ValueError:
            self.angle_var.set(90)

    def _send_command(self):
        if not self.ser or not self.ser.is_open:
            self.status_var.set("Not connected.")
            return

        channel = self.channel_var.get()
        angle = int(self.angle_var.get())
        angle = max(0, min(180, angle))

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
