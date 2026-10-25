#!/usr/bin/env python3
"""
========================================================================================
ExoArm Real-Time Force Vector & Kinematics Visualizer (STM32 + MPU6050 + Strain Gauge)
========================================================================================

Visualizes:
  1. Real-time exoskeleton arm kinematics (Upper arm & Forearm rotated at angle theta).
  2. Load Cell / Strain Gauge interaction force vector (magnitude in kg & Newtons).
  3. MPU-6050 inertial acceleration / gravity vector (Ax, Ay, Az).
  4. Resultant net dynamic force vector acting on the arm cuff.
  5. Live telemetry strip chart (Angle, Mass in kg, Interaction Force in N, and Motor PWM).

Usage:
  python visualize_force_vector.py --port COM3      # Connects to STM32 Bluepill on COM3
  python visualize_force_vector.py --demo           # Interactive simulation without hardware
========================================================================================
"""

import sys
import time
import math
import argparse
from collections import deque

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import matplotlib.patches as patches
except ImportError:
    print("[ERROR] Required libraries not installed. Please run: pip install matplotlib numpy pyserial")
    sys.exit(1)

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ======================================================================================
# DATA PARSER & TELEMETRY CONTAINER
# ======================================================================================
class ExoArmDataReceiver:
    def __init__(self, port=None, baudrate=115200, demo_mode=False):
        self.demo_mode = demo_mode
        self.serial_port = None
        self.port_name = port
        self.baudrate = baudrate

        # Telemetry State
        self.angle_deg = 90.0
        self.weight_kg = 0.0
        self.force_n = 0.0
        self.accel_x = 0.0
        self.accel_y = 1.0
        self.accel_z = 0.0
        self.pwm = 0
        self.mode = 1
        self.status = "OK"

        # History Buffers for Strip Charts (last 100 samples)
        self.history_len = 120
        self.time_hist = deque(maxlen=self.history_len)
        self.angle_hist = deque(maxlen=self.history_len)
        self.weight_hist = deque(maxlen=self.history_len)
        self.force_hist = deque(maxlen=self.history_len)
        self.pwm_hist = deque(maxlen=self.history_len)
        self.start_time = time.time()

        if not self.demo_mode:
            self._connect_serial()

    def _connect_serial(self):
        if not SERIAL_AVAILABLE:
            print("[WARN] pySerial not available. Falling back to Demo Simulation Mode.")
            self.demo_mode = True
            return

        if not self.port_name:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                print("[WARN] No COM ports found! Starting in Demo Simulation Mode.")
                self.demo_mode = True
                return
            self.port_name = ports[0].device
            print(f"[INFO] Auto-selected COM port: {self.port_name}")

        try:
            self.serial_port = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            print(f"[CONNECTED] Serial connection established on {self.port_name} at {self.baudrate} baud.")
        except Exception as e:
            print(f"[ERROR] Failed to open {self.port_name}: {e}")
            print("[INFO] Switching to Demo Simulation Mode.")
            self.demo_mode = True

    def update(self):
        current_t = time.time() - self.start_time

        if self.demo_mode:
            # Simulated sinusoidal arm curling motion & load variations
            self.angle_deg = 85.0 + 45.0 * math.sin(current_t * 1.2)
            # Simulated load: user applies upward force as arm flexes
            self.weight_kg = 1.25 * math.sin(current_t * 1.8) + 0.15 * math.sin(current_t * 6.0)
            self.force_n = self.weight_kg * 9.80665
            # Gravitational acceleration components relative to arm angle
            phi_rad = math.radians(self.angle_deg - 90.0)
            self.accel_x = math.sin(phi_rad)
            self.accel_y = math.cos(phi_rad)
            self.accel_z = 0.05 * math.sin(current_t * 3.0)
            self.pwm = int(60.0 * math.cos(current_t * 1.2))
            self.mode = 1
            self.status = "SIMULATED_DEMO"
        else:
            if self.serial_port and self.serial_port.is_open:
                try:
                    while self.serial_port.in_waiting:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("$TELEM,"):
                            # Packet: $TELEM,ANGLE,WEIGHT_KG,FORCE_N,ACCEL_X,ACCEL_Y,ACCEL_Z,PWM,MODE,STATUS
                            parts = line.split(',')
                            if len(parts) >= 10:
                                self.angle_deg = float(parts[1])
                                self.weight_kg = float(parts[2])
                                self.force_n = float(parts[3])
                                self.accel_x = float(parts[4])
                                self.accel_y = float(parts[5])
                                self.accel_z = float(parts[6])
                                self.pwm = int(parts[7])
                                self.mode = int(parts[8])
                                self.status = parts[9]
                except Exception as err:
                    print(f"[SERIAL ERR] {err}")

        # Append to strip chart buffers
        self.time_hist.append(current_t)
        self.angle_hist.append(self.angle_deg)
        self.weight_hist.append(self.weight_kg)
        self.force_hist.append(self.force_n)
        self.pwm_hist.append(self.pwm)


# ======================================================================================
# VISUALIZATION GUI (MATPLOTLIB)
# ======================================================================================
class ExoArmVisualizer:
    def __init__(self, data_receiver):
        self.data = data_receiver
        self.forearm_len = 0.22  # meters

        # Configure Matplotlib dark theme
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 8), dpi=100)
        self.fig.canvas.manager.set_window_title("ExoArm Dynamic Force Vector Visualizer (STM32 Edition)")

        # Create Grid: Left = 2D Vector & Kinematics Field, Right = Strip Charts
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.2, 1.0], hspace=0.35, wspace=0.25)

        self.ax_kinematics = self.fig.add_subplot(gs[:, 0])
        self.ax_angle = self.fig.add_subplot(gs[0, 1])
        self.ax_force = self.fig.add_subplot(gs[1, 1])
        self.ax_pwm   = self.fig.add_subplot(gs[2, 1])

        self._setup_kinematics_plot()
        self._setup_strip_charts()

    def _setup_kinematics_plot(self):
        ax = self.ax_kinematics
        ax.set_title("Exoskeleton Kinematics & Force Vector Field", fontsize=13, fontweight='bold', color='#4fc3f7', pad=12)
        ax.set_xlim(-0.15, 0.35)
        ax.set_ylim(-0.30, 0.30)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.3, color='#555555')

        # Static reference: Upper Arm (Shoulder to Elbow)
        self.upper_arm_line, = ax.plot([0, 0], [0.22, 0], color='#78909c', linewidth=8, solid_capstyle='round', label="Upper Arm (Fixed)")
        # Shoulder Joint Marker
        ax.plot(0, 0.22, marker='o', markersize=12, color='#90a4ae')

        # Elbow Pivot Joint Marker
        self.elbow_marker, = ax.plot([0], [0], marker='o', markersize=14, color='#ffb74d', zorder=5, label="Elbow Joint Pivot")

        # Dynamic Forearm Link
        self.forearm_line, = ax.plot([], [], color='#00e676', linewidth=9, solid_capstyle='round', label="Exoskeleton Forearm", zorder=4)

        # Load Cell Cuff Marker
        self.cuff_marker, = ax.plot([], [], marker='s', markersize=12, color='#ff5252', label="20kg Load Cell Cuff", zorder=6)

        # Dynamic Force Vector Arrow (Red)
        self.quiver_force = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='#ff1744',
                                      width=0.015, zorder=8, label="Interaction Force Vector (F_user)")

        # Dynamic Acceleration / Gravity Vector Arrow (Cyan)
        self.quiver_accel = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='#00e5ff',
                                      width=0.012, zorder=7, label="MPU6050 Gravity/Accel Vector (g)")

        # Resultant Net Dynamic Vector (Yellow-Green)
        self.quiver_net = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='#eeff41',
                                    width=0.010, alpha=0.85, zorder=7, label="Net Resultant Force Vector")

        # Live HUD text annotations
        self.txt_hud = ax.text(-0.13, 0.24, "", fontsize=10, family='monospace',
                               bbox=dict(boxstyle='round,pad=0.6', facecolor='#1e1e1e', edgecolor='#424242', alpha=0.9))

        ax.legend(loc='lower left', fontsize=8, facecolor='#212121', edgecolor='#424242')

    def _setup_strip_charts(self):
        # 1. Elbow Angle Chart
        ax1 = self.ax_angle
        ax1.set_title("Elbow Joint Angle (θ)", fontsize=10, fontweight='bold', color='#ffb74d')
        ax1.set_ylabel("Degrees (°)", fontsize=8)
        ax1.set_ylim(20, 160)
        ax1.grid(True, linestyle=':', alpha=0.3)
        self.line_angle, = ax1.plot([], [], color='#ffb74d', linewidth=2)
        ax1.axhspan(32, 153, color='#00e676', alpha=0.08, label="Safe Range")

        # 2. Force & Weight Chart
        ax2 = self.ax_force
        ax2.set_title("Load Cell Weight (kg) & Interaction Force (N)", fontsize=10, fontweight='bold', color='#ff5252')
        ax2.set_ylabel("Force (N) / Mass (kg)", fontsize=8)
        ax2.set_ylim(-35, 35)
        ax2.grid(True, linestyle=':', alpha=0.3)
        self.line_force, = ax2.plot([], [], color='#ff1744', linewidth=2, label="Force (N)")
        self.line_weight, = ax2.plot([], [], color='#40c4ff', linewidth=1.5, linestyle='--', label="Mass (kg)")
        ax2.legend(loc='upper right', fontsize=7, facecolor='#212121')

        # 3. Motor PWM Chart
        ax3 = self.ax_pwm
        ax3.set_title("BTS7960 Motor Command (PWM)", fontsize=10, fontweight='bold', color='#00e676')
        ax3.set_ylabel("PWM Duty (-255 to 255)", fontsize=8)
        ax3.set_xlabel("Time (s)", fontsize=8)
        ax3.set_ylim(-260, 260)
        ax3.grid(True, linestyle=':', alpha=0.3)
        self.line_pwm, = ax3.plot([], [], color='#00e676', linewidth=1.8)

    def _animate(self, frame):
        self.data.update()

        # 1. Kinematics Geometry:
        # 0 deg = hanging vertically downward (-90 deg from horizontal)
        # 90 deg = horizontal (0 deg from horizontal)
        # 150 deg = flexed (+60 deg from horizontal)
        theta_rad = math.radians(self.data.angle_deg - 90.0)
        cuff_x = self.forearm_len * math.cos(theta_rad)
        cuff_y = self.forearm_len * math.sin(theta_rad)

        # Update Forearm Link and Cuff Marker
        self.forearm_line.set_data([0, cuff_x], [0, cuff_y])
        self.cuff_marker.set_data([cuff_x], [cuff_y])

        # 2. Interaction Force Vector (from Strain Gauge):
        # Normal to the forearm (tangential curl direction):
        tangent_x = -math.sin(theta_rad)
        tangent_y =  math.cos(theta_rad)

        # Scale force for clear visual display (e.g. 20 N = 0.12 meters arrow)
        force_scale = 0.0065
        f_vec_x = self.data.force_n * tangent_x * force_scale
        f_vec_y = self.data.force_n * tangent_y * force_scale

        # 3. Acceleration / Gravity Vector (from MPU6050):
        # MPU6050 measured vector [Ax, Ay, Az] in sensor frame:
        # Sensor X is along forearm, Sensor Y is perpendicular
        accel_world_x = (self.data.accel_x * math.cos(theta_rad) - self.data.accel_y * math.sin(theta_rad))
        accel_world_y = (self.data.accel_x * math.sin(theta_rad) + self.data.accel_y * math.cos(theta_rad))

        accel_scale = 0.08  # 1.0g = 0.08 meters arrow
        a_vec_x = -accel_world_x * accel_scale
        a_vec_y = -accel_world_y * accel_scale

        # 4. Resultant Vector:
        net_x = f_vec_x + a_vec_x
        net_y = f_vec_y + a_vec_y

        # Update Vectors in Kinematics Plot
        self.quiver_force.set_offsets([cuff_x, cuff_y])
        self.quiver_force.set_UVC(f_vec_x, f_vec_y)

        # Position IMU midway on forearm
        imu_x = cuff_x * 0.55
        imu_y = cuff_y * 0.55
        self.quiver_accel.set_offsets([imu_x, imu_y])
        self.quiver_accel.set_UVC(a_vec_x, a_vec_y)

        self.quiver_net.set_offsets([cuff_x, cuff_y])
        self.quiver_net.set_UVC(net_x, net_y)

        # Update HUD Text
        hud_str = (
            f"Elbow Angle : {self.data.angle_deg:6.1f}°\n"
            f"Weight (kg) : {self.data.weight_kg:6.3f} kg\n"
            f"Force (N)   : {self.data.force_n:6.2f} N\n"
            f"Accel Vector: [{self.data.accel_x:+.2f}, {self.data.accel_y:+.2f}, {self.data.accel_z:+.2f}] g\n"
            f"Motor PWM   : {self.data.pwm:+4d}\n"
            f"System State: {self.data.status}"
        )
        self.txt_hud.set_text(hud_str)

        # 5. Update Strip Charts:
        if len(self.data.time_hist) > 1:
            t = list(self.data.time_hist)
            t_min, t_max = t[0], t[-1]
            if t_max - t_min < 5:
                t_max = t_min + 5

            self.ax_angle.set_xlim(t_min, t_max)
            self.ax_force.set_xlim(t_min, t_max)
            self.ax_pwm.set_xlim(t_min, t_max)

            self.line_angle.set_data(t, list(self.data.angle_hist))
            self.line_force.set_data(t, list(self.data.force_hist))
            self.line_weight.set_data(t, list(self.data.weight_hist))
            self.line_pwm.set_data(t, list(self.data.pwm_hist))

        return self.forearm_line, self.cuff_marker, self.quiver_force, self.quiver_accel, self.quiver_net

    def run(self):
        ani = FuncAnimation(self.fig, self._animate, interval=40, blit=False)
        plt.show()


# ======================================================================================
# MAIN ENTRY POINT
# ======================================================================================
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ExoArm Real-Time Force Vector & Kinematics Visualizer")
    parser.add_argument('--port', type=str, default=None, help="Serial COM Port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument('--baud', type=int, default=115200, help="Serial Baudrate (default: 115200)")
    parser.add_argument('--demo', action='store_true', help="Run in simulated demo mode without hardware")
    args = parser.parse_args()

    print("\n======================================================================")
    print(" ExoArm Force Vector & MPU6050 Acceleration Visualizer")
    print("======================================================================")
    if args.demo:
        print("[MODE] Running in Interactive Demo Simulation Mode.")
    else:
        print("[MODE] Attempting Hardware Serial Connection (use --demo for simulation).")

    receiver = ExoArmDataReceiver(port=args.port, baudrate=args.baud, demo_mode=args.demo)
    app = ExoArmVisualizer(receiver)
    app.run()
