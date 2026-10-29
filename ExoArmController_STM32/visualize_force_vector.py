#!/usr/bin/env python3
"""
========================================================================================
ExoArm Real-Time Arm Tracking, Force Vector & Kinematics Visualizer
(STM32F411 + MPU-6050 upper arm + AS5600 / 1000 PPR optical elbow encoders + load cell)
========================================================================================

Visualizes:
  1. Real-time arm kinematics in the sagittal plane: upper arm (from the upper-arm MPU-6050),
     forearm (upper arm + elbow angle from the AS5600 / optical-encoder fusion) and the wrist/cuff placement.
  2. Load cell interaction force vector (magnitude in kg & Newtons).
  3. MPU-6050 gravity / acceleration vector (Ax, Ay, Az) in the upper-arm frame.
  4. Resultant net force vector acting on the cuff.
  5. Live strip charts: joint angles, force, motor PWM, elbow speed and AS5600-vs-optical mismatch.

Telemetry packet (firmware 20 Hz, 115200 baud):
  $TELEM,FOREARM_DEG,WEIGHT_KG,FORCE_N,AX,AY,AZ,PWM,MODE,STATUS,UA_DEG,ELBOW_DEG,ELBOW_DPS,MISMATCH_DEG,WRIST_X_CM,WRIST_Y_CM
  (an older 10-field packet is still accepted; the extra fields then default to 0.)

Angle conventions:  upper arm / forearm = inclination from gravity (0 = hanging down, 90 = horizontal
forward, 180 = pointing up).  Elbow = joint angle (0 = fully extended, + = flexion).
Forearm inclination = upper-arm inclination + elbow angle.

Usage:
  python visualize_force_vector.py --port COM3      # Connects to the STM32 Black Pill on COM3
  python visualize_force_vector.py --demo           # Interactive simulation without hardware
  (--upper-len / --fore-len / --elbow-min / --elbow-max must match include/Config.h)
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
except ImportError:
    print("[ERROR] Required libraries not installed. Please run: pip install matplotlib numpy pyserial")
    sys.exit(1)

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def limb_dir(inclination_deg):
    """Unit vector along a limb (x forward, y up) for an inclination measured from hanging-down."""
    a = math.radians(inclination_deg)
    return math.sin(a), -math.cos(a)


def limb_perp(inclination_deg):
    """Unit vector perpendicular to a limb, pointing in the flexion (curl-up) direction."""
    a = math.radians(inclination_deg)
    return math.cos(a), math.sin(a)


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
        self.angle_deg = 90.0          # forearm inclination from gravity
        self.weight_kg = 0.0
        self.force_n = 0.0
        self.accel_x = -1.0            # upper-arm IMU
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.pwm = 0
        self.mode = 1
        self.status = "OK"
        self.ua_deg = 0.0              # upper-arm inclination (MPU-6050)
        self.elbow_deg = 90.0          # fused elbow joint angle (AS5600 + optical encoder)
        self.elbow_dps = 0.0
        self.mismatch_deg = 0.0        # AS5600 - optical disagreement
        self.wrist_x_cm = 0.0          # firmware forearm placement (relative to the shoulder)
        self.wrist_y_cm = 0.0

        # History Buffers for Strip Charts
        self.history_len = 120
        self.time_hist = deque(maxlen=self.history_len)
        self.angle_hist = deque(maxlen=self.history_len)       # forearm inclination
        self.ua_hist = deque(maxlen=self.history_len)
        self.elbow_hist = deque(maxlen=self.history_len)
        self.weight_hist = deque(maxlen=self.history_len)
        self.force_hist = deque(maxlen=self.history_len)
        self.pwm_hist = deque(maxlen=self.history_len)
        self.speed_hist = deque(maxlen=self.history_len)
        self.mismatch_hist = deque(maxlen=self.history_len)
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

    @staticmethod
    def parse_packet(line):
        """Parse one $TELEM line into a dict, or return None if it is not a valid packet."""
        if not line.startswith("$TELEM,"):
            return None
        parts = line.split(',')
        if len(parts) < 10:
            return None
        try:
            d = dict(angle_deg=float(parts[1]), weight_kg=float(parts[2]), force_n=float(parts[3]),
                     accel_x=float(parts[4]), accel_y=float(parts[5]), accel_z=float(parts[6]),
                     pwm=int(parts[7]), mode=int(parts[8]), status=parts[9])
            if len(parts) >= 16:
                d.update(ua_deg=float(parts[10]), elbow_deg=float(parts[11]), elbow_dps=float(parts[12]),
                         mismatch_deg=float(parts[13]), wrist_x_cm=float(parts[14]), wrist_y_cm=float(parts[15]))
            else:                       # legacy packet: no upper-arm / elbow information
                d.update(ua_deg=0.0, elbow_deg=d['angle_deg'], elbow_dps=0.0, mismatch_deg=0.0,
                         wrist_x_cm=0.0, wrist_y_cm=0.0)
            return d
        except ValueError:
            return None

    def update(self):
        current_t = time.time() - self.start_time

        if self.demo_mode:
            # Simulated curl: elbow flexes 15..135 deg while the shoulder sways a little
            self.ua_deg = 20.0 + 12.0 * math.sin(current_t * 0.5)
            self.elbow_deg = 75.0 + 60.0 * math.sin(current_t * 1.2)
            self.elbow_dps = 60.0 * 1.2 * math.cos(current_t * 1.2)      # d(elbow)/dt in deg/s
            self.angle_deg = self.ua_deg + self.elbow_deg
            self.mismatch_deg = 0.4 * math.sin(current_t * 5.0)
            self.weight_kg = 1.25 * math.sin(current_t * 1.8) + 0.15 * math.sin(current_t * 6.0)
            self.force_n = self.weight_kg * 9.80665
            # Upper-arm IMU: specific force in the sensor frame (X along the upper arm)
            a = math.radians(self.ua_deg)
            self.accel_x = -math.cos(a)
            self.accel_y = math.sin(a)
            self.accel_z = 0.05 * math.sin(current_t * 3.0)
            self.pwm = int(250.0 * math.cos(current_t * 1.2))
            self.mode = 1
            self.status = "SIMULATED_DEMO"
            ux, uy = limb_dir(self.ua_deg)
            fx, fy = limb_dir(self.angle_deg)
            self.wrist_x_cm = 100.0 * (self.upper_len * ux + self.fore_len * fx)
            self.wrist_y_cm = 100.0 * (self.upper_len * uy + self.fore_len * fy)
        else:
            if self.serial_port and self.serial_port.is_open:
                while self.serial_port.in_waiting:
                    try:
                        line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        d = self.parse_packet(line)
                        if d is None:
                            continue
                        for k, v in d.items():
                            setattr(self, k, v)
                    except UnicodeDecodeError:
                        continue            # one corrupt line must not discard the rest of the buffer
                    except Exception as err:
                        print(f"[SERIAL ERR] {err}")
                        break

        # Append to strip chart buffers
        self.time_hist.append(current_t)
        self.angle_hist.append(self.angle_deg)
        self.ua_hist.append(self.ua_deg)
        self.elbow_hist.append(self.elbow_deg)
        self.weight_hist.append(self.weight_kg)
        self.force_hist.append(self.force_n)
        self.pwm_hist.append(self.pwm)
        self.speed_hist.append(self.elbow_dps)
        self.mismatch_hist.append(self.mismatch_deg)

    # Link lengths used by demo mode (real mode draws from the received angles)
    upper_len = 0.30
    fore_len = 0.27


# ======================================================================================
# VISUALIZATION GUI (MATPLOTLIB)
# ======================================================================================
class ExoArmVisualizer:
    def __init__(self, data_receiver, pwm_max=1000, upper_len=0.30, fore_len=0.27,
                 elbow_min=10.0, elbow_max=140.0, mismatch_limit=8.0):
        self.data = data_receiver
        self.pwm_max = pwm_max
        self.upper_len = upper_len
        self.fore_len = fore_len
        self.elbow_min = elbow_min
        self.elbow_max = elbow_max
        self.mismatch_limit = mismatch_limit
        self.data.upper_len = upper_len
        self.data.fore_len = fore_len

        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 9), dpi=100)
        self.fig.canvas.manager.set_window_title("ExoArm Arm Tracking & Force Vector Visualizer (STM32F411)")

        # Grid: Left = 2D kinematics field, Right = 4 strip charts
        gs = self.fig.add_gridspec(4, 2, width_ratios=[1.2, 1.0], hspace=0.55, wspace=0.25)
        self.ax_kinematics = self.fig.add_subplot(gs[:, 0])
        self.ax_angle = self.fig.add_subplot(gs[0, 1])
        self.ax_force = self.fig.add_subplot(gs[1, 1])
        self.ax_pwm = self.fig.add_subplot(gs[2, 1])
        self.ax_speed = self.fig.add_subplot(gs[3, 1])

        self._setup_kinematics_plot()
        self._setup_strip_charts()

    def _setup_kinematics_plot(self):
        ax = self.ax_kinematics
        reach = self.upper_len + self.fore_len + 0.08
        ax.set_title("Arm Tracking, Forearm Placement & Force Vectors (sagittal plane)",
                     fontsize=12, fontweight='bold', color='#4fc3f7', pad=12)
        ax.set_xlim(-0.45 * reach, 1.05 * reach)
        ax.set_ylim(-1.05 * reach, 0.80 * reach)
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.3, color='#555555')
        ax.set_xlabel("forward (m)", fontsize=8)
        ax.set_ylabel("up (m)", fontsize=8)

        # Shoulder marker at the origin
        ax.plot(0, 0, marker='o', markersize=12, color='#90a4ae', zorder=5)

        self.upper_arm_line, = ax.plot([], [], color='#78909c', linewidth=9, solid_capstyle='round',
                                       label="Upper arm (MPU-6050)", zorder=3)
        self.elbow_marker, = ax.plot([], [], marker='o', markersize=14, color='#ffb74d', zorder=6,
                                     label="Elbow (AS5600 + optical encoder)")
        self.forearm_line, = ax.plot([], [], color='#00e676', linewidth=9, solid_capstyle='round',
                                     label="Forearm", zorder=4)
        self.cuff_marker, = ax.plot([], [], marker='s', markersize=12, color='#ff5252',
                                    label="Cuff / wrist (load cell)", zorder=7)
        self.trail_line, = ax.plot([], [], color='#00e676', linewidth=1, alpha=0.35, label="Wrist path")
        self.trail_x, self.trail_y = deque(maxlen=150), deque(maxlen=150)

        self.quiver_force = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='#ff1744',
                                      width=0.012, zorder=8, label="Interaction force (F_user)")
        self.quiver_accel = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='#00e5ff',
                                      width=0.010, zorder=7, label="MPU-6050 gravity/accel (g)")
        self.quiver_net = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='#eeff41',
                                    width=0.008, alpha=0.85, zorder=7, label="Net resultant vector")

        self.txt_hud = ax.text(0.02, 0.98, "", transform=ax.transAxes, fontsize=9, family='monospace',
                               va='top', bbox=dict(boxstyle='round,pad=0.6', facecolor='#1e1e1e',
                                                   edgecolor='#424242', alpha=0.9))
        ax.legend(loc='lower right', fontsize=7, facecolor='#212121', edgecolor='#424242')

    def _setup_strip_charts(self):
        # 1. Joint angles
        ax1 = self.ax_angle
        ax1.set_title("Joint angles: elbow (AS5600+optical), upper arm (IMU), forearm", fontsize=9,
                      fontweight='bold', color='#ffb74d')
        ax1.set_ylabel("Degrees", fontsize=8)
        ax1.set_ylim(-20, 230)
        ax1.grid(True, linestyle=':', alpha=0.3)
        ax1.axhspan(self.elbow_min, self.elbow_max, color='#00e676', alpha=0.08, label="Elbow safe range")
        self.line_elbow, = ax1.plot([], [], color='#ffb74d', linewidth=2, label="Elbow")
        self.line_ua, = ax1.plot([], [], color='#90a4ae', linewidth=1.3, linestyle='--', label="Upper arm")
        self.line_angle, = ax1.plot([], [], color='#00e676', linewidth=1.5, label="Forearm")
        ax1.legend(loc='upper right', fontsize=6, facecolor='#212121', ncol=4)

        # 2. Force & Weight
        ax2 = self.ax_force
        ax2.set_title("Load Cell Weight (kg) & Interaction Force (N)", fontsize=9, fontweight='bold', color='#ff5252')
        ax2.set_ylabel("Force (N) / Mass (kg)", fontsize=8)
        ax2.set_ylim(-35, 35)
        ax2.grid(True, linestyle=':', alpha=0.3)
        self.line_force, = ax2.plot([], [], color='#ff1744', linewidth=2, label="Force (N)")
        self.line_weight, = ax2.plot([], [], color='#40c4ff', linewidth=1.5, linestyle='--', label="Mass (kg)")
        ax2.legend(loc='upper right', fontsize=7, facecolor='#212121')

        # 3. Motor PWM
        ax3 = self.ax_pwm
        ax3.set_title("BTS7960 Motor Command (PWM)", fontsize=9, fontweight='bold', color='#00e676')
        ax3.set_ylabel(f"PWM (permille, +/-{self.pwm_max})", fontsize=8)
        ax3.set_ylim(-1.05 * self.pwm_max, 1.05 * self.pwm_max)
        ax3.grid(True, linestyle=':', alpha=0.3)
        self.line_pwm, = ax3.plot([], [], color='#00e676', linewidth=1.8)

        # 4. Elbow speed (optical encoder) and encoder cross-check
        ax4 = self.ax_speed
        ax4.set_title("Elbow speed (optical) & AS5600-vs-optical mismatch", fontsize=9, fontweight='bold', color='#b388ff')
        ax4.set_ylabel("deg/s", fontsize=8)
        ax4.set_xlabel("Time (s)", fontsize=8)
        ax4.set_ylim(-450, 450)
        ax4.grid(True, linestyle=':', alpha=0.3)
        self.line_speed, = ax4.plot([], [], color='#b388ff', linewidth=1.5, label="Speed (deg/s)")
        self.ax_mis = ax4.twinx()
        self.ax_mis.set_ylabel("mismatch (deg)", fontsize=8, color='#ff9100')
        self.ax_mis.set_ylim(-2 * self.mismatch_limit, 2 * self.mismatch_limit)
        for lim in (self.mismatch_limit, -self.mismatch_limit):
            self.ax_mis.axhline(lim, color='#ff9100', linestyle=':', alpha=0.6)
        self.line_mis, = self.ax_mis.plot([], [], color='#ff9100', linewidth=1.3, label="Mismatch (deg)")

    def _animate(self, frame):
        self.data.update()
        d = self.data

        # ---- Kinematics: shoulder at origin, x forward, y up ----
        ux, uy = limb_dir(d.ua_deg)
        fx, fy = limb_dir(d.angle_deg)
        elbow_x, elbow_y = self.upper_len * ux, self.upper_len * uy
        cuff_x, cuff_y = elbow_x + self.fore_len * fx, elbow_y + self.fore_len * fy

        self.upper_arm_line.set_data([0, elbow_x], [0, elbow_y])
        self.elbow_marker.set_data([elbow_x], [elbow_y])
        self.forearm_line.set_data([elbow_x, cuff_x], [elbow_y, cuff_y])
        self.cuff_marker.set_data([cuff_x], [cuff_y])
        self.trail_x.append(cuff_x)
        self.trail_y.append(cuff_y)
        self.trail_line.set_data(list(self.trail_x), list(self.trail_y))

        # ---- Interaction force: perpendicular to the forearm, positive = pushing UP (flexion) ----
        px, py = limb_perp(d.angle_deg)
        force_scale = 0.0065            # 20 N ~ 0.13 m arrow
        f_vec_x, f_vec_y = d.force_n * px * force_scale, d.force_n * py * force_scale

        # ---- Gravity / acceleration vector from the UPPER-ARM IMU (sensor X along the upper arm) ----
        qx, qy = limb_perp(d.ua_deg)
        accel_world_x = d.accel_x * ux + d.accel_y * qx
        accel_world_y = d.accel_x * uy + d.accel_y * qy
        accel_scale = 0.08              # 1.0 g ~ 0.08 m arrow
        a_vec_x, a_vec_y = -accel_world_x * accel_scale, -accel_world_y * accel_scale

        net_x, net_y = f_vec_x + a_vec_x, f_vec_y + a_vec_y

        self.quiver_force.set_offsets([cuff_x, cuff_y])
        self.quiver_force.set_UVC(f_vec_x, f_vec_y)
        self.quiver_accel.set_offsets([0.5 * elbow_x, 0.5 * elbow_y])      # IMU sits mid upper arm
        self.quiver_accel.set_UVC(a_vec_x, a_vec_y)
        self.quiver_net.set_offsets([cuff_x, cuff_y])
        self.quiver_net.set_UVC(net_x, net_y)

        hud_str = (
            f"Upper arm   : {d.ua_deg:7.1f}°  (IMU)\n"
            f"Elbow       : {d.elbow_deg:7.1f}°  (AS5600+optical)\n"
            f"Forearm     : {d.angle_deg:7.1f}°  (from gravity)\n"
            f"Elbow speed : {d.elbow_dps:7.1f} °/s\n"
            f"Enc mismatch: {d.mismatch_deg:7.1f}°\n"
            f"Wrist (fw)  : x{d.wrist_x_cm:6.1f} y{d.wrist_y_cm:6.1f} cm\n"
            f"Weight      : {d.weight_kg:7.3f} kg\n"
            f"Force       : {d.force_n:7.2f} N\n"
            f"Accel [g]   : {d.accel_x:+.2f} {d.accel_y:+.2f} {d.accel_z:+.2f}\n"
            f"Motor PWM   : {d.pwm:+5d}\n"
            f"State       : {d.status}"
        )
        self.txt_hud.set_text(hud_str)

        # ---- Strip charts ----
        if len(d.time_hist) > 1:
            t = list(d.time_hist)
            t_min, t_max = t[0], t[-1]
            if t_max - t_min < 5:
                t_max = t_min + 5
            for axx in (self.ax_angle, self.ax_force, self.ax_pwm, self.ax_speed):
                axx.set_xlim(t_min, t_max)
            self.line_elbow.set_data(t, list(d.elbow_hist))
            self.line_ua.set_data(t, list(d.ua_hist))
            self.line_angle.set_data(t, list(d.angle_hist))
            self.line_force.set_data(t, list(d.force_hist))
            self.line_weight.set_data(t, list(d.weight_hist))
            self.line_pwm.set_data(t, list(d.pwm_hist))
            self.line_speed.set_data(t, list(d.speed_hist))
            self.line_mis.set_data(t, list(d.mismatch_hist))

        return (self.upper_arm_line, self.forearm_line, self.cuff_marker, self.quiver_force,
                self.quiver_accel, self.quiver_net)

    def run(self):
        self.ani = FuncAnimation(self.fig, self._animate, interval=40, blit=False, cache_frame_data=False)
        plt.show()


# ======================================================================================
# MAIN ENTRY POINT
# ======================================================================================
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ExoArm Real-Time Arm Tracking & Force Vector Visualizer")
    parser.add_argument('--port', type=str, default=None, help="Serial COM Port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument('--baud', type=int, default=115200, help="Serial Baudrate (default: 115200)")
    parser.add_argument('--pwm-max', type=int, default=1000, help="Full-scale PWM in telemetry (permille, default 1000)")
    parser.add_argument('--upper-len', type=float, default=0.30, help="Upper-arm length in m (Config.h UPPER_ARM_LENGTH_M)")
    parser.add_argument('--fore-len', type=float, default=0.27, help="Forearm length in m (Config.h FOREARM_LENGTH_M)")
    parser.add_argument('--elbow-min', type=float, default=10.0, help="Elbow soft limit min, deg (Config.h ELBOW_MIN_DEG)")
    parser.add_argument('--elbow-max', type=float, default=140.0, help="Elbow soft limit max, deg (Config.h ELBOW_MAX_DEG)")
    parser.add_argument('--demo', action='store_true', help="Run in simulated demo mode without hardware")
    args = parser.parse_args()

    print("\n======================================================================")
    print(" ExoArm Arm Tracking, Force Vector & Kinematics Visualizer")
    print("======================================================================")
    if args.demo:
        print("[MODE] Running in Interactive Demo Simulation Mode.")
    else:
        print("[MODE] Attempting Hardware Serial Connection (use --demo for simulation).")

    receiver = ExoArmDataReceiver(port=args.port, baudrate=args.baud, demo_mode=args.demo)
    app = ExoArmVisualizer(receiver, pwm_max=args.pwm_max, upper_len=args.upper_len, fore_len=args.fore_len,
                           elbow_min=args.elbow_min, elbow_max=args.elbow_max)
    app.run()
