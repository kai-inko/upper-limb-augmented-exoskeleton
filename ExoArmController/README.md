# Advanced Assistive Exoskeleton Arm Controller

Firmware and control suite adapted for:
* **Processor:** Arduino Nano (ATmega328P)
* **Joint Angle IMU:** GY-521 (MPU-6050 6-Axis Gyroscope & Accelerometer via I2C)
* **Interaction Force:** 20 kg Parallel Beam Load Cell + HX711 24-bit ADC
* **Motor Driver:** BTS7960 43A High-Power Dual H-Bridge Module (IBT-2)
* **Control Principle:** Inspired by Kristjan Berce's ExoArm ([Hackaday #20663](https://hackaday.io/project/20663-affordable-exoskeleton-arm-exoarm) / [#166841](https://hackaday.io/project/166841-3d-printed-affordable-exoskeleton-arm-v2))

---

## 1. Hardware Architecture & Wiring Diagram

```
                       +-----------------------------------+
                       |           Arduino Nano            |
                       |                                   |
[GY-521 IMU SDA] ----->| A4 (I2C SDA)                      |
[GY-521 IMU SCL] ----->| A5 (I2C SCL)                      |
                       |                                   |
[HX711 DOUT] --------->| D2 (Data Output, INT0)            |
[HX711 SCK]  <---------| D3 (Serial Clock)                 |
                       |                                   |
[BTS7960 R_EN + L_EN] <| D4 (Driver Gate Enable)           |
[BTS7960 RPWM] <-------| D5 (Flexion / UP Hardware PWM)    |
[BTS7960 LPWM] <-------| D6 (Extension / DOWN Hardware PWM)|
                       |                                   |
[Button Flex (UP)] --->| D7 (Internal Pullup to GND)       |
[Button Ext (DOWN)] -->| D8 (Internal Pullup to GND)       |
[Emergency Stop] ----->| D9 (Normally Closed to GND)       |
[Diagnostic LED] <-----| D13 (Built-in Nano LED)           |
                       +-----------------------------------+
```

### Pin Assignment Table (Arduino Nano)

| Arduino Nano Pin | Connected Device | Purpose | Notes |
| :--- | :--- | :--- | :--- |
| **`A4`** | GY-521 `SDA` | I2C Data line for joint inclination | Hardware I2C with internal pullups |
| **`A5`** | GY-521 `SCL` | I2C Clock line | 400 kHz Fast I2C communication |
| **`D2`** | HX711 `DOUT` | Load cell 24-bit serial output | Low-latency polling / Interrupt pin |
| **`D3`** | HX711 `SCK` | HX711 clock trigger | Generates 25 pulses per conversion |
| **`D4`** | BTS7960 `R_EN` & `L_EN` | Bridge Enable lines | Bridge both pins to D4 to enable FETs |
| **`D5`** | BTS7960 `RPWM` | Forward PWM (Flexion / Move UP) | Timer 0 Hardware PWM ($0-250$) |
| **`D6`** | BTS7960 `LPWM` | Reverse PWM (Extension / Move DOWN) | Timer 0 Hardware PWM ($0-250$) |
| **`D7`** | Pushbutton UP | Manual Flexion command | Wire switch to GND (Active LOW) |
| **`D8`** | Pushbutton DOWN | Manual Extension command | Wire switch to GND (Active LOW) |
| **`D9`** | E-Stop Switch | Emergency power cutoff | Wire NC switch to GND (Active LOW) |
| **`5V` / `GND`** | All modules | Logic power rail | Common system ground |

---

## 2. Key Hardware Enhancements

### A. GY-521 (MPU-6050) vs. Rotary Potentiometer
In the original 2017 ExoArm, a rotary potentiometer was mounted at the elbow joint. This introduced:
- Backlash and gear play in the 3D-printed gears (`pot_gear1.stl`).
- Mechanical wear and wiper noise over repeated cycles.

**How the GY-521 solves this:**
Mounted directly onto the forearm structure, the GY-521 measures absolute orientation without mechanical contact. The included `IMU_GY521` library combines the accelerometer and gyroscope via a **Complementary Filter**:
$$\theta(t) = 0.96 \cdot (\theta(t-1) + \omega_{gyro} \cdot \Delta t) + 0.04 \cdot \theta_{accel}$$
* The **gyroscope** provides instantaneous, low-latency angular rate without being fooled by sudden linear movements.
* The **accelerometer** eliminates long-term gyroscope integration drift.

### B. BTS7960 High-Power Driver vs. VNH2SP30
The BTS7960 module (IBT-2) handles up to 43A peak current and uses two separate PWM lines (`RPWM` and `LPWM`):
* **Break-Before-Make Protection:** To prevent internal H-bridge shoot-through, the firmware automatically enforces a dead-time delay whenever motor direction inverts.
* **Slew-Rate Limiter:** Acceleration is capped at 10 PWM units per cycle to prevent stripping the window motor's nylon gears.

### C. 20 kg Load Cell & Structural Protection
With a 20 kg load cell and a $0.22\text{ m}$ forearm leverage distance:
* Measurable interaction torque ranges up to $\pm 43.1\text{ N}\cdot\text{m}$.
* The `SafetyWatchdog` actively monitors load cell readings: if force exceeds $92\%$ of rated capacity ($\approx 18.4\text{ kg} = 180\text{ N}$), the motor cuts power to protect the user's arm and the mechanical structure.

---

## 3. Supported Operational Modes

You can change modes live via the Serial Monitor at **115200 baud**:

1. **Mode 0: Admittance Force Assist (`m 0`)**  
   Target interaction force is $0\text{ N}$. When the user flexes or extends their arm, the load cell measures cuff pressure, and the PID controller drives the motor to eliminate the user's effort, making external loads feel weightless.
2. **Mode 1: Gravity-Compensated Force Assist (`m 1`, Default)**  
   Calculates the gravitational holding torque in real time using the IMU pitch angle:
   $$\tau_{gravity}(\theta) = K_{gravity} \cdot \sin(\theta_{elbow})$$
   Automatically counterbalances the arm's weight when holding horizontal positions.
3. **Mode 2: Angle Position Hold (`m 2`)**  
   Maintains a commanded angle (e.g. `p 90.0`) using closed-loop PID servo tracking based on the GY-521 IMU.
4. **Mode 3: Manual Pushbutton Ramping (`m 3`)**  
   Enables smooth speed ramp control using the tactile switches on pins D7 and D8.

---

## 4. Calibration & Quick-Start Guide

1. Open [`ExoArmController.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController/ExoArmController.ino) in the Arduino IDE.
2. Select **Board: "Arduino Nano"** and **Processor: "ATmega328P"** (or "Old Bootloader" depending on your Nano board).
3. Connect the hardware according to the pin table above.
4. Open the Serial Monitor at **115200 baud**:
   * **IMU Bias:** Keep the arm still during startup; the GY-521 will automatically sample and zero out gyro drift. To re-calibrate anytime, type `c`.
   * **Load Cell Tare:** Keep the arm cuff unloaded and send `t` to zero the 20kg load cell.
   * **Live Telemetry:** Open the **Arduino Serial Plotter** to observe real-time graphs of `Angle`, `Target`, `ForceN`, and `PWM`.
