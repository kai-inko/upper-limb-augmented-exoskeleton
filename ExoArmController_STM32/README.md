# Advanced Assistive Exoskeleton Arm Controller (STM32 Bluepill Edition)

A high-performance clone of the ExoArm assistive control suite ported to the **STM32 Bluepill (STM32F103C8T6)** 72MHz ARM Cortex-M3 processor.

---

## 1. System Features & Key Changes

1. **High-Performance 32-bit Architecture:** Runs on the STM32F103C8T6 with hardware Timer 2 PWM channels and 5V-tolerant peripheral I/O.
2. **Hardware EXTI Degree Range Interruption:** Uses the MPU-6050 `INT` pin connected directly to STM32 external interrupt pin **`PB14`** to trip an instantaneous hardware stop if the arm reaches an unsafe angle range ($< 32^\circ$ or $> 153^\circ$).
3. **Exact Mass Calibration in Kilograms (kg):** Two-point manual calibration formula directly measuring payload and user force in kilograms with 3-decimal gram precision ($0.001\text{ kg}$).
4. **Real-Time Force Vector Visualizer:** Includes a standalone Python visualizer ([`visualize_force_vector.py`](file:///C:/Users/kainko-win/Desktop/ExoArmController_STM32/visualize_force_vector.py)) that connects via Serial and displays:
   - Exoskeleton arm kinematics rotated at real-time elbow angle $\theta$.
   - The interaction force vector $\vec{F}$ at the cuff.
   - The 3D gravitational/inertial acceleration vector $\vec{a}$ from the MPU-6050.
   - The resultant net dynamic force vector.

---

## 2. STM32 Bluepill Complete Wiring Schematic

```
                      +------------------------------------------+
                      |         STM32 Bluepill (F103C8)          |
                      |                                          |
[GY-521 IMU SCL] ---->| PB6 (Hardware I2C1 SCL, 5V Tolerant)     |
[GY-521 IMU SDA] ---->| PB7 (Hardware I2C1 SDA, 5V Tolerant)     |
[GY-521 IMU INT] ---->| PB14 (EXTI System Interruption Pin)     |
                      |                                          |
[HX711 DOUT] -------->| PB12 (24-bit Load Cell Data, 5V Tol.)   |
[HX711 SCK]  <--------| PB13 (Load Cell Serial Clock, 5V Tol.)   |
                      |                                          |
[BTS7960 RPWM] <------| PA0 (Timer 2 Ch 1 Hardware PWM - Flexion)|
[BTS7960 LPWM] <------| PA1 (Timer 2 Ch 2 Hardware PWM - Ext.)   |
[BTS7960 R_EN + L_EN]<| PA2 (Driver Gate Enable)                 |
                      |                                          |
[Button Flex (UP)] -->| PA4 (Internal Pullup to GND)             |
[Button Ext (DOWN)] ->| PA5 (Internal Pullup to GND)             |
[Emergency Stop] ---->| PA6 (NC Switch to GND)                   |
[Diagnostic LED] <----| PC13 (Built-in Bluepill LED, Active LOW) |
                      +------------------------------------------+
```

---

## 3. Manual Calibration in Kilograms (kg)

The firmware is calibrated directly in **kilograms (kg)**:
$$\text{Weight (kg)} = \frac{\text{Raw ADC Counts} - \text{Zero Offset}}{\text{LOAD\_CELL\_CAL\_FAC\_KG}}$$

### Calibration Procedure:
1. Open the Serial Monitor at **115200 baud**.
2. With no weight applied to the forearm cuff, send `t` to tare the zero point.
3. Place a known reference weight on the cuff (e.g. `1.000` kg or `2.500` kg).
4. Send `c <weight_in_kg>` (e.g. `c 1.000` for a 1kg mass).
5. The sketch will compute your exact `LOAD_CELL_CAL_FAC_KG` value. Update this value in [`Config.h`](file:///C:/Users/kainko-win/Desktop/ExoArmController_STM32/Config.h):
   ```cpp
   #define LOAD_CELL_CAL_FAC_KG  21500.0f  // Counts per kg
   ```

---

## 4. Real-Time Python Force Vector Visualizer

A companion Python GUI ([`visualize_force_vector.py`](file:///C:/Users/kainko-win/Desktop/ExoArmController_STM32/visualize_force_vector.py)) renders the exoskeleton arm kinematics, the interaction force vector, and the MPU6050 acceleration vector in real time.

### How to Run:
```bash
# Connect to STM32 Bluepill on COM port (e.g. COM3)
python visualize_force_vector.py --port COM3

# Or run in interactive simulation demo mode without hardware:
python visualize_force_vector.py --demo
```

### Visual Elements in the GUI:
* **Green Arm Link:** Renders the actual exoskeleton forearm angle $\theta$ relative to the fixed upper arm.
* **Red Arrow:** Interaction force vector $\vec{F}_{user}$ measured by the strain gauge load cell at the cuff.
* **Cyan Arrow:** Gravitational/inertial acceleration vector $\vec{a}$ measured by the MPU-6050.
* **Yellow Arrow:** Net resultant dynamic force vector.
* **Live Strip Charts:** Historical plots of joint angle, payload mass (kg), force (N), and motor PWM duty cycle.

---

## 5. Arduino IDE Flashing Guide (STM32duino)

1. In the Arduino IDE, open **Tools > Board > STM32 boards > Generic STM32F1 series**.
2. Select:
   * **Board part number:** `BluePill F103C8` (or `F103CB` 128k)
   * **U(S)ART support:** `Enabled (generic 'Serial')`
   * **USB support:** `CDC (generic 'Serial' supersede U(S)ART)` (if using Bluepill USB port) or `ST-Link` / `FTDI Serial`
3. Connect your programmer (ST-Link V2 or USB-to-UART FTDI on PA9/PA10).
4. Open [`ExoArmController.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController_STM32/ExoArmController.ino) and click **Upload**.
