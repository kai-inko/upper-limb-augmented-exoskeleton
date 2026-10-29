# ExoArmController STM32F411 (Black Pill, STM32Cube HAL, PlatformIO)

Elbow-assist exoskeleton controller for lifting. A 100 Hz loop tracks the arm with **three position sensors**
(an MPU-6050 on the upper arm, an AS5600 magnetic encoder and a 1000 PPR optical encoder on the elbow),
reads a 20 kg HX711 load cell (how hard the user pushes the cuff) and drives a BTS7960 H-bridge so the motor
moves **with** the user. Everything is wrapped in latched safety faults.

> **Safety first.** The firmware E-stop is *not* a safety-rated device. Fit a real E-stop that cuts
> motor POWER, add 10 kΩ pull-downs on PA8, PA9 and PB12 (so the driver stays off while the MCU
> boots or resets), and always commission with the arm unloaded and mechanically clamped.

## Hardware

| Part | Role |
| :-- | :-- |
| **STM32F411CEU6** Black Pill (100 MHz Cortex-M4F, 25 MHz crystal) | controller (replaces the STM32F103C8T6 Blue Pill) |
| **MPU-6050** (GY-521) on the **upper arm** | upper-arm inclination from gravity (+ gyro, complementary filter) |
| **AS5600** magnetic encoder on the **elbow / forearm** axis | absolute elbow angle, 12 bit (0.088°), I2C |
| **1000 PPR optical encoder** | fast incremental elbow angle/speed, 4000 counts/rev (X4), hardware decoded by TIM3 |
| HX711 + 20 kg load cell | cuff interaction force |
| BTS7960 + motor | actuator |

### How the arm is tracked

```
 upper arm  --MPU-6050-->  UA   (inclination from gravity: 0 = hanging down, 90 = forward horizontal)
 elbow      --AS5600 + optical encoder, fused-->  phi  (0 = fully extended, + = flexion)
 forearm placement:   forearm inclination = UA + phi
                      wrist (x, y) = L1*[sin UA, -cos UA] + L2*[sin(UA+phi), -cos(UA+phi)]   (shoulder = origin)
```

The elbow estimate (`JointEstimator`, `include/ArmKinematics.h`) integrates the optical encoder every tick (fine
resolution, no latency) and is pulled toward the AS5600 (absolute, drift-free) with a small gain. The disagreement
between the two sensors is monitored: a slipped magnet, a loose encoder coupling or lost counts show up as a
persistent mismatch and fault the system.

## Pin map (single source of truth: `include/Config.h`)

| Function | Pin | Notes |
| :-- | :-- | :-- |
| E-Stop | PA0 | active LOW, internal pull-up (NC switch to GND is the safest wiring). The Black Pill's on-board KEY button is also on PA0 and acts as an E-stop press |
| MODE button | PA1 | active LOW, internal pull-up |
| BTS7960 RPWM (UP) | PA8 | TIM1_CH1, 20 kHz |
| BTS7960 LPWM (DOWN) | PA9 | TIM1_CH2, 20 kHz |
| BTS7960 R_EN + L_EN | PB12 | tie both together; LOW at boot |
| **MPU-6050** SCL / SDA | PB6 / PB7 | **I2C1**, 400 kHz, 4.7 kΩ pull-ups to 3V3 (upper arm) |
| **AS5600** SCL / SDA | PB10 / PB9 | **I2C2**, 400 kHz, 4.7 kΩ pull-ups to 3V3 (elbow). Address 0x36; tie DIR to GND (or use `AS5600_SIGN`) |
| **Optical encoder** A / B | PA6 / PA7 | **TIM3_CH1 / CH2**, hardware X4 quadrature, internal pull-ups. Z (index) not used |
| HX711 DOUT / SCK | PB14 / PB15 | |
| Telemetry TX | PA2 | USART2, 115200 8N1, to a 3V3 USB-TTL adapter RX |
| Status LED | PC13 | on-board, active LOW |
| ST-Link | PA13 / PA14 | SWDIO / SWCLK |

Notes
* Two separate I2C buses on purpose: the sensors sit on different limb segments, so a bad cable or a stuck bus on one
  cannot take the other down. Each has its own timeout fault.
* Encoder logic levels: PA6/PA7 are 5 V-tolerant on the F411, but for open-collector encoder outputs pull A and B up
  to **3V3** (4.7 kΩ), not 5 V. Share GND with the encoder supply. Power the AS5600 from 3V3.
* Where the optical encoder sits: the defaults assume it is on the **elbow axis** (`ENCODER_REVS_PER_JOINT_REV = 1.0`).
  If it is on a motor shaft behind an N:1 gearbox, set that constant to N. The AS5600 magnet must be on the elbow axis (1:1).
* Do not use USART1 for telemetry: PA9 is needed by TIM1_CH2.

## Behaviour

* **IDLE** – driver disabled. LED: short blip every second. MODE → ASSIST.
* **ASSIST** – LED solid. `force → low-pass → deadband → PID → elbow limits → slew limit → PWM`. MODE → IDLE.
* **FAULT** – latched, driver disabled, LED 5 Hz. Clear with E-stop released + MODE; if the cause persists it faults again.

| Fault (telemetry status) | Cause |
| :-- | :-- |
| `ESTOP` | E-stop pressed |
| `IMU_FAIL` | no valid upper-arm IMU sample for 50 ms |
| `AS5600_FAIL` | no valid AS5600 sample for 50 ms (I2C error or **no magnet detected**) |
| `LC_FAIL` | no load-cell sample for 500 ms, or saturated ADC |
| `ENC_MISMATCH` | AS5600 and optical encoder disagree by more than 8° for 50 ms (any state) |
| `OVERFORCE` | filtered force > 150 N (ASSIST) |
| `OVERSPEED` | elbow speed > 400 °/s (ASSIST) |
| `JOINT_LIMIT` | elbow more than 10° beyond its soft limits (ASSIST) |

* **Joint limits are now on the real elbow angle** (`ELBOW_MIN_DEG` 10°, `ELBOW_MAX_DEG` 140°, 0° = fully extended): the
  motor may not drive further into a limit but can always drive back out of it. Previously the limits were applied to a
  gravity-referenced forearm angle.
* A hardware **IWDG** (~0.4 s) resets the MCU if the loop ever stalls.
* Telemetry (20 Hz):
  `$TELEM,forearm_deg,kg,N,ax,ay,az,pwm_permille,mode,status,ua_deg,elbow_deg,elbow_dps,mismatch_deg,wrist_x_cm,wrist_y_cm`
  `python visualize_force_vector.py --port COM3` (or `--demo`). The first 10 fields keep the original layout.

## Bring-up order (do not skip steps)

1. Wire everything with the motor **power disconnected**. `pio run -e test_01_buttons -t upload` – check E-stop, MODE, LED, serial.
2. `test_02_imu` – upper-arm IMU: I2C1 scan (0x68), WHO_AM_I, verify the angle convention (0° hanging, 90° forward horizontal)
   and the gyro sign (`UA_GYRO_SIGN`).
3. `test_03_as5600` – I2C2 scan (0x36), magnet flags and AGC (adjust magnet height until MD = 1 and no MH/ML), check that flexion
   increases the angle (`AS5600_SIGN`), then **fully extend the elbow, press MODE and paste `AS5600_ZERO_DEG`** into Config.h.
4. `test_04_optical_encoder` – one encoder revolution must read ±4000 counts; flexion must increase the joint angle
   (`ENCODER_SIGN`); set `ENCODER_REVS_PER_JOINT_REV` if there is a gearbox.
5. `test_05_loadcell` – tare, hang a known mass, press MODE, copy the printed value into `LOADCELL_SCALE_COUNTS_PER_KG`.
   Check the sign: pushing UP must read positive (else `LOADCELL_FORCE_SIGN = -1.0f`).
6. `test_06_motor` – motor power on, arm unloaded. Confirm RPWM moves the arm **UP** (flexion); if not, swap the RPWM/LPWM wires.
7. `test_07_diagnostic` – everything together with the Python visualizer. Move the elbow by hand and confirm the AS5600 and
   optical angles agree (mismatch near 0) and the wrist position follows the arm.
8. `main_app` – start with the arm unloaded, then add load gradually. Tune `PID_KP/KI/KD`, `ASSIST_GAIN`,
   and only then raise `ASSIST_PWM_LIMIT` (starts at 30 %).

## Project layout

```text
include/  Config.h (all pins/limits/gains)  Board.h  Filters.h  ArmKinematics.h (joint fusion + forearm placement)
          PIDController.h  SafetyWatchdog.h  IMU_GY521.h  AS5600_Encoder.h  OpticalEncoder.h  HX711_LoadCell.h
          Telemetry.h  stm32f4xx_hal_conf.h
src/      ExoArmController.cpp (main app)  board.cpp (clock, GPIO, TIM1 PWM, TIM3 encoder, I2C1/I2C2, UART, IRQs, MSP)
          IMU_GY521.cpp  AS5600_Encoder.cpp  HX711_LoadCell.cpp  PIDController.cpp  Telemetry.cpp
hardware_tests/  01..07 (+ README)
visualize_force_vector.py
```

`board.cpp` is linked into every environment, so each test and the main app initialise the hardware identically.
If your PlatformIO setup already provides `SysTick_Handler` or `stm32f4xx_hal_conf.h`, delete the duplicate
(`SysTick_Handler` is in `board.cpp`; the HAL conf is in `include/`).
If your Black Pill has an 8 MHz crystal instead of 25 MHz, set `HSE_VALUE` and the PLLM value in `SystemClock_Config()`.

## Build / flash

```bash
pio run -e main_app -t upload      # production firmware
pio run -e test_03_as5600 -t upload   # any test: test_01_buttons ... test_07_diagnostic
pio device monitor -b 115200
```
