# ExoArmController STM32 (Blue Pill, STM32Cube HAL, PlatformIO)

Elbow-assist exoskeleton controller. A 100 Hz loop reads an MPU-6050 (elbow angle) and a 20 kg
HX711 load cell (how hard the user pushes the cuff) and drives a BTS7960 H-bridge so the motor
moves **with** the user. Everything is wrapped in latched safety faults.

> **Safety first.** The firmware E-stop is *not* a safety-rated device. Fit a real E-stop that cuts
> motor POWER, add 10 kΩ pull-downs on PA8, PA9 and PB12 (so the driver stays off while the MCU
> boots or resets), and always commission with the arm unloaded and mechanically clamped.

## Pin map (single source of truth: `include/Config.h`)

| Function | Pin | Notes |
| :-- | :-- | :-- |
| E-Stop | PA0 | active LOW, internal pull-up (NC switch to GND is the safest wiring) |
| MODE button | PA1 | active LOW, internal pull-up |
| BTS7960 RPWM (UP) | PA8 | TIM1_CH1, 20 kHz |
| BTS7960 LPWM (DOWN) | PA9 | TIM1_CH2, 20 kHz |
| BTS7960 R_EN + L_EN | PB12 | tie both together; LOW at boot |
| IMU SCL / SDA | PB6 / PB7 | I2C1, 400 kHz, 4.7 kΩ pull-ups to 3V3 |
| HX711 DOUT / SCK | PB14 / PB15 | moved off PB6/PB7 (they collide with I2C1) |
| Telemetry TX | PA2 | USART2, 115200 8N1, to a 3V3 USB-TTL adapter RX |
| Status LED | PC13 | on-board, active LOW |
| ST-Link | PA13 / PA14 | SWDIO / SWCLK |

Do not use USART1 for telemetry: PA9 is needed by TIM1_CH2.

## Behaviour

* **IDLE** – driver disabled. LED: short blip every second. MODE → ASSIST.
* **ASSIST** – LED solid. `force → low-pass → deadband → PID → angle limits → slew limit → PWM`.
  MODE → IDLE.
* **FAULT** – latched, driver disabled, LED 5 Hz. Causes: `ESTOP`, `IMU_FAIL` (no valid sample for 50 ms),
  `LC_FAIL` (none for 500 ms, or saturated ADC), `OVERFORCE` (> 150 N). Clear with E-stop released + MODE;
  if the cause persists it faults again.
* A hardware **IWDG** (~0.4 s) resets the MCU if the loop ever stalls.
* Telemetry: `$TELEM,angle,kg,N,ax,ay,az,pwm_permille,mode,status` at 20 Hz.
  `python visualize_force_vector.py --port COM3` (or `--demo`).

## Bring-up order (do not skip steps)

1. Wire everything with the motor **power disconnected**. `pio run -e test_01_buttons -t upload` – check E-stop, MODE, LED, serial.
2. `test_02_imu` – I2C scan, WHO_AM_I, then verify the angle convention (0° hanging, 90° horizontal) and the gyro sign
   (`IMU_GYRO_SIGN` in Config.h).
3. `test_03_loadcell` – tare, hang a known mass, press MODE, copy the printed value into `LOADCELL_SCALE_COUNTS_PER_KG`.
   Check the sign: pushing UP must read positive (else `LOADCELL_FORCE_SIGN = -1.0f`).
4. `test_04_motor` – motor power on, arm unloaded. Confirm RPWM moves the arm **UP**; if not, swap the RPWM/LPWM wires.
5. `test_05_diagnostic` – everything together with the Python visualizer.
6. `main_app` – start with the arm unloaded, then add load gradually. Tune `PID_KP/KI/KD`, `ASSIST_GAIN`,
   and only then raise `ASSIST_PWM_LIMIT` (starts at 30 %).

## Project layout

```text
include/  Config.h (all pins/limits/gains) Board.h  Filters.h  PIDController.h  SafetyWatchdog.h
          IMU_GY521.h  HX711_LoadCell.h  Telemetry.h  stm32f1xx_hal_conf.h
src/      ExoArmController.cpp (main app)  board.cpp (clock, GPIO, TIM1, I2C, UART, IRQs, MSP)
          IMU_GY521.cpp  HX711_LoadCell.cpp  PIDController.cpp  Telemetry.cpp
hardware_tests/  01..05 (+ README)
visualize_force_vector.py
```

`board.cpp` is linked into every environment, so each test and the main app initialise the hardware identically.
If your PlatformIO setup already provides `SysTick_Handler` or `stm32f1xx_hal_conf.h`, delete the duplicate
(`SysTick_Handler` is in `board.cpp`; the HAL conf is in `include/`).

## Build / flash

```bash
pio run -e main_app -t upload      # production firmware
pio run -e test_02_imu -t upload   # any test: test_01_buttons ... test_05_diagnostic
pio device monitor -b 115200
```
