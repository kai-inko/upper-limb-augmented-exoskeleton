# ExoArmController STM32 Firmware (STM32Cube HAL)

Firmware for the Human Upper Limb Exoskeleton Controller based on the **STM32F103C8T6 (Blue Pill)**. This project provides real-time signal processing, safety monitoring, and closed-loop motor control to assist human arm movement during heavy lifting. 

The project is built natively using **STM32Cube HAL (C++)** on VS Code with PlatformIO, offering deterministic execution, direct hardware register configuration, zero Arduino framework overhead, and isolated hardware diagnostic environments.

---

## Table of Contents
1. [Architecture & Framework Migration](#1-architecture--framework-migration)
2. [Hardware Pinout & Wiring](#2-hardware-pinout--wiring)
3. [System Clock & Peripheral Initialization](#3-system-clock--peripheral-initialization)
4. [Project Directory Structure](#4-project-directory-structure)
5. [PlatformIO Multi-Environment Setup](#5-platformio-multi-environment-setup)
6. [Building & Flashing Code](#6-building--flashing-code)
7. [Control Loop Execution Flow](#7-control-loop-execution-flow)

---

## 1. Architecture & Framework Migration

This codebase has been refactored from legacy Arduino dependencies (`<Arduino.h>`, `<Wire.h>`, `digitalRead`, `analogWrite`, `millis`) to native **STM32Cube HAL**:

* **Framework:** Updated `platformio.ini` from `framework = arduino` to `framework = stm32cube`.
* **Standard C++ Support:** Configured the compiler for `-std=gnu++17` to support C++17 utilities (e.g., `std::clamp` in `PIDController`), with fallback macros for standard C++11 compatibility (`std::min`/`std::max`).
* **Modular HX711 Driver:** Extracted load-cell hardware driver logic into dedicated `HX711_LoadCell.h` and `HX711_LoadCell.cpp` modules.
* **Deterministic Timing:** Replaced standard loop delays with precise tick-based interval timing (`HAL_GetTick()`) running a **100Hz control loop (10ms sampling period)**.
* **Isolated Multi-Target Testing:** Configured `build_src_filter` in `platformio.ini` to enable single-click building and flashing of standalone hardware diagnostic tests without `main()` symbol redefinition conflicts.

---

## 2. Hardware Pinout & Wiring

### 2.1 MCU Pinout Configuration

| Peripheral / Component | Signal Name | STM32 Pin | Mode / Configuration | Hardware Function |
| :--- | :--- | :--- | :--- | :--- |
| **Emergency Stop Switch** | `ESTOP` | `PA0` | Input with Internal Pull-Up | Active-LOW hardware interlock switch |
| **Mode Toggle Button** | `MODE` | `PA1` | Input with Internal Pull-Up | Active-LOW mode selection switch |
| **BTS7960 Motor Driver** | `L_PWM` | `PA8` | TIM1 Channel 1 (`TIM1_CH1`) | Counter-clockwise / Upward PWM drive |
| **BTS7960 Motor Driver** | `R_PWM` | `PA9` | TIM1 Channel 2 (`TIM1_CH2`) | Clockwise / Downward PWM drive |
| **BTS7960 Motor Driver** | `L_EN` / `R_EN` | `PB12` | General Output Push-Pull | Bridge driver gate enable (Active High) |
| **HX711 Load Cell** | `DOUT` | `PB6` | General Input (Floating) | Serial data line from 20kg load cell ADC |
| **HX711 Load Cell** | `SCK` | `PB7` | Output Push-Pull (High Speed)| Serial clock pulse output to HX711 |
| **GY-521 (MPU-6050) IMU** | `SCL` | `PB6`* | I2C1 Alternate Function Open-Drain | I2C Clock Line (Requires pull-ups) |
| **GY-521 (MPU-6050) IMU** | `SDA` | `PB7`* | I2C1 Alternate Function Open-Drain | I2C Data Line (Requires pull-ups) |
| **ST-Link V2 Debugger** | `SWCLK` | `PA14` | Dedicated Serial Wire Debug | Hardware Flashing & Debugging Clock |
| **ST-Link V2 Debugger** | `SWDIO` | `PA13` | Dedicated Serial Wire Debug | Hardware Flashing & Debugging Data |

> **Note on Pin Sharing:** When running hardware diagnostic test `05_All_Components_Diagnostic`, ensure I2C1 remap flags are properly configured in `platformio.ini` or select separate GPIO lines (`PB8`/`PB9` for I2C1) if using HX711 and GY-521 simultaneously.

### 2.2 ST-Link V2 Wiring & Jumper Configurations

* **3.3V** $\rightarrow$ `3.3V` on Blue Pill
* **GND** $\rightarrow$ `GND` on Blue Pill
* **SWDIO** $\rightarrow$ `PA13`
* **SWCLK** $\rightarrow$ `PA14`
* **Boot Jumpers:** Set **BOOT0 = 0** and **BOOT1 = 0** (Jumpers closest to the micro-USB connector).

---

## 3. System Clock & Peripheral Initialization

### 3.1 72MHz System Clock (`SystemClock_Config`)
Configures the High-Speed External (HSE) 8MHz crystal via the phase-locked loop (PLL) multiplier to drive the core at maximum rated speed:
* **Core Clock (SYSCLK):** 72 MHz
* **AHB Prescaler:** DIV1 (72 MHz HCLK)
* **APB1 Prescaler:** DIV2 (36 MHz PCLK1)
* **APB2 Prescaler:** DIV1 (72 MHz PCLK2)
* **Flash Latency:** 2 Wait States

### 3.2 Timer 1 PWM Generation (`MX_TIM1_Init`)
Generates 1kHz complimentary PWM signals for directional power delivery:
* **Prescaler:** `71` ($72\,\text{MHz} / (71 + 1) = 1\,\text{MHz}$ counter frequency).
* **Period (ARR):** `1000` ($1\,\text{MHz} / 1000 = 1\,\text{kHz}$ PWM frequency).
* **Duty Cycle Clamping:** Modified dynamically from `0` to `1000` using HAL macros:
  ```cpp
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseWidth);
  ```

### 3.3 I2C1 Configuration (`MX_I2C1_Init`)
Configures hardware I2C peripheral for communication with the GY-521 MPU-6050 module:
* **Clock Speed:** 400,000 Hz (Fast Mode)
* **Device Target Address:** `0x68 << 1` (Shifted to 8-bit format `0xD0` for HAL drivers)

---

## 4. Project Directory Structure

```text
ExoArmController_STM32/
├── platformio.ini                 # PlatformIO environment configurations & build filters
├── include/
│   ├── Config.h                   # Pin assignments, frequencies, threshold constants
│   ├── Filters.h                  # Low-pass exponential moving average filter
│   ├── HX711_LoadCell.h           # Class declaration for 20kg Load Cell driver
│   ├── IMU_GY521.h                # Class declaration for MPU-6050 IMU driver
│   ├── PIDController.h            # PID controller class with anti-windup protection
│   └── SafetyWatchdog.h           # System timeout watchdog observer
├── src/
│   ├── ExoArmController.cpp       # Main production firmware entry point (100Hz loop)
│   ├── HX711_LoadCell.cpp         # Bit-banging & tare implementation for HX711
│   ├── IMU_GY521.cpp              # I2C communication implementation for MPU-6050
│   └── PIDController.cpp          # PID error computation and state clamping
├── hardware_tests/                # Isolated hardware verification tests
│   ├── 01_Test_Buttons_and_Estop/
│   │   └── 01_Test_Buttons_and_Estop.cpp
│   ├── 02_Test_GY521_IMU/
│   │   └── 02_Test_GY521_IMU.cpp
│   ├── 03_Test_HX711_20kg_LoadCell/
│   │   └── 03_Test_HX711_20kg_LoadCell.cpp
│   ├── 04_Test_BTS7960_Motor/
│   │   └── 04_Test_BTS7960_Motor.cpp
│   └── 05_All_Components_Diagnostic/
│       └── 05_All_Components_Diagnostic.cpp
└── visualize_force_vector.py     # Python script for real-time telemetry rendering
```

---

## 5. PlatformIO Multi-Environment Setup

The project `platformio.ini` uses `build_src_filter` to allow independent compilation of individual hardware diagnostic routines or the primary production code:

```ini
[env]
platform = ststm32
board = bluepill_f103c8
framework = stm32cube
board_build.mcu = stm32f103c8t6
board_build.f_cpu = 72000000L
build_flags = 
    -D STM32F103xB
    -I include
    -std=gnu++17
upload_protocol = stlink
debug_tool = stlink

; Main Arm Controller Application
[env:main_app]
build_src_filter = 
    +<*>

; Diagnostic Test 01: Buttons & Emergency Stop Switch
[env:test_01_buttons]
build_src_filter = 
    -<*> 
    +<../hardware_tests/01_Test_Buttons_and_Estop/01_Test_Buttons_and_Estop.cpp>

; Diagnostic Test 02: MPU-6050 GY-521 IMU
[env:test_02_imu]
build_src_filter = 
    -<*> 
    +<IMU_GY521.cpp>
    +<../hardware_tests/02_Test_GY521_IMU/02_Test_GY521_IMU.cpp>

; Diagnostic Test 03: HX711 Load Cell
[env:test_03_loadcell]
build_src_filter = 
    -<*> 
    +<HX711_LoadCell.cpp>
    +<../hardware_tests/03_Test_HX711_20kg_LoadCell/03_Test_HX711_20kg_LoadCell.cpp>

; Diagnostic Test 04: BTS7960 Motor Driver Sweep
[env:test_04_motor]
build_src_filter = 
    -<*> 
    +<../hardware_tests/04_Test_BTS7960_Motor/04_Test_BTS7960_Motor.cpp>

; Diagnostic Test 05: Full Integrated Diagnostic
[env:test_05_diagnostic]
build_src_filter = 
    -<*> 
    +<IMU_GY521.cpp>
    +<HX711_LoadCell.cpp>
    +<../hardware_tests/05_All_Components_Diagnostic/05_All_Components_Diagnostic.cpp>
```

---

## 6. Building & Flashing Code

### 6.1 Using VS Code GUI

1. Open the project folder in VS Code.
2. Click on the **PlatformIO Alien Icon** (🛸) on the left activity bar.
3. Under **Project Tasks**, expand the desired environment (e.g., `env:main_app` or `env:test_03_loadcell`).
4. Click **Build** to compile the source code.
5. Connect your ST-Link V2 programmer to your computer and the Blue Pill board.
6. Click **Upload** to flash the target program.

### 6.2 Using PlatformIO Integrated Terminal (CLI)

Run commands directly from the terminal (`Ctrl + ~`):

* **Build & Upload Production Main Code:**
  ```bash
  pio run -e main_app -t upload
  ```

* **Build & Upload Button & E-Stop Test:**
  ```bash
  pio run -e test_01_buttons -t upload
  ```

* **Build & Upload GY-521 IMU Test:**
  ```bash
  pio run -e test_02_imu -t upload
  ```

* **Build & Upload Load Cell Test:**
  ```bash
  pio run -e test_03_loadcell -t upload
  ```

* **Build & Upload Motor Driver Ramp Test:**
  ```bash
  pio run -e test_04_motor -t upload
  ```

* **Build & Upload Integrated System Diagnostic:**
  ```bash
  pio run -e test_05_diagnostic -t upload
  ```

---

## 7. Control Loop Execution Flow

The production entry point (`src/ExoArmController.cpp`) runs a non-blocking, deterministic 100Hz state machine:

```text
               +-------------------------------------------+
               |             Power-On / Reset              |
               |  HAL_Init() & Clock Config (72MHz)        |
               +-------------------------------------------+
                                     |
                                     v
               +-------------------------------------------+
               |       Peripheral Initialization           |
               |  MX_GPIO_Init()                           |
               |  MX_I2C1_Init()                           |
               |  MX_TIM1_Init() (PWM 1kHz)                |
               |  loadcell.begin() & tare()                |
               |  imu.begin()                              |
               +-------------------------------------------+
                                     |
                                     v
               +-------------------------------------------+
               |        100Hz Periodic Scheduler Loop      |
               |        (Tick interval: 10 ms)             |
               +-------------------------------------------+
                                     |
                    +----------------+----------------+
                    |                                 |
         [E-Stop Pin Active (LOW)]             [Normal Operation]
                    |                                 |
                    v                                 v
        +-----------------------+         +-----------------------+
        | Immediate Shutdown:   |         | Read IMU Accelerometer|
        | TIM1_CH1 Duty = 0     |         | Read Load Cell Force  |
        | TIM1_CH2 Duty = 0     |         +-----------------------+
        +-----------------------+                     |
                                                      v
                                          +-----------------------+
                                          | Compute PID Output &  |
                                          | Anti-Windup Guard     |
                                          +-----------------------+
                                                      |
                                                      v
                                          +-----------------------+
                                          | Direct Motor Control: |
                                          | Set TIM1 Duty Cycles  |
                                          | Kick Safety Watchdog  |
                                          +-----------------------+
```