# ExoArm STM32 Bluepill Hardware Testing Suite

Dedicated diagnostic suite for the **STM32 Bluepill (STM32F103C8T6)** exoskeleton arm build.

---

## Hardware Pinout Reference (STM32 Bluepill)

| Peripheral | Bluepill Pin | Pin Characteristics | Function |
| :--- | :--- | :--- | :--- |
| **MPU-6050 SCL** | **`PB6`** | 5V-Tolerant I2C1 | Hardware I2C Clock (400 kHz) |
| **MPU-6050 SDA** | **`PB7`** | 5V-Tolerant I2C1 | Hardware I2C Data Line |
| **MPU-6050 INT** | **`PB14`** | 5V-Tolerant EXTI | System Interruption for Degree Range Limit |
| **HX711 DOUT** | **`PB12`** | 5V-Tolerant Input | 24-bit Load Cell Data Output |
| **HX711 SCK** | **`PB13`** | 5V-Tolerant Output | 24-bit Load Cell Serial Clock |
| **BTS7960 RPWM** | **`PA0`** | Timer 2 Channel 1 PWM | Flexion / Move UP PWM ($0 - 250$) |
| **BTS7960 LPWM** | **`PA1`** | Timer 2 Channel 2 PWM | Extension / Move DOWN PWM ($0 - 250$) |
| **BTS7960 EN** | **`PA2`** | 3.3V GPIO Output | Gate Enable (connects to both R_EN & L_EN) |
| **Button UP** | **`PA4`** | Input with Pullup | Manual Flexion Command |
| **Button DOWN** | **`PA5`** | Input with Pullup | Manual Extension Command |
| **E-Stop Switch** | **`PA6`** | Input with Pullup | Hardware Emergency Cutoff |
| **Diagnostic LED** | **`PC13`** | Active LOW Onboard LED | Visual status & fault alert |

---

## Testing Workflow

1. [**`01_Test_Buttons_and_Estop.cpp`**](01_Test_Buttons_and_Estop/01_Test_Buttons_and_Estop.cpp)  
   Verifies pullups on PA4/PA5/PA6 and validates active-LOW LED drive on PC13.
2. [**`02_Test_GY521_IMU.cpp`**](02_Test_GY521_IMU/02_Test_GY521_IMU.cpp)  
   Scans I2C1 on PB6/PB7, verifies WHO_AM_I, and confirms hardware EXTI interruption on PB14.
3. [**`03_Test_HX711_20kg_LoadCell.cpp`**](03_Test_HX711_20kg_LoadCell/03_Test_HX711_20kg_LoadCell.cpp)  
   Tests Wheatstone bridge continuity and performs guided manual calibration in **KILOGRAMS (kg)**.
4. [**`04_Test_BTS7960_Motor.cpp`**](04_Test_BTS7960_Motor/04_Test_BTS7960_Motor.cpp)  
   Runs automated forward/reverse ramp tests on PA0/PA1 and verifies gate cutoff on PA2.
5. [**`05_All_Components_Diagnostic.cpp`**](05_All_Components_Diagnostic/05_All_Components_Diagnostic.cpp)  
   Master bench diagnostic dashboard emitting live `$TELEM` packets for the Python visualizer.
