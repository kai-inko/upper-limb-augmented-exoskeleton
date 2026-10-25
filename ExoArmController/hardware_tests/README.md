# ExoArm Hardware Integrity & Subsystem Test Suite

This directory contains standalone, dedicated diagnostic sketches to verify each electrical component and subsystem **before** integrating them into the full autonomous exoskeleton system.

Testing each component independently isolates wiring errors, cold solder joints, noisy power rails, and defective modules before you connect the high-power 12V motor drive.

---

## Testing Workflow & Sequence

Run the test sketches in the following numerical order:

```
┌─────────────────────────────────┐
│ 01. Test Buttons & E-Stop       │ -> Verifies logic pullups & safety switch
└────────────────┬────────────────┘
                 ▼
┌─────────────────────────────────┐
│ 02. Test GY-521 (MPU-6050) IMU  │ -> Validates I2C address, gravity vector & drift
└────────────────┬────────────────┘
                 ▼
┌─────────────────────────────────┐
│ 03. Test HX711 20kg Load Cell   │ -> Validates bridge balance, noise & calibration
└────────────────┬────────────────┘
                 ▼
┌─────────────────────────────────┐
│ 04. Test BTS7960 Motor Driver   │ -> Verifies PWM, direction, break-before-make
└────────────────┬────────────────┘
                 ▼
┌─────────────────────────────────┐
│ 05. All-in-One Live Diagnostic  │ -> Complete integrated pre-flight bench checkout
└─────────────────────────────────┘
```

---

## 1. Test Descriptions & Expected Results

### Test 01: Pushbuttons, E-Stop & LED
* **Folder:** [`01_Test_Buttons_and_Estop/01_Test_Buttons_and_Estop.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController/hardware_tests/01_Test_Buttons_and_Estop/01_Test_Buttons_and_Estop.ino)
* **What it tests:**
  * Arduino Nano internal pullups on pins D7, D8, and D9.
  * Switch contact debounce and actuation counts.
  * Built-in diagnostic LED on pin D13.
* **Pass Criteria:**
  * When unpressed, pins D7 and D8 report `[PASS] Pin is HIGH`.
  * Pressing Button UP increments `upPressCount` by 1.
  * Tripping E-Stop immediately flags `[ALERT] EMERGENCY STOP TRIPPED`.

---

### Test 02: GY-521 (MPU-6050) 6-DOF IMU
* **Folder:** [`02_Test_GY521_IMU/02_Test_GY521_IMU.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController/hardware_tests/02_Test_GY521_IMU/02_Test_GY521_IMU.ino)
* **What it tests:**
  * Scans I2C bus on pins A4 (SDA) and A5 (SCL).
  * Validates chip signature register `0x75` (`WHO_AM_I` = `0x68`).
  * Measures earth gravity vector magnitude ($|\vec{a}| \approx 1.00\text{g} \pm 0.15\text{g}$).
  * Computes stationary gyroscope zero-rate bias.
  * Streams real-time fused inclination angle (pitch).
* **Pass Criteria:**
  * I2C device detected at address `0x68`.
  * Total acceleration magnitude is approximately $1.0\text{ g}$.
  * Rotating the sensor forward changes angle smoothly from $0^\circ$ to $90^\circ$ to $150^\circ$.

---

### Test 03: HX711 20kg Load Cell & Calibration
* **Folder:** [`03_Test_HX711_20kg_LoadCell/03_Test_HX711_20kg_LoadCell.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController/hardware_tests/03_Test_HX711_20kg_LoadCell/03_Test_HX711_20kg_LoadCell.ino)
* **What it tests:**
  * Fast clock and data synchronization on pins D2 (DOUT) and D3 (SCK).
  * Checks for open-circuit Wheatstone bridge faults (saturated ADC readings).
  * Evaluates peak-to-peak baseline noise floor (should be $< 5000$ counts).
  * **Interactive Calibration:** Allows you to place any known weight (e.g. 500g or 1000g) and automatically calculates the exact `LOAD_CELL_CAL_FAC` value to paste into `Config.h`.
* **Pass Criteria:**
  * ADC raw readings are within healthy range ($\pm 300,000$ counts, not saturated at $\pm 8,388,600$).
  * Pressing on the load cell produces positive delta counts; pulling produces negative counts.

---

### Test 04: BTS7960 43A Motor Driver
* **Folder:** [`04_Test_BTS7960_Motor/04_Test_BTS7960_Motor.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController/hardware_tests/04_Test_BTS7960_Motor/04_Test_BTS7960_Motor.ino)
* **What it tests:**
  * Verifies forward PWM on pin D5 (`RPWM`) and reverse PWM on pin D6 (`LPWM`).
  * Verifies break-before-make dead-time delay between direction switches.
  * Tests hardware gate enable cutoff on pin D4 (`R_EN` + `L_EN`).
  * Interactive keyboard jog mode (`w` = UP, `s` = DOWN, `space` = STOP).
* **Safety Feature:** The test **will not move the motor** until you open the Serial Monitor (115200 baud) and explicitly type `START`.
* **Pass Criteria:**
  * Motor smoothly ramps up in forward direction, stops for 1.5s, then ramps up in reverse.
  * Pulling D4 LOW instantly disables all motor torque.

---

### Test 05: All-in-One Comprehensive Pre-Flight Diagnostic
* **Folder:** [`05_All_Components_Diagnostic/05_All_Components_Diagnostic.ino`](file:///C:/Users/kainko-win/Desktop/ExoArmController/hardware_tests/05_All_Components_Diagnostic/05_All_Components_Diagnostic.ino)
* **What it tests:**
  * Runs all 4 subsystems simultaneously on one live dashboard screen:
    ```
    [IMU: 89.4 deg] | [LOAD: 250g (2.45N)] | [BTNS: UP=0 DN=0 ESTOP=OK] | [MOTOR: PWM=0]
    ```
  * Allows live jog testing of the motor while observing real-time load cell deflection and IMU joint angle tracking.

---

## 2. Common Troubleshooting Reference

| Symptom | Probable Cause | Fix |
| :--- | :--- | :--- |
| **`No I2C devices detected` in Test 02** | Reversed SDA/SCL or missing power | Ensure `SDA` is wired to Nano **`A4`** and `SCL` to Nano **`A5`**. Verify GY-521 power LED is lit. |
| **`HX711 not responding (DOUT HIGH)` in Test 03** | DT/SCK swapped or bad GND | Ensure `DT` $\rightarrow$ **`D2`**, `SCK` $\rightarrow$ **`D3`**, and common GND is connected. |
| **ADC counts saturated ($\pm 8,388,600$)** | Broken load cell wire | Check the 4 thin strain gauge wires: Red (`E+`), Black (`E-`), Green (`A+`), White (`A-`). |
| **Motor does not spin in Test 04** | Enable pins not powered or 12V supply off | Ensure `R_EN` and `L_EN` are jumpered together and connected to **`D4`**. Verify 12V power supply is on. |
| **Motor direction is inverted** | Motor polarity reversed | Swap the `M+` and `M-` wires on the BTS7960 terminal block. |
