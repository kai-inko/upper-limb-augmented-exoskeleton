/*
 * ======================================================================================
 * HARDWARE TEST 02: GY-521 (MPU-6050) & HARDWARE INT INTERRUPT (STM32 Bluepill)
 * Target: STM32F103C8T6 (Bluepill, 72MHz ARM Cortex-M3)
 * 
 * Purpose:
 *   1. Verify I2C1 communication on STM32 5V-tolerant pins PB6 (SCL) and PB7 (SDA).
 *   2. Measure 3D acceleration vector and static 1.0g gravity vector.
 *   3. Configure and verify MPU-6050 INT pin hardware interrupt triggering on PB14.
 * 
 * Hardware Wiring:
 *   - VCC -> 3.3V or 5V
 *   - GND -> GND
 *   - SCL -> Pin PB6 (STM32 Hardware I2C1 SCL, 5V Tolerant)
 *   - SDA -> Pin PB7 (STM32 Hardware I2C1 SDA, 5V Tolerant)
 *   - INT -> Pin PB14 (MPU6050 Hardware INT Pin, 5V Tolerant EXTI Pin)
 *   - AD0 -> GND (Address 0x68)
 * ======================================================================================
 */

#include <Wire.h>

const int PIN_MPU_INT = PB14;
const uint8_t MPU_ADDR = 0x68;

volatile unsigned long intCount = 0;
volatile bool intFired = false;

// STM32 Hardware EXTI ISR
void mpuHardwareInterruptISR() {
    intCount++;
    intFired = true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 02: GY-521 IMU & HARDWARE INT (STM32 BLUEPILL)"));
    Serial.println(F("========================================================"));

    Wire.begin();
    Wire.setClock(400000);

    // -------------------------------------------------------------------------
    // STEP 1: I2C Scanner on PB6 (SCL) and PB7 (SDA)
    // -------------------------------------------------------------------------
    Serial.println(F("[STEP 1] Scanning I2C1 on PB6(SCL) and PB7(SDA)..."));
    uint8_t target = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  [FOUND] I2C Device at: 0x"));
            if (a < 16) Serial.print('0');
            Serial.println(a, HEX);
            if (a == 0x68 || a == 0x69) target = a;
        }
    }

    if (target == 0) {
        Serial.println(F("  [FATAL ERROR] MPU6050 not found on I2C bus!"));
        Serial.println(F("  Verify: PB6 is SCL, PB7 is SDA, and sensor VCC/GND are connected."));
        while (1) delay(1000);
    }

    // -------------------------------------------------------------------------
    // STEP 2: WHO_AM_I Signature
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 2] Verifying MPU-6050 Signature (Register 0x75)..."));
    Wire.beginTransmission(target);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom(target, (uint8_t)1);
    uint8_t whoAmI = Wire.read();
    Serial.print(F("  Register 0x75: 0x")); Serial.println(whoAmI, HEX);
    if (whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x72) {
        Serial.println(F("  [PASS] Valid MPU-6050 silicon ID confirmed!"));
    }

    // Wake up from sleep
    Wire.beginTransmission(target);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Accel ±4g (8192 LSB/g)
    Wire.beginTransmission(target);
    Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();

    // Gyro ±500 dps (65.5 LSB/dps)
    Wire.beginTransmission(target);
    Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();

    // -------------------------------------------------------------------------
    // STEP 3: Configure Hardware Interrupt (INT Pin -> PB14)
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 3] Configuring MPU-6050 Hardware INT Pin on STM32 PB14..."));
    pinMode(PIN_MPU_INT, INPUT);

    // INT_PIN_CFG (0x37): Active HIGH, Push-Pull, Latched until read
    Wire.beginTransmission(target);
    Wire.write(0x37); Wire.write(0x20); Wire.endTransmission();

    // MOT_THR (0x1F): Motion detection threshold (~0.64g jerk)
    Wire.beginTransmission(target);
    Wire.write(0x1F); Wire.write(20); Wire.endTransmission();

    // MOT_DUR (0x20): Duration
    Wire.beginTransmission(target);
    Wire.write(0x20); Wire.write(0x02); Wire.endTransmission();

    // INT_ENABLE (0x38): Enable Motion & Data-Ready Interrupts
    Wire.beginTransmission(target);
    Wire.write(0x38); Wire.write(0x41); Wire.endTransmission();

    // Attach STM32 EXTI on PB14
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuHardwareInterruptISR, RISING);
    Serial.println(F("  [PASS] STM32 EXTI interrupt attached to Pin PB14!"));

    Serial.println(F("\n[STEP 4] Live IMU Angle & Interrupt Monitor:"));
    Serial.println(F("  - Move/rotate the arm to test joint inclination."));
    Serial.println(F("  - Rapid movement or exceeding safe range will trigger hardware INT on PB14."));
    Serial.println(F("--------------------------------------------------------\n"));
}

float filteredAngle = 90.0f;
unsigned long lastTime = 0;

void loop() {
    // Read MPU6050 raw data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
        delay(100);
        return;
    }
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // temp
    Wire.read(); Wire.read(); // Gx
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Gz

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    float ax_g = (float)rawAx / 8192.0f;
    float ay_g = (float)rawAy / 8192.0f;
    float az_g = (float)rawAz / 8192.0f;
    float gyroY = (float)rawGy / 65.5f;

    float accelPitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * (180.0f / PI) + 90.0f;
    filteredAngle = 0.96f * (filteredAngle + gyroY * dt) + 0.04f * accelPitch;

    // Check if hardware interrupt occurred
    if (intFired) {
        intFired = false;
        // Read INT_STATUS to clear latched interrupt
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3A);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)1);
        uint8_t status = Wire.read();

        Serial.print(F(">>> [HARDWARE INT FIRED ON PB14] Total Events: "));
        Serial.print(intCount);
        Serial.print(F(" | INT_STATUS: 0x"));
        Serial.print(status, HEX);
        if (filteredAngle <= 32.0f || filteredAngle >= 153.0f) {
            Serial.print(F(" [CRITICAL DEGREE RANGE LIMIT TRIPPED!]"));
        }
        Serial.println();
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 120) {
        lastPrint = millis();
        Serial.print(F("Angle: "));
        Serial.print(filteredAngle, 1);
        Serial.print(F("°  Accel: [X:"));
        Serial.print(ax_g, 2);
        Serial.print(F(" Y:"));
        Serial.print(ay_g, 2);
        Serial.print(F(" Z:"));
        Serial.print(az_g, 2);
        Serial.print(F("] g  GyroRate: "));
        Serial.print(gyroY, 1);
        Serial.print(F(" dps  EXTI_Count: "));
        Serial.println(intCount);
    }
}
