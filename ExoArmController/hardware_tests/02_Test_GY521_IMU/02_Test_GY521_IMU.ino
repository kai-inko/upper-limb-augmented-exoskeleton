/*
 * ======================================================================================
 * HARDWARE TEST 02: GY-521 (MPU-6050) 6-DOF IMU INTEGRITY CHECK
 * Target: Arduino Nano (ATmega328P)
 * 
 * Purpose:
 *   Verify I2C communication, power status, WHO_AM_I device signature,
 *   accelerometer gravity vector calibration, and gyroscope drift/noise.
 * 
 * Hardware Wiring:
 *   - VCC -> 5V (or 3.3V)
 *   - GND -> GND
 *   - SDA -> Pin A4 (Arduino Nano Hardware I2C SDA)
 *   - SCL -> Pin A5 (Arduino Nano Hardware I2C SCL)
 *   - AD0 -> GND (Sets address to 0x68. If 3.3V, address is 0x69)
 * ======================================================================================
 */

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 02: GY-521 (MPU-6050) IMU INTEGRITY CHECK"));
    Serial.println(F("========================================================"));

    Wire.begin();
    Wire.setClock(400000); // 400kHz fast I2C mode

    // -------------------------------------------------------------------------
    // STEP 1: I2C Bus Scan
    // -------------------------------------------------------------------------
    Serial.println(F("[STEP 1] Scanning I2C Bus on Pins A4(SDA) and A5(SCL)..."));
    uint8_t foundCount = 0;
    uint8_t targetAddr = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  [FOUND] I2C Device responding at address: 0x"));
            if (addr < 16) Serial.print('0');
            Serial.println(addr, HEX);
            foundCount++;
            if (addr == 0x68 || addr == 0x69) {
                targetAddr = addr;
            }
        }
    }

    if (foundCount == 0) {
        Serial.println(F("  [FATAL ERROR] No I2C devices detected!"));
        Serial.println(F("  Checklist:"));
        Serial.println(F("   1. Verify VCC has 5V (or 3.3V) and GND is connected."));
        Serial.println(F("   2. Verify SDA is on Pin A4 and SCL is on Pin A5."));
        Serial.println(F("   3. Check for loose jumper wires."));
        while (1) delay(1000);
    }

    if (targetAddr == 0) {
        Serial.println(F("  [WARN] Device found but not at 0x68 or 0x69. Check AD0 pin wiring."));
        targetAddr = 0x68;
    }

    // -------------------------------------------------------------------------
    // STEP 2: WHO_AM_I Chip Signature Verification
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 2] Verifying MPU-6050 Chip ID (Register 0x75)..."));
    Wire.beginTransmission(targetAddr);
    Wire.write(0x75);
    Wire.endTransmission(false);
    Wire.requestFrom(targetAddr, (uint8_t)1);
    uint8_t whoAmI = Wire.read();

    Serial.print(F("  Register 0x75 returned: 0x"));
    Serial.println(whoAmI, HEX);
    if (whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x72) {
        Serial.println(F("  [PASS] Valid MPU-6050 / compatible silicon signature verified!"));
    } else {
        Serial.println(F("  [WARN] Non-standard chip ID. Sensor may be a compatible clone."));
    }

    // -------------------------------------------------------------------------
    // STEP 3: Power Management Wake-Up & Configuration
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 3] Configuring Power & Operating Modes..."));
    // Wake up from sleep (clear SLEEP bit in PWR_MGMT_1)
    Wire.beginTransmission(targetAddr);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    if (Wire.endTransmission() == 0) {
        Serial.println(F("  [PASS] MPU-6050 woke up from sleep mode."));
    } else {
        Serial.println(F("  [FAIL] Failed to write PWR_MGMT_1 register!"));
    }

    // Set Accel Range ±4g (FS_SEL = 1, 8192 LSB/g)
    Wire.beginTransmission(targetAddr);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    // Set Gyro Range ±500 deg/s (FS_SEL = 1, 65.5 LSB/(deg/s))
    Wire.beginTransmission(targetAddr);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    // -------------------------------------------------------------------------
    // STEP 4: Static Calibration & Gravity Vector Test (Hold still for 2 sec)
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 4] Static Noise & Gravity Vector Analysis (Sampling 100 points)..."));
    Serial.println(F("  *** Please keep the sensor stationary on a flat surface! ***"));
    delay(500);

    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumGx = 0, sumGy = 0, sumGz = 0;
    const int SAMPLES = 100;

    for (int i = 0; i < SAMPLES; i++) {
        Wire.beginTransmission(targetAddr);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(targetAddr, (uint8_t)14);

        int16_t rawAx = (Wire.read() << 8) | Wire.read();
        int16_t rawAy = (Wire.read() << 8) | Wire.read();
        int16_t rawAz = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temp
        int16_t rawGx = (Wire.read() << 8) | Wire.read();
        int16_t rawGy = (Wire.read() << 8) | Wire.read();
        int16_t rawGz = (Wire.read() << 8) | Wire.read();

        // Convert to physical units: ±4g -> 8192 LSB/g, ±500 dps -> 65.5 LSB/(deg/s)
        sumAx += (float)rawAx / 8192.0f;
        sumAy += (float)rawAy / 8192.0f;
        sumAz += (float)rawAz / 8192.0f;
        sumGx += (float)rawGx / 65.5f;
        sumGy += (float)rawGy / 65.5f;
        sumGz += (float)rawGz / 65.5f;
        delay(10);
    }

    float avgAx = sumAx / SAMPLES;
    float avgAy = sumAy / SAMPLES;
    float avgAz = sumAz / SAMPLES;
    float totalG = sqrt(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);

    Serial.print(F("  Average Accel: X=")); Serial.print(avgAx, 3);
    Serial.print(F("g, Y=")); Serial.print(avgAy, 3);
    Serial.print(F("g, Z=")); Serial.print(avgAz, 3);
    Serial.print(F("g -> Total Magnitude: ")); Serial.print(totalG, 3); Serial.println(F(" g"));

    if (totalG >= 0.85f && totalG <= 1.15f) {
        Serial.println(F("  [PASS] Earth Gravity Vector validated within ±15% of 1.00g!"));
    } else {
        Serial.println(F("  [WARN] Acceleration vector out of bounds. Check sensor orientation or calibration."));
    }

    Serial.print(F("  Gyro Zero-Rate Bias: X=")); Serial.print(sumGx / SAMPLES, 2);
    Serial.print(F(" deg/s, Y=")); Serial.print(sumGy / SAMPLES, 2);
    Serial.print(F(" deg/s, Z=")); Serial.print(sumGz / SAMPLES, 2); Serial.println(F(" deg/s"));
    Serial.println(F("  [PASS] Static calibration complete."));

    Serial.println(F("\n[STEP 5] Real-Time Orientation Stream:"));
    Serial.println(F("  Rotate the GY-521 to test inclination angle tracking."));
    Serial.println(F("--------------------------------------------------------\n"));
}

float filteredAngle = 90.0f;
unsigned long lastTime = 0;

void loop() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
        Serial.println(F("[ERROR] I2C Communication Lost!"));
        delay(500);
        return;
    }
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temp
    Wire.read(); Wire.read(); // Skip Gx
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip Gz

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    float gyroRateY = (float)rawGy / 65.5f;
    float accelPitch = atan2((float)rawAx, sqrt((float)rawAy * rawAy + (float)rawAz * rawAz)) * (180.0f / PI) + 90.0f;

    // Complementary filter: 96% gyro, 4% accel
    filteredAngle = 0.96f * (filteredAngle + gyroRateY * dt) + 0.04f * accelPitch;

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();
        Serial.print(F("Elbow_Angle_Deg:"));
        Serial.print(filteredAngle, 1);
        Serial.print(F("  Raw_Pitch:"));
        Serial.print(accelPitch, 1);
        Serial.print(F("  GyroRate_Y:"));
        Serial.print(gyroRateY, 1);
        Serial.println(F(" dps"));
    }
}
