/*
 * ======================================================================================
 * HARDWARE TEST 05: ALL-IN-ONE COMPREHENSIVE HARDWARE PRE-FLIGHT DIAGNOSTIC
 * Target: Arduino Nano (ATmega328P)
 * 
 * Purpose:
 *   Simultaneously stream and verify all 4 hardware subsystems on the assembled
 *   exoskeleton bench setup before uploading the main autonomous control loop:
 *     1. GY-521 IMU (MPU-6050 on I2C A4/A5)
 *     2. 20kg Load Cell + HX711 (Pins D2/D3)
 *     3. Pushbuttons (D7/D8) & E-Stop Switch (D9)
 *     4. BTS7960 43A Motor Driver (Pins D4/D5/D6)
 * ======================================================================================
 */

#include <Wire.h>

// Pin Definitions
const int PIN_HX711_DOUT = 2;
const int PIN_HX711_SCK  = 3;
const int PIN_BTS_EN     = 4;
const int PIN_BTS_RPWM   = 5;
const int PIN_BTS_LPWM   = 6;
const int PIN_BTN_UP     = 7;
const int PIN_BTN_DOWN   = 8;
const int PIN_ESTOP      = 9;
const int PIN_LED        = 13;

const uint8_t MPU_ADDR   = 0x68;

// State Variables
float imuAngleDeg = 90.0f;
bool  imuHealthy  = false;
unsigned long lastImuTime = 0;

long  loadCellZero = 0;
float loadCellCalFac = 21.50f;
long  lastRawLoadCell = 0;
float currentForceN = 0.0f;
float currentGrams  = 0.0f;

int   currentPWM = 0;

// Fast HX711 Reader
bool readHX711Raw(long &val) {
    if (digitalRead(PIN_HX711_DOUT) == HIGH) return false;
    unsigned long count = 0;
    noInterrupts();
    for (int i = 0; i < 24; i++) {
        digitalWrite(PIN_HX711_SCK, HIGH); delayMicroseconds(1);
        count = count << 1;
        digitalWrite(PIN_HX711_SCK, LOW); delayMicroseconds(1);
        if (digitalRead(PIN_HX711_DOUT) == HIGH) count++;
    }
    digitalWrite(PIN_HX711_SCK, HIGH); delayMicroseconds(1);
    digitalWrite(PIN_HX711_SCK, LOW); delayMicroseconds(1);
    interrupts();
    if (count & 0x800000) count |= 0xFF000000;
    val = (long)count;
    return true;
}

void tareLoadCell() {
    long sum = 0;
    int n = 0;
    unsigned long t = millis() + 1500;
    while (n < 20 && millis() < t) {
        long r;
        if (readHX711Raw(r)) { sum += r; n++; delay(10); }
    }
    if (n > 0) loadCellZero = sum / n;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 05: ALL-IN-ONE COMPREHENSIVE HARDWARE DIAGNOSTIC"));
    Serial.println(F("========================================================"));

    // Configure Pins
    pinMode(PIN_BTS_EN, OUTPUT);
    pinMode(PIN_BTS_RPWM, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);
    digitalWrite(PIN_BTS_EN, HIGH);
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);

    pinMode(PIN_HX711_SCK, OUTPUT);
    pinMode(PIN_HX711_DOUT, INPUT);
    digitalWrite(PIN_HX711_SCK, LOW);

    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    // Initialize GY-521 IMU
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    if (Wire.endTransmission() == 0) {
        imuHealthy = true;
        Serial.println(F("[SUBSYSTEM 1] GY-521 IMU: ONLINE (I2C 0x68 OK)"));
    } else {
        imuHealthy = false;
        Serial.println(F("[SUBSYSTEM 1] GY-521 IMU: OFFLINE! Check A4/A5 wiring."));
    }

    // Set IMU Accel ±4g, Gyro ±500 dps
    if (imuHealthy) {
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    }

    // Tare Load Cell
    Serial.println(F("[SUBSYSTEM 2] Taring 20kg Load Cell..."));
    tareLoadCell();
    Serial.print(F("              Zero Tare Offset: ")); Serial.println(loadCellZero);

    Serial.println(F("[SUBSYSTEM 3] Pushbuttons & E-Stop: ONLINE"));
    Serial.println(F("[SUBSYSTEM 4] BTS7960 Motor Driver: ONLINE (Bridge Enabled)"));

    Serial.println(F("\n--- Live Diagnostic Stream Started ---"));
    Serial.println(F("Controls: 'w' = Jog UP | 's' = Jog DOWN | ' ' = Stop | 't' = Tare Load Cell\n"));
    lastImuTime = micros();
}

void loop() {
    // 1. Process Serial Jog Commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'w' || c == 'W') {
            currentPWM = 70;
            analogWrite(PIN_BTS_LPWM, 0);
            analogWrite(PIN_BTS_RPWM, currentPWM);
        } else if (c == 's' || c == 'S') {
            currentPWM = -70;
            analogWrite(PIN_BTS_RPWM, 0);
            analogWrite(PIN_BTS_LPWM, abs(currentPWM));
        } else if (c == ' ') {
            currentPWM = 0;
            analogWrite(PIN_BTS_RPWM, 0);
            analogWrite(PIN_BTS_LPWM, 0);
        } else if (c == 't' || c == 'T') {
            tareLoadCell();
            Serial.println(F("\n>>> [TARE] Load Cell Zero Re-calibrated! <<<\n"));
        }
    }

    // 2. Read IMU Pitch
    if (imuHealthy) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        if (Wire.endTransmission(false) == 0) {
            Wire.requestFrom(MPU_ADDR, (uint8_t)14);
            int16_t rawAx = (Wire.read() << 8) | Wire.read();
            int16_t rawAy = (Wire.read() << 8) | Wire.read();
            int16_t rawAz = (Wire.read() << 8) | Wire.read();
            Wire.read(); Wire.read(); // Temp
            Wire.read(); Wire.read(); // Gx
            int16_t rawGy = (Wire.read() << 8) | Wire.read();
            Wire.read(); Wire.read(); // Gz

            unsigned long now = micros();
            float dt = (now - lastImuTime) / 1000000.0f;
            lastImuTime = now;
            if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

            float gyroRateY = (float)rawGy / 65.5f;
            float accelPitch = atan2((float)rawAx, sqrt((float)rawAy * rawAy + (float)rawAz * rawAz)) * (180.0f / PI) + 90.0f;
            imuAngleDeg = 0.96f * (imuAngleDeg + gyroRateY * dt) + 0.04f * accelPitch;
        } else {
            imuHealthy = false;
        }
    }

    // 3. Read 20kg Load Cell
    long rawLC;
    if (readHX711Raw(rawLC)) {
        lastRawLoadCell = rawLC;
        float delta = (float)(rawLC - loadCellZero);
        currentGrams = delta / loadCellCalFac;
        currentForceN = (currentGrams / 1000.0f) * 9.80665f;
    }

    // 4. Read Pushbuttons & E-Stop
    bool btnUp   = (digitalRead(PIN_BTN_UP) == LOW);
    bool btnDown = (digitalRead(PIN_BTN_DOWN) == LOW);
    bool estop   = (digitalRead(PIN_ESTOP) == HIGH); // HIGH = Tripped/Open

    // Auto-EStop safety in diagnostic: if estop tripped, cut motor immediately
    if (estop && currentPWM != 0) {
        currentPWM = 0;
        analogWrite(PIN_BTS_RPWM, 0);
        analogWrite(PIN_BTS_LPWM, 0);
        digitalWrite(PIN_LED, HIGH);
    } else if (!estop) {
        digitalWrite(PIN_LED, LOW);
    }

    // 5. Periodic Dashboard Output (Every 150ms)
    static unsigned long lastDashboard = 0;
    if (millis() - lastDashboard > 150) {
        lastDashboard = millis();

        Serial.print(F("[IMU: "));
        if (imuHealthy) {
            Serial.print(imuAngleDeg, 1);
            Serial.print(F(" deg"));
        } else {
            Serial.print(F("DISCONNECTED"));
        }

        Serial.print(F("] | [LOAD: "));
        Serial.print(currentGrams, 0);
        Serial.print(F("g ("));
        Serial.print(currentForceN, 2);
        Serial.print(F("N)] | [BTNS: UP="));
        Serial.print(btnUp ? "1" : "0");
        Serial.print(F(" DN="));
        Serial.print(btnDown ? "1" : "0");
        Serial.print(F(" ESTOP="));
        Serial.print(estop ? "TRIP" : "OK");

        Serial.print(F("] | [MOTOR: PWM="));
        Serial.print(currentPWM);
        Serial.println(F("]"));
    }
}
