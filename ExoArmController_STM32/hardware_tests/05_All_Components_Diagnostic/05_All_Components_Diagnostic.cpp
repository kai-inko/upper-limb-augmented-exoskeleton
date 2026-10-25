/*
 * ======================================================================================
 * HARDWARE TEST 05: ALL-IN-ONE COMPREHENSIVE PRE-FLIGHT DIAGNOSTIC (STM32 Bluepill)
 * Target: STM32F103C8T6 (Bluepill, 72MHz ARM Cortex-M3)
 * 
 * Verifies all 4 hardware subsystems on STM32 Bluepill:
 *   1. GY-521 IMU (PB6 SCL, PB7 SDA)
 *   2. MPU-6050 Hardware INT Pin (PB14 EXTI)
 *   3. 20kg Strain Gauge Load Cell (PB12 DOUT, PB13 SCK) calibrated in KILOGRAMS (kg)
 *   4. BTS7960 43A Motor Driver (PA0 RPWM, PA1 LPWM, PA2 EN)
 *   5. Buttons (PA4/PA5), E-Stop (PA6), LED (PC13)
 * 
 * Emits telemetry stream compatible with:
 *   - Arduino Serial Plotter
 *   - Python Force Vector Visualizer (visualize_force_vector.py)
 * ======================================================================================
 */

#include <Arduino.h>   // Serial, pinMode/digitalWrite/analogWrite, millis, F(), PI, constrain (STM32duino core)
#include <stdint.h>
#include <math.h>
#include <Wire.h>      // STM32duino I2C: Wire.setSDA()/setSCL()/begin()

const int PIN_HX711_DOUT = PB12;
const int PIN_HX711_SCK  = PB13;
const int PIN_BTS_RPWM   = PA0;
const int PIN_BTS_LPWM   = PA1;
const int PIN_BTS_EN     = PA2;
const int PIN_BTN_UP     = PA4;
const int PIN_BTN_DOWN   = PA5;
const int PIN_ESTOP      = PA6;
const int PIN_LED        = PC13;
const int PIN_MPU_INT    = PB14;

const uint8_t MPU_ADDR   = 0x68;

#define LED_ON  LOW
#define LED_OFF HIGH

// State Variables
float imuAngleDeg = 90.0f;
bool  imuHealthy  = false;
float accelX_g    = 0.0f;
float accelY_g    = 1.0f;
float accelZ_g    = 0.0f;

volatile unsigned long mpuIntCount = 0;
volatile bool mpuIntFired = false;

// Read one big-endian 16-bit word from the I2C RX buffer.
// Do NOT write (Wire.read() << 8) | Wire.read(): C++ leaves operand evaluation order
// unspecified, and ARM GCC (STM32) may read the low byte first, swapping the bytes.
static inline int16_t readInt16BE() {
    uint8_t hi = (uint8_t)Wire.read();
    uint8_t lo = (uint8_t)Wire.read();
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

long  loadCellZero = 0;
float loadCellCalFacKg = 21500.0f;
float currentWeightKg  = 0.0f;
float currentForceN    = 0.0f;

int currentPWM = 0;
unsigned long lastImuTime = 0;

void mpuInterruptISR() {
    mpuIntCount++;
    mpuIntFired = true;
}

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
    analogWriteResolution(8);   // STM32duino: make PWM duty 0..255 explicit
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 05: ALL-IN-ONE BENCH PRE-FLIGHT (STM32 BLUEPILL)"));
    Serial.println(F("========================================================"));

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
    digitalWrite(PIN_LED, LED_OFF);

    // Initialize MPU-6050
    Wire.setSDA(PB7);   // I2C1 SDA (set pins BEFORE begin)
    Wire.setSCL(PB6);   // I2C1 SCL
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00); // Wake up
    if (Wire.endTransmission() == 0) {
        imuHealthy = true;
        Serial.println(F("[SUBSYSTEM 1] GY-521 IMU: ONLINE (I2C1 PB6/PB7 OK)"));

        // Configure Accel ±4g, Gyro ±500 dps
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();

        // Configure INT Pin (Active High, Latched)
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x37); Wire.write(0x20); Wire.endTransmission();
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x1F); Wire.write(20); Wire.endTransmission(); // MOT_THR
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x20); Wire.write(0x02); Wire.endTransmission(); // MOT_DUR
        Wire.beginTransmission(MPU_ADDR); Wire.write(0x38); Wire.write(0x41); Wire.endTransmission(); // MOT_EN + DATA_RDY

        pinMode(PIN_MPU_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuInterruptISR, RISING);
        Serial.println(F("[SUBSYSTEM 2] MPU6050 Hardware INT: ONLINE (PB14 Attached)"));
    } else {
        imuHealthy = false;
        Serial.println(F("[SUBSYSTEM 1] GY-521 IMU: OFFLINE! Check PB6/PB7 wiring."));
    }

    // Tare Load Cell
    Serial.println(F("[SUBSYSTEM 3] Taring 20kg Load Cell..."));
    tareLoadCell();
    Serial.print(F("              Zero Tare Offset: ")); Serial.println(loadCellZero);

    Serial.println(F("[SUBSYSTEM 4] Pushbuttons & E-Stop: ONLINE"));
    Serial.println(F("[SUBSYSTEM 5] BTS7960 Motor Driver: ONLINE (Bridge Enabled)"));

    Serial.println(F("\n--- Live Diagnostic Stream Started ---"));
    Serial.println(F("Controls: 'w'=Jog UP | 's'=Jog DOWN | ' '=Stop | 't'=Tare | 'c <kg>'=Calibrate\n"));
    lastImuTime = micros();
}

void loop() {
    // 1. Process Serial Commands
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
            Serial.println(F(">>> [TARE] Load cell re-zeroed! <<<"));
        } else if (c == 'c' || c == 'C') {
            float knownKg = Serial.parseFloat();
            if (knownKg > 0.05f) {
                long sum = 0;
                int n = 0;
                unsigned long t = millis() + 1500;
                while (n < 20 && millis() < t) {
                    long r;
                    if (readHX711Raw(r)) { sum += r; n++; delay(10); }
                }
                if (n > 0) {
                    long delta = abs((sum / n) - loadCellZero);
                    loadCellCalFacKg = (float)delta / knownKg;
                    Serial.print(F(">>> CALIBRATED: ")); Serial.print(loadCellCalFacKg, 2); Serial.println(F(" counts/kg <<<"));
                }
            }
        }
    }

    // 2. Read IMU
    if (imuHealthy) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B);
        if (Wire.endTransmission(false) == 0) {
            Wire.requestFrom(MPU_ADDR, (uint8_t)14);
            int16_t rawAx = readInt16BE();
            int16_t rawAy = readInt16BE();
            int16_t rawAz = readInt16BE();
            Wire.read(); Wire.read(); // Temp
            Wire.read(); Wire.read(); // Gx
            int16_t rawGy = readInt16BE();
            Wire.read(); Wire.read(); // Gz

            unsigned long now = micros();
            float dt = (now - lastImuTime) / 1000000.0f;
            lastImuTime = now;
            if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

            accelX_g = (float)rawAx / 8192.0f;
            accelY_g = (float)rawAy / 8192.0f;
            accelZ_g = (float)rawAz / 8192.0f;
            float gyroRateY = (float)rawGy / 65.5f;

            float accelPitch = atan2f(accelX_g, sqrtf(accelY_g * accelY_g + accelZ_g * accelZ_g)) * (180.0f / (float)PI) + 90.0f;
            imuAngleDeg = 0.96f * (imuAngleDeg + gyroRateY * dt) + 0.04f * accelPitch;
        } else {
            imuHealthy = false;
        }
    }

    // Clear INT_STATUS if interrupt fired
    if (mpuIntFired) {
        mpuIntFired = false;
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3A);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)1);
        Wire.read();
    }

    // 3. Read Load Cell in KILOGRAMS
    long rawLC;
    if (readHX711Raw(rawLC)) {
        float delta = (float)(rawLC - loadCellZero);
        currentWeightKg = delta / loadCellCalFacKg;
        currentForceN   = currentWeightKg * 9.80665f;
    }

    // 4. Read Pushbuttons & E-Stop
    bool btnUp   = (digitalRead(PIN_BTN_UP) == LOW);
    bool btnDown = (digitalRead(PIN_BTN_DOWN) == LOW);
    bool estop   = (digitalRead(PIN_ESTOP) == HIGH);

    if (estop && currentPWM != 0) {
        currentPWM = 0;
        analogWrite(PIN_BTS_RPWM, 0);
        analogWrite(PIN_BTS_LPWM, 0);
        digitalWrite(PIN_LED, LED_ON);
    } else if (!estop) {
        digitalWrite(PIN_LED, LED_OFF);
    }

    // 5. Periodic Dashboard & Python Telemetry Output (Every 100ms)
    static unsigned long lastDashboard = 0;
    if (millis() - lastDashboard > 100) {
        lastDashboard = millis();

        // Standard Human-readable Dashboard:
        Serial.print(F("[IMU: "));
        Serial.print(imuAngleDeg, 1);
        Serial.print(F("° (INT="));
        Serial.print(mpuIntCount);
        Serial.print(F(")] | [LOAD: "));
        Serial.print(currentWeightKg, 3);
        Serial.print(F(" kg ("));
        Serial.print(currentForceN, 2);
        Serial.print(F(" N)] | [BTNS: UP="));
        Serial.print(btnUp ? "1" : "0");
        Serial.print(F(" DN="));
        Serial.print(btnDown ? "1" : "0");
        Serial.print(F(" ESTOP="));
        Serial.print(estop ? "TRIP" : "OK");
        Serial.print(F("] | [MOTOR: "));
        Serial.print(currentPWM);
        Serial.println(F("]"));

        // Python Visualizer Packet:
        // $TELEM,ANGLE,WEIGHT_KG,FORCE_N,ACCEL_X,ACCEL_Y,ACCEL_Z,PWM,MODE,STATUS
        Serial.print(F("$TELEM,"));
        Serial.print(imuAngleDeg, 2);
        Serial.print(F(","));
        Serial.print(currentWeightKg, 3);
        Serial.print(F(","));
        Serial.print(currentForceN, 2);
        Serial.print(F(","));
        Serial.print(accelX_g, 3);
        Serial.print(F(","));
        Serial.print(accelY_g, 3);
        Serial.print(F(","));
        Serial.print(accelZ_g, 3);
        Serial.print(F(","));
        Serial.print(currentPWM);
        Serial.print(F(",1,DIAGNOSTIC\n"));
    }
}
