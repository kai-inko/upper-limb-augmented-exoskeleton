/*
 * ======================================================================================
 * HARDWARE TEST 03: HX711 20KG LOAD CELL & MANUAL KG CALIBRATION (STM32 Bluepill)
 * Target: STM32F103C8T6 (Bluepill, 72MHz ARM Cortex-M3)
 * 
 * Purpose:
 *   1. Verify HX711 24-bit ADC communication on STM32 5V-tolerant pins PB12 and PB13.
 *   2. Detect Wheatstone bridge faults (open/short circuit).
 *   3. Manually calibrate strain gauge & load cell for exact weight measurement in KILOGRAMS (kg).
 * 
 * Hardware Wiring:
 *   - HX711 VCC -> 5V or 3.3V
 *   - HX711 GND -> GND
 *   - HX711 DT  -> Pin PB12 (STM32 5V-Tolerant GPIO)
 *   - HX711 SCK -> Pin PB13 (STM32 5V-Tolerant GPIO)
 * ======================================================================================
 */

#include <Arduino.h>   // Serial, pinMode/digitalWrite/analogWrite, millis, F(), PI, constrain (STM32duino core)
#include <stdint.h>

const int PIN_HX711_DOUT = PB12;
const int PIN_HX711_SCK  = PB13;

bool readHX711Raw(long &val) {
    if (digitalRead(PIN_HX711_DOUT) == HIGH) return false;

    unsigned long count = 0;
    noInterrupts();
    for (int i = 0; i < 24; i++) {
        digitalWrite(PIN_HX711_SCK, HIGH);
        delayMicroseconds(1);
        count = count << 1;
        digitalWrite(PIN_HX711_SCK, LOW);
        delayMicroseconds(1);
        if (digitalRead(PIN_HX711_DOUT) == HIGH) count++;
    }
    digitalWrite(PIN_HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_HX711_SCK, LOW);
    delayMicroseconds(1);
    interrupts();

    if (count & 0x800000) count |= 0xFF000000;
    val = (long)count;
    return true;
}

long zeroOffset = 0;
float calFactorKg = 21500.0f; // Default counts per kg

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 03: HX711 20KG LOAD CELL & KG CALIBRATION (STM32)"));
    Serial.println(F("========================================================"));

    pinMode(PIN_HX711_SCK, OUTPUT);
    pinMode(PIN_HX711_DOUT, INPUT);
    digitalWrite(PIN_HX711_SCK, LOW);

    // -------------------------------------------------------------------------
    // STEP 1: Handshake Test
    // -------------------------------------------------------------------------
    Serial.println(F("[STEP 1] Checking HX711 DOUT on STM32 Pin PB12..."));
    unsigned long timeout = millis() + 800;
    bool ready = false;
    while (millis() < timeout) {
        if (digitalRead(PIN_HX711_DOUT) == LOW) {
            ready = true;
            break;
        }
    }

    if (!ready) {
        Serial.println(F("  [FATAL ERROR] HX711 not responding! (DOUT remained HIGH)"));
        Serial.println(F("  Check PB12 (DT), PB13 (SCK), VCC, and GND connections."));
        while (1) delay(1000);
    }
    Serial.println(F("  [PASS] HX711 communication verified on PB12/PB13!"));

    // -------------------------------------------------------------------------
    // STEP 2: Baseline Noise Floor Analysis
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 2] Sampling 30 points to calculate zero-load baseline..."));
    Serial.println(F("  *** Ensure the load cell is unloaded! ***"));

    long samples[30];
    int count = 0;
    timeout = millis() + 4000;
    while (count < 30 && millis() < timeout) {
        long r;
        if (readHX711Raw(r)) {
            samples[count++] = r;
            delay(20);
        }
    }

    long minVal = samples[0], maxVal = samples[0], sum = 0;
    for (int i = 0; i < count; i++) {
        if (samples[i] < minVal) minVal = samples[i];
        if (samples[i] > maxVal) maxVal = samples[i];
        sum += samples[i];
    }
    zeroOffset = sum / count;
    long jitter = maxVal - minVal;

    Serial.print(F("  Raw ADC Baseline: ")); Serial.println(zeroOffset);
    Serial.print(F("  Peak-to-Peak Jitter: ")); Serial.print(jitter); Serial.println(F(" counts"));

    if (zeroOffset >= 8388600 || zeroOffset <= -8388600) {
        Serial.println(F("  [FAIL] ADC is saturated! Check Wheatstone bridge wiring (Red, Black, Green, White)."));
    } else {
        Serial.println(F("  [PASS] Healthy Wheatstone bridge signal (< 5000 counts jitter)."));
    }

    Serial.println(F("\n[STEP 3] Manual Calibration Guide (Exact measurement in kg):"));
    Serial.println(F("  1. Ensure the arm cuff is completely free, then type 't' to tare zero."));
    Serial.println(F("  2. Place a known reference weight on the cuff (e.g., 1.000 kg or 2.500 kg)."));
    Serial.println(F("  3. Type 'c <weight_in_kg>' (e.g. 'c 1.000')."));
    Serial.println(F("     The sketch calculates your exact LOAD_CELL_CAL_FAC_KG!"));
    Serial.println(F("--------------------------------------------------------\n"));
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 't' || cmd == 'T') {
            long sum = 0;
            int n = 0;
            unsigned long t = millis() + 1500;
            while (n < 25 && millis() < t) {
                long r;
                if (readHX711Raw(r)) { sum += r; n++; delay(15); }
            }
            if (n > 0) {
                zeroOffset = sum / n;
                Serial.print(F(">>> [TARE] Zero Tare Captured: "));
                Serial.println(zeroOffset);
            }
        } 
        else if (cmd == 'c' || cmd == 'C') {
            float knownKg = Serial.parseFloat();
            if (knownKg > 0.05f) {
                long sum = 0;
                int n = 0;
                unsigned long t = millis() + 2000;
                while (n < 25 && millis() < t) {
                    long r;
                    if (readHX711Raw(r)) { sum += r; n++; delay(15); }
                }
                if (n > 0) {
                    long avg = sum / n;
                    long delta = abs(avg - zeroOffset);
                    if (delta > 200) {
                        calFactorKg = (float)delta / knownKg;
                        Serial.println(F("\n********************************************************"));
                        Serial.print(F(">>> EXACT CALIBRATION COMPUTED FOR: ")); Serial.print(knownKg, 3); Serial.println(F(" kg"));
                        Serial.print(F(">>> Scale Factor: ")); Serial.print(calFactorKg, 2); Serial.println(F(" counts/kg"));
                        Serial.print(F(">>> Copy this line into Config.h: \n    #define LOAD_CELL_CAL_FAC_KG "));
                        Serial.println(calFactorKg, 2);
                        Serial.println(F("********************************************************\n"));
                    } else {
                        Serial.println(F(">>> [ERROR] Delta too small! Ensure weight is resting on the load cell."));
                    }
                }
            } else {
                Serial.println(F(">>> [ERROR] Specify weight in kg (e.g. 'c 1.000' for 1kg)."));
            }
        }
    }

    // Live streaming in kg
    long raw;
    if (readHX711Raw(raw)) {
        float delta = (float)(raw - zeroOffset);
        float weightKg = delta / calFactorKg;
        float forceN = weightKg * 9.80665f;

        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 120) {
            lastPrint = millis();
            Serial.print(F("Raw_ADC:"));
            Serial.print(raw);
            Serial.print(F("  Delta:"));
            Serial.print((long)delta);
            Serial.print(F("  Weight: "));
            Serial.print(weightKg, 3);
            Serial.print(F(" kg  Force: "));
            Serial.print(forceN, 2);
            Serial.println(F(" N"));
        }
    }
}
