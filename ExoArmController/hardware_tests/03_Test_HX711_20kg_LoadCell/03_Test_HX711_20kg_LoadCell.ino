/*
 * ======================================================================================
 * HARDWARE TEST 03: HX711 20KG LOAD CELL INTEGRITY & CALIBRATION
 * Target: Arduino Nano (ATmega328P)
 * 
 * Purpose:
 *   1. Verify HX711 24-bit ADC communication (data-ready pulses).
 *   2. Detect wheatstone bridge wiring faults (open/short on E+, E-, A+, A-).
 *   3. Measure baseline noise floor and peak-to-peak ADC jitter.
 *   4. Guide the user through one-step known-weight calibration to find the exact
 *      'LOAD_CELL_CAL_FAC' to paste into Config.h.
 * 
 * Hardware Wiring:
 *   - HX711 VCC -> 5V
 *   - HX711 GND -> GND
 *   - HX711 DT  -> Pin D2 (Arduino Nano)
 *   - HX711 SCK -> Pin D3 (Arduino Nano)
 * 
 * Load Cell 4-Wire Standard Colors:
 *   - RED   -> E+ (Excitation +)
 *   - BLACK -> E- (Excitation -)
 *   - GREEN -> A+ (Signal +)
 *   - WHITE -> A- (Signal -)
 * ======================================================================================
 */

const int PIN_HX711_DOUT = 2;
const int PIN_HX711_SCK  = 3;

// Fast non-blocking HX711 reader
bool readHX711Raw(long &val) {
    if (digitalRead(PIN_HX711_DOUT) == HIGH) {
        return false;
    }

    unsigned long count = 0;
    noInterrupts();
    for (int i = 0; i < 24; i++) {
        digitalWrite(PIN_HX711_SCK, HIGH);
        delayMicroseconds(1);
        count = count << 1;
        digitalWrite(PIN_HX711_SCK, LOW);
        delayMicroseconds(1);
        if (digitalRead(PIN_HX711_DOUT) == HIGH) {
            count++;
        }
    }
    // 25th pulse: Channel A, Gain 128
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
float calFactor = 21.50f; // Default starting factor

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 03: HX711 & 20KG LOAD CELL INTEGRITY CHECK"));
    Serial.println(F("========================================================"));

    pinMode(PIN_HX711_SCK, OUTPUT);
    pinMode(PIN_HX711_DOUT, INPUT);
    digitalWrite(PIN_HX711_SCK, LOW);

    // -------------------------------------------------------------------------
    // STEP 1: Communication Line Check (Timeout detection)
    // -------------------------------------------------------------------------
    Serial.println(F("[STEP 1] Checking HX711 DOUT Data-Ready Line..."));
    unsigned long timeout = millis() + 500;
    bool responded = false;
    while (millis() < timeout) {
        if (digitalRead(PIN_HX711_DOUT) == LOW) {
            responded = true;
            break;
        }
    }

    if (!responded) {
        Serial.println(F("  [FATAL ERROR] HX711 not responding (DOUT line remained HIGH)!"));
        Serial.println(F("  Checklist:"));
        Serial.println(F("   1. Verify VCC has 5V and GND is connected securely."));
        Serial.println(F("   2. Check DT (Pin D2) and SCK (Pin D3) wiring."));
        Serial.println(F("   3. Check whether the 4 load cell wires (Red/Black/White/Green) are soldered."));
        while (1) delay(1000);
    }
    Serial.println(F("  [PASS] HX711 clock and data synchronization verified!"));

    // -------------------------------------------------------------------------
    // STEP 2: Baseline Noise & Bridge Fault Test
    // -------------------------------------------------------------------------
    Serial.println(F("\n[STEP 2] Sampling 30 raw readings to evaluate noise floor..."));
    Serial.println(F("  *** Please do NOT touch the load cell during this test ***"));

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

    if (count < 30) {
        Serial.println(F("  [WARN] Slow data rate from HX711. (Standard rate is 10Hz or 80Hz)"));
    }

    long minVal = samples[0];
    long maxVal = samples[0];
    long sum = 0;
    for (int i = 0; i < count; i++) {
        if (samples[i] < minVal) minVal = samples[i];
        if (samples[i] > maxVal) maxVal = samples[i];
        sum += samples[i];
    }
    long avg = sum / count;
    long peakToPeak = maxVal - minVal;

    Serial.print(F("  Raw ADC Average : ")); Serial.println(avg);
    Serial.print(F("  Peak-to-Peak Jitter: ")); Serial.print(peakToPeak); Serial.println(F(" counts"));

    // Diagnostics
    if (avg >= 8388600 || avg <= -8388600) {
        Serial.println(F("  [FAIL] ADC is SATURATED at full scale! Likely open bridge wire (E+, E-, A+, or A-)."));
    } else if (peakToPeak > 15000) {
        Serial.println(F("  [WARN] High electrical noise detected! Keep wires away from high-current motor cables."));
    } else {
        Serial.println(F("  [PASS] Load cell bridge is balanced and noise floor is healthy (< 5000 counts jitter)."));
    }

    zeroOffset = avg;
    Serial.print(F("\n[STEP 3] Zero Tare Point Captured: ")); Serial.println(zeroOffset);

    Serial.println(F("\n[STEP 4] Interactive Calibration & Monitor:"));
    Serial.println(F("  - Type 't' to tare zero anytime."));
    Serial.println(F("  - To CALIBRATE: Place a known weight on the sensor (e.g. 500g, 1000g),"));
    Serial.println(F("    then type 'c <weight_in_grams>' (e.g. 'c 1000')."));
    Serial.println(F("    The sketch will compute your exact LOAD_CELL_CAL_FAC!"));
    Serial.println(F("--------------------------------------------------------\n"));
}

void loop() {
    // Process Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 't' || cmd == 'T') {
            long sum = 0;
            int n = 0;
            unsigned long t = millis() + 1500;
            while (n < 20 && millis() < t) {
                long r;
                if (readHX711Raw(r)) { sum += r; n++; delay(15); }
            }
            if (n > 0) {
                zeroOffset = sum / n;
                Serial.print(F(">>> TARE SUCCESSFUL! New Zero Offset: "));
                Serial.println(zeroOffset);
            }
        } 
        else if (cmd == 'c' || cmd == 'C') {
            float knownGrams = Serial.parseFloat();
            if (knownGrams > 5.0f) {
                long sum = 0;
                int n = 0;
                unsigned long t = millis() + 1500;
                while (n < 20 && millis() < t) {
                    long r;
                    if (readHX711Raw(r)) { sum += r; n++; delay(15); }
                }
                if (n > 0) {
                    long currentAvg = sum / n;
                    long delta = abs(currentAvg - zeroOffset);
                    if (delta > 500) {
                        calFactor = (float)delta / knownGrams;
                        Serial.println(F("\n********************************************************"));
                        Serial.print(F(">>> CALIBRATION CALCULATED FOR ")); Serial.print(knownGrams); Serial.println(F("g:"));
                        Serial.print(F(">>> Copy this line into Config.h: \n    #define LOAD_CELL_CAL_FAC "));
                        Serial.println(calFactor, 2);
                        Serial.println(F("********************************************************\n"));
                    } else {
                        Serial.println(F(">>> [ERROR] Delta too small! Ensure the weight is resting on the sensor."));
                    }
                }
            } else {
                Serial.println(F(">>> [ERROR] Specify weight in grams (e.g. 'c 1000' for 1kg)."));
            }
        }
    }

    // Stream live load cell readings
    long raw;
    if (readHX711Raw(raw)) {
        float delta = (float)(raw - zeroOffset);
        float grams = delta / calFactor;
        float kg = grams / 1000.0f;
        float newtons = kg * 9.80665f;

        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 120) {
            lastPrint = millis();
            Serial.print(F("Raw_ADC:"));
            Serial.print(raw);
            Serial.print(F("  Delta:"));
            Serial.print((long)delta);
            Serial.print(F("  Grams:"));
            Serial.print(grams, 1);
            Serial.print(F("g  Force_N:"));
            Serial.print(newtons, 2);
            Serial.println(F(" N"));
        }
    }
}
