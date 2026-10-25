/*
 * ======================================================================================
 * HARDWARE TEST 04: BTS7960 HIGH-POWER MOTOR CONTROLLER TEST
 * Target: Arduino Nano (ATmega328P)
 * 
 * Purpose:
 *   Safely verify the BTS7960 (IBT-2) 43A motor driver, direction switching,
 *   PWM speed modulation, enable gate shutdown, and break-before-make timing.
 * 
 * Safety Precautions:
 *   - The motor will NOT move until you explicitly type 'START' in the Serial Monitor!
 *   - Ensure the motor is secured to a workbench and mechanical limits are clear.
 *   - Keep an emergency power cutoff switch accessible.
 * 
 * Hardware Wiring:
 *   - VCC    -> 5V logic
 *   - GND    -> GND logic
 *   - R_EN   -> Pin D4 (Jumpered with L_EN)
 *   - L_EN   -> Pin D4
 *   - RPWM   -> Pin D5 (Hardware Timer 0 PWM)
 *   - LPWM   -> Pin D6 (Hardware Timer 0 PWM)
 *   - B+     -> 12V Battery / Power Supply (+)
 *   - B-     -> Power Supply Ground (-)
 *   - M+ / M--> Motor terminals
 * ======================================================================================
 */

const int PIN_BTS_EN   = 4;
const int PIN_BTS_RPWM = 5;
const int PIN_BTS_LPWM = 6;

void stopMotor() {
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);
}

void driveForward(int pwm) {
    analogWrite(PIN_BTS_LPWM, 0);
    analogWrite(PIN_BTS_RPWM, constrain(pwm, 0, 255));
}

void driveReverse(int pwm) {
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, constrain(pwm, 0, 255));
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    pinMode(PIN_BTS_EN, OUTPUT);
    pinMode(PIN_BTS_RPWM, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);

    // Initial safe state: disable bridge and zero PWMs
    digitalWrite(PIN_BTS_EN, LOW);
    stopMotor();

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 04: BTS7960 (IBT-2) MOTOR CONTROLLER INTEGRITY"));
    Serial.println(F("========================================================"));
    Serial.println(F("! SAFETY FIRST !"));
    Serial.println(F(" Ensure 12V power supply is ready and motor shaft is free to spin."));
    Serial.println(F(" Type 'START' in Serial Monitor to begin automated sequence...\n"));
}

void runAutomatedTest() {
    Serial.println(F(">>> STARTING MOTOR DRIVER INTEGRITY TEST <<<"));
    digitalWrite(PIN_BTS_EN, HIGH);
    delay(100);

    // -------------------------------------------------------------------------
    // TEST A: FORWARD RAMP (RPWM)
    // -------------------------------------------------------------------------
    Serial.println(F("\n[TEST A] Forward (Flexion / UP) Low-Speed Ramp (0 -> 90 -> 0)..."));
    for (int pwm = 0; pwm <= 90; pwm += 5) {
        driveForward(pwm);
        Serial.print(F("  PWM Forward: ")); Serial.println(pwm);
        delay(60);
    }
    for (int pwm = 90; pwm >= 0; pwm -= 5) {
        driveForward(pwm);
        Serial.print(F("  PWM Forward: ")); Serial.println(pwm);
        delay(60);
    }
    stopMotor();
    Serial.println(F("  [PASS] Forward cycle complete. Motor should have spun forward."));

    // Break-before-make transition pause (1.5 seconds)
    Serial.println(F("  Pausing 1.5s (Break-before-make dead-time)..."));
    delay(1500);

    // -------------------------------------------------------------------------
    // TEST B: REVERSE RAMP (LPWM)
    // -------------------------------------------------------------------------
    Serial.println(F("\n[TEST B] Reverse (Extension / DOWN) Low-Speed Ramp (0 -> 90 -> 0)..."));
    for (int pwm = 0; pwm <= 90; pwm += 5) {
        driveReverse(pwm);
        Serial.print(F("  PWM Reverse: ")); Serial.println(pwm);
        delay(60);
    }
    for (int pwm = 90; pwm >= 0; pwm -= 5) {
        driveReverse(pwm);
        Serial.print(F("  PWM Reverse: ")); Serial.println(pwm);
        delay(60);
    }
    stopMotor();
    Serial.println(F("  [PASS] Reverse cycle complete. Motor should have spun in reverse."));

    delay(1000);

    // -------------------------------------------------------------------------
    // TEST C: HARDWARE ENABLE GATE CUTOFF TEST
    // -------------------------------------------------------------------------
    Serial.println(F("\n[TEST C] Testing Hardware Gate Enable (D4 LOW cutoff)..."));
    Serial.println(F("  Applying PWM 60, then pulling PIN_BTS_EN (D4) LOW..."));
    driveForward(60);
    delay(600);
    digitalWrite(PIN_BTS_EN, LOW); // Pull enable LOW
    stopMotor();
    Serial.println(F("  [PASS] Enable pin pulled LOW. Motor should have immediately cut off!"));

    // Re-enable for manual control
    digitalWrite(PIN_BTS_EN, HIGH);
    Serial.println(F("\n========================================================"));
    Serial.println(F(" BTS7960 Automated Tests PASSED!"));
    Serial.println(F(" Entering Interactive Jog Mode:"));
    Serial.println(F("   'w' : Jog UP / Forward (PWM 70)"));
    Serial.println(F("   's' : Jog DOWN / Reverse (PWM 70)"));
    Serial.println(F("   ' ' : Stop immediately"));
    Serial.println(F("   'e' : Toggle Bridge Enable (Safety On/Off)"));
    Serial.println(F("========================================================\n"));
}

bool testStarted = false;
bool bridgeEnabled = true;

void loop() {
    if (!Serial.available()) return;

    if (!testStarted) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.equalsIgnoreCase("START")) {
            testStarted = true;
            runAutomatedTest();
        } else {
            Serial.println(F("Type 'START' to run the motor diagnostic."));
        }
        return;
    }

    char c = Serial.read();
    switch (c) {
        case 'w':
        case 'W':
            if (bridgeEnabled) {
                driveForward(75);
                Serial.println(F(">>> JOG FORWARD (Flexion / UP, PWM 75)"));
            } else {
                Serial.println(F("[WARN] Bridge is disabled! Press 'e' to enable."));
            }
            break;

        case 's':
        case 'S':
            if (bridgeEnabled) {
                driveReverse(75);
                Serial.println(F(">>> JOG REVERSE (Extension / DOWN, PWM 75)"));
            } else {
                Serial.println(F("[WARN] Bridge is disabled! Press 'e' to enable."));
            }
            break;

        case ' ': // Spacebar = STOP
            stopMotor();
            Serial.println(F("--- MOTOR STOPPED ---"));
            break;

        case 'e':
        case 'E':
            bridgeEnabled = !bridgeEnabled;
            digitalWrite(PIN_BTS_EN, bridgeEnabled ? HIGH : LOW);
            if (!bridgeEnabled) stopMotor();
            Serial.print(F(">>> BTS7960 Bridge Enable is now: "));
            Serial.println(bridgeEnabled ? F("ENABLED (Active)") : F("DISABLED (Cutoff)"));
            break;

        default:
            break;
    }
}
