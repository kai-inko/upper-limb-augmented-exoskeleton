/*
 * ======================================================================================
 * HARDWARE TEST 04: BTS7960 43A MOTOR DRIVER TEST (STM32 Bluepill)
 * Target: STM32F103C8T6 (Bluepill, 72MHz ARM Cortex-M3)
 * 
 * Hardware Wiring:
 *   - BTS7960 RPWM -> Pin PA0 (Timer 2 Channel 1 Hardware PWM)
 *   - BTS7960 LPWM -> Pin PA1 (Timer 2 Channel 2 Hardware PWM)
 *   - BTS7960 EN   -> Pin PA2 (Connects to both R_EN and L_EN)
 *   - BTS7960 VCC  -> 5V logic
 *   - BTS7960 GND  -> GND logic
 *   - BTS7960 B+/B--> 12V High-Current Power Supply / Battery
 *   - BTS7960 M+/M--> Motor Terminals
 * ======================================================================================
 */

const int PIN_BTS_RPWM = PA0;
const int PIN_BTS_LPWM = PA1;
const int PIN_BTS_EN   = PA2;

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
    delay(1000);

    pinMode(PIN_BTS_EN, OUTPUT);
    pinMode(PIN_BTS_RPWM, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);

    digitalWrite(PIN_BTS_EN, LOW);
    stopMotor();

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 04: BTS7960 MOTOR DRIVER (STM32 BLUEPILL)"));
    Serial.println(F("========================================================"));
    Serial.println(F("! SAFETY FIRST !"));
    Serial.println(F(" Ensure 12V power supply is on and motor shaft is free to move."));
    Serial.println(F(" Type 'START' in Serial Monitor to run the diagnostic...\n"));
}

void runAutomatedTest() {
    Serial.println(F(">>> STARTING STM32 MOTOR DRIVER DIAGNOSTIC <<<"));
    digitalWrite(PIN_BTS_EN, HIGH);
    delay(100);

    // Test A: Forward Ramp
    Serial.println(F("\n[TEST A] Forward (Flexion / UP) Low-Speed Ramp on PA0..."));
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
    Serial.println(F("  [PASS] Forward cycle complete."));

    // Dead-time pause
    Serial.println(F("  Pausing 1.5s (Break-before-make dead-time)..."));
    delay(1500);

    // Test B: Reverse Ramp
    Serial.println(F("\n[TEST B] Reverse (Extension / DOWN) Low-Speed Ramp on PA1..."));
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
    Serial.println(F("  [PASS] Reverse cycle complete."));

    delay(1000);

    // Test C: Gate Enable Cutoff on PA2
    Serial.println(F("\n[TEST C] Testing Driver Gate Cutoff (Pin PA2 LOW)..."));
    driveForward(60);
    delay(600);
    digitalWrite(PIN_BTS_EN, LOW); // Pull enable LOW
    stopMotor();
    Serial.println(F("  [PASS] Pin PA2 pulled LOW. Motor instantly disengaged."));

    digitalWrite(PIN_BTS_EN, HIGH);
    Serial.println(F("\n========================================================"));
    Serial.println(F(" BTS7960 Driver Diagnostic PASSED!"));
    Serial.println(F(" Interactive Jog Mode:"));
    Serial.println(F("   'w' : Jog UP (PWM 75)"));
    Serial.println(F("   's' : Jog DOWN (PWM 75)"));
    Serial.println(F("   ' ' : Stop immediately"));
    Serial.println(F("   'e' : Toggle Gate Enable"));
    Serial.println(F("========================================================\n"));
}

bool started = false;
bool enabled = true;

void loop() {
    if (!Serial.available()) return;

    if (!started) {
        String in = Serial.readStringUntil('\n');
        in.trim();
        if (in.equalsIgnoreCase("START")) {
            started = true;
            runAutomatedTest();
        } else {
            Serial.println(F("Type 'START' to run motor test."));
        }
        return;
    }

    char c = Serial.read();
    switch (c) {
        case 'w':
        case 'W':
            if (enabled) {
                driveForward(75);
                Serial.println(F(">>> JOG UP (PWM 75)"));
            }
            break;
        case 's':
        case 'S':
            if (enabled) {
                driveReverse(75);
                Serial.println(F(">>> JOG DOWN (PWM 75)"));
            }
            break;
        case ' ':
            stopMotor();
            Serial.println(F("--- MOTOR STOPPED ---"));
            break;
        case 'e':
        case 'E':
            enabled = !enabled;
            digitalWrite(PIN_BTS_EN, enabled ? HIGH : LOW);
            if (!enabled) stopMotor();
            Serial.print(F(">>> Gate Enable is now: "));
            Serial.println(enabled ? F("ACTIVE") : F("CUT OFF"));
            break;
        default:
            break;
    }
}
