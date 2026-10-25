/*
 * ======================================================================================
 * HARDWARE TEST 01: Pushbuttons, E-Stop Switch & Diagnostic LED
 * Target: Arduino Nano (ATmega328P)
 * 
 * Purpose:
 *   Verify digital inputs, internal pullup integrity, switch contact bounce,
 *   and emergency stop circuit before connecting high-power motor stages.
 * 
 * Hardware Wiring:
 *   - Button UP (Flexion)   : Pin D7 -> Pushbutton -> GND
 *   - Button DOWN (Ext)     : Pin D8 -> Pushbutton -> GND
 *   - E-Stop Switch         : Pin D9 -> NC (Normally Closed) Switch -> GND
 *   - Diagnostic LED        : Built-in LED on Pin D13
 * ======================================================================================
 */

const int PIN_BTN_UP   = 7;
const int PIN_BTN_DOWN = 8;
const int PIN_ESTOP    = 9;
const int PIN_LED      = 13;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait for Serial Monitor

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 01: PUSHBUTTONS, E-STOP & LED INTEGRITY CHECK"));
    Serial.println(F("========================================================"));

    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    // Initial electrical state check (unpressed buttons should read HIGH due to pullups)
    delay(100);
    int upVal   = digitalRead(PIN_BTN_UP);
    int downVal = digitalRead(PIN_BTN_DOWN);
    int estopVal = digitalRead(PIN_ESTOP);

    Serial.println(F("[STEP 1] Testing Internal Pullup Resistors (No buttons pressed)..."));
    bool pullupPass = true;
    if (upVal != HIGH) {
        Serial.println(F("  [FAIL] Pin D7 (UP) is LOW! Check for short to GND or stuck switch."));
        pullupPass = false;
    } else {
        Serial.println(F("  [PASS] Pin D7 (UP) is HIGH (Pullup working)."));
    }

    if (downVal != HIGH) {
        Serial.println(F("  [FAIL] Pin D8 (DOWN) is LOW! Check for short to GND or stuck switch."));
        pullupPass = false;
    } else {
        Serial.println(F("  [PASS] Pin D8 (DOWN) is HIGH (Pullup working)."));
    }

    if (estopVal != LOW && estopVal != HIGH) {
        Serial.println(F("  [WARN] Pin D9 (ESTOP) floating state."));
    } else {
        Serial.print(F("  [INFO] Pin D9 (ESTOP) initial state: "));
        Serial.println(estopVal == LOW ? F("CLOSED to GND (Safe)") : F("OPEN (Tripped)"));
    }

    // LED Blink Test
    Serial.println(F("\n[STEP 2] Testing Builtin LED (Pin D13)..."));
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(150);
        digitalWrite(PIN_LED, LOW);
        delay(150);
    }
    Serial.println(F("  [PASS] LED cycled 3 times. Verify visual flash on Arduino Nano."));

    Serial.println(F("\n[STEP 3] Interactive Switch Monitor:"));
    Serial.println(F("  - Press Button UP (D7)   -> Should toggle to [PRESSED]"));
    Serial.println(F("  - Press Button DOWN (D8) -> Should toggle to [PRESSED]"));
    Serial.println(F("  - Press / Release E-STOP -> Should toggle status"));
    Serial.println(F("  - LED will illuminate whenever ANY input is triggered."));
    Serial.println(F("--------------------------------------------------------\n"));
}

int lastUp = -1;
int lastDown = -1;
int lastEstop = -1;
unsigned long upPressCount = 0;
unsigned long downPressCount = 0;

void loop() {
    int upVal    = digitalRead(PIN_BTN_UP);
    int downVal  = digitalRead(PIN_BTN_DOWN);
    int estopVal = digitalRead(PIN_ESTOP);

    // Track transitions with basic debounce
    static unsigned long lastDebounce = 0;
    if (millis() - lastDebounce > 20) {
        lastDebounce = millis();

        if (upVal != lastUp) {
            lastUp = upVal;
            if (upVal == LOW) {
                upPressCount++;
                Serial.print(F(">>> BUTTON UP (D7) PRESSED! Total count: "));
                Serial.println(upPressCount);
            } else {
                Serial.println(F("    BUTTON UP (D7) Released."));
            }
        }

        if (downVal != lastDown) {
            lastDown = downVal;
            if (downVal == LOW) {
                downPressCount++;
                Serial.print(F(">>> BUTTON DOWN (D8) PRESSED! Total count: "));
                Serial.println(downPressCount);
            } else {
                Serial.println(F("    BUTTON DOWN (D8) Released."));
            }
        }

        if (estopVal != lastEstop) {
            lastEstop = estopVal;
            if (estopVal == HIGH) {
                Serial.println(F("!!! [ALERT] EMERGENCY STOP TRIPPED (Circuit Open) !!!"));
            } else {
                Serial.println(F("--- [INFO] EMERGENCY STOP CLEARED (Circuit Closed to GND) ---"));
            }
        }
    }

    // Mirror active input to status LED
    if (upVal == LOW || downVal == LOW || estopVal == HIGH) {
        digitalWrite(PIN_LED, HIGH);
    } else {
        digitalWrite(PIN_LED, LOW);
    }
}
