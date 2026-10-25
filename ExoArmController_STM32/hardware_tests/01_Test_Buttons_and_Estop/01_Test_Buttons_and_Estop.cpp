/*
 * ======================================================================================
 * HARDWARE TEST 01: Pushbuttons, E-Stop Switch & LED (STM32 Bluepill Edition)
 * Target: STM32F103C8T6 (Bluepill, 72MHz ARM Cortex-M3)
 * 
 * Hardware Wiring:
 *   - Button UP (Flexion)   : Pin PA4 -> Pushbutton -> GND
 *   - Button DOWN (Ext)     : Pin PA5 -> Pushbutton -> GND
 *   - E-Stop Switch         : Pin PA6 -> NC (Normally Closed) Switch -> GND
 *   - Diagnostic LED        : Built-in LED on Pin PC13 (Active LOW on Bluepill!)
 * ======================================================================================
 */

#include <Arduino.h>   // Serial, pinMode/digitalWrite/analogWrite, millis, F(), PI, constrain (STM32duino core)
#include <stdint.h>

const int PIN_BTN_UP   = PA4;
const int PIN_BTN_DOWN = PA5;
const int PIN_ESTOP    = PA6;
const int PIN_LED      = PC13;

// Active level definition for Bluepill PC13 LED
#define LED_ON  LOW
#define LED_OFF HIGH

void setup() {
    Serial.begin(115200);
    delay(1000); // Wait for STM32 USB / Serial initialization

    Serial.println(F("\n========================================================"));
    Serial.println(F(" TEST 01: PUSHBUTTONS & E-STOP (STM32 BLUEPILL)"));
    Serial.println(F("========================================================"));

    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LED_OFF);

    // Initial electrical state check
    delay(100);
    int upVal    = digitalRead(PIN_BTN_UP);
    int downVal  = digitalRead(PIN_BTN_DOWN);
    int estopVal = digitalRead(PIN_ESTOP);

    Serial.println(F("[STEP 1] Testing Internal Pullup Resistors (No buttons pressed)..."));
    if (upVal != HIGH) {
        Serial.println(F("  [FAIL] Pin PA4 (UP) is LOW! Check for short to GND or stuck switch."));
    } else {
        Serial.println(F("  [PASS] Pin PA4 (UP) is HIGH (Internal pullup active)."));
    }

    if (downVal != HIGH) {
        Serial.println(F("  [FAIL] Pin PA5 (DOWN) is LOW! Check for short to GND or stuck switch."));
    } else {
        Serial.println(F("  [PASS] Pin PA5 (DOWN) is HIGH (Internal pullup active)."));
    }

    Serial.print(F("  [INFO] Pin PA6 (ESTOP) initial state: "));
    Serial.println(estopVal == LOW ? F("CLOSED to GND (Safe / Armed)") : F("OPEN (Tripped)"));

    // LED Flash Test (Active LOW on Bluepill)
    Serial.println(F("\n[STEP 2] Testing Builtin LED (Pin PC13 - Active LOW)..."));
    for (int i = 0; i < 4; i++) {
        digitalWrite(PIN_LED, LED_ON);
        delay(120);
        digitalWrite(PIN_LED, LED_OFF);
        delay(120);
    }
    Serial.println(F("  [PASS] PC13 LED cycled. Verify visual green/blue blink on Bluepill board."));

    Serial.println(F("\n[STEP 3] Interactive Switch Monitor:"));
    Serial.println(F("  - Press Button UP (PA4)   -> Triggers [PRESSED]"));
    Serial.println(F("  - Press Button DOWN (PA5) -> Triggers [PRESSED]"));
    Serial.println(F("  - Toggle E-Stop (PA6)     -> Triggers [ESTOP TRIPPED]"));
    Serial.println(F("  - LED on PC13 lights up whenever any input is active."));
    Serial.println(F("--------------------------------------------------------\n"));
}

int lastUp = -1;
int lastDown = -1;
int lastEstop = -1;
unsigned long upCount = 0;
unsigned long downCount = 0;

void loop() {
    int upVal    = digitalRead(PIN_BTN_UP);
    int downVal  = digitalRead(PIN_BTN_DOWN);
    int estopVal = digitalRead(PIN_ESTOP);

    static unsigned long lastDebounce = 0;
    if (millis() - lastDebounce > 20) {
        lastDebounce = millis();

        if (upVal != lastUp) {
            lastUp = upVal;
            if (upVal == LOW) {
                upCount++;
                Serial.print(F(">>> BUTTON UP (PA4) PRESSED! Total count: "));
                Serial.println(upCount);
            } else {
                Serial.println(F("    BUTTON UP (PA4) Released."));
            }
        }

        if (downVal != lastDown) {
            lastDown = downVal;
            if (downVal == LOW) {
                downCount++;
                Serial.print(F(">>> BUTTON DOWN (PA5) PRESSED! Total count: "));
                Serial.println(downCount);
            } else {
                Serial.println(F("    BUTTON DOWN (PA5) Released."));
            }
        }

        if (estopVal != lastEstop) {
            lastEstop = estopVal;
            if (estopVal == HIGH) {
                Serial.println(F("!!! [ALERT] EMERGENCY STOP TRIPPED (PA6 Circuit Open) !!!"));
            } else {
                Serial.println(F("--- [INFO] EMERGENCY STOP ARMED (PA6 Closed to GND) ---"));
            }
        }
    }

    // Mirror input activity to LED (Active LOW)
    if (upVal == LOW || downVal == LOW || estopVal == HIGH) {
        digitalWrite(PIN_LED, LED_ON);
    } else {
        digitalWrite(PIN_LED, LED_OFF);
    }
}
