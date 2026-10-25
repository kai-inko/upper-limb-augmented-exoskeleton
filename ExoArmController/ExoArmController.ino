/*
 * ======================================================================================
 * ExoArm Advanced Controller Firmware (Adapted for Arduino Nano + BTS7960 + GY-521 + 20kg Cell)
 * 
 * Hardware Architecture:
 *  - Processor: Arduino Nano (ATmega328P, 16 MHz)
 *  - Joint Angle IMU: GY-521 (MPU-6050 6-DOF Accelerometer & Gyroscope on I2C A4/A5)
 *  - Interaction Force: 20kg Parallel Beam Load Cell + HX711 24-bit ADC (Pins D2/D3)
 *  - Motor Controller: BTS7960 43A High-Power Dual H-Bridge Module (IBT-2 on Pins D4/D5/D6)
 *  - Manual Controls: Flexion/Extension Pushbuttons (Pins D7/D8) & E-Stop (Pin D9)
 * 
 * Operating Modes:
 *  - Mode 0: Admittance Force Assist (Zero-interaction torque following)
 *  - Mode 1: Gravity-Compensated Force Assist (Active sine model counteracting arm weight)
 *  - Mode 2: Joint Angle Position Hold (Closed-loop PID servo holding via IMU)
 *  - Mode 3: Manual Pushbutton Control (Smooth S-curve velocity ramping)
 * ======================================================================================
 */

#include "Config.h"
#include "PIDController.h"
#include "Filters.h"
#include "SafetyWatchdog.h"
#include "IMU_GY521.h"

// =============================================================================
// GLOBAL OBJECTS & STATE
// =============================================================================

// IMU Sensor Instance
IMU_GY521 imu(MPU6050_I2C_ADDR);

// PID Controllers
PIDController torquePID(TORQUE_KP, TORQUE_KI, TORQUE_KD, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);
PIDController positionPID(POS_KP, POS_KI, POS_KD, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);

// Signal Filters
LowPassFilter forceFilter(0.25f);       // Smooth load cell reading noise
SlewRateLimiter pwmSlew(PWM_SLEW_RATE); // Smooth motor acceleration ramp to protect gearbox

// Safety Watchdog
SafetyWatchdog watchdog;

// System State Variables
ControlMode currentMode = MODE_GRAVITY_COMP_ASSIST;
float currentAngleDeg = 90.0f;
float angleSetpointDeg = 90.0f;
float interactionForceN = 0.0f;
float interactionTorqueNm = 0.0f;
float commandedPWM = 0.0f;
float actualOutputPWM = 0.0f;

// 20kg Load Cell Calibration State
long loadCellZeroOffset = 0;

// Loop Timing
unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;

// Manual Mode Speed Ramp Memory
float manualSpeedStep = 0.0f;

// =============================================================================
// HARDWARE DRIVER ROUTINES: 20KG LOAD CELL (HX711)
// =============================================================================

// Fast, non-blocking / low-latency HX711 24-bit reader
bool readHX711(long &rawResult) {
    if (digitalRead(PIN_HX711_DOUT) == HIGH) {
        return false; // Conversion still in progress
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
    // 25th pulse sets Channel A, 128 gain for the next conversion
    digitalWrite(PIN_HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_HX711_SCK, LOW);
    delayMicroseconds(1);
    interrupts();

    // Sign extend 24-bit two's complement to 32-bit signed long
    if (count & 0x800000) {
        count |= 0xFF000000;
    }
    rawResult = (long)count;
    return true;
}

// Tare / Zero the Load Cell
void tareLoadCell(int samples = 25) {
    long sum = 0;
    int validCount = 0;
    unsigned long timeout = millis() + 1500;
    while (validCount < samples && millis() < timeout) {
        long raw = 0;
        if (readHX711(raw)) {
            sum += raw;
            validCount++;
            delay(10);
        }
    }
    if (validCount > 0) {
        loadCellZeroOffset = sum / validCount;
        Serial.print(F("[INFO] 20kg Load cell tared. Offset: "));
        Serial.println(loadCellZeroOffset);
    }
}

// Read interaction force (Newtons) from 20kg load cell and calculate torque (N*m)
void updateForceSensor() {
    long raw = 0;
    if (readHX711(raw)) {
        // Calibration for 20kg load cell: convert delta counts to grams, then kg, then Newtons
        float deltaCounts = (float)(raw - loadCellZeroOffset);
        float calibratedGrams = deltaCounts / LOAD_CELL_CAL_FAC;
        float calibratedKg = calibratedGrams / 1000.0f;
        calibratedKg = constrain(calibratedKg, -LOAD_CELL_MAX_KG, LOAD_CELL_MAX_KG);

        float rawForceN = calibratedKg * 9.80665f;
        interactionForceN = forceFilter.update(rawForceN);
        interactionTorqueNm = interactionForceN * FOREARM_LENGTH_M;
    }
}

// =============================================================================
// HARDWARE DRIVER ROUTINES: BTS7960 HIGH-POWER MOTOR DRIVER
// =============================================================================

// Drives the BTS7960 H-Bridge using separate RPWM and LPWM lines with break-before-make protection
void setMotorDriveBTS7960(float pwmValue) {
    // 1. Enforce deadband to eliminate steady-state hum/whine
    float targetPWM = pwmValue;
    if (abs(targetPWM) < PWM_DEADBAND) {
        targetPWM = 0.0f;
    }

    // 2. Slew-rate limiting to prevent mechanical shock to gearbox & motor mounts
    targetPWM = pwmSlew.update(targetPWM);

    // 3. Absolute clamp to max limit
    targetPWM = constrain(targetPWM, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);
    actualOutputPWM = targetPWM;

    static int lastDirection = 0; // -1 = Extension (Down), 0 = Stop, 1 = Flexion (Up)

    if (targetPWM > 0) {
        // FLEXION (MOVE UP) -> Drive RPWM pin
        if (lastDirection == -1) {
            // Break before make: shut off both gates briefly to prevent shoot-through
            analogWrite(PIN_BTS_LPWM, 0);
            analogWrite(PIN_BTS_RPWM, 0);
            delayMicroseconds(60);
        }
        analogWrite(PIN_BTS_LPWM, 0);
        analogWrite(PIN_BTS_RPWM, (int)targetPWM);
        lastDirection = 1;
    } 
    else if (targetPWM < 0) {
        // EXTENSION (MOVE DOWN) -> Drive LPWM pin
        if (lastDirection == 1) {
            analogWrite(PIN_BTS_RPWM, 0);
            analogWrite(PIN_BTS_LPWM, 0);
            delayMicroseconds(60);
        }
        analogWrite(PIN_BTS_RPWM, 0);
        analogWrite(PIN_BTS_LPWM, (int)(-targetPWM));
        lastDirection = -1;
    } 
    else {
        // STOP / ACTIVE COAST
        analogWrite(PIN_BTS_RPWM, 0);
        analogWrite(PIN_BTS_LPWM, 0);
        lastDirection = 0;
    }
}

// Emergency stop shutdown
void emergencyShutdown(const char* reason) {
    currentMode = MODE_EMERGENCY_STOP;
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);
    digitalWrite(PIN_BTS_EN, LOW); // Disable driver bridge completely

    digitalWrite(PIN_LED_STATUS, HIGH); // Turn on warning LED
    Serial.print(F("[EMERGENCY STOP] Motor Cutoff! Reason: "));
    Serial.println(reason);
}

// =============================================================================
// SERIAL COMMAND INTERPRETER (Interactive Debug & Tuning)
// =============================================================================
void handleSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    switch (cmd) {
        case 'm': // Change Mode
        case 'M': {
            int newMode = Serial.parseInt();
            if (newMode >= 0 && newMode <= 3) {
                currentMode = (ControlMode)newMode;
                torquePID.reset();
                positionPID.reset();
                pwmSlew.reset(0);
                manualSpeedStep = 0;
                angleSetpointDeg = currentAngleDeg;
                Serial.print(F("[MODE] Switched to mode: "));
                Serial.println(currentMode);
            }
            break;
        }
        case 't': // Tare Load Cell
        case 'T':
            tareLoadCell();
            break;

        case 'p': // Set Target Position (Angle Deg)
        case 'P': {
            float target = Serial.parseFloat();
            if (target >= ANGLE_MIN_DEG && target <= ANGLE_MAX_DEG) {
                angleSetpointDeg = target;
                Serial.print(F("[TARGET] Angle setpoint: "));
                Serial.println(angleSetpointDeg);
            } else {
                Serial.println(F("[ERROR] Target angle out of safe limits!"));
            }
            break;
        }
        case 'c': // Calibrate IMU Gyro Zero
        case 'C':
            Serial.println(F("[IMU] Calibrating gyro bias... Hold arm completely still!"));
            imu.calibrateGyro(200);
            Serial.println(F("[IMU] Gyro bias calibrated."));
            break;

        case 'r': // Reset Emergency Fault
        case 'R':
            watchdog.resetFault();
            digitalWrite(PIN_BTS_EN, HIGH); // Re-enable BTS7960 bridge
            digitalWrite(PIN_LED_STATUS, LOW);
            currentMode = MODE_GRAVITY_COMP_ASSIST;
            torquePID.reset();
            positionPID.reset();
            Serial.println(F("[RESET] Faults cleared. Resumed operation."));
            break;

        case '?': // Help menu
            Serial.println(F("\n--- ExoArm Controller CLI (Nano + BTS7960 + GY-521) ---"));
            Serial.println(F(" m <0-3> : Set Mode (0:Assist, 1:GravComp, 2:PosHold, 3:Manual)"));
            Serial.println(F(" t       : Tare 20kg load cell"));
            Serial.println(F(" c       : Re-calibrate IMU Gyro zero bias"));
            Serial.println(F(" p <deg> : Set Target Angle for Mode 2 (e.g. p 90)"));
            Serial.println(F(" r       : Reset faults and re-enable BTS7960"));
            Serial.println(F(" ?       : Print this help"));
            break;
        default:
            break;
    }
}

// =============================================================================
// ARDUINO SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println(F("======================================================"));
    Serial.println(F(" ExoArm Controller (Nano + BTS7960 + GY-521 + 20kg Cell)"));
    Serial.println(F("======================================================"));

    // Configure BTS7960 Motor Driver Pins
    pinMode(PIN_BTS_RPWM, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);
    pinMode(PIN_BTS_EN, OUTPUT);
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);
    digitalWrite(PIN_BTS_EN, HIGH); // Enable H-Bridge gates

    // Configure HX711 Load Cell Pins
    pinMode(PIN_HX711_SCK, OUTPUT);
    pinMode(PIN_HX711_DOUT, INPUT);
    digitalWrite(PIN_HX711_SCK, LOW);

    // Configure Pushbuttons & Indicators
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    watchdog.init();

    // Initialize GY-521 IMU via I2C (Pins A4/A5)
    Serial.println(F("[INIT] Initializing GY-521 IMU (MPU6050)..."));
    if (imu.begin()) {
        Serial.println(F("[INIT] GY-521 Connected. Calibrating Gyro (hold arm still)..."));
        imu.calibrateGyro(150);
        Serial.println(F("[INIT] GY-521 Ready!"));
    } else {
        Serial.println(F("[ERROR] GY-521 Not Found! Check SDA(A4)/SCL(A5) wiring."));
    }

    // Read initial angle from IMU
    imu.update();
    currentAngleDeg = imu.getAngleDeg();
    angleSetpointDeg = currentAngleDeg;

    // Tare 20kg Load Cell
    Serial.println(F("[INIT] Taring 20kg Load Cell... Keep arm relaxed!"));
    tareLoadCell(30);

    lastControlTime = millis();
    lastTelemetryTime = millis();
    Serial.println(F("[READY] System Operational. Type '?' for command menu."));
}

// =============================================================================
// MAIN CONTROL LOOP (Deterministic 50Hz Period)
// =============================================================================
void loop() {
    unsigned long now = millis();

    // 1. Process Serial Commands
    handleSerialCommands();

    // 2. Periodic Control Cycle (50 Hz / 20ms)
    if (now - lastControlTime >= CONTROL_PERIOD_MS) {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        // A. Read Sensors: IMU Joint Angle & 20kg Load Cell
        bool imuSuccess = imu.update();
        if (imuSuccess) {
            currentAngleDeg = imu.getAngleDeg();
        }
        updateForceSensor();

        // B. Check Safety Watchdog
        SafetyState safety = watchdog.checkSafety(imu.isHealthy(), currentAngleDeg, interactionForceN, commandedPWM);
        if (safety == SAFETY_ESTOP_TRIGGERED || 
            safety == SAFETY_IMU_SENSOR_FAULT || 
            safety == SAFETY_MOTOR_STALL_DETECTED ||
            safety == SAFETY_LOAD_CELL_OVERLOAD) {
            emergencyShutdown(watchdog.getStateDescription());
            return;
        }

        // C. Calculate Motor Control Value Based on Active Mode
        switch (currentMode) {
            // MODE 0: Standard Admittance Force Assist
            case MODE_FORCE_ASSIST: {
                if (abs(interactionForceN) < FORCE_DEADBAND_N) {
                    commandedPWM = 0.0f;
                    torquePID.reset();
                } else {
                    // Motor direction: Lift when interaction force is positive (user pulling up)
                    commandedPWM = -torquePID.compute(0.0f, interactionTorqueNm, dt);
                }
                break;
            }

            // MODE 1: Gravity-Compensated Assist (Default)
            case MODE_GRAVITY_COMP_ASSIST: {
                // Gravity model: peak holding torque needed when forearm is horizontal (90 deg)
                float angleRad = currentAngleDeg * (PI / 180.0f);
                float gravityTorqueNm = K_GRAVITY_COMP * sin(angleRad);

                if (abs(interactionForceN) < FORCE_DEADBAND_N) {
                    // User is relaxed: provide feedforward holding torque so the arm holds position against gravity
                    commandedPWM = map(gravityTorqueNm * 100, 0, K_GRAVITY_COMP * 100, 0, 75);
                    torquePID.reset();
                } else {
                    float assistDemand = -torquePID.compute(0.0f, interactionTorqueNm, dt);
                    float feedforwardGravityPWM = map(gravityTorqueNm * 100, 0, K_GRAVITY_COMP * 100, 0, 75);
                    commandedPWM = assistDemand + feedforwardGravityPWM;
                }
                break;
            }

            // MODE 2: Angle Position Hold (Closed-Loop Servo)
            case MODE_POSITION_HOLD: {
                commandedPWM = positionPID.compute(angleSetpointDeg, currentAngleDeg, dt);
                break;
            }

            // MODE 3: Manual Pushbutton Control (V1 style with S-curve ramping)
            case MODE_MANUAL_BUTTON: {
                bool btnUp = (digitalRead(PIN_BTN_UP) == LOW);
                bool btnDown = (digitalRead(PIN_BTN_DOWN) == LOW);

                if (btnUp && !btnDown) {
                    manualSpeedStep += 6.0f; // Ramp up speed
                } else if (btnDown && !btnUp) {
                    manualSpeedStep -= 6.0f; // Ramp down speed
                } else {
                    if (manualSpeedStep > 0) manualSpeedStep -= 12.0f;
                    if (manualSpeedStep < 0) manualSpeedStep += 12.0f;
                    if (abs(manualSpeedStep) < 12.0f) manualSpeedStep = 0.0f;
                }
                manualSpeedStep = constrain(manualSpeedStep, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);
                commandedPWM = manualSpeedStep;
                break;
            }

            case MODE_EMERGENCY_STOP:
            default:
                commandedPWM = 0.0f;
                break;
        }

        // D. Boundary Governor: Smoothly slow down when nearing joint limits
        commandedPWM = watchdog.governOutputNearLimits(commandedPWM, currentAngleDeg);

        // E. Apply Output to BTS7960 Motor Driver
        setMotorDriveBTS7960(commandedPWM);
    }

    // 3. Telemetry Stream for Serial Plotter / Data Logging (Every 100ms)
    if (now - lastTelemetryTime >= 100) {
        lastTelemetryTime = now;
        Serial.print(F("Angle:"));
        Serial.print(currentAngleDeg, 1);
        Serial.print(F(",Target:"));
        Serial.print(angleSetpointDeg, 1);
        Serial.print(F(",ForceN:"));
        Serial.print(interactionForceN, 2);
        Serial.print(F(",PWM:"));
        Serial.print(actualOutputPWM, 0);
        Serial.print(F(",Mode:"));
        Serial.print((int)currentMode);
        Serial.print(F(",Status:"));
        Serial.println(watchdog.getStateDescription());
    }
}
