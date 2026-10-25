/*
 * ======================================================================================
 * ExoArm Advanced Controller Firmware (STM32 Bluepill STM32F103C8T6 Edition)
 * 
 * Hardware Architecture:
 *  - Processor: STM32 Bluepill (STM32F103C8T6, 72 MHz ARM Cortex-M3)
 *  - Joint Angle IMU: GY-521 (MPU-6050 on I2C1 PB6/PB7)
 *  - Hardware Interrupt: MPU6050 INT pin -> Pin PB14 (EXTI Interlock for Angle Range)
 *  - Interaction Force: 20kg Strain Gauge / Load Cell + HX711 (Pins PB12/PB13)
 *  - Motor Driver: BTS7960 43A Dual Half-Bridge (RPWM=PA0, LPWM=PA1, EN=PA2)
 *  - Tactile Inputs: Button UP (PA4), Button DOWN (PA5), E-Stop (PA6), LED (PC13)
 * 
 * Features:
 *  - Hardware EXTI System Interruption when arm exceeds safe degree range
 *  - Exact manual calibration in KILOGRAMS (kg) with multi-point tare & span
 *  - Streams formatted 3D acceleration vector & force data for Python visualization
 *  - Real-time gravity compensation & closed-loop admittance assist
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

// IMU Instance
IMU_GY521 imu(MPU6050_I2C_ADDR);

// PID Controllers
PIDController torquePID(TORQUE_KP, TORQUE_KI, TORQUE_KD, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);
PIDController positionPID(POS_KP, POS_KI, POS_KD, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);

// Signal Filters
LowPassFilter forceFilter(0.25f);
SlewRateLimiter pwmSlew(PWM_SLEW_RATE);

// Safety Watchdog
SafetyWatchdog watchdog;

// System State Variables
ControlMode currentMode = MODE_GRAVITY_COMP_ASSIST;
float currentAngleDeg = 90.0f;
float angleSetpointDeg = 90.0f;

// Force & Mass Measurement (in kg and N)
float measuredWeightKg = 0.0f;
float interactionForceN = 0.0f;
float interactionTorqueNm = 0.0f;

float commandedPWM = 0.0f;
float actualOutputPWM = 0.0f;

// 20kg Load Cell Manual Calibration State
long  loadCellZeroOffset = 0;
float loadCellCalFactorKg = LOAD_CELL_CAL_FAC_KG;

// Loop Timing
unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;

// Manual Mode Speed Ramp Memory
float manualSpeedStep = 0.0f;

// =============================================================================
// MPU6050 HARDWARE INTERRUPT SERVICE ROUTINE (EXTI on PB14)
// =============================================================================
void mpuInterruptISR() {
    // Immediate hardware interruption when MPU6050 triggers
    watchdog.triggerHardwareInterruptFromISR(currentAngleDeg);
}

// =============================================================================
// HARDWARE DRIVER ROUTINES: 20KG LOAD CELL (HX711)
// =============================================================================

// Fast non-blocking HX711 24-bit reader for STM32
bool readHX711(long &rawResult) {
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
    // 25th pulse: Channel A, 128 gain
    digitalWrite(PIN_HX711_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_HX711_SCK, LOW);
    delayMicroseconds(1);
    interrupts();

    if (count & 0x800000) {
        count |= 0xFF000000;
    }
    rawResult = (long)count;
    return true;
}

// Tare / Zero the Load Cell
void tareLoadCell(int samples = 30) {
    long sum = 0;
    int validCount = 0;
    unsigned long timeout = millis() + 2000;
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
        Serial.print(F("[INFO] Load cell tared. Zero Offset: "));
        Serial.println(loadCellZeroOffset);
    }
}

// Read weight in KILOGRAMS and calculate interaction force (N) & torque (N*m)
void updateForceSensor() {
    long raw = 0;
    if (readHX711(raw)) {
        float delta = (float)(raw - loadCellZeroOffset);
        float rawKg = delta / loadCellCalFactorKg;
        rawKg = constrain(rawKg, -LOAD_CELL_MAX_KG, LOAD_CELL_MAX_KG);

        // Filter weight in kg
        measuredWeightKg = forceFilter.update(rawKg);

        // Compute physical Force in Newtons: F = m * g
        interactionForceN = measuredWeightKg * 9.80665f;

        // Compute Torque in N*m: tau = F * L
        interactionTorqueNm = interactionForceN * FOREARM_LENGTH_M;
    }
}

// =============================================================================
// HARDWARE DRIVER ROUTINES: BTS7960 ON STM32 BLUEPILL
// =============================================================================
void setMotorDriveBTS7960(float pwmValue) {
    float targetPWM = pwmValue;
    if (abs(targetPWM) < PWM_DEADBAND) {
        targetPWM = 0.0f;
    }

    targetPWM = pwmSlew.update(targetPWM);
    targetPWM = constrain(targetPWM, -PWM_MAX_LIMIT, PWM_MAX_LIMIT);
    actualOutputPWM = targetPWM;

    static int lastDirection = 0; // -1 = Extension (Down), 0 = Stop, 1 = Flexion (Up)

    if (targetPWM > 0) {
        // FLEXION (MOVE UP) -> Drive PA0 (RPWM)
        if (lastDirection == -1) {
            analogWrite(PIN_BTS_LPWM, 0);
            analogWrite(PIN_BTS_RPWM, 0);
            delayMicroseconds(60);
        }
        analogWrite(PIN_BTS_LPWM, 0);
        analogWrite(PIN_BTS_RPWM, (int)targetPWM);
        lastDirection = 1;
    } 
    else if (targetPWM < 0) {
        // EXTENSION (MOVE DOWN) -> Drive PA1 (LPWM)
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
        analogWrite(PIN_BTS_RPWM, 0);
        analogWrite(PIN_BTS_LPWM, 0);
        lastDirection = 0;
    }
}

void emergencyShutdown(const char* reason) {
    currentMode = MODE_EMERGENCY_STOP;
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);
    digitalWrite(PIN_BTS_EN, LOW); // Cut off driver gate

    digitalWrite(PIN_LED_STATUS, LED_ACTIVE_STATE); // Turn ON alert LED on PC13
    Serial.print(F("[EMERGENCY STOP] Motor Cutoff! Reason: "));
    Serial.println(reason);
}

// =============================================================================
// SERIAL INTERACTIVE CLI & CALIBRATION (IN KILOGRAMS)
// =============================================================================
void handleSerialCommands() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    switch (cmd) {
        case 'm':
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
        case 't': // Tare Zero
        case 'T':
            tareLoadCell();
            break;

        case 'c': // Exact Manual Calibration in kg
        case 'C': {
            float knownKg = Serial.parseFloat();
            if (knownKg > 0.05f) {
                long sum = 0;
                int n = 0;
                unsigned long t = millis() + 2000;
                while (n < 25 && millis() < t) {
                    long r;
                    if (readHX711(r)) { sum += r; n++; delay(15); }
                }
                if (n > 0) {
                    long currentAvg = sum / n;
                    long delta = abs(currentAvg - loadCellZeroOffset);
                    if (delta > 200) {
                        loadCellCalFactorKg = (float)delta / knownKg;
                        Serial.println(F("\n********************************************************"));
                        Serial.print(F(">>> CALIBRATED FOR EXACT WEIGHT: ")); Serial.print(knownKg, 3); Serial.println(F(" kg"));
                        Serial.print(F(">>> New Factor: ")); Serial.print(loadCellCalFactorKg, 2); Serial.println(F(" counts/kg"));
                        Serial.print(F(">>> Update Config.h: \n    #define LOAD_CELL_CAL_FAC_KG "));
                        Serial.println(loadCellCalFactorKg, 2);
                        Serial.println(F("********************************************************\n"));
                    } else {
                        Serial.println(F(">>> [ERROR] Delta too small! Place weight firmly on cuff."));
                    }
                }
            } else {
                Serial.println(F(">>> [ERROR] Usage: 'c <weight_in_kg>' (e.g. 'c 1.000' for 1kg)"));
            }
            break;
        }

        case 'p':
        case 'P': {
            float target = Serial.parseFloat();
            if (target >= ANGLE_MIN_DEG && target <= ANGLE_MAX_DEG) {
                angleSetpointDeg = target;
                Serial.print(F("[TARGET] Angle setpoint: "));
                Serial.println(angleSetpointDeg);
            }
            break;
        }

        case 'r':
        case 'R':
            watchdog.resetFault();
            imu.readAndClearInterruptStatus();
            digitalWrite(PIN_BTS_EN, HIGH);
            digitalWrite(PIN_LED_STATUS, LED_INACTIVE_STATE);
            currentMode = MODE_GRAVITY_COMP_ASSIST;
            torquePID.reset();
            positionPID.reset();
            Serial.println(F("[RESET] Faults cleared. BTS7960 re-enabled."));
            break;

        case '?':
            Serial.println(F("\n--- ExoArm Controller CLI (STM32 Bluepill) ---"));
            Serial.println(F(" m <0-3> : Set Mode (0:Assist, 1:GravComp, 2:PosHold, 3:Manual)"));
            Serial.println(F(" t       : Tare load cell zero"));
            Serial.println(F(" c <kg>  : Calibrate exact weight in kg (e.g. 'c 1.0')"));
            Serial.println(F(" p <deg> : Set Target Angle for Mode 2"));
            Serial.println(F(" r       : Reset emergency faults & clear MPU INT"));
            Serial.println(F(" ?       : Print this menu"));
            break;
        default:
            break;
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("======================================================"));
    Serial.println(F(" ExoArm Controller (STM32 Bluepill 72MHz Edition)"));
    Serial.println(F("======================================================"));

    // Configure BTS7960 Pins
    pinMode(PIN_BTS_RPWM, OUTPUT);
    pinMode(PIN_BTS_LPWM, OUTPUT);
    pinMode(PIN_BTS_EN, OUTPUT);
    analogWrite(PIN_BTS_RPWM, 0);
    analogWrite(PIN_BTS_LPWM, 0);
    digitalWrite(PIN_BTS_EN, HIGH);

    // Configure HX711 Pins
    pinMode(PIN_HX711_SCK, OUTPUT);
    pinMode(PIN_HX711_DOUT, INPUT);
    digitalWrite(PIN_HX711_SCK, LOW);

    // Pushbuttons & LED
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LED_INACTIVE_STATE);

    watchdog.init();

    // Initialize GY-521 IMU on PB6/PB7
    Serial.println(F("[INIT] Initializing GY-521 IMU on I2C1 (PB6/PB7)..."));
    if (imu.begin()) {
        Serial.println(F("[INIT] GY-521 Connected. Calibrating Gyro (hold arm still)..."));
        imu.calibrateGyro(150);

        // Configure MPU6050 Hardware INT Pin
        Serial.println(F("[INIT] Configuring MPU6050 INT Pin for System Interruption..."));
        imu.configureHardwareInterrupt(true, MPU_MOT_THR_VAL);

        // Attach EXTI interrupt on STM32 Pin PB14
        attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuInterruptISR, RISING);
        Serial.println(F("[INIT] Hardware Interrupt attached to STM32 Pin PB14."));
    } else {
        Serial.println(F("[ERROR] GY-521 Not Found! Check PB6/PB7 wiring."));
    }

    imu.update();
    currentAngleDeg = imu.getAngleDeg();
    angleSetpointDeg = currentAngleDeg;

    // Tare Load Cell
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

        // A. Read IMU
        bool imuSuccess = imu.update();
        if (imuSuccess) {
            currentAngleDeg = imu.getAngleDeg();
        }

        // B. Read Load Cell (Weight in kg & Force in N)
        updateForceSensor();

        // C. Check Safety Watchdog
        SafetyState safety = watchdog.checkSafety(imu.isHealthy(), currentAngleDeg, measuredWeightKg, commandedPWM);
        if (safety != SAFETY_OK && safety != SAFETY_ANGLE_LIMIT_MIN && safety != SAFETY_ANGLE_LIMIT_MAX) {
            emergencyShutdown(watchdog.getStateDescription());
            return;
        }

        // D. Calculate Motor PWM Based on Active Mode
        switch (currentMode) {
            // MODE 0: Standard Admittance Force Assist
            case MODE_FORCE_ASSIST: {
                if (abs(interactionForceN) < FORCE_DEADBAND_N) {
                    commandedPWM = 0.0f;
                    torquePID.reset();
                } else {
                    commandedPWM = -torquePID.compute(0.0f, interactionTorqueNm, dt);
                }
                break;
            }

            // MODE 1: Gravity-Compensated Assist (Default)
            case MODE_GRAVITY_COMP_ASSIST: {
                float angleRad = currentAngleDeg * (PI / 180.0f);
                float gravityTorqueNm = K_GRAVITY_COMP * sin(angleRad);

                if (abs(interactionForceN) < FORCE_DEADBAND_N) {
                    // Feedforward holding torque
                    commandedPWM = map(gravityTorqueNm * 100, 0, K_GRAVITY_COMP * 100, 0, 75);
                    torquePID.reset();
                } else {
                    float assistDemand = -torquePID.compute(0.0f, interactionTorqueNm, dt);
                    float feedforwardGravityPWM = map(gravityTorqueNm * 100, 0, K_GRAVITY_COMP * 100, 0, 75);
                    commandedPWM = assistDemand + feedforwardGravityPWM;
                }
                break;
            }

            // MODE 2: Angle Position Hold (Servo)
            case MODE_POSITION_HOLD: {
                commandedPWM = positionPID.compute(angleSetpointDeg, currentAngleDeg, dt);
                break;
            }

            // MODE 3: Manual Pushbutton Control
            case MODE_MANUAL_BUTTON: {
                bool btnUp = (digitalRead(PIN_BTN_UP) == LOW);
                bool btnDown = (digitalRead(PIN_BTN_DOWN) == LOW);

                if (btnUp && !btnDown) {
                    manualSpeedStep += 6.0f;
                } else if (btnDown && !btnUp) {
                    manualSpeedStep -= 6.0f;
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

        // E. Boundary Governor & Output Apply
        commandedPWM = watchdog.governOutputNearLimits(commandedPWM, currentAngleDeg);
        setMotorDriveBTS7960(commandedPWM);
    }

    // 3. Telemetry Stream for Python Force Vector Visualizer & Plotter (Every 80ms)
    if (now - lastTelemetryTime >= 80) {
        lastTelemetryTime = now;

        // Structured Telemetry Packet for Python Visualizer:
        // $TELEM,ANGLE,WEIGHT_KG,FORCE_N,ACCEL_X,ACCEL_Y,ACCEL_Z,PWM,MODE,STATUS
        Serial.print(F("$TELEM,"));
        Serial.print(currentAngleDeg, 2);
        Serial.print(F(","));
        Serial.print(measuredWeightKg, 3);
        Serial.print(F(","));
        Serial.print(interactionForceN, 2);
        Serial.print(F(","));
        Serial.print(imu.getAccelX_g(), 3);
        Serial.print(F(","));
        Serial.print(imu.getAccelY_g(), 3);
        Serial.print(F(","));
        Serial.print(imu.getAccelZ_g(), 3);
        Serial.print(F(","));
        Serial.print(actualOutputPWM, 0);
        Serial.print(F(","));
        Serial.print((int)currentMode);
        Serial.print(F(","));
        Serial.println(watchdog.getStateDescription());
    }
}
