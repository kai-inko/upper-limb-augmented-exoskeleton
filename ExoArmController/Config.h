#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// PROCESSOR: ARDUINO NANO (ATmega328P) PIN CONFIGURATION
// =============================================================================

// Motor Controller: BTS7960 High-Power 43A H-Bridge Module (IBT-2)
#define PIN_BTS_RPWM       5     // Right PWM (Flexion / Move UP) - Hardware PWM on Nano
#define PIN_BTS_LPWM       6     // Left PWM (Extension / Move DOWN) - Hardware PWM on Nano
#define PIN_BTS_EN         4     // Bridge Enable (Connects to both R_EN and L_EN)
#define PIN_BTS_RIS        A0    // Optional: R_IS Right Current Sense analog input (-1 if unused)
#define PIN_BTS_LIS        A1    // Optional: L_IS Left Current Sense analog input (-1 if unused)

// IMU: GY-521 (MPU-6050 6-Axis Gyroscope & Accelerometer)
// Fixed Hardware I2C Pins on Arduino Nano:
//   SDA -> Pin A4
//   SCL -> Pin A5
#define USE_IMU_GY521      true  // Set to true to use GY-521 IMU for joint angle feedback
#define PIN_AUX_POT        A2    // Optional auxiliary backup potentiometer (-1 if unused)

// Interaction Force Sensor: HX711 24-bit ADC with 20kg Bar Load Cell
#define PIN_HX711_DOUT     2     // HX711 Data Output (Hardware Interrupt INT0 on Nano)
#define PIN_HX711_SCK      3     // HX711 Serial Clock Output

// Manual Tactile Interface & Safety Inputs
#define PIN_BTN_UP         7     // Pushbutton for manual Flexion (Up) - active LOW with pullup
#define PIN_BTN_DOWN       8     // Pushbutton for manual Extension (Down) - active LOW with pullup
#define PIN_ESTOP          9     // Hardware Emergency Stop switch - active LOW with pullup
#define PIN_LED_STATUS     13    // Arduino Nano Onboard Diagnostic LED

// =============================================================================
// PHYSICAL & SENSOR CALIBRATION (20kg Load Cell & Exoskeleton Geometry)
// =============================================================================
#define FOREARM_LENGTH_M   0.22f // Distance from elbow pivot to load cell cuff (meters)

// 20kg Load Cell Calibration Parameters
#define LOAD_CELL_MAX_KG   20.0f // Maximum capacity: 20kg (approx 196.1 N)
#define LOAD_CELL_CAL_FAC  21.50f // Ticks/calibration factor per gram for typical 20kg cell

// Joint Angle Soft Limits (Degrees)
// 0 deg = Full extension (straight down), 90 deg = Horizontal, 150 deg = Flexed upward
#define ANGLE_MIN_DEG      30.0f // Maximum safe extension limit (prevent hyperextension)
#define ANGLE_MAX_DEG      155.0f // Maximum safe flexion limit
#define ANGLE_SAFETY_MARGIN 8.0f // Deceleration buffer before hard stop

// Backup Potentiometer Calibration (if used)
#define POT_ADC_MIN        130
#define POT_ADC_MAX        980

// =============================================================================
// CONTROL TUNING PARAMETERS (Optimized for BTS7960 + 20kg Sensor)
// =============================================================================

// Torque / Admittance PID Gains (Mode: User Force Following)
#define TORQUE_KP          0.55f
#define TORQUE_KI          0.02f
#define TORQUE_KD          0.06f

// Position Hold PID Gains (Mode: Setpoint Angle Holding)
#define POS_KP             3.00f
#define POS_KI             0.04f
#define POS_KD             0.35f

// Gravity Compensation Constant (Peak gravity torque in N*m when forearm is horizontal)
#define K_GRAVITY_COMP     2.10f // In N*m (adjust according to wearer + harness weight)

// Deadband & Limits for BTS7960
#define FORCE_DEADBAND_N   2.0f  // Force deadband (Newtons) to eliminate drift / resting jitter
#define PWM_DEADBAND       20    // Minimum PWM to overcome motor & worm-gear static friction
#define PWM_MAX_LIMIT      250   // Maximum allowable motor PWM clamp (0-255)
#define PWM_SLEW_RATE      10    // Maximum change in PWM per control cycle (smooth acceleration)

// Loop Timing
#define CONTROL_LOOP_HZ    50    // Control loop frequency (50 Hz = 20ms period)
#define CONTROL_PERIOD_MS  (1000 / CONTROL_LOOP_HZ)

// =============================================================================
// OPERATIONAL MODES
// =============================================================================
enum ControlMode {
    MODE_FORCE_ASSIST = 0,         // Closed-loop admittance (zero-effort weightless feel)
    MODE_GRAVITY_COMP_ASSIST = 1,  // Force assist + Sine-wave active gravity compensation
    MODE_POSITION_HOLD = 2,        // Servo hold at commanded joint angle
    MODE_MANUAL_BUTTON = 3,        // Pushbutton speed ramp control (V1 style)
    MODE_EMERGENCY_STOP = 4        // Safety lockdown
};

#endif // CONFIG_H
