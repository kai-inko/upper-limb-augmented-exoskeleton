#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =============================================================================
// PROCESSOR: STM32F103C8T6 (STM32 BLUEPILL - 72MHz ARM Cortex-M3)
// =============================================================================

// Motor Controller: BTS7960 43A High-Power Dual H-Bridge Module (IBT-2)
#define PIN_BTS_RPWM       PA0   // Right PWM (Flexion / UP) - Timer 2 Channel 1 Hardware PWM
#define PIN_BTS_LPWM       PA1   // Left PWM (Extension / DOWN) - Timer 2 Channel 2 Hardware PWM
#define PIN_BTS_EN         PA2   // Driver Gate Enable (Connects to both R_EN and L_EN)
#define PIN_BTS_RIS        PA3   // Optional: Right current sense analog input (-1 if unused)

// IMU: GY-521 (MPU-6050 6-Axis Gyroscope & Accelerometer)
// STM32 Hardware I2C1 Pins (5V Tolerant):
//   SCL -> Pin PB6
//   SDA -> Pin PB7
#define PIN_MPU_SCL        PB6   // Hardware I2C1 SCL
#define PIN_MPU_SDA        PB7   // Hardware I2C1 SDA
#define PIN_MPU_INT        PB14  // MPU6050 Hardware Interrupt Pin (5V Tolerant EXTI Pin)

// Interaction Force Sensor: HX711 24-bit ADC with 20kg Load Cell / Strain Gauge
// Connected to 5V-Tolerant GPIOs on STM32:
#define PIN_HX711_DOUT     PB12  // HX711 Data Output (5V Tolerant, EXTI capable)
#define PIN_HX711_SCK      PB13  // HX711 Serial Clock Output (5V Tolerant)

// Manual Tactile Interface & Safety Inputs
#define PIN_BTN_UP         PA4   // Manual Flexion switch (active LOW with internal pullup)
#define PIN_BTN_DOWN       PA5   // Manual Extension switch (active LOW with internal pullup)
#define PIN_ESTOP          PA6   // Emergency Stop switch (Normally Closed to GND)
#define PIN_LED_STATUS     PC13  // STM32 Bluepill Onboard LED (Note: Active LOW on Bluepill!)

// Active level definition for Bluepill PC13 LED
#define LED_ACTIVE_STATE   LOW   // Bluepill PC13 lights up on LOW
#define LED_INACTIVE_STATE HIGH

// =============================================================================
// LOAD CELL & STRAIN GAUGE CALIBRATION (IN KILOGRAMS)
// =============================================================================
#define FOREARM_LENGTH_M   0.22f // Distance from elbow pivot to load cell cuff (meters)
#define LOAD_CELL_MAX_KG   20.0f // Rated capacity: 20.000 kg

// Default counts per 1.000 kg (Can be tuned live via 'c <kg>' serial command)
// Calibration Formula: weight_kg = (raw_counts - zero_offset) / LOAD_CELL_CAL_FAC_KG
#define LOAD_CELL_CAL_FAC_KG 21500.0f // ADC counts per 1.0 kg mass

// =============================================================================
// JOINT ANGLE & HARDWARE MPU6050 INTERRUPT BOUNDARIES
// =============================================================================
// Allowable Joint Angle Range (Degrees)
// 0 deg = Full extension hanging down, 90 deg = Horizontal, 150 deg = Flexion
#define ANGLE_MIN_DEG      30.0f // Maximum safe extension limit
#define ANGLE_MAX_DEG      155.0f // Maximum safe flexion limit
#define ANGLE_SAFETY_MARGIN 8.0f // Buffer zone where motor starts to decelerate smoothly

// MPU6050 Hardware Interrupt Trigger Range
// When the arm reaches or exceeds these degree bounds, MPU6050 INT trips STM32 EXTI
#define MPU_INT_MIN_DEG    32.0f
#define MPU_INT_MAX_DEG    153.0f

// Motion detection threshold (1 LSB = 32mg, threshold for Wake-on-Motion interrupt)
#define MPU_MOT_THR_VAL    20    // Approx 0.64g rapid jerk threshold

// =============================================================================
// CONTROL TUNING PARAMETERS (STM32 72MHz 50Hz Loop)
// =============================================================================
#define TORQUE_KP          0.60f
#define TORQUE_KI          0.02f
#define TORQUE_KD          0.07f

#define POS_KP             3.20f
#define POS_KI             0.04f
#define POS_KD             0.38f

#define K_GRAVITY_COMP     2.10f // Peak gravitational torque in N*m at 90 degrees

// BTS7960 Limits
#define FORCE_DEADBAND_KG  0.150f // Deadband in kg (150 grams = ~1.47 N)
#define FORCE_DEADBAND_N   (FORCE_DEADBAND_KG * 9.80665f)
#define PWM_DEADBAND       18
#define PWM_MAX_LIMIT      250   // Maximum PWM duty cycle
#define PWM_SLEW_RATE      10    // Maximum PWM delta per 20ms cycle

// Control Loop Frequency
#define CONTROL_LOOP_HZ    50
#define CONTROL_PERIOD_MS  (1000 / CONTROL_LOOP_HZ)

// =============================================================================
// OPERATIONAL MODES
// =============================================================================
enum ControlMode {
    MODE_FORCE_ASSIST = 0,         // Admittance assist (zero interaction force following)
    MODE_GRAVITY_COMP_ASSIST = 1,  // Force assist + Sine gravity compensation
    MODE_POSITION_HOLD = 2,        // Closed-loop angle servo holding
    MODE_MANUAL_BUTTON = 3,        // Pushbutton speed ramping
    MODE_EMERGENCY_STOP = 4        // Safety lockdown
};

#endif // CONFIG_H
