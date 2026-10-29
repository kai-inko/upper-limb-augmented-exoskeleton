#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx_hal.h"

// =====================================================================
//  SINGLE SOURCE OF TRUTH FOR PINS, TIMING, LIMITS AND TUNING VALUES
//  Target: STM32F411CEU6 "Black Pill" (100 MHz Cortex-M4F, 25 MHz HSE)
// =====================================================================

// ---------------- Timing ----------------
#define CONTROL_LOOP_PERIOD_MS   10
#define CONTROL_LOOP_DT_SEC      0.01f
#define TELEMETRY_EVERY_N_TICKS  5          // 100 Hz / 5 = 20 Hz packets
#define IMU_TIMEOUT_MS           50         // no valid upper-arm IMU sample for this long -> FAULT
#define AS5600_TIMEOUT_MS        50         // no valid AS5600 sample for this long -> FAULT
#define LOADCELL_TIMEOUT_MS      500        // HX711 runs at 10 SPS (100 ms) -> 5 missed samples
#define IWDG_RELOAD_COUNTS       400        // PRESCALER_32 @ ~32 kHz LSI = ~1 kHz -> ~0.4 s hardware reset

// ---------------- Pin map (Black Pill STM32F411CEU6) ----------------
// Inputs (active LOW, internal pull-up).
// NOTE: the Black Pill's on-board KEY button is wired to PA0, so pressing it behaves like the E-stop.
#define ESTOP_BUTTON_PORT       GPIOA
#define ESTOP_BUTTON_PIN        GPIO_PIN_0
#define MODE_BUTTON_PORT        GPIOA
#define MODE_BUTTON_PIN         GPIO_PIN_1

// BTS7960: PWM on TIM1 (PA8/PA9, AF1), enable on PB12 (wire to BOTH R_EN and L_EN)
#define MOTOR_PWM_UP_PORT       GPIOA       // BTS7960 RPWM  (flexion / UP)     TIM1_CH1
#define MOTOR_PWM_UP_PIN        GPIO_PIN_8
#define MOTOR_PWM_DOWN_PORT     GPIOA       // BTS7960 LPWM  (extension / DOWN) TIM1_CH2
#define MOTOR_PWM_DOWN_PIN      GPIO_PIN_9
#define MOTOR_EN_PORT           GPIOB
#define MOTOR_EN_PIN            GPIO_PIN_12

// HX711
#define HX711_DOUT_PORT         GPIOB
#define HX711_DOUT_PIN          GPIO_PIN_14
#define HX711_SCK_PORT          GPIOB
#define HX711_SCK_PIN           GPIO_PIN_15

// Fixed by silicon (see board.cpp):
//   I2C1  PB6 = SCL, PB7 = SDA   -> MPU-6050 (GY-521) on the UPPER ARM   (4.7k pull-ups to 3V3)
//   I2C2  PB10 = SCL, PB9 = SDA  -> AS5600 magnetic encoder on the ELBOW / FOREARM (4.7k pull-ups to 3V3)
//   TIM3  PA6 = CH1 (A), PA7 = CH2 (B), AF2 -> 1000 PPR optical quadrature encoder
//   USART2 TX PA2 (AF7)          -> telemetry to a 3V3 USB-TTL adapter RX, 115200 8N1
// Two separate I2C buses on purpose: the two sensors sit on different limb segments, so a bad cable or a
// stuck bus on one of them cannot take the other one down.
#define TELEMETRY_BAUD          115200

// Status LED (Black Pill on-board, active LOW)
#define STATUS_LED_PORT         GPIOC
#define STATUS_LED_PIN          GPIO_PIN_13

// ---------------- PWM ----------------
// TIM1 (APB2 timer clock = 100 MHz): prescaler 0, ARR = 4999 -> 100 MHz / 5000 = 20 kHz (inaudible, BTS7960 max 25 kHz)
#define PWM_TIM_PERIOD_COUNTS   5000
#define PWM_PERMILLE_MAX        1000        // all motor commands are -1000..+1000 (permille duty)

// ---------------- Upper-arm IMU (MPU-6050 on GY-521, AD0 = GND, mounted on the UPPER ARM) ----------------
#define MPU6050_I2C_ADDR        ((uint16_t)(0x68 << 1))   // HAL wants the 8-bit shifted address
// Mounting assumption (verify with test 02): sensor +X runs along the upper arm from shoulder toward elbow,
// +Y is "up" when the upper arm is horizontal-forward, +Z is the shoulder flexion axis.
//   upper-arm angle 0 deg = hanging down, 90 = horizontal forward, 180 = pointing straight up.
#define UA_GYRO_SIGN            1.0f        // flip to -1.0f if fused angle moves opposite to accel angle
#define UA_ANGLE_OFFSET_DEG     0.0f        // mechanical zero trim
#define UA_COMP_ALPHA           0.98f

// ---------------- Elbow absolute encoder (AS5600, 12-bit, magnet on the elbow axis) ----------------
#define AS5600_I2C_ADDR         ((uint16_t)(0x36 << 1))   // fixed address of the AS5600
// elbow angle phi = AS5600_SIGN * (raw_angle_deg - AS5600_ZERO_DEG)    0 deg = arm fully extended, 90 = right angle
// Run test 03: with the elbow fully extended press MODE and paste the printed AS5600_ZERO_DEG here.
#define AS5600_SIGN             1.0f        // -1.0f if flexion makes the angle DEcrease (or flip the DIR pin)
#define AS5600_ZERO_DEG         0.0f        // PLACEHOLDER - calibrate with test 03

// ---------------- Optical incremental encoder (1000 PPR, A/B quadrature, TIM3 hardware X4 decode) ----------------
#define ENCODER_PPR                  1000
#define ENCODER_COUNTS_PER_REV       (ENCODER_PPR * 4)          // X4 decoding -> 4000 counts per encoder revolution
#define ENCODER_SIGN                 1.0f   // -1.0f if flexion makes the optical angle DEcrease
#define ENCODER_REVS_PER_JOINT_REV   1.0f   // 1.0 = encoder on the elbow axis; N = encoder on a motor shaft behind an N:1 gear
#define ENCODER_IC_FILTER            10     // TIM input-capture filter code 0..15 (10 -> ~0.64 us glitch rejection)
// Joint degrees per X4 count (0.09 deg with the defaults)
#define ENCODER_DEG_PER_COUNT        (360.0f * ENCODER_SIGN / ((float)ENCODER_COUNTS_PER_REV * ENCODER_REVS_PER_JOINT_REV))

// ---------------- Elbow joint estimator (AS5600 absolute  +  optical incremental) ----------------
#define JOINT_FUSION_GAIN       0.02f       // per-tick pull of the optical angle toward the AS5600 (removes drift / lost counts)
#define JOINT_VEL_LPF_HZ        10.0f       // low-pass on the optical-encoder joint speed
#define ENCODER_MISMATCH_DEG    8.0f        // AS5600 vs optical disagreement that counts as an error ...
#define ENCODER_MISMATCH_TICKS  5           // ... if it persists this many ticks (50 ms) -> FAULT

// ---------------- Arm geometry (for the forearm placement / wrist position estimate) ----------------
#define UPPER_ARM_LENGTH_M      0.30f       // shoulder -> elbow axis
#define FOREARM_LENGTH_M        0.27f       // elbow axis -> cuff / wrist

// ---------------- Load cell ----------------
// PLACEHOLDER: run hardware test 05, hang a known mass and paste the printed value here.
#define LOADCELL_SCALE_COUNTS_PER_KG   228000.0f
#define LOADCELL_FORCE_SIGN            1.0f   // +1: user pushing arm UP gives positive force
#define GRAVITY_MS2                    9.80665f

// ---------------- Safety limits ----------------
// Limits are now on the true ELBOW JOINT angle (AS5600/optical), 0 deg = fully extended.
#define ELBOW_MIN_DEG           10.0f       // motor may not drive further toward extension below this
#define ELBOW_MAX_DEG           140.0f      // motor may not drive further toward flexion above this
#define ELBOW_HARD_MARGIN_DEG   10.0f       // beyond a limit by this much while assisting -> FAULT (JOINT_LIMIT)
#define JOINT_SPEED_FAULT_DPS   400.0f      // elbow speed above this while assisting -> FAULT (OVERSPEED)
#define FORCE_FAULT_N           150.0f      // 20 kg cell = 196 N full scale; fault well below it
#define FORCE_DEADBAND_N        1.5f

// ---------------- Assist controller (STARTING VALUES - tune on the bench, arm unloaded first) -------
#define FORCE_LPF_CUTOFF_HZ     5.0f
#define PID_KP                  10.0f       // permille duty per Newton
#define PID_KI                  3.0f
#define PID_KD                  0.2f
#define ASSIST_GAIN             1.0f
#define ASSIST_PWM_LIMIT        300         // permille -> max 30 % duty until you trust the system
#define ASSIST_SLEW_PER_TICK    25.0f       // permille per 10 ms tick

#endif // CONFIG_H
