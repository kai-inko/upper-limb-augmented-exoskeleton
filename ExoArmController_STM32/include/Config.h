#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// =====================================================================
//  SINGLE SOURCE OF TRUTH FOR PINS, TIMING, LIMITS AND TUNING VALUES
// =====================================================================

// ---------------- Timing ----------------
#define CONTROL_LOOP_PERIOD_MS   10
#define CONTROL_LOOP_DT_SEC      0.01f
#define TELEMETRY_EVERY_N_TICKS  5          // 100 Hz / 5 = 20 Hz packets
#define IMU_TIMEOUT_MS           50         // no valid IMU sample for this long -> FAULT
#define LOADCELL_TIMEOUT_MS      500        // HX711 runs at 10 SPS (100 ms) -> 5 missed samples
#define IWDG_RELOAD_COUNTS       500        // PRESCALER_32 @ ~40 kHz LSI -> ~0.4 s hardware reset

// ---------------- Pin map (Blue Pill STM32F103C8) ----------------
// Inputs (active LOW, internal pull-up)
#define ESTOP_BUTTON_PORT       GPIOA
#define ESTOP_BUTTON_PIN        GPIO_PIN_0
#define MODE_BUTTON_PORT        GPIOA
#define MODE_BUTTON_PIN         GPIO_PIN_1

// BTS7960: PWM on TIM1 (PA8/PA9), enable on PB12 (wire to BOTH R_EN and L_EN)
#define MOTOR_PWM_UP_PORT       GPIOA       // BTS7960 RPWM  (flexion / UP)    TIM1_CH1
#define MOTOR_PWM_UP_PIN        GPIO_PIN_8
#define MOTOR_PWM_DOWN_PORT     GPIOA       // BTS7960 LPWM  (extension / DOWN) TIM1_CH2
#define MOTOR_PWM_DOWN_PIN      GPIO_PIN_9
#define MOTOR_EN_PORT           GPIOB
#define MOTOR_EN_PIN            GPIO_PIN_12

// HX711 (moved off PB6/PB7, which belong to I2C1)
#define HX711_DOUT_PORT         GPIOB
#define HX711_DOUT_PIN          GPIO_PIN_14
#define HX711_SCK_PORT          GPIOB
#define HX711_SCK_PIN           GPIO_PIN_15

// I2C1 -> GY-521 (fixed by silicon): SCL = PB6, SDA = PB7 (needs 4.7k pull-ups to 3V3)
// USART2 TX -> telemetry (fixed by silicon): PA2 -> USB-TTL RX (3V3 logic), 115200 8N1
// (USART1 is NOT used because PA9 is TIM1_CH2.)
#define TELEMETRY_BAUD          115200

// Status LED (Blue Pill on-board, active LOW)
#define STATUS_LED_PORT         GPIOC
#define STATUS_LED_PIN          GPIO_PIN_13

// ---------------- PWM ----------------
// TIM1: prescaler 0, ARR = 3599 -> 72 MHz / 3600 = 20 kHz (inaudible, BTS7960 max 25 kHz)
#define PWM_TIM_PERIOD_COUNTS   3600
#define PWM_PERMILLE_MAX        1000        // all motor commands are -1000..+1000 (permille duty)

// ---------------- IMU (MPU-6050 on GY-521, AD0 = GND) ----------------
#define MPU6050_I2C_ADDR        ((uint16_t)(0x68 << 1))   // HAL wants the 8-bit shifted address
// Mounting assumption (verify with test 02): sensor +X runs along the forearm toward the hand,
// +Y is "up" when forearm is horizontal, +Z is the elbow axis.
//   angle 0 deg = forearm hanging down, 90 = horizontal, 150 = flexed.
#define IMU_GYRO_SIGN           1.0f        // flip to -1.0f if fused angle moves opposite to accel angle
#define IMU_ANGLE_OFFSET_DEG    0.0f        // mechanical zero trim
#define IMU_COMP_ALPHA          0.98f

// ---------------- Load cell ----------------
// PLACEHOLDER: run hardware test 03, hang a known mass and paste the printed value here.
#define LOADCELL_SCALE_COUNTS_PER_KG   228000.0f
#define LOADCELL_FORCE_SIGN            1.0f   // +1: user pushing arm UP gives positive force
#define GRAVITY_MS2                    9.80665f

// ---------------- Safety limits ----------------
#define ANGLE_MIN_DEG           30.0f       // motor may not drive further DOWN below this
#define ANGLE_MAX_DEG           140.0f      // motor may not drive further UP above this
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
