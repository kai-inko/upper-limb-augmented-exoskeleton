#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// System Timing Configuration
#define CONTROL_LOOP_FREQ_HZ    100
#define CONTROL_LOOP_DT_SEC     0.01f
#define WATCHDOG_TIMEOUT_MS     200

// GPIO Mapping (STM32 Pin Out)
#define ESTOP_BUTTON_PORT       GPIOA
#define ESTOP_BUTTON_PIN        GPIO_PIN_0

#define MODE_BUTTON_PORT        GPIOA
#define MODE_BUTTON_PIN        GPIO_PIN_1

// BTS7960 Motor Driver Pins
#define MOTOR_PWM_LEFT_PORT     GPIOA
#define MOTOR_PWM_LEFT_PIN      GPIO_PIN_8   // TIM1_CH1
#define MOTOR_PWM_RIGHT_PORT    GPIOA
#define MOTOR_PWM_RIGHT_PIN     GPIO_PIN_9   // TIM1_CH2
#define MOTOR_EN_PORT           GPIOB
#define MOTOR_EN_PIN            GPIO_PIN_12

// HX711 Load Cell Pins
#define HX711_DOUT_PORT         GPIOB
#define HX711_DOUT_PIN          GPIO_PIN_6
#define HX711_SCK_PORT          GPIOB
#define HX711_SCK_PIN           GPIO_PIN_7

// I2C Hardware Instance (GY-521 IMU)
#define MPU6050_I2C_ADDR        (0x68 << 1)  // HAL requires 8-bit shifted address

#endif // CONFIG_H