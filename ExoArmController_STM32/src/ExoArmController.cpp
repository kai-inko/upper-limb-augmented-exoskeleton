#include "stm32f1xx_hal.h"
#include "Config.h"
#include "IMU_GY521.h"
#include "PIDController.h"
#include "SafetyWatchdog.h"

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

PIDController pid(2.5f, 0.1f, 0.05f, -100.0f, 100.0f);
SafetyWatchdog watchdog(WATCHDOG_TIMEOUT_MS);
IMU_GY521 imu(&hi2c1, MPU6050_I2C_ADDR);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    imu.begin();
    uint32_t last_tick = HAL_GetTick();

    while (1) {
        // Enforce 100Hz deterministic control loop
        if ((HAL_GetTick() - last_tick) >= 10) {
            last_tick = HAL_GetTick();

            // Read E-Stop
            if (HAL_GPIO_ReadPin(ESTOP_BUTTON_PORT, ESTOP_BUTTON_PIN) == GPIO_PIN_RESET) {
                // E-Stop Triggered: Stop Motor
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
                continue;
            }

            float ax, ay, az, gx, gy, gz;
            if (imu.readSensor(ax, ay, az, gx, gy, gz)) {
                float output = pid.compute(0.0f, ay, CONTROL_LOOP_DT_SEC);
                
                // Drive BTS7960 PWM
                if (output >= 0) {
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)output);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(-output));
                }
            }

            watchdog.kick();
        }
    }
}

void SystemClock_Config(void) {
    // Configures 72MHz HSE Clock for BluePill (F103)
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure E-Stop Pin
    GPIO_InitStruct.Pin = ESTOP_BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ESTOP_BUTTON_PORT, &GPIO_InitStruct);

    // Configure Motor Enable Pin
    GPIO_InitStruct.Pin = MOTOR_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_EN_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN, GPIO_PIN_SET);
}

static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    HAL_I2C_Init(&hi2c1);
}

static void MX_TIM1_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1000;
    HAL_TIM_PWM_Init(&htim1);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
}