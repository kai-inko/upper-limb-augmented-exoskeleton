// Board support for the STM32F411CEU6 (Black Pill): clock, GPIO, TIM1 PWM, TIM3 encoder, I2C1, I2C2,
// USART2, interrupt handlers and HAL MSP hooks.
// Every environment (main app AND every hardware test) links this file, so the hardware is
// initialised identically everywhere.
#include "Board.h"
#include "Config.h"
#include <cstring>
#include <cstdlib>

#ifndef GPIO_AF9_I2C2
  #define GPIO_AF9_I2C2 ((uint8_t)0x09)     // PB9 = I2C2_SDA is on AF9 (PB10 = I2C2_SCL is on AF4)
#endif

I2C_HandleTypeDef  hi2c1;
I2C_HandleTypeDef  hi2c2;
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim3;
UART_HandleTypeDef huart2;

static void Error_Handler(void) {
    // Clock/peripheral bring-up failed. Nothing dangerous is enabled yet, so just halt.
    __disable_irq();
    while (1) { }
}

// ---------------------------------------------------------------------------------------
// Interrupt handlers (the HAL time base does not tick without SysTick_Handler)
// ---------------------------------------------------------------------------------------
extern "C" void SysTick_Handler(void)  { HAL_IncTick(); }
extern "C" void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }

// ---------------------------------------------------------------------------------------
// HAL MSP hooks (GPIO alternate functions + peripheral clocks)
// ---------------------------------------------------------------------------------------
extern "C" void HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

extern "C" void HAL_I2C_MspInit(I2C_HandleTypeDef* h) {
    GPIO_InitTypeDef g = {0};
    if (h->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();
        g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;      // SCL, SDA
        g.Mode      = GPIO_MODE_AF_OD;
        g.Pull      = GPIO_NOPULL;                  // external 4.7k pull-ups (GY-521 has its own)
        g.Speed     = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &g);
        __HAL_RCC_I2C1_FORCE_RESET();               // clear a stuck BUSY flag (known I2C erratum)
        __HAL_RCC_I2C1_RELEASE_RESET();
    } else if (h->Instance == I2C2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C2_CLK_ENABLE();
        g.Pin       = GPIO_PIN_10;                  // SCL (AF4)
        g.Mode      = GPIO_MODE_AF_OD;
        g.Pull      = GPIO_NOPULL;                  // external 4.7k pull-ups (AS5600 breakouts usually have their own)
        g.Speed     = GPIO_SPEED_FREQ_HIGH;
        g.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &g);
        g.Pin       = GPIO_PIN_9;                   // SDA (AF9)
        g.Alternate = GPIO_AF9_I2C2;
        HAL_GPIO_Init(GPIOB, &g);
        __HAL_RCC_I2C2_FORCE_RESET();
        __HAL_RCC_I2C2_RELEASE_RESET();
    }
}

extern "C" void HAL_I2C_MspDeInit(I2C_HandleTypeDef* h) {
    if (h->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    } else if (h->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9 | GPIO_PIN_10);
    }
}

extern "C" void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* h) {
    if (h->Instance != TIM1) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin       = MOTOR_PWM_UP_PIN | MOTOR_PWM_DOWN_PIN;   // PA8 = CH1, PA9 = CH2
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &g);
}

extern "C" void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* h) {
    if (h->Instance != TIM3) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;                 // PA6 = TIM3_CH1 (A), PA7 = TIM3_CH2 (B)
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;                             // keeps the lines defined if the encoder is unplugged
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &g);
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef* h) {
    if (h->Instance != USART2) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin       = GPIO_PIN_2;                              // USART2_TX
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

// ---------------------------------------------------------------------------------------
// Clock: 25 MHz HSE /25 x200 /2 = 100 MHz SYSCLK, APB1 = 50 MHz (timers 100 MHz), APB2 = 100 MHz,
// voltage scale 1, 3 flash wait states.
// (If your Black Pill carries an 8 MHz crystal instead: set HSE_VALUE to 8000000U and PLLM to 8.)
// ---------------------------------------------------------------------------------------
void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 25;
    osc.PLL.PLLN       = 200;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    // Motor driver DISABLED at boot
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN, GPIO_PIN_RESET);
    g.Pin = MOTOR_EN_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_EN_PORT, &g);

    // Status LED (active low) -> off
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);
    g.Pin = STATUS_LED_PIN;
    HAL_GPIO_Init(STATUS_LED_PORT, &g);

    // Buttons (active low)
    g.Pin = ESTOP_BUTTON_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ESTOP_BUTTON_PORT, &g);
    g.Pin = MODE_BUTTON_PIN;
    HAL_GPIO_Init(MODE_BUTTON_PORT, &g);
}

static void MX_TIM1_Init(void) {
    TIM_OC_InitTypeDef oc = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = PWM_TIM_PERIOD_COUNTS - 1;     // ARR = 4999 -> 20 kHz @ 100 MHz
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

    // Start outputs at 0 % duty. (HAL_TIM_PWM_Start also sets MOE, which TIM1 requires.)
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
}

// TIM3 as a hardware quadrature decoder: both edges of A and B are counted (X4) -> 4 counts per PPR.
// 16-bit counter free-runs; OpticalEncoder.cpp turns it into wrap-safe deltas.
static void MX_TIM3_Init(void) {
    TIM_Encoder_InitTypeDef enc = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFF;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc.EncoderMode  = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter    = ENCODER_IC_FILTER;
    enc.IC2Polarity  = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter    = ENCODER_IC_FILTER;
    if (HAL_TIM_Encoder_Init(&htim3, &enc) != HAL_OK) Error_Handler();
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
}

static void MX_I2C_Init(I2C_HandleTypeDef* h, I2C_TypeDef* inst) {
    h->Instance             = inst;
    h->Init.ClockSpeed      = 400000;
    h->Init.DutyCycle       = I2C_DUTYCYCLE_2;
    h->Init.OwnAddress1     = 0;
    h->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    h->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    h->Init.OwnAddress2     = 0;
    h->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    h->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(h) != HAL_OK) Error_Handler();
}

static void MX_USART2_Init(void) {
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = TELEMETRY_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

void Board_Init(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_I2C_Init(&hi2c1, I2C1);
    MX_I2C_Init(&hi2c2, I2C2);
    MX_USART2_Init();
}

// ---------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------
bool Estop_Pressed(void)      { return HAL_GPIO_ReadPin(ESTOP_BUTTON_PORT, ESTOP_BUTTON_PIN) == GPIO_PIN_RESET; }
bool ModeButton_Pressed(void) { return HAL_GPIO_ReadPin(MODE_BUTTON_PORT, MODE_BUTTON_PIN) == GPIO_PIN_RESET; }
void Led_Set(bool on)         { HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); }
void Led_Toggle(void)         { HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN); }

void Motor_Enable(bool enable) {
    HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Set(int16_t permille) {
    if (permille >  PWM_PERMILLE_MAX) permille =  PWM_PERMILLE_MAX;
    if (permille < -PWM_PERMILLE_MAX) permille = -PWM_PERMILLE_MAX;
    uint32_t counts = (uint32_t)std::abs((int)permille) * PWM_TIM_PERIOD_COUNTS / PWM_PERMILLE_MAX;
    if (permille >= 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);        // release the other side first
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, counts);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, counts);
    }
}

void Motor_Stop(void) { Motor_Set(0); }
void Motor_Safe(void) { Motor_Set(0); Motor_Enable(false); }

void Uart_Write(const char* s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)std::strlen(s), 100);
}

bool Uart_WriteAsync(const char* s, size_t len) {
    static uint8_t tx_buf[192];
    if (huart2.gState != HAL_UART_STATE_READY) return false;     // previous packet still going out
    if (len > sizeof(tx_buf)) len = sizeof(tx_buf);
    std::memcpy(tx_buf, s, len);
    return HAL_UART_Transmit_IT(&huart2, tx_buf, (uint16_t)len) == HAL_OK;
}
