#ifndef BOARD_H
#define BOARD_H

#include "stm32f1xx_hal.h"
#include <cstddef>
#include <cstdint>

// Peripheral handles (defined in board.cpp)
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;
extern UART_HandleTypeDef huart2;

// One call brings the whole board up: HAL, 72 MHz clock, GPIO, TIM1 PWM, I2C1, USART2.
// The motor driver is left DISABLED with 0 % duty.
void Board_Init(void);
void SystemClock_Config(void);

// Inputs / indicators
bool Estop_Pressed(void);
bool ModeButton_Pressed(void);
void Led_Set(bool on);
void Led_Toggle(void);

// Motor (command range -1000..+1000 permille; + = UP / flexion, - = DOWN / extension)
void Motor_Enable(bool enable);
void Motor_Set(int16_t permille);
void Motor_Stop(void);      // 0 % duty, driver stays enabled
void Motor_Safe(void);      // 0 % duty AND driver disabled (use for every fault path)

// UART (USART2, TX only)
void Uart_Write(const char* s);                       // blocking, for tests / boot messages
bool Uart_WriteAsync(const char* s, size_t len);      // interrupt driven; returns false if busy

#endif // BOARD_H
