// TEST 04 - BTS7960 motor driver (RPWM = PA8, LPWM = PA9, R_EN+L_EN = PB12)
// SAFETY: run with the exoskeleton arm UNLOADED and clear of your body. Keep a hand on the E-stop.
// Sequence (after you press MODE): ramp UP to TEST_PWM_MAX, ramp down, pause, ramp DOWN, ramp down, repeat.
// The E-stop immediately zeroes PWM and disables the driver; press MODE to restart.
#include "Board.h"
#include "Config.h"

#define TEST_PWM_MAX   250      // permille (25 %)
#define RAMP_STEP      10
#define RAMP_STEP_MS   40

static void waitMode() {
    Uart_Write("Press MODE to start (E-stop must be released)\r\n");
    while (!(ModeButton_Pressed() && !Estop_Pressed())) { Led_Set((HAL_GetTick() / 250) & 1); HAL_Delay(10); }
    while (ModeButton_Pressed()) { HAL_Delay(10); }
    HAL_Delay(50);
}

// Applies one PWM step; returns false if the E-stop fired.
static bool applyStep(int16_t pwm) {
    if (Estop_Pressed()) {
        Motor_Safe();
        Uart_Write("E-STOP -> motor disabled\r\n");
        return false;
    }
    Motor_Set(pwm);
    HAL_Delay(RAMP_STEP_MS);
    return true;
}

static bool ramp(int dir, const char* name) {
    Uart_Write("Ramp "); Uart_Write(name); Uart_Write("\r\n");
    for (int v = 0; v <= TEST_PWM_MAX; v += RAMP_STEP) if (!applyStep((int16_t)(dir * v))) return false;
    HAL_Delay(300);
    for (int v = TEST_PWM_MAX; v >= 0; v -= RAMP_STEP) if (!applyStep((int16_t)(dir * v))) return false;
    Motor_Stop();
    HAL_Delay(500);
    return true;
}

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 04] BTS7960 motor driver\r\n");

    while (1) {
        waitMode();
        Motor_Enable(true);
        Led_Set(true);
        while (1) {
            if (!ramp(+1, "UP   (RPWM/PA8)")) break;
            if (!ramp(-1, "DOWN (LPWM/PA9)")) break;
        }
        Motor_Safe();
        Led_Set(false);
        while (Estop_Pressed()) { HAL_Delay(10); }
    }
}
