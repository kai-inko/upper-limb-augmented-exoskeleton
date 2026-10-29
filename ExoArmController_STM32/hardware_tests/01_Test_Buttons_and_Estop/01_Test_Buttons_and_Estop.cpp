// TEST 01 - E-Stop (PA0), MODE button (PA1), status LED (PC13), UART link (PA2)
// Open a serial monitor at 115200. Press each button: the LED lights and a line is printed.
#include "Board.h"
#include "Config.h"

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 01] Buttons / E-Stop / LED / UART\r\n");

    bool lastE = !Estop_Pressed();      // force a first print
    bool lastM = !ModeButton_Pressed();

    while (1) {
        bool e = Estop_Pressed();
        bool m = ModeButton_Pressed();
        if (e != lastE || m != lastM) {
            HAL_Delay(30);                          // crude debounce, then re-read
            e = Estop_Pressed();
            m = ModeButton_Pressed();
            if (e != lastE) Uart_Write(e ? "E-STOP : PRESSED\r\n" : "E-STOP : released\r\n");
            if (m != lastM) Uart_Write(m ? "MODE   : PRESSED\r\n" : "MODE   : released\r\n");
            lastE = e; lastM = m;
        }
        Led_Set(e || m);
        HAL_Delay(10);
    }
}

