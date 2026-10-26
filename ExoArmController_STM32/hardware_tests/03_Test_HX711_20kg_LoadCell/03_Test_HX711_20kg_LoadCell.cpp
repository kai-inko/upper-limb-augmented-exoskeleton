// TEST 03 - HX711 + 20 kg load cell (DOUT = PB14, SCK = PB15)
// 1) Leave the cell UNLOADED; it tares after 3 s.
// 2) Readings are printed in kg and N.
// 3) CALIBRATION: hang a known mass (CAL_MASS_KG) and press MODE. The printed counts-per-kg
//    value must be pasted into LOADCELL_SCALE_COUNTS_PER_KG in include/Config.h.
// Press the E-stop button at any time to re-tare.
#include "Board.h"
#include "Config.h"
#include "HX711_LoadCell.h"
#include "Telemetry.h"

#define CAL_MASS_KG  2.000f      // <- the reference mass you actually hang on the cell

static HX711_LoadCell loadcell(HX711_DOUT_PORT, HX711_DOUT_PIN, HX711_SCK_PORT, HX711_SCK_PIN);

static void tareNow() {
    Uart_Write("Taring - cell must be UNLOADED...\r\n");
    HAL_Delay(3000);
    if (loadcell.tare(10)) {
        char buf[64]; LineBuilder b(buf, sizeof(buf));
        b.str("Tare ok, offset = ").i(loadcell.getOffset()).str(" counts\r\n"); Uart_Write(b.c_str());
    } else {
        Uart_Write("TARE FAILED: no data. Check 5V/GND, E+/E-/A+/A- wiring, DOUT/SCK pins\r\n");
    }
}

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 03] HX711 load cell\r\n");

    loadcell.begin();
    loadcell.setScale(LOADCELL_SCALE_COUNTS_PER_KG);
    tareNow();
    Uart_Write("Hang the reference mass and press MODE to calibrate.\r\n");

    bool lastMode = false;
    while (1) {
        int32_t raw;
        char buf[120]; LineBuilder b(buf, sizeof(buf));
        if (loadcell.read(raw, 200)) {
            float kg = LOADCELL_FORCE_SIGN * loadcell.rawToUnits(raw);
            b.str("raw ").i(raw).str("  ").f(kg, 3).str(" kg  ").f(kg * GRAVITY_MS2, 2).str(" N\r\n");
            Led_Set(false);
        } else if (loadcell.wasSaturated()) {
            b.str("SATURATED (bridge open / swapped wires / no cell)\r\n");
            Led_Set(true);
        } else {
            b.str("NO DATA from HX711\r\n");
            Led_Set(true);
        }
        Uart_Write(b.c_str());

        bool m = ModeButton_Pressed();
        if (m && !lastMode) {
            int32_t avg;
            if (loadcell.readAverage(avg, 10, 300)) {
                float counts_per_kg = (float)(avg - loadcell.getOffset()) / CAL_MASS_KG;
                loadcell.setScale(counts_per_kg);
                char c[96]; LineBuilder cb(c, sizeof(c));
                cb.str(">>> LOADCELL_SCALE_COUNTS_PER_KG = ").f(counts_per_kg, 1).str("f\r\n"); Uart_Write(c);
            } else {
                Uart_Write("Calibration read failed\r\n");
            }
        }
        lastMode = m;

        if (Estop_Pressed()) { tareNow(); }
        HAL_Delay(100);
    }
}
