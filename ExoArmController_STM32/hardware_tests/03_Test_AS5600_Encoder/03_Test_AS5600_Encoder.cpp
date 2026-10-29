// TEST 03 - AS5600 magnetic encoder on I2C2 (PB10 = SCL, PB9 = SDA), magnet on the ELBOW axis
// 1) scans I2C2 (expect 0x36), 2) checks the magnet (MD / ML / MH flags + AGC),
// 3) streams raw angle (0..360 deg) and the elbow angle phi = AS5600_SIGN * (raw - AS5600_ZERO_DEG) at 10 Hz.
//
// CALIBRATION: put the elbow in the FULLY EXTENDED position and press MODE. The firmware averages 20 samples
// and prints   >>> AS5600_ZERO_DEG = xxx.xxf   -> paste it into include/Config.h.
// DIRECTION CHECK: flex the elbow; phi must INCREASE. If it decreases set AS5600_SIGN = -1.0f.
// MAGNET: AGC should sit mid-range (roughly 80..180). MH = too strong (move the magnet away),
//         ML = too weak (move it closer). MD must be 1.
#include "Board.h"
#include "Config.h"
#include "AS5600_Encoder.h"
#include "Filters.h"
#include "Telemetry.h"

static AS5600_Encoder as5600(&hi2c2, AS5600_I2C_ADDR);

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 03] AS5600 elbow encoder (I2C2)\r\n");

    Uart_Write("Scanning I2C2...\r\n");
    int found = 0;
    for (uint16_t a = 1; a < 127; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, a << 1, 1, 5) == HAL_OK) {
            char buf[48]; LineBuilder b(buf, sizeof(buf));
            b.str("  device at 7-bit address ").x(a, 2).str("\r\n"); Uart_Write(b.c_str());
            found++;
        }
    }
    if (!found) Uart_Write("  NOTHING FOUND: check SDA(PB9)/SCL(PB10), 3V3, GND, pull-ups (expect 0x36)\r\n");

    if (!as5600.begin()) {
        char buf[96]; LineBuilder b(buf, sizeof(buf));
        b.str("begin() FAILED, STATUS = ").x(as5600.status(), 2).str("  (0x20 = magnet detected)\r\n");
        Uart_Write(b.c_str());
        Uart_Write("Continuing so you can watch the magnet flags while you adjust the magnet.\r\n");
    }

    bool lastMode = false;
    uint32_t next = HAL_GetTick();
    while (1) {
        while ((int32_t)(HAL_GetTick() - next) < 0) { }
        next += 100;                                            // 10 Hz

        float deg;
        bool ok = as5600.readAngleDeg(deg);
        uint8_t agc = 0; as5600.readAgc(agc);

        char buf[160]; LineBuilder b(buf, sizeof(buf));
        if (ok) {
            float phi = wrap180(AS5600_SIGN * (deg - AS5600_ZERO_DEG));
            b.str("raw ").f(deg, 2).str(" deg | elbow phi ").f(phi, 2).str(" deg | AGC ").u(agc)
             .str(as5600.magnetTooStrong() ? " | MH: too STRONG" : "")
             .str(as5600.magnetTooWeak()   ? " | ML: too WEAK"   : "")
             .str("\r\n");
            Led_Set(false);
        } else {
            b.str("NO DATA  STATUS ").x(as5600.status(), 2).str("  (I2C error or no magnet: MD=0)  AGC ").u(agc).str("\r\n");
            Led_Set(true);
        }
        Uart_Write(b.c_str());

        bool m = ModeButton_Pressed();
        if (m && !lastMode) {
            float ref = 0, sum = 0; int got = 0;
            for (int i = 0; i < 20; i++) {
                float d;
                if (as5600.readAngleDeg(d)) {
                    if (!got) ref = d;
                    sum += wrap180(d - ref);                    // average around the first sample: safe across the 359 -> 0 wrap
                    got++;
                }
                HAL_Delay(10);
            }
            if (got >= 10) {
                float zero = ref + sum / (float)got;
                while (zero <   0.0f) zero += 360.0f;
                while (zero >= 360.0f) zero -= 360.0f;
                char c[96]; LineBuilder cb(c, sizeof(c));
                cb.str(">>> AS5600_ZERO_DEG = ").f(zero, 2).str("f\r\n"); Uart_Write(c);
            } else {
                Uart_Write("Zero calibration failed (not enough valid samples)\r\n");
            }
        }
        lastMode = m;
    }
}
