// TEST 04 - 1000 PPR optical encoder, A = PA6 (TIM3_CH1), B = PA7 (TIM3_CH2), hardware X4 decoding
// Prints at 10 Hz: total counts, encoder revolutions, joint angle (relative to the last zero) and joint speed.
// Press MODE to zero the counter.
//
// CHECKS
//  * PPR / decoding: turn the ENCODER SHAFT exactly one full revolution -> counts must change by +/-4000
//    (1000 PPR x 4 edges). If it shows +/-2000 you only count one channel; if it is erratic, check the
//    A/B wiring, the 3V3 pull-ups and the common ground.
//  * DIRECTION: flex the elbow; "joint" must INCREASE. If it decreases set ENCODER_SIGN = -1.0f.
//  * RATIO: if the encoder is on a gearbox/motor shaft, set ENCODER_REVS_PER_JOINT_REV to the gear ratio,
//    then rotate the elbow by a known angle (e.g. 90 deg) and compare.
// The Z (index) channel is not used: the AS5600 provides the absolute reference.
#include "Board.h"
#include "Config.h"
#include "OpticalEncoder.h"
#include "Filters.h"
#include "Telemetry.h"

static OpticalEncoder enc(&htim3);

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 04] Optical encoder (TIM3 X4)\r\n");
    enc.begin();

    LowPassFilter speedLpf(JOINT_VEL_LPF_HZ, CONTROL_LOOP_DT_SEC);
    bool lastMode = false;
    uint32_t next = HAL_GetTick(), n = 0;
    uint32_t lastMove = HAL_GetTick();
    while (1) {
        while ((int32_t)(HAL_GetTick() - next) < 0) { }
        next += CONTROL_LOOP_PERIOD_MS;                          // 100 Hz sampling, same as the controller

        int32_t d = enc.readDeltaCounts();
        float speed = speedLpf.update((float)d * ENCODER_DEG_PER_COUNT / CONTROL_LOOP_DT_SEC);
        if (d != 0) lastMove = HAL_GetTick();
        Led_Set(d != 0);                                         // LED flickers while the encoder moves

        bool m = ModeButton_Pressed();
        if (m && !lastMode) { enc.resetTotal(); Uart_Write(">>> counter zeroed\r\n"); }
        lastMode = m;

        if ((++n % 10) == 0) {
            int32_t total = enc.totalCounts();
            char buf[140]; LineBuilder b(buf, sizeof(buf));
            b.str("counts ").i(total)
             .str(" | enc rev ").f((float)total / (float)ENCODER_COUNTS_PER_REV, 3)
             .str(" | joint ").f((float)total * ENCODER_DEG_PER_COUNT, 2).str(" deg")
             .str(" | speed ").f(speed, 1).str(" dps");
            if ((HAL_GetTick() - lastMove) > 3000) b.str(" | (no motion)");
            b.str("\r\n");
            Uart_Write(b.c_str());
        }
    }
}
