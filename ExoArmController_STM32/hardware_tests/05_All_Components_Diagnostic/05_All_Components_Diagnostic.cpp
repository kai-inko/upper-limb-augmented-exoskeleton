// TEST 05 - Full bench diagnostic, streams $TELEM packets for visualize_force_vector.py
//   python visualize_force_vector.py --port COMx
// The motor is OFF unless you HOLD the MODE button with the E-stop released: then it applies a
// gentle +/- 150 permille square wave (period 2 s). Releasing MODE or pressing E-stop stops it at once.
#include "Board.h"
#include "Config.h"
#include "IMU_GY521.h"
#include "HX711_LoadCell.h"
#include "Filters.h"
#include "Telemetry.h"

static IMU_GY521      imu(&hi2c1, MPU6050_I2C_ADDR);
static HX711_LoadCell loadcell(HX711_DOUT_PORT, HX711_DOUT_PIN, HX711_SCK_PORT, HX711_SCK_PIN);

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 05] Full diagnostic - keep arm still and unloaded during boot\r\n");

    bool imuOk = imu.begin() && imu.calibrateGyro(200);
    loadcell.begin();
    loadcell.setScale(LOADCELL_SCALE_COUNTS_PER_KG);
    bool lcOk = loadcell.tare(10);
    Uart_Write(imuOk ? "IMU      : OK\r\n" : "IMU      : FAIL\r\n");
    Uart_Write(lcOk  ? "LOADCELL : OK\r\n" : "LOADCELL : FAIL\r\n");

    ComplementaryFilter filt(IMU_COMP_ALPHA);
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (imuOk && imu.readSensor(ax, ay, az, gx, gy, gz)) filt.reset(elbowAngleFromAccel(ax, ay));

    float angle = 90.0f, kg = 0.0f;
    uint32_t next = HAL_GetTick(), n = 0;
    uint32_t imuLast = HAL_GetTick(), lcLast = HAL_GetTick();

    while (1) {
        while ((int32_t)(HAL_GetTick() - next) < 0) { }
        next += CONTROL_LOOP_PERIOD_MS;
        n++;

        if (imuOk && imu.readSensor(ax, ay, az, gx, gy, gz)) {
            imuLast = HAL_GetTick();
            angle = filt.update(IMU_GYRO_SIGN * gz, elbowAngleFromAccel(ax, ay) + IMU_ANGLE_OFFSET_DEG,
                                accelMagnitudeValid(ax, ay, az), CONTROL_LOOP_DT_SEC);
        }
        int32_t raw;
        if (lcOk && loadcell.dataReady() && loadcell.read(raw, 0)) {
            lcLast = HAL_GetTick();
            kg = LOADCELL_FORCE_SIGN * loadcell.rawToUnits(raw);
        }

        bool estop = Estop_Pressed();
        const char* status = "OK";
        if (estop)                                                         status = "ESTOP";
        else if (!imuOk || (HAL_GetTick() - imuLast) > IMU_TIMEOUT_MS)     status = "IMU_FAIL";
        else if (!lcOk || (HAL_GetTick() - lcLast) > LOADCELL_TIMEOUT_MS)  status = "LC_FAIL";

        // Gated motor exercise
        int pwm = 0;
        if (!estop && ModeButton_Pressed()) {
            Motor_Enable(true);
            pwm = ((n / 100) & 1) ? -150 : 150;
            Motor_Set((int16_t)pwm);
        } else {
            Motor_Safe();
        }
        Led_Set(status[0] != 'O');

        if ((n % TELEMETRY_EVERY_N_TICKS) == 0) {
            TelemetryPacket p = { angle, kg, kg * GRAVITY_MS2, ax, ay, az, pwm, pwm ? 1 : 0, status };
            char buf[128];
            size_t len = Telemetry_Format(buf, sizeof(buf), p);
            Uart_WriteAsync(buf, len);
        }
    }
}
