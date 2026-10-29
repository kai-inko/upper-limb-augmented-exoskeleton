// TEST 02 - GY-521 / MPU-6050 on I2C1 (PB6 = SCL, PB7 = SDA), mounted on the UPPER ARM
// 1) scans the bus, 2) checks WHO_AM_I, 3) calibrates the gyro (keep still),
// 4) prints accel, gyro, accel-only angle and fused angle at 10 Hz.
//
// MOUNTING CHECK (upper arm): hang the upper arm down -> angle ~ 0; hold it horizontal forward -> ~ 90;
// raise it further -> > 90. Rotate the upper arm UP (shoulder flexion): the FUSED angle must follow the
// ACCEL angle. If it moves the opposite way while you rotate, set UA_GYRO_SIGN to -1.0f in Config.h.
#include "Board.h"
#include "Config.h"
#include "IMU_GY521.h"
#include "Filters.h"
#include "Telemetry.h"

static IMU_GY521 imu(&hi2c1, MPU6050_I2C_ADDR);

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 02] GY-521 IMU (upper arm, I2C1)\r\n");

    // --- I2C scan ---
    Uart_Write("Scanning I2C1...\r\n");
    int found = 0;
    for (uint16_t a = 1; a < 127; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 1, 5) == HAL_OK) {
            char buf[48]; LineBuilder b(buf, sizeof(buf));
            b.str("  device at 7-bit address ").x(a, 2).str("\r\n"); Uart_Write(b.c_str());
            found++;
        }
    }
    if (!found) Uart_Write("  NOTHING FOUND: check SDA/SCL, 3V3, GND, pull-ups (expect 0x68)\r\n");

    uint8_t id = 0;
    if (!imu.whoAmI(id)) {
        Uart_Write("WHO_AM_I read FAILED\r\n");
        while (1) { Led_Toggle(); HAL_Delay(100); }
    }
    { char buf[64]; LineBuilder b(buf, sizeof(buf)); b.str("WHO_AM_I = ").x(id, 2).str(" (0x68 = MPU-6050)\r\n"); Uart_Write(b.c_str()); }

    if (!imu.begin()) {
        Uart_Write("begin() FAILED (unknown chip ID or write error)\r\n");
        while (1) { Led_Toggle(); HAL_Delay(100); }
    }

    Uart_Write("Calibrating gyro - keep the sensor STILL...\r\n");
    if (!imu.calibrateGyro(200)) Uart_Write("Gyro calibration failed (moving or I2C error) - continuing uncalibrated\r\n");

    ComplementaryFilter filt(UA_COMP_ALPHA);
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    if (imu.readSensor(ax, ay, az, gx, gy, gz)) filt.reset(limbAngleFromAccel(ax, ay) + UA_ANGLE_OFFSET_DEG);

    uint32_t next = HAL_GetTick(), n = 0;
    while (1) {
        while ((int32_t)(HAL_GetTick() - next) < 0) { }
        next += CONTROL_LOOP_PERIOD_MS;

        if (!imu.readSensor(ax, ay, az, gx, gy, gz)) { Led_Set(true); continue; }
        float accAngle = limbAngleFromAccel(ax, ay) + UA_ANGLE_OFFSET_DEG;
        float fused = filt.update(UA_GYRO_SIGN * gz, accAngle, accelMagnitudeValid(ax, ay, az), CONTROL_LOOP_DT_SEC);
        Led_Set(false);

        if ((++n % 10) == 0) {
            char buf[160]; LineBuilder b(buf, sizeof(buf));
            b.str("A[g] ").f(ax, 2).ch(' ').f(ay, 2).ch(' ').f(az, 2)
             .str(" | G[dps] ").f(gx, 1).ch(' ').f(gy, 1).ch(' ').f(gz, 1)
             .str(" | UA accAngle ").f(accAngle, 1).str(" fused ").f(fused, 1).str("\r\n");
            Uart_Write(b.c_str());
        }
    }
}
