// TEST 07 - Full bench diagnostic, streams $TELEM packets for visualize_force_vector.py
//   python visualize_force_vector.py --port COMx
// Sensors: upper-arm MPU-6050 (I2C1), elbow AS5600 (I2C2), 1000 PPR optical encoder (TIM3), HX711 load cell.
// The motor is OFF unless you HOLD the MODE button with the E-stop released: then it applies a
// gentle +/- 150 permille square wave (period 2 s). Releasing MODE or pressing E-stop stops it at once.
// Status strings: OK, ESTOP, IMU_FAIL, AS5600_FAIL, LC_FAIL, ENC_MISMATCH.
#include "Board.h"
#include "Config.h"
#include "IMU_GY521.h"
#include "AS5600_Encoder.h"
#include "OpticalEncoder.h"
#include "HX711_LoadCell.h"
#include "Filters.h"
#include "ArmKinematics.h"
#include "Telemetry.h"

static IMU_GY521      imu(&hi2c1, MPU6050_I2C_ADDR);
static AS5600_Encoder as5600(&hi2c2, AS5600_I2C_ADDR);
static OpticalEncoder optEnc(&htim3);
static HX711_LoadCell loadcell(HX711_DOUT_PORT, HX711_DOUT_PIN, HX711_SCK_PORT, HX711_SCK_PIN);

int main(void) {
    Board_Init();
    Motor_Safe();
    Uart_Write("\r\n[TEST 07] Full diagnostic - keep arm still and unloaded during boot\r\n");

    bool imuOk = imu.begin() && imu.calibrateGyro(200);
    bool asOk  = as5600.begin();
    optEnc.begin();
    loadcell.begin();
    loadcell.setScale(LOADCELL_SCALE_COUNTS_PER_KG);
    bool lcOk = loadcell.tare(10);
    Uart_Write(imuOk ? "IMU (upper arm) : OK\r\n" : "IMU (upper arm) : FAIL\r\n");
    Uart_Write(asOk  ? "AS5600 (elbow)  : OK\r\n" : "AS5600 (elbow)  : FAIL\r\n");
    Uart_Write(lcOk  ? "LOADCELL        : OK\r\n" : "LOADCELL        : FAIL\r\n");

    ComplementaryFilter uaFilt(UA_COMP_ALPHA);
    JointEstimator joint(JOINT_FUSION_GAIN, JOINT_VEL_LPF_HZ, CONTROL_LOOP_DT_SEC);
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, rawDeg = 0;
    float ua = 0, asDeg = 0, kg = 0;
    if (imuOk && imu.readSensor(ax, ay, az, gx, gy, gz)) { ua = limbAngleFromAccel(ax, ay) + UA_ANGLE_OFFSET_DEG; uaFilt.reset(ua); }
    if (asOk && as5600.readAngleDeg(rawDeg)) { asDeg = wrap180(AS5600_SIGN * (rawDeg - AS5600_ZERO_DEG)); joint.reset(asDeg); }
    optEnc.readDeltaCounts();

    uint32_t next = HAL_GetTick(), n = 0;
    uint32_t imuLast = HAL_GetTick(), asLast = HAL_GetTick(), lcLast = HAL_GetTick();
    uint8_t mismatchTicks = 0;

    while (1) {
        while ((int32_t)(HAL_GetTick() - next) < 0) { }
        next += CONTROL_LOOP_PERIOD_MS;
        n++;

        if (imuOk && imu.readSensor(ax, ay, az, gx, gy, gz)) {
            imuLast = HAL_GetTick();
            ua = uaFilt.update(UA_GYRO_SIGN * gz, limbAngleFromAccel(ax, ay) + UA_ANGLE_OFFSET_DEG,
                               accelMagnitudeValid(ax, ay, az), CONTROL_LOOP_DT_SEC);
        }
        bool asValid = asOk && as5600.readAngleDeg(rawDeg);
        if (asValid) { asLast = HAL_GetTick(); asDeg = wrap180(AS5600_SIGN * (rawDeg - AS5600_ZERO_DEG)); }
        joint.update((float)optEnc.readDeltaCounts() * ENCODER_DEG_PER_COUNT, asDeg, asValid);
        if (asValid) mismatchTicks = (std::fabs(joint.mismatch()) > ENCODER_MISMATCH_DEG)
                                     ? (uint8_t)(mismatchTicks < 255 ? mismatchTicks + 1 : 255) : 0;

        int32_t raw;
        if (lcOk && loadcell.dataReady() && loadcell.read(raw, 0)) {
            lcLast = HAL_GetTick();
            kg = LOADCELL_FORCE_SIGN * loadcell.rawToUnits(raw);
        }

        const float elbow = joint.angle();
        const float fore  = wrap180(ua + elbow);
        const ArmPlacement pl = forearmPlacement(ua, fore, UPPER_ARM_LENGTH_M, FOREARM_LENGTH_M);

        bool estop = Estop_Pressed();
        const char* status = "OK";
        if (estop)                                                         status = "ESTOP";
        else if (!imuOk || (HAL_GetTick() - imuLast) > IMU_TIMEOUT_MS)     status = "IMU_FAIL";
        else if (!asOk  || (HAL_GetTick() - asLast)  > AS5600_TIMEOUT_MS)  status = "AS5600_FAIL";
        else if (!lcOk  || (HAL_GetTick() - lcLast)  > LOADCELL_TIMEOUT_MS) status = "LC_FAIL";
        else if (mismatchTicks >= ENCODER_MISMATCH_TICKS)                  status = "ENC_MISMATCH";

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
            TelemetryPacket p = { fore, kg, kg * GRAVITY_MS2, ax, ay, az, pwm, pwm ? 1 : 0, status,
                                  ua, elbow, joint.velocity(), joint.mismatch(),
                                  pl.wrist_x * 100.0f, pl.wrist_y * 100.0f };
            char buf[TELEM_BUF_SIZE];
            size_t len = Telemetry_Format(buf, sizeof(buf), p);
            Uart_WriteAsync(buf, len);
        }
    }
}
