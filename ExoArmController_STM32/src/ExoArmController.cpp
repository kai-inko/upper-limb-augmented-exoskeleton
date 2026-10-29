// ExoArm main application: 100 Hz force-following assist controller with layered safety.
//
// Sensing (arm tracking):
//   upper arm  : MPU-6050 (I2C1)  -> upper-arm inclination from gravity
//   elbow joint: AS5600 absolute magnetic encoder (I2C2)  +  1000 PPR optical encoder (TIM3, X4)
//                fused by JointEstimator -> elbow angle, elbow speed, sensor-disagreement check
//   forearm    : inclination = upper arm + elbow, plus the wrist position in the sagittal plane
//
//   IDLE   : driver disabled, 0 % PWM. MODE button -> ASSIST.
//   ASSIST : load-cell force (user pushing the cuff) drives a PID -> motor assists in the push direction.
//   FAULT  : latched. Driver disabled. Cleared only with E-stop released + MODE press.
#include "Board.h"
#include "Config.h"
#include "IMU_GY521.h"
#include "AS5600_Encoder.h"
#include "OpticalEncoder.h"
#include "HX711_LoadCell.h"
#include "PIDController.h"
#include "SafetyWatchdog.h"
#include "Filters.h"
#include "ArmKinematics.h"
#include "Telemetry.h"

enum class State : uint8_t { IDLE = 0, ASSIST = 1, FAULT = 2 };
enum class Fault : uint8_t { NONE = 0, ESTOP, IMU, AS5600, LOADCELL, OVERFORCE, ENC_MISMATCH, OVERSPEED, JOINT_LIMIT };

static const char* faultName(Fault f) {
    switch (f) {
        case Fault::ESTOP:        return "ESTOP";
        case Fault::IMU:          return "IMU_FAIL";
        case Fault::AS5600:       return "AS5600_FAIL";
        case Fault::LOADCELL:     return "LC_FAIL";
        case Fault::OVERFORCE:    return "OVERFORCE";
        case Fault::ENC_MISMATCH: return "ENC_MISMATCH";
        case Fault::OVERSPEED:    return "OVERSPEED";
        case Fault::JOINT_LIMIT:  return "JOINT_LIMIT";
        default:                  return "OK";
    }
}

// ---- Objects ----
static IMU_GY521       imu(&hi2c1, MPU6050_I2C_ADDR);          // upper arm
static AS5600_Encoder  as5600(&hi2c2, AS5600_I2C_ADDR);        // elbow axis, absolute
static OpticalEncoder  optEnc(&htim3);                         // elbow axis, incremental
static HX711_LoadCell  loadcell(HX711_DOUT_PORT, HX711_DOUT_PIN, HX711_SCK_PORT, HX711_SCK_PIN);
static PIDController   pid(PID_KP, PID_KI, PID_KD, -(float)ASSIST_PWM_LIMIT, (float)ASSIST_PWM_LIMIT);
static LowPassFilter   forceLpf(FORCE_LPF_CUTOFF_HZ, CONTROL_LOOP_DT_SEC);
static ComplementaryFilter upperArmFilter(UA_COMP_ALPHA);
static JointEstimator  joint(JOINT_FUSION_GAIN, JOINT_VEL_LPF_HZ, CONTROL_LOOP_DT_SEC);
static SlewLimiter     slew(ASSIST_SLEW_PER_TICK);
static SafetyWatchdog  imuWd(IMU_TIMEOUT_MS);
static SafetyWatchdog  asWd(AS5600_TIMEOUT_MS);
static SafetyWatchdog  loadWd(LOADCELL_TIMEOUT_MS);
static IWDG_HandleTypeDef hiwdg;

// ---- State ----
static State g_state = State::IDLE;
static Fault g_fault = Fault::NONE;
static float g_ax = 0, g_ay = 0, g_az = 0;
static float g_ua = 0.0f;                  // upper-arm inclination (deg, 0 = hanging down)
static float g_as_deg = 0.0f;              // elbow angle from the AS5600 alone (deg)
static float g_elbow = 0.0f;               // fused elbow angle (deg, 0 = extended)
static float g_elbow_dps = 0.0f;
static float g_mismatch = 0.0f;
static float g_fore = 0.0f;                // forearm inclination from gravity (deg)
static ArmPlacement g_place = {0, 0, 0, 0};
static float g_weight_kg = 0, g_force_n = 0;
static int   g_pwm = 0;
static uint32_t g_tick = 0;
static uint8_t  g_mismatchTicks = 0;
static uint8_t  g_overspeedTicks = 0;

static void enterFault(Fault f) {
    Motor_Safe();
    pid.reset();
    slew.reset();
    g_pwm = 0;
    if (g_state != State::FAULT) {                 // keep the FIRST cause
        g_state = State::FAULT;
        g_fault = f;
    }
}

static void startHardwareWatchdog() {
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload    = IWDG_RELOAD_COUNTS;
    HAL_IWDG_Init(&hiwdg);                          // cannot be stopped once started
}

// Debounced MODE-button press event (30 ms stable, fires once per press)
static bool modePressedEvent() {
    static uint8_t stable = 0;
    static bool    armed  = true;
    if (ModeButton_Pressed()) {
        if (stable < 255) stable++;
        if (stable >= 3 && armed) { armed = false; return true; }
    } else {
        stable = 0;
        armed  = true;
    }
    return false;
}

// AS5600 sample -> elbow angle in the joint frame (0 = extended, + = flexion)
static float elbowFromAs5600(float raw_deg) {
    return wrap180(AS5600_SIGN * (raw_deg - AS5600_ZERO_DEG));
}

static void bootSequence() {
    Motor_Safe();
    Uart_Write("\r\n[ExoArm] boot - keep the arm still and UNLOADED\r\n");

    bool imuOk = imu.begin();
    if (imuOk) imuOk = imu.calibrateGyro(200);
    Uart_Write(imuOk ? "[ExoArm] upper-arm IMU ok, gyro calibrated\r\n" : "[ExoArm] upper-arm IMU FAILED\r\n");

    bool asOk = as5600.begin();
    Uart_Write(asOk ? "[ExoArm] AS5600 ok, magnet detected\r\n" : "[ExoArm] AS5600 FAILED (no ACK or no magnet)\r\n");

    optEnc.begin();
    optEnc.readDeltaCounts();

    loadcell.begin();
    loadcell.setScale(LOADCELL_SCALE_COUNTS_PER_KG);
    bool lcOk = loadcell.tare(10);
    Uart_Write(lcOk ? "[ExoArm] load cell ok, tared\r\n" : "[ExoArm] LOAD CELL FAILED\r\n");

    float ax, ay, az, gx, gy, gz;
    if (imuOk && imu.readSensor(ax, ay, az, gx, gy, gz)) {
        g_ua = limbAngleFromAccel(ax, ay) + UA_ANGLE_OFFSET_DEG;
        upperArmFilter.reset(g_ua);
    }
    float rawDeg;
    if (asOk && as5600.readAngleDeg(rawDeg)) {                  // home the optical encoder to the absolute sensor
        g_as_deg = elbowFromAs5600(rawDeg);
        joint.reset(g_as_deg);
        g_elbow = g_as_deg;
    } else {
        asOk = false;
    }
    g_fore = wrap180(g_ua + g_elbow);

    imuWd.kick();
    asWd.kick();
    loadWd.kick();
    if (!imuOk)      enterFault(Fault::IMU);
    else if (!asOk)  enterFault(Fault::AS5600);
    else if (!lcOk)  enterFault(Fault::LOADCELL);
}

static void controlStep() {
    g_tick++;
    const bool estop = Estop_Pressed();
    const bool modeEvt = modePressedEvent();

    // ---------- 1. Sensors ----------
    // Upper arm (MPU-6050)
    float ax, ay, az, gx, gy, gz;
    if (imu.readSensor(ax, ay, az, gx, gy, gz)) {
        imuWd.kick();
        g_ax = ax; g_ay = ay; g_az = az;
        g_ua = upperArmFilter.update(UA_GYRO_SIGN * gz,
                                     limbAngleFromAccel(ax, ay) + UA_ANGLE_OFFSET_DEG,
                                     accelMagnitudeValid(ax, ay, az), CONTROL_LOOP_DT_SEC);
    }

    // Elbow: AS5600 (absolute, I2C2) + optical encoder (incremental, TIM3) -> fused joint angle
    float rawDeg;
    bool  asValid = as5600.readAngleDeg(rawDeg);
    if (asValid) {
        asWd.kick();
        g_as_deg = elbowFromAs5600(rawDeg);
    }
    const float optDelta = (float)optEnc.readDeltaCounts() * ENCODER_DEG_PER_COUNT;   // joint degrees this tick
    joint.update(optDelta, g_as_deg, asValid);
    g_elbow     = joint.angle();
    g_elbow_dps = joint.velocity();
    g_mismatch  = joint.mismatch();

    // Forearm placement = upper arm + elbow
    g_fore  = wrap180(g_ua + g_elbow);
    g_place = forearmPlacement(g_ua, g_fore, UPPER_ARM_LENGTH_M, FOREARM_LENGTH_M);

    // Load cell
    int32_t raw;
    if (loadcell.dataReady() && loadcell.read(raw, 0)) {      // non-blocking: HX711 only makes 10 samples/s
        loadWd.kick();
        g_weight_kg = LOADCELL_FORCE_SIGN * loadcell.rawToUnits(raw);
        g_force_n   = g_weight_kg * GRAVITY_MS2;
    }
    const float force_f = forceLpf.update(g_force_n);

    // Persistence counters for the encoder cross-check and the speed limit
    if (asValid) g_mismatchTicks = (std::fabs(g_mismatch) > ENCODER_MISMATCH_DEG)
                                   ? (uint8_t)(g_mismatchTicks < 255 ? g_mismatchTicks + 1 : 255) : 0;
    g_overspeedTicks = (std::fabs(g_elbow_dps) > JOINT_SPEED_FAULT_DPS)
                       ? (uint8_t)(g_overspeedTicks < 255 ? g_overspeedTicks + 1 : 255) : 0;

    // ---------- 2. Supervision (any state) ----------
    if (estop)                                    enterFault(Fault::ESTOP);
    else if (imuWd.expired())                     enterFault(Fault::IMU);
    else if (asWd.expired())                      enterFault(Fault::AS5600);
    else if (loadWd.expired())                    enterFault(Fault::LOADCELL);
    else if (g_mismatchTicks >= ENCODER_MISMATCH_TICKS)
                                                  enterFault(Fault::ENC_MISMATCH);
    else if (g_state == State::ASSIST) {
        if (force_f > FORCE_FAULT_N || force_f < -FORCE_FAULT_N)
                                                  enterFault(Fault::OVERFORCE);
        else if (g_overspeedTicks >= 3)           enterFault(Fault::OVERSPEED);
        else if (g_elbow > ELBOW_MAX_DEG + ELBOW_HARD_MARGIN_DEG || g_elbow < ELBOW_MIN_DEG - ELBOW_HARD_MARGIN_DEG)
                                                  enterFault(Fault::JOINT_LIMIT);
    }

    // ---------- 3. State machine ----------
    switch (g_state) {
    case State::IDLE:
        Motor_Safe();
        if (modeEvt) {
            pid.reset(); slew.reset(); forceLpf.reset(g_force_n);
            Motor_Enable(true);
            g_state = State::ASSIST;
        }
        break;

    case State::ASSIST: {
        if (modeEvt) { Motor_Safe(); g_state = State::IDLE; g_pwm = 0; break; }

        // Force-following: drive the measured interaction force to zero by moving WITH the user.
        // error = 0 - (-F) = +F  ->  positive output = UP.
        float f   = applyDeadband(force_f, FORCE_DEADBAND_N);
        float cmd = pid.compute(0.0f, -f, CONTROL_LOOP_DT_SEC) * ASSIST_GAIN;

        // Soft joint limits on the true elbow angle: block motion INTO a limit, still allow motion back out of it.
        if (g_elbow >= ELBOW_MAX_DEG && cmd > 0.0f) cmd = 0.0f;
        if (g_elbow <= ELBOW_MIN_DEG && cmd < 0.0f) cmd = 0.0f;

        cmd = clampf(cmd, -(float)ASSIST_PWM_LIMIT, (float)ASSIST_PWM_LIMIT);
        g_pwm = (int)slew.update(cmd);
        Motor_Set((int16_t)g_pwm);
        break;
    }

    case State::FAULT:
        Motor_Safe();
        if (modeEvt && !estop) {                    // operator acknowledged; supervision re-faults if still bad
            g_fault = Fault::NONE;
            g_state = State::IDLE;
            g_mismatchTicks = 0;
            g_overspeedTicks = 0;
        }
        break;
    }

    // ---------- 4. Status LED ----------
    switch (g_state) {
        case State::IDLE:   Led_Set((g_tick % 100) < 10); break;   // short blip once a second
        case State::ASSIST: Led_Set(true); break;                  // solid
        case State::FAULT:  Led_Set((g_tick % 20) < 10);  break;   // 5 Hz blink
    }

    // ---------- 5. Telemetry (20 Hz, interrupt driven - never blocks the loop) ----------
    if ((g_tick % TELEMETRY_EVERY_N_TICKS) == 0) {
        TelemetryPacket p = { g_fore, g_weight_kg, g_force_n, g_ax, g_ay, g_az,
                              g_pwm, (int)g_state, faultName(g_fault),
                              g_ua, g_elbow, g_elbow_dps, g_mismatch,
                              g_place.wrist_x * 100.0f, g_place.wrist_y * 100.0f };
        char buf[TELEM_BUF_SIZE];
        size_t n = Telemetry_Format(buf, sizeof(buf), p);
        Uart_WriteAsync(buf, n);
    }
}

int main(void) {
    Board_Init();               // clock, GPIO, PWM (0 %, driver disabled), encoder, I2C x2, UART
    bootSequence();
    startHardwareWatchdog();    // after the slow boot steps

    uint32_t next = HAL_GetTick() + CONTROL_LOOP_PERIOD_MS;
    while (1) {
        while ((int32_t)(HAL_GetTick() - next) < 0) { __WFI(); }   // sleep until the next SysTick
        next += CONTROL_LOOP_PERIOD_MS;
        if ((int32_t)(HAL_GetTick() - next) > 5 * CONTROL_LOOP_PERIOD_MS) next = HAL_GetTick();  // overrun: resync

        HAL_IWDG_Refresh(&hiwdg);
        controlStep();
    }
}
