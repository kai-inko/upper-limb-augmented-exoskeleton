// ExoArm main application: 100 Hz force-following assist controller with layered safety.
//
//   IDLE   : driver disabled, 0 % PWM. MODE button -> ASSIST.
//   ASSIST : load-cell force (user pushing the cuff) drives a PID -> motor assists in the push direction.
//   FAULT  : latched. Driver disabled. Cleared only with E-stop released + MODE press.
#include "Board.h"
#include "Config.h"
#include "IMU_GY521.h"
#include "HX711_LoadCell.h"
#include "PIDController.h"
#include "SafetyWatchdog.h"
#include "Filters.h"
#include "Telemetry.h"

enum class State : uint8_t { IDLE = 0, ASSIST = 1, FAULT = 2 };
enum class Fault : uint8_t { NONE = 0, ESTOP, IMU, LOADCELL, OVERFORCE };

static const char* faultName(Fault f) {
    switch (f) {
        case Fault::ESTOP:     return "ESTOP";
        case Fault::IMU:       return "IMU_FAIL";
        case Fault::LOADCELL:  return "LC_FAIL";
        case Fault::OVERFORCE: return "OVERFORCE";
        default:               return "OK";
    }
}

// ---- Objects ----
static IMU_GY521      imu(&hi2c1, MPU6050_I2C_ADDR);
static HX711_LoadCell loadcell(HX711_DOUT_PORT, HX711_DOUT_PIN, HX711_SCK_PORT, HX711_SCK_PIN);
static PIDController  pid(PID_KP, PID_KI, PID_KD, -(float)ASSIST_PWM_LIMIT, (float)ASSIST_PWM_LIMIT);
static LowPassFilter  forceLpf(FORCE_LPF_CUTOFF_HZ, CONTROL_LOOP_DT_SEC);
static ComplementaryFilter angleFilter(IMU_COMP_ALPHA);
static SlewLimiter    slew(ASSIST_SLEW_PER_TICK);
static SafetyWatchdog imuWd(IMU_TIMEOUT_MS);
static SafetyWatchdog loadWd(LOADCELL_TIMEOUT_MS);
static IWDG_HandleTypeDef hiwdg;

// ---- State ----
static State g_state = State::IDLE;
static Fault g_fault = Fault::NONE;
static float g_ax = 0, g_ay = 0, g_az = 0;
static float g_angle = 90.0f;
static float g_weight_kg = 0, g_force_n = 0;
static int   g_pwm = 0;
static uint32_t g_tick = 0;

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

static void bootSequence() {
    Motor_Safe();
    Uart_Write("\r\n[ExoArm] boot - keep the arm still and UNLOADED\r\n");

    bool imuOk = imu.begin();
    if (imuOk) imuOk = imu.calibrateGyro(200);
    Uart_Write(imuOk ? "[ExoArm] IMU ok, gyro calibrated\r\n" : "[ExoArm] IMU FAILED\r\n");

    loadcell.begin();
    loadcell.setScale(LOADCELL_SCALE_COUNTS_PER_KG);
    bool lcOk = loadcell.tare(10);
    Uart_Write(lcOk ? "[ExoArm] load cell ok, tared\r\n" : "[ExoArm] LOAD CELL FAILED\r\n");

    float ax, ay, az, gx, gy, gz;
    if (imuOk && imu.readSensor(ax, ay, az, gx, gy, gz))
        angleFilter.reset(elbowAngleFromAccel(ax, ay) + IMU_ANGLE_OFFSET_DEG);

    imuWd.kick();
    loadWd.kick();
    if (!imuOk)      enterFault(Fault::IMU);
    else if (!lcOk)  enterFault(Fault::LOADCELL);
}

static void controlStep() {
    g_tick++;
    const bool estop = Estop_Pressed();
    const bool modeEvt = modePressedEvent();

    // ---------- 1. Sensors ----------
    float ax, ay, az, gx, gy, gz;
    if (imu.readSensor(ax, ay, az, gx, gy, gz)) {
        imuWd.kick();
        g_ax = ax; g_ay = ay; g_az = az;
        g_angle = angleFilter.update(IMU_GYRO_SIGN * gz,
                                     elbowAngleFromAccel(ax, ay) + IMU_ANGLE_OFFSET_DEG,
                                     accelMagnitudeValid(ax, ay, az), CONTROL_LOOP_DT_SEC);
    }

    int32_t raw;
    if (loadcell.dataReady() && loadcell.read(raw, 0)) {      // non-blocking: HX711 only makes 10 samples/s
        loadWd.kick();
        g_weight_kg = LOADCELL_FORCE_SIGN * loadcell.rawToUnits(raw);
        g_force_n   = g_weight_kg * GRAVITY_MS2;
    }
    const float force_f = forceLpf.update(g_force_n);

    // ---------- 2. Supervision (any state) ----------
    if (estop)                       enterFault(Fault::ESTOP);
    else if (imuWd.expired())        enterFault(Fault::IMU);
    else if (loadWd.expired())       enterFault(Fault::LOADCELL);
    else if (g_state == State::ASSIST && (force_f > FORCE_FAULT_N || force_f < -FORCE_FAULT_N))
                                     enterFault(Fault::OVERFORCE);

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

        // Soft joint limits: block motion INTO a limit, still allow motion back out of it.
        if (g_angle >= ANGLE_MAX_DEG && cmd > 0.0f) cmd = 0.0f;
        if (g_angle <= ANGLE_MIN_DEG && cmd < 0.0f) cmd = 0.0f;

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
        TelemetryPacket p = { g_angle, g_weight_kg, g_force_n, g_ax, g_ay, g_az,
                              g_pwm, (int)g_state, faultName(g_fault) };
        char buf[128];
        size_t n = Telemetry_Format(buf, sizeof(buf), p);
        Uart_WriteAsync(buf, n);
    }
}

int main(void) {
    Board_Init();               // clock, GPIO, PWM (0 %, driver disabled), I2C, UART
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
