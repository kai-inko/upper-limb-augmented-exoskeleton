#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <cstddef>
#include <cstdint>

#define TELEM_BUF_SIZE 192

// Tiny printf replacement. Avoids newlib's float printf (which needs "-u _printf_float"
// and ~10 kB of flash).
class LineBuilder {
public:
    LineBuilder(char* buf, size_t cap) : buf_(buf), cap_(cap), len_(0) { if (cap_) buf_[0] = '\0'; }
    LineBuilder& str(const char* s);
    LineBuilder& ch(char c);
    LineBuilder& i(int32_t v);
    LineBuilder& u(uint32_t v);
    LineBuilder& x(uint32_t v, uint8_t digits = 2);      // "0x.." hexadecimal
    LineBuilder& f(float v, uint8_t decimals = 2);
    size_t      len() const { return len_; }
    const char* c_str() const { return buf_; }
private:
    char*  buf_;
    size_t cap_;
    size_t len_;
};

// Packet understood by visualize_force_vector.py (fields 1..9 are the original layout, 10..15 are new):
// $TELEM,FOREARM_DEG,WEIGHT_KG,FORCE_N,AX,AY,AZ,PWM_PERMILLE,MODE,STATUS,UA_DEG,ELBOW_DEG,ELBOW_DPS,MISMATCH_DEG,WRIST_X_CM,WRIST_Y_CM\r\n
struct TelemetryPacket {
    float angle_deg;        // forearm inclination from gravity = upper-arm angle + elbow angle
    float weight_kg, force_n;
    float ax, ay, az;       // upper-arm IMU accelerometer (g)
    int   pwm;              // -1000..1000
    int   mode;             // 0 = IDLE, 1 = ASSIST, 2 = FAULT
    const char* status;
    float ua_deg;           // upper-arm inclination (MPU-6050)
    float elbow_deg;        // elbow joint angle (AS5600 + optical encoder fusion), 0 = extended
    float elbow_dps;        // elbow speed (optical encoder)
    float mismatch_deg;     // AS5600 vs optical disagreement
    float wrist_x_cm, wrist_y_cm;   // forearm placement: wrist/cuff relative to the shoulder (x forward, y up)
};

size_t Telemetry_Format(char* buf, size_t cap, const TelemetryPacket& p);

#endif // TELEMETRY_H
