#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <cstddef>
#include <cstdint>

// Tiny printf replacement. Avoids newlib's float printf (which needs "-u _printf_float"
// and ~10 kB of flash).
class LineBuilder {
public:
    LineBuilder(char* buf, size_t cap) : buf_(buf), cap_(cap), len_(0) { if (cap_) buf_[0] = '\0'; }
    LineBuilder& str(const char* s);
    LineBuilder& ch(char c);
    LineBuilder& i(int32_t v);
    LineBuilder& u(uint32_t v);
    LineBuilder& f(float v, uint8_t decimals = 2);
    size_t      len() const { return len_; }
    const char* c_str() const { return buf_; }
private:
    char*  buf_;
    size_t cap_;
    size_t len_;
};

// Packet understood by visualize_force_vector.py:
// $TELEM,ANGLE_DEG,WEIGHT_KG,FORCE_N,AX,AY,AZ,PWM_PERMILLE,MODE,STATUS\r\n
struct TelemetryPacket {
    float angle_deg, weight_kg, force_n, ax, ay, az;
    int   pwm;       // -1000..1000
    int   mode;      // 0 = IDLE, 1 = ASSIST, 2 = FAULT
    const char* status;
};

size_t Telemetry_Format(char* buf, size_t cap, const TelemetryPacket& p);

#endif // TELEMETRY_H
