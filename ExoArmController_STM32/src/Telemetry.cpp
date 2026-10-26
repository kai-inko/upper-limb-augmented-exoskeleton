#include "Telemetry.h"

LineBuilder& LineBuilder::ch(char c) {
    if (len_ + 1 < cap_) { buf_[len_++] = c; buf_[len_] = '\0'; }
    return *this;
}

LineBuilder& LineBuilder::str(const char* s) {
    while (*s) ch(*s++);
    return *this;
}

LineBuilder& LineBuilder::u(uint32_t v) {
    char tmp[11]; int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n) ch(tmp[--n]);
    return *this;
}

LineBuilder& LineBuilder::i(int32_t v) {
    if (v < 0) { ch('-'); return u((uint32_t)(-(int64_t)v)); }
    return u((uint32_t)v);
}

LineBuilder& LineBuilder::f(float v, uint8_t decimals) {
    if (v != v) return str("nan");
    if (decimals > 5) decimals = 5;
    uint32_t scale = 1;
    for (uint8_t k = 0; k < decimals; k++) scale *= 10;

    bool  neg = v < 0.0f;
    float a   = neg ? -v : v;
    float s   = a * (float)scale;
    if (!(s < 4.0e9f)) s = 4.0e9f;                    // clamp (also catches +inf)
    uint32_t n = (uint32_t)(s + 0.5f);
    if (n == 0) neg = false;                          // never print "-0.00"

    if (neg) ch('-');
    u(n / scale);
    if (decimals) {
        ch('.');
        uint32_t frac = n % scale;
        for (uint32_t d = scale / 10; d >= 1; d /= 10) {
            ch((char)('0' + (frac / d) % 10));
            if (d == 1) break;
        }
    }
    return *this;
}

size_t Telemetry_Format(char* buf, size_t cap, const TelemetryPacket& p) {
    LineBuilder b(buf, cap);
    b.str("$TELEM,").f(p.angle_deg, 1).ch(',').f(p.weight_kg, 3).ch(',').f(p.force_n, 2).ch(',')
     .f(p.ax, 2).ch(',').f(p.ay, 2).ch(',').f(p.az, 2).ch(',')
     .i(p.pwm).ch(',').i(p.mode).ch(',').str(p.status).str("\r\n");
    return b.len();
}
