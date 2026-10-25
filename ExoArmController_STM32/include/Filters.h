#ifndef FILTERS_H
#define FILTERS_H

#include <cmath>
#include <algorithm>
#include <cstdint>

class LowPassFilter {
public:
    LowPassFilter(float cutoff_freq, float sample_time) {
        float rc = 1.0f / (2.0f * 3.14159265f * cutoff_freq);
        alpha = sample_time / (rc + sample_time);
        prev_output = 0.0f;
    }

    float update(float input) {
        prev_output = prev_output + alpha * (input - prev_output);
        return prev_output;
    }

    void reset(float val = 0.0f) {
        prev_output = val;
    }

private:
    float alpha;
    float prev_output;
};

#endif // FILTERS_H