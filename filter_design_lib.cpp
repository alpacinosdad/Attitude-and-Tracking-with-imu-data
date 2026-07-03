#include "filter_design_lib.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int mil_design_ba_coeffs(int order,
                         float sampleHz,
                         float cutoffHz,
                         int type,
                         double ba[6])
{
    if (!ba || sampleHz <= 0.0f || cutoffHz <= 0.0f)
        return -1;

    if (order != 1)
        return -2;

    const double fs = sampleHz;
    const double fc = cutoffHz;
    const double k = std::tan(M_PI * fc / fs);
    const double a1 = (k - 1.0) / (k + 1.0);

    double b0 = 0.0;
    double b1 = 0.0;

    if (type == 0) {
        b0 = k / (1.0 + k);
        b1 = b0;
    } else {
        b0 = 1.0 / (1.0 + k);
        b1 = -b0;
    }

    ba[0] = b0;
    ba[1] = b1;
    ba[2] = 0.0;
    ba[3] = 1.0;
    ba[4] = a1;
    ba[5] = 0.0;
    return 0;
}


