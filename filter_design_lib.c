int mil_design_ba_coeffs(int order,
    float sampleHz,
    float cutoffHz,
    int type,
    double ba[6])
{
if (!ba || sampleHz <= 0.0f || cutoffHz <= 0.0f)
return -1;

const double fs = sampleHz;
const double fc = cutoffHz;
const double k = tan(M_PI * fc / fs);

if (order == 1) {
double b0, b1, b2 = 0.0;
double a1, a2 = 0.0;

if (type == 0) {
b0 = k / (1.0 + k);
b1 = b0;
} else {
b0 = 1.0 / (1.0 + k);
b1 = -b0;
}

a1 = (k - 1.0) / (k + 1.0);

ba[0] = b0;
ba[1] = b1;
ba[2] = b2;
ba[3] = 1.0;
ba[4] = a1;
ba[5] = a2;
return 0;
}

return -2;
}