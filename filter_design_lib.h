#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 设计单个二阶节的 ba 系数。
 * 输出格式固定为 6 个数：
 *   [b0, b1, b2, a0, a1, a2]
 * 其中 a0 恒为 1，保留 6 维以兼容现有滤波链路。
 *
 * order 目前按二阶滤波器设计，保留参数用于后续扩展和校验。
 * type: 0=低通, 1=高通
 * 返回值：0 成功，非 0 失败。
 */
int mil_design_ba_coeffs(int order,
                         float sampleHz,
                         float cutoffHz,
                         int type,
                         double ba[6]);

#ifdef __cplusplus
}
#endif
