#include "micro_imu_lib.h"   // 引入 IMU 库对应的头文件，包含结构体、函数声明等
#include <math.h>            // 引入数学库，使用 sqrtf / fabsf / atan2f / asinf / cosf / sinf / expf 等函数

// 是否计算绝对坐标系加速度：1=启用，0=禁用
#define EN_ACC_ABS 1

// 是否使用重力加速度修正姿态：1=启用，0=禁用
#define EN_ACC_FIX 1

// 重力加速度常量，单位 m/s^2
#define GRAV_FACT 9.8f

// 加速度计修正系数最大值，用于限制姿态修正强度
#define ACC_FIX_FACT_MAX 0.2f

// 加速度计修正指数衰减系数，越大则偏离重力越多时修正衰减越快
#define ATTEN_FACT 2

// 四元数初始化所需的原始样本数
#define INIT_QUAT_SAMPLE_COUNT 2

// 静止判定所需连续样本数（约 1s@50Hz）
#define STATIONARY_SAMPLE_COUNT 50

// 陀螺仪静止判定阈值，单位 rad/s
#define GYRO_STATIONARY_THRESH 0.05f

// 加速度模长与重力值的允许偏差，单位 m/s^2
#define ACC_STATIONARY_TOL 0.5f


// 初始化句柄：清空内部状态、初始化四元数、旋转矩阵、角度、偏置等
void mil_handle_init(MIL_Handle_t* p);

// 写入一帧 IMU 原始数据：角速度(rad/s)、加速度(m/s^2)、时间戳(s)
void mil_get_imu(MIL_Handle_t* p, float gx, float gy, float gz, float ax, float ay, float az, float time);

// 获取绝对坐标系加速度（世界坐标系/导航坐标系）
void mil_get_aaccel(MIL_Handle_t* p, float* aax, float* aay, float* aaz);

// 获取线性加速度（去掉重力项后的 body 系加速度）
void mil_get_alin(MIL_Handle_t* p, float* lax, float* lay, float* laz);

// 获取欧拉角（roll / pitch / yaw）
void mil_get_ang(MIL_Handle_t* p, float* roll, float* pitch, float* yaw);

// 实时解算主函数：每次输入一帧 imuData 后调用一次
void mil_while_run(MIL_Handle_t* p);

// ------------------------------
// 以下为静态内部函数，仅在本文件内可见
// ------------------------------

// 加速度计姿态修正：根据当前四元数估计重力方向，与实际加速度方向比较生成误差项
static void acce_fix(float q0, float q1, float q2, float q3,
                     float ax, float ay, float az,
                     float* egx, float* egy, float* egz);

// 陀螺仪姿态解算：使用角速度对四元数积分
static void gyro_solu(float iq0, float iq1, float iq2, float iq3,
                      float gx, float gy, float gz, float time,
                      float *oq0, float *oq1, float *oq2, float *oq3);

// 由四元数计算旋转矩阵
static void quat_to_rotmat(float q0, float q1, float q2, float q3, MIL_RotMat_t* R);

// 将 body 系加速度转换为绝对坐标系加速度
static void acce_to_abs(float ax, float ay, float az, MIL_RotMat_t* R,
                        float *aax, float *aay, float *aaz);

// 计算去重力后的线性加速度
static void acce_to_linear(float ax, float ay, float az, MIL_RotMat_t* R,
                           float *lax, float *lay, float *laz);

// 根据加速度初始值估算初始四元数（只能确定 roll / pitch，yaw 默认为 0）
static void quat_from_accel(float ax, float ay, float az, MIL_Quat_t* q);

// 旋转矩阵转欧拉角
static void quat_to_attu(MIL_RotMat_t* R, float* roll, float* pitch, float* yaw);

// 重置静止检测窗口状态
static void reset_stationary_window(MIL_Handle_t* p);

// 对单帧数据执行“是否静止”的判断
static int is_stationary_sample(float gx, float gy, float gz, float ax, float ay, float az);

// 更新陀螺仪零偏（gyro bias）
static void update_gyro_bias(MIL_Handle_t* p, float newBiasX, float newBiasY, float newBiasZ);


void mil_handle_init(MIL_Handle_t* p)
{
    // 空指针保护：句柄为空则直接返回
    if(!p)
    {
        return;
    }

    // 初始化 IMU 原始数据：加速度全置 0
    p->imuData.ax = 0.0f;
    p->imuData.ay = 0.0f;
    p->imuData.az = 0.0f;

    // 初始化 IMU 原始数据：角速度全置 0
    p->imuData.gx = 0.0f;
    p->imuData.gy = 0.0f;
    p->imuData.gz = 0.0f;

    // 初始化时间戳为 0
    p->imuData.time = 0.0f;

    // 初始化四元数为单位四元数，表示“无旋转”
    p->quat.w = 1.0f;
    p->quat.x = 0.0f;
    p->quat.y = 0.0f;
    p->quat.z = 0.0f;

    // 初始化旋转矩阵为单位阵
    p->rotMat.R11 = 1.0f; p->rotMat.R12 = 0.0f; p->rotMat.R13 = 0.0f;
    p->rotMat.R21 = 0.0f; p->rotMat.R22 = 1.0f; p->rotMat.R23 = 0.0f;
    p->rotMat.R31 = 0.0f; p->rotMat.R32 = 0.0f; p->rotMat.R33 = 1.0f;

    // 初始化欧拉角为 0
    p->attiAng.roll  = 0.0f;
    p->attiAng.pitch = 0.0f;
    p->attiAng.yaw   = 0.0f;

    // 初始化绝对坐标系加速度为 0
    p->acceAbs.x = 0.0f;
    p->acceAbs.y = 0.0f;
    p->acceAbs.z = 0.0f;

    // 初始化线性加速度为 0
    p->acceLin.x = 0.0f;
    p->acceLin.y = 0.0f;
    p->acceLin.z = 0.0f;

    // 上一帧时间初始化为 0
    p->lastTime = 0.0f;

    // 初始化四元数时，已经采集到的样本数置 0
    p->initSampleCount = 0;

    // 初始化四元数目标样本数，使用宏定义值
    p->initSampleTarget = INIT_QUAT_SAMPLE_COUNT;

    // 初始化累加器：用于求取初始加速度均值
    p->initAxSum = 0.0f;
    p->initAySum = 0.0f;
    p->initAzSum = 0.0f;

    // 标记四元数尚未初始化
    p->isQuatInited = 0;

    // 静止计数器初始化为 0
    p->stationaryCount = 0;

    // 连续静止目标帧数
    p->stationaryTarget = STATIONARY_SAMPLE_COUNT;

    // 陀螺仪零偏初始为 0
    p->gyroBiasX = 0.0f;
    p->gyroBiasY = 0.0f;
    p->gyroBiasZ = 0.0f;

    // bias 采样和初始化为 0
    p->biasGxSum = 0.0f;
    p->biasGySum = 0.0f;
    p->biasGzSum = 0.0f;

    /*
     * 新增静止窗口状态。
     *
     * stationaryPhase:
     *   0 = 静止确认阶段；
     *   1 = bias采样阶段。
     *
     * biasSampleCount:
     *   bias采样阶段的采样计数。
     *
     * isGyroBiasInited:
     *   0 = 还没有完成第一次gyro bias初始化；
     *   1 = 已经有初始gyro bias，后续只做慢更新。
     */

    // 静止状态机阶段：0 表示连续静止确认阶段
    p->stationaryPhase = 0;

    // bias 采样阶段采到的样本数
    p->biasSampleCount = 0;

    // 标记陀螺仪零偏尚未完成首次初始化
    p->isGyroBiasInited = 0;
}


void mil_get_imu(MIL_Handle_t* p, float gx, float gy, float gz,
                 float ax, float ay, float az, float time)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    // 更新当前帧陀螺仪数据
    p->imuData.gx = gx;
    p->imuData.gy = gy;
    p->imuData.gz = gz;

    // 更新当前帧加速度计数据
    p->imuData.ax = ax;
    p->imuData.ay = ay;
    p->imuData.az = az;

    // 更新时间戳
    p->imuData.time = time;
}


void mil_get_aaccel(MIL_Handle_t* p, float* aax, float* aay, float* aaz)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    // 若输出指针不为空，则返回绝对坐标系 X 方向加速度
    if(aax) *aax = p->acceAbs.x;

    // 若输出指针不为空，则返回绝对坐标系 Y 方向加速度
    if(aay) *aay = p->acceAbs.y;

    // 若输出指针不为空，则返回绝对坐标系 Z 方向加速度
    if(aaz) *aaz = p->acceAbs.z;
}


void mil_get_alin(MIL_Handle_t* p, float* lax, float* lay, float* laz)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    // 若输出指针不为空，则返回线性加速度 X 分量
    if(lax) *lax = p->acceLin.x;

    // 若输出指针不为空，则返回线性加速度 Y 分量
    if(lay) *lay = p->acceLin.y;

    // 若输出指针不为空，则返回线性加速度 Z 分量
    if(laz) *laz = p->acceLin.z;
}


void mil_get_ang(MIL_Handle_t* p, float* roll, float* pitch, float* yaw)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    // 若输出指针不为空，则返回横滚角 roll
    if(roll) *roll = p->attiAng.roll;

    // 若输出指针不为空，则返回俯仰角 pitch
    if(pitch) *pitch = p->attiAng.pitch;

    // 若输出指针不为空，则返回偏航角 yaw
    if(yaw) *yaw = p->attiAng.yaw;
}


static void reset_stationary_window(MIL_Handle_t* p)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    // 连续静止计数清零
    p->stationaryCount = 0;

    // 恢复到“静止确认阶段”
    p->stationaryPhase = 0;

    // bias 采样计数清零
    p->biasSampleCount = 0;

    // bias 累加器清零
    p->biasGxSum = 0.0f;
    p->biasGySum = 0.0f;
    p->biasGzSum = 0.0f;
}


static int is_stationary_sample(float gx, float gy, float gz,
                                float ax, float ay, float az)
{
    // 计算当前陀螺仪角速度模长
    float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    // 计算当前加速度模长
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    // 同时满足：
    // 1. 角速度足够小（近似不转动）
    // 2. 加速度模长接近重力（近似没有线性运动）
    return (gyro_norm < GYRO_STATIONARY_THRESH &&
            fabsf(acc_norm - GRAV_FACT) < ACC_STATIONARY_TOL);
}


static void update_gyro_bias(MIL_Handle_t* p,
                             float newBiasX, float newBiasY, float newBiasZ)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    /*
     * 第一次获得可靠静止窗口 bias：
     * 直接赋值。
     *
     * 原因：
     * 初始阶段如果 gyro bias 还没有建立，姿态积分会持续带入零偏。
     * 所以第一次可靠 bias 应该直接使用。
     */
    if(!p->isGyroBiasInited)
    {
        // 第一次 bias 初始化，直接覆盖当前零偏
        p->gyroBiasX = newBiasX;
        p->gyroBiasY = newBiasY;
        p->gyroBiasZ = newBiasZ;

        // 标记 gyro bias 已完成首次初始化
        p->isGyroBiasInited = 1;
        return;
    }

    /*
     * 后续运行时 bias 更新：
     * 不直接覆盖，而是慢更新 + 单次限幅。
     *
     * biasUpdateAlpha = 0.05:
     *   每次有效静止窗口只吸收 5% 的新bias差值。
     *
     * biasMaxStep = 0.003 rad/s:
     *   单次bias最大变化约 0.17 deg/s。
     *
     * 这样可以降低静止误判导致 gyroBias 被突然拉飞的风险。
     */
    {
        // bias 更新比例，越小越平滑
        const float biasUpdateAlpha = 0.15f;

        // 单次更新最大步长，防止 bias 突然跳变
        const float biasMaxStep = 0.003f;

        // 按比例计算本次偏置增量
        float dx = biasUpdateAlpha * (newBiasX - p->gyroBiasX);
        float dy = biasUpdateAlpha * (newBiasY - p->gyroBiasY);
        float dz = biasUpdateAlpha * (newBiasZ - p->gyroBiasZ);

        // 对 X 方向增量限幅
        if(dx > biasMaxStep) dx = biasMaxStep;
        if(dx < -biasMaxStep) dx = -biasMaxStep;

        // 对 Y 方向增量限幅
        if(dy > biasMaxStep) dy = biasMaxStep;
        if(dy < -biasMaxStep) dy = -biasMaxStep;

        // 对 Z 方向增量限幅
        if(dz > biasMaxStep) dz = biasMaxStep;
        if(dz < -biasMaxStep) dz = -biasMaxStep;

        // 更新 bias
        p->gyroBiasX += dx;
        p->gyroBiasY += dy;
        p->gyroBiasZ += dz;
    }
}


void mil_while_run(MIL_Handle_t* p)
{
    // 空指针保护
    if(!p)
    {
        return;
    }

    /*
     * 四元数初始化。
     * 当前保持你的参数 INIT_QUAT_SAMPLE_COUNT = 2 不变。
     */
    if(!p->isQuatInited)
    {
        // 累加加速度样本，用于求均值
        p->initAxSum += p->imuData.ax;
        p->initAySum += p->imuData.ay;
        p->initAzSum += p->imuData.az;

        // 已收集样本数 +1
        p->initSampleCount++;

        // 如果已达到初始化所需样本数，则开始利用平均加速度初始化姿态
        if(p->initSampleCount >= p->initSampleTarget)
        {
            // 计算加速度平均值
            float avgAx = p->initAxSum / (float)p->initSampleCount;
            float avgAy = p->initAySum / (float)p->initSampleCount;
            float avgAz = p->initAzSum / (float)p->initSampleCount;

            // 根据平均加速度估算初始四元数（主要确定 roll/pitch）
            quat_from_accel(avgAx, avgAy, avgAz, &p->quat);

            // 根据四元数生成旋转矩阵
            quat_to_rotmat(p->quat.w, p->quat.x, p->quat.y, p->quat.z, &p->rotMat);

            // 根据旋转矩阵生成欧拉角
            quat_to_attu(&p->rotMat, &p->attiAng.roll, &p->attiAng.pitch, &p->attiAng.yaw);

            // 标记四元数已完成初始化
            p->isQuatInited = 1;

            // 记录当前时间作为后续积分的上一时刻
            p->lastTime = p->imuData.time;
        }

        // 初始化阶段处理完毕后直接返回
        return;
    }

    /*
     * 静止点与gyro bias更新逻辑。
     *
     * 两阶段：
     *   phase 0: 连续静止确认阶段；
     *   phase 1: 确认静止后，重新采样gyro求bias均值。
     *
     * 注意：
     *   phase 0 的数据不参与 bias 均值；
     *   phase 1 的数据才参与 bias 均值。
     */
    if(is_stationary_sample(p->imuData.gx, p->imuData.gy, p->imuData.gz,
                            p->imuData.ax, p->imuData.ay, p->imuData.az))
    {
        // 若当前帧满足静止条件
        if(p->stationaryPhase == 0)
        {
            /*
             * 第一阶段：只确认连续静止。
             * 这里不累计 biasGxSum/biasGySum/biasGzSum。
             */

            // 连续静止计数 +1
            p->stationaryCount++;

            // 连续静止达到目标帧数，则进入第二阶段：重新采样求 bias
            if(p->stationaryCount >= p->stationaryTarget)
            {
                /*
                 * 已经连续 stationaryTarget 帧满足静止条件。
                 * 切换到第二阶段：重新采样 gyro 求 bias。
                 */

                // 状态机切换到 bias 采样阶段
                p->stationaryPhase = 1;

                // 采样计数清零
                p->biasSampleCount = 0;

                // bias 累加器清零，为重新采样做准备
                p->biasGxSum = 0.0f;
                p->biasGySum = 0.0f;
                p->biasGzSum = 0.0f;
            }
        }
        else
        {
            /*
             * 第二阶段：确认静止后，重新采样 gyro 数据。
             * 这一阶段的样本才用于计算 newBias。
             */

            // bias 采样计数 +1
            p->biasSampleCount++;

            // 累加静止状态下的陀螺仪原始输出，用于计算平均零偏
            p->biasGxSum += p->imuData.gx;
            p->biasGySum += p->imuData.gy;
            p->biasGzSum += p->imuData.gz;

            // 达到目标样本数后，计算新的 gyro bias
            if(p->biasSampleCount >= p->stationaryTarget)
            {
                // 求平均值的倒数，减少重复除法
                float inv = 1.0f / (float)p->biasSampleCount;

                // 计算三个方向上的平均零偏
                float newBiasX = p->biasGxSum * inv;
                float newBiasY = p->biasGySum * inv;
                float newBiasZ = p->biasGzSum * inv;

                // 更新内部保存的 gyro bias
                update_gyro_bias(p, newBiasX, newBiasY, newBiasZ);

                // 完成一次偏置更新后，重置静止检测窗口
                reset_stationary_window(p);
            }
        }
    }
    else
    {
        /*
         * 只要中途任意一帧不满足静止条件，
         * 无论处于静止确认阶段还是bias采样阶段，都全部重来。
         */

        // 出现非静止帧，则静止检测状态全部清空重置
        reset_stationary_window(p);
    }

    /*
     * 加速度计姿态修正。
     * acce_fix 内部已经对加速度进行了归一化。
     */

    // 最终施加到误差项上的修正系数
    float fix_fact = 0.0f;

    // 三轴姿态误差项
    float egx = 0.0f;
    float egy = 0.0f;
    float egz = 0.0f;

    // 如果启用加速度计修正
    if(EN_ACC_FIX)
    {
        // 根据当前四元数估计重力方向，并与加速度方向比较，得到误差项
        acce_fix(p->quat.w, p->quat.x, p->quat.y, p->quat.z,
                 p->imuData.ax, p->imuData.ay, p->imuData.az,
                 &egx, &egy, &egz);

        // 计算当前加速度模长
        float acc_norm = sqrtf(p->imuData.ax * p->imuData.ax +
                               p->imuData.ay * p->imuData.ay +
                               p->imuData.az * p->imuData.az);

        // 计算当前加速度模长与重力值的偏差
        float grav_diff = fabsf(acc_norm - GRAV_FACT);

        // 偏差越大，修正权重越小；偏差越接近重力，修正权重越大
        fix_fact = ACC_FIX_FACT_MAX * expf(-ATTEN_FACT * grav_diff);
    }

    // 如果 lastTime 尚未有效初始化，则记录当前时间并返回
    if(p->lastTime <= 0.0f)
    {
        p->lastTime = p->imuData.time;
        return;
    }

    // 当前时刻
    float curr_time = p->imuData.time;

    // 计算与上一帧的时间差 dt
    float err_time = curr_time - p->lastTime;

    // 更新 lastTime
    p->lastTime = curr_time;

    // 对陀螺仪原始值进行修正：
    // 1. 减去零偏
    // 2. 加上加速度计姿态误差反馈项
    float gx = p->imuData.gx - p->gyroBiasX + egx * fix_fact;
    float gy = p->imuData.gy - p->gyroBiasY + egy * fix_fact;
    float gz = p->imuData.gz - p->gyroBiasZ + egz * fix_fact;

    // 用修正后的角速度对四元数积分更新姿态
    gyro_solu(p->quat.w, p->quat.x, p->quat.y, p->quat.z,
              gx, gy, gz, err_time,
              &p->quat.w, &p->quat.x, &p->quat.y, &p->quat.z);

    // 根据更新后的四元数计算旋转矩阵
    quat_to_rotmat(p->quat.w, p->quat.x, p->quat.y, p->quat.z, &p->rotMat);

    // 若启用绝对坐标加速度计算
    if(EN_ACC_ABS)
    {
        // 将 body 系加速度旋转到绝对坐标系
        acce_to_abs(p->imuData.ax, p->imuData.ay, p->imuData.az,
                    &p->rotMat,
                    &p->acceAbs.x, &p->acceAbs.y, &p->acceAbs.z);

        // 计算去掉重力后的线性加速度
        acce_to_linear(p->imuData.ax, p->imuData.ay, p->imuData.az,
                       &p->rotMat,
                       &p->acceLin.x, &p->acceLin.y, &p->acceLin.z);
    }

    // 最后根据旋转矩阵求欧拉角输出
    quat_to_attu(&p->rotMat,
                 &p->attiAng.roll, &p->attiAng.pitch, &p->attiAng.yaw);
}


static void acce_fix(float q0, float q1, float q2, float q3,
                     float ax, float ay, float az,
                     float* egx, float* egy, float* egz)
{
    // 计算加速度向量模长
    float norm = sqrtf(ax*ax + ay*ay + az*az);

    // 若模长过小，说明当前加速度无效，直接返回零误差
    if(norm < 1e-6f)
    {
        *egx = 0.0f;
        *egy = 0.0f;
        *egz = 0.0f;
        return;
    }

    // 将加速度归一化，只保留方向信息
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 预先计算四元数组合项，减少重复运算
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    // 根据当前四元数，估算机体坐标系下的重力方向分量
    float qgx = 2.0f * (q1q3 - q0q2);
    float qgy = 2.0f * (q0q1 + q2q3);
    float qgz = q0q0 - q1q1 - q2q2 + q3q3;

    // 通过“实测加速度方向 × 估计重力方向”得到姿态误差
    *egx = ay*qgz - az*qgy;
    *egy = az*qgx - ax*qgz;
    *egz = ax*qgy - ay*qgx;
}


static void gyro_solu(float iq0, float iq1, float iq2, float iq3,
                      float gx, float gy, float gz, float time,
                      float *oq0, float *oq1, float *oq2, float *oq3)
{
    // 四元数微分方程离散积分（欧拉法），根据角速度更新四元数
    float temq0 = iq0 + (-iq1*gx - iq2*gy - iq3*gz) * time / 2.0f;
    float temq1 = iq1 + ( iq0*gx + iq2*gz - iq3*gy) * time / 2.0f;
    float temq2 = iq2 + ( iq0*gy - iq1*gz + iq3*gx) * time / 2.0f;
    float temq3 = iq3 + ( iq0*gz + iq1*gy - iq2*gx) * time / 2.0f;

    // 计算积分后四元数模长
    float norm = sqrtf(temq0*temq0 + temq1*temq1 + temq2*temq2 + temq3*temq3);

    // 归一化四元数，防止数值积分导致长度漂移
    if(norm > 1e-6f)
    {
        temq0 /= norm;
        temq1 /= norm;
        temq2 /= norm;
        temq3 /= norm;
    }

    // 输出更新后的四元数
    *oq0 = temq0;
    *oq1 = temq1;
    *oq2 = temq2;
    *oq3 = temq3;
}


static void quat_to_rotmat(float q0, float q1, float q2, float q3, MIL_RotMat_t* R)
{
    // 预先计算若干乘积项，减少重复乘法
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    // 根据四元数计算方向余弦矩阵（旋转矩阵）
    R->R11 = 1.0f - 2.0f*(q2q2 + q3q3);
    R->R12 = 2.0f*(q1q2 - q0q3);
    R->R13 = 2.0f*(q1q3 + q0q2);

    R->R21 = 2.0f*(q1q2 + q0q3);
    R->R22 = 1.0f - 2.0f*(q1q1 + q3q3);
    R->R23 = 2.0f*(q2q3 - q0q1);

    R->R31 = 2.0f*(q1q3 - q0q2);
    R->R32 = 2.0f*(q2q3 + q0q1);
    R->R33 = 1.0f - 2.0f*(q1q1 + q2q2);
}


static void acce_to_abs(float ax, float ay, float az, MIL_RotMat_t* R,
                        float *aax, float *aay, float *aaz)
{
    // 使用旋转矩阵将 body 系加速度映射到绝对坐标系
    *aax = R->R11*ax + R->R12*ay + R->R13*az;
    *aay = R->R21*ax + R->R22*ay + R->R23*az;
    *aaz = R->R31*ax + R->R32*ay + R->R33*az;
}


// mike nian 26.6.10 修改旋转矩阵
static void acce_to_linear(float ax, float ay, float az, MIL_RotMat_t* R,
                           float *lax, float *lay, float *laz)
{
    // 先求 R 的转置矩阵 Rt = R^T
    // 这里实际上只取到了转置矩阵第三列需要用到的三个元素
    float Rt13 = R->R31;
    float Rt23 = R->R32;
    float Rt33 = R->R33;

    // 世界系重力向量定义为 [0, 0, GRAV_FACT]
    // 利用 R^T 将世界系重力投影回 body 系，得到 body 系下的重力分量
    float gravity_body_x = Rt13 * GRAV_FACT;
    float gravity_body_y = Rt23 * GRAV_FACT;
    float gravity_body_z = Rt33 * GRAV_FACT;

    // 用原始加速度减去 body 系重力，即得到线性加速度
    if(lax) *lax = ax - gravity_body_x;
    if(lay) *lay = ay - gravity_body_y;
    if(laz) *laz = az - gravity_body_z;
}


static void quat_from_accel(float ax, float ay, float az, MIL_Quat_t* q)
{
    // 计算加速度模长
    float norm = sqrtf(ax*ax + ay*ay + az*az);

    // 若模长过小，则无法初始化，退回单位四元数
    if(norm < 1e-6f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }

    // 加速度归一化，只保留方向
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 根据重力方向估算 roll
    float roll = atan2f(ay, az);

    // 根据重力方向估算 pitch
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));

    // 仅凭加速度无法估算 yaw，因此设为 0
    float yaw = 0.0f;

    // 计算三个半角三角函数，用于欧拉角转四元数
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    // 由欧拉角构造四元数
    q->w = cr*cp*cy + sr*sp*sy;
    q->x = sr*cp*cy - cr*sp*sy;
    q->y = cr*sp*cy + sr*cp*sy;
    q->z = cr*cp*sy - sr*sp*cy;

    // 计算四元数模长
    float qnorm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);

    // 对四元数归一化，防止数值误差
    if(qnorm > 1e-6f)
    {
        q->w /= qnorm;
        q->x /= qnorm;
        q->y /= qnorm;
        q->z /= qnorm;
    }
}


static void quat_to_attu(MIL_RotMat_t* R, float* roll, float* pitch, float* yaw)
{
    // 根据旋转矩阵提取 roll 角，结果乘 57.3 近似实现弧度转角度
    *roll  = atan2f(R->R32, R->R33) * 57.3f;

    // 根据旋转矩阵提取 pitch 角
    *pitch = -asinf(R->R13) * 57.3f;

    // 根据旋转矩阵提取 yaw 角
    *yaw   = atan2f(R->R12, R->R11) * 57.3f;
}