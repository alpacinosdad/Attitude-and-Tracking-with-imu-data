#include"micro_imu_lib.h"
#include<math.h>

//是否计算绝对坐标系加速度
#define EN_ACC_ABS 1
//是否使用重力加速度修正
#define EN_ACC_FIX 1
//重力系数
#define GRAV_FACT 9.8f
//加速度计修正系数
#define ACC_FIX_FACT_MAX 0.8f
//加速度计修正指数衰减系数
#define ATTEN_FACT 100
//四元数初始化所需的原始样本数
#define INIT_QUAT_SAMPLE_COUNT 20
//静止判定所需样本数（约 1s@50Hz）
#define STATIONARY_SAMPLE_COUNT 50
//静止判定阈值
#define GYRO_STATIONARY_THRESH 0.15f   // rad/s
#define ACC_STATIONARY_TOL 1.0f        // m/s^2

//初始化
void mil_handle_init(MIL_Handle_t*p);
//获取传感器值（rad/s）（m/s）（s）
void mil_get_imu(MIL_Handle_t*p, float gx, float gy, float gz, float ax, float ay, float az, float time);
//获取绝对坐标系加速度
void mil_get_aaccel(MIL_Handle_t*p, float* aax, float* aay, float* aaz);
//获取线性加速度
void mil_get_alin(MIL_Handle_t*p, float* lax, float* lay, float* laz);
//获取欧拉角
void mil_get_ang(MIL_Handle_t*p, float* roll, float* pitch, float* yaw);
//实时解算
void mil_while_run(MIL_Handle_t*p);
//加速度计姿态修正
static void acce_fix(float q0, float q1, float q2, float q3, float ax, float ay, float az, float* egx, float* egy, float* egz);
//陀螺仪姿态解算
static void gyro_solu(float iq0, float iq1, float iq2, float iq3, float gx, float gy, float gz, float time, float *oq0, float *oq1, float *oq2, float *oq3);
//四元数旋转矩阵计算
static void quat_to_rotmat(float q0, float q1, float q2, float q3, MIL_RotMat_t* R);
//绝对坐标系加速度
static void acce_to_abs(float ax, float ay, float az, MIL_RotMat_t* R, float *aax, float *aay, float *aaz);
//去重力后的线性加速度
static void acce_to_linear(float ax, float ay, float az, MIL_RotMat_t* R, float *lax, float *lay, float *laz);
//由加速度估算初始四元数
static void quat_from_accel(float ax, float ay, float az, MIL_Quat_t* q);
//四元数转欧拉角
static void quat_to_attu(MIL_RotMat_t* R, float* roll, float* pitch, float* yaw);

void mil_handle_init(MIL_Handle_t*p)
{
    if(!p)
    {
        return;
    }
    p->imuData.ax = 0.0f;
    p->imuData.ay = 0.0f;
    p->imuData.az = 0.0f;
    p->imuData.gx = 0.0f;
    p->imuData.gy = 0.0f;
    p->imuData.gz = 0.0f;
    p->imuData.time = 0.0f;

    p->quat.w = 1.0f;
    p->quat.x = 0.0f;
    p->quat.y = 0.0f;
    p->quat.z = 0.0f;

    p->rotMat.R11 = 1.0f; p->rotMat.R12 = 0.0f; p->rotMat.R13 = 0.0f;
    p->rotMat.R21 = 0.0f; p->rotMat.R22 = 1.0f; p->rotMat.R23 = 0.0f;
    p->rotMat.R31 = 0.0f; p->rotMat.R32 = 0.0f; p->rotMat.R33 = 1.0f;

    p->attiAng.roll  = 0.0f;
    p->attiAng.pitch = 0.0f;
    p->attiAng.yaw   = 0.0f;

    p->acceAbs.x = 0.0f;
    p->acceAbs.y = 0.0f;
    p->acceAbs.z = 0.0f;
    p->acceLin.x = 0.0f;
    p->acceLin.y = 0.0f;
    p->acceLin.z = 0.0f;

    p->lastTime = 0.0f;
    p->initSampleCount = 0;
    p->initSampleTarget = INIT_QUAT_SAMPLE_COUNT;
    p->initAxSum = 0.0f;
    p->initAySum = 0.0f;
    p->initAzSum = 0.0f;
    p->isQuatInited = 0;
    p->stationaryCount = 0;
    p->stationaryTarget = STATIONARY_SAMPLE_COUNT;
    p->gyroBiasX = 0.0f;
    p->gyroBiasY = 0.0f;
    p->gyroBiasZ = 0.0f;
    p->biasGxSum = 0.0f;
    p->biasGySum = 0.0f;
    p->biasGzSum = 0.0f;
}

void mil_get_imu(MIL_Handle_t*p, float gx, float gy, float gz, float ax, float ay, float az, float time)
{
    if(!p)
    {
        return;
    }
    p->imuData.gx = gx;
    p->imuData.gy = gy;
    p->imuData.gz = gz;
    p->imuData.ax = ax;
    p->imuData.ay = ay;
    p->imuData.az = az;
    p->imuData.time = time;
}

void mil_get_aaccel(MIL_Handle_t*p, float* aax, float* aay, float* aaz)
{
    if(!p)
    {
        return;
    }
    if(aax) *aax = p->acceAbs.x;
    if(aay) *aay = p->acceAbs.y;
    if(aaz) *aaz = p->acceAbs.z;
}

void mil_get_alin(MIL_Handle_t*p, float* lax, float* lay, float* laz)
{
    if(!p)
    {
        return;
    }
    if(lax) *lax = p->acceLin.x;
    if(lay) *lay = p->acceLin.y;
    if(laz) *laz = p->acceLin.z;
}

void mil_get_ang(MIL_Handle_t*p, float* roll, float* pitch, float* yaw)
{
    if(!p)
    {
        return;
    }
    if(roll) *roll = p->attiAng.roll;
    if(pitch) *pitch = p->attiAng.pitch;
    if(yaw) *yaw = p->attiAng.yaw;
}

static void reset_stationary_window(MIL_Handle_t* p)
{
    p->stationaryCount = 0;
    p->biasGxSum = 0.0f;
    p->biasGySum = 0.0f;
    p->biasGzSum = 0.0f;
}

static int is_stationary_sample(float gx, float gy, float gz, float ax, float ay, float az)
{
    float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    return (gyro_norm < GYRO_STATIONARY_THRESH && fabsf(acc_norm - GRAV_FACT) < ACC_STATIONARY_TOL);
}

void mil_while_run(MIL_Handle_t*p)
{
    if(!p)
    {
        return;
    }

    if(!p->isQuatInited)
    {
        p->initAxSum += p->imuData.ax;
        p->initAySum += p->imuData.ay;
        p->initAzSum += p->imuData.az;
        p->initSampleCount++;

        if(p->initSampleCount >= p->initSampleTarget)
        {
            float avgAx = p->initAxSum / (float)p->initSampleCount;
            float avgAy = p->initAySum / (float)p->initSampleCount;
            float avgAz = p->initAzSum / (float)p->initSampleCount;
            quat_from_accel(avgAx, avgAy, avgAz, &p->quat);
            quat_to_rotmat(p->quat.w, p->quat.x, p->quat.y, p->quat.z, &p->rotMat);
            quat_to_attu(&p->rotMat, &p->attiAng.roll, &p->attiAng.pitch, &p->attiAng.yaw);
            p->isQuatInited = 1;
            p->lastTime = p->imuData.time;
        }
        return;
    }

    if(is_stationary_sample(p->imuData.gx, p->imuData.gy, p->imuData.gz, p->imuData.ax, p->imuData.ay, p->imuData.az))
    {
        p->stationaryCount++;
        p->biasGxSum += p->imuData.gx;
        p->biasGySum += p->imuData.gy;
        p->biasGzSum += p->imuData.gz;
        if(p->stationaryCount >= p->stationaryTarget)
        {
            float inv = 1.0f / (float)p->stationaryCount;
            p->gyroBiasX = p->biasGxSum * inv;
            p->gyroBiasY = p->biasGySum * inv;
            p->gyroBiasZ = p->biasGzSum * inv;
            reset_stationary_window(p);
        }
    }
    else
    {
        reset_stationary_window(p);
    }

    float fix_fact=0;
    float egx=0,egy=0,egz=0;
    if(EN_ACC_FIX)
    {
        acce_fix(p->quat.w, p->quat.x, p->quat.y, p->quat.z, p->imuData.ax, p->imuData.ay, p->imuData.az, &egx, &egy, &egz);
        float acc_norm = sqrtf(p->imuData.ax * p->imuData.ax + p->imuData.ay * p->imuData.ay + p->imuData.az * p->imuData.az);
        float grav_diff = fabsf(acc_norm - GRAV_FACT);
        fix_fact = ACC_FIX_FACT_MAX * expf(-ATTEN_FACT * grav_diff);
    }

    if(p->lastTime <= 0.0f)
    {
        p->lastTime = p->imuData.time;
        return;
    }

    float curr_time = p->imuData.time;
    float err_time = curr_time - p->lastTime;
    p->lastTime = curr_time;

    float gx = p->imuData.gx - p->gyroBiasX + egx * fix_fact;
    float gy = p->imuData.gy - p->gyroBiasY + egy * fix_fact;
    float gz = p->imuData.gz - p->gyroBiasZ + egz * fix_fact;

    gyro_solu(p->quat.w, p->quat.x, p->quat.y, p->quat.z, gx, gy, gz, err_time, &p->quat.w, &p->quat.x, &p->quat.y, &p->quat.z);
    quat_to_rotmat(p->quat.w, p->quat.x, p->quat.y, p->quat.z, &p->rotMat);
    if(EN_ACC_ABS)
    {
        acce_to_abs(p->imuData.ax, p->imuData.ay, p->imuData.az, &p->rotMat, &p->acceAbs.x, &p->acceAbs.y, &p->acceAbs.z);
        acce_to_linear(p->imuData.ax, p->imuData.ay, p->imuData.az, &p->rotMat, &p->acceLin.x, &p->acceLin.y, &p->acceLin.z);
    }
    quat_to_attu(&p->rotMat, &p->attiAng.roll, &p->attiAng.pitch, &p->attiAng.yaw);
}

static void acce_fix(float q0, float q1, float q2, float q3, float ax, float ay, float az, float* egx, float* egy, float* egz)
{
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q1q1 = q1*q1;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    float qgx = 2*(q1q3 - q0q2);
    float qgy = 2*(q0q1 + q2q3);
    float qgz = q0q0 - q1q1 - q2q2 + q3q3;
    *egx = (ay*qgz - az*qgy);
    *egy = (az*qgx - ax*qgz);
    *egz = (ax*qgy - ay*qgx);
}

static void gyro_solu(float iq0, float iq1, float iq2, float iq3, float gx, float gy, float gz, float time, float *oq0, float *oq1, float *oq2, float *oq3)
{
    float temq0 = iq0 + (-iq1*gx - iq2*gy - iq3*gz) * time / 2;
    float temq1 = iq1 + ( iq0*gx + iq2*gz - iq3*gy) * time / 2;
    float temq2 = iq2 + ( iq0*gy - iq1*gz + iq3*gx) * time / 2;
    float temq3 = iq3 + ( iq0*gz + iq1*gy - iq2*gx) * time / 2;
    float norm = sqrtf(temq0*temq0 + temq1*temq1 + temq2*temq2 + temq3*temq3);
    if (norm > 1e-6f)
    {
        temq0 /= norm;
        temq1 /= norm;
        temq2 /= norm;
        temq3 /= norm;
    }
    *oq0 = temq0;
    *oq1 = temq1;
    *oq2 = temq2;
    *oq3 = temq3;
}

static void quat_to_rotmat(float q0, float q1, float q2, float q3, MIL_RotMat_t* R)
{
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;
    R->R11 = 1 - 2*(q2q2 + q3q3);
    R->R12 = 2*(q1q2 - q0q3);
    R->R13 = 2*(q1q3 + q0q2);
    R->R21 = 2*(q1q2 + q0q3);
    R->R22 = 1 - 2*(q1q1 + q3q3);
    R->R23 = 2*(q2q3 - q0q1);
    R->R31 = 2*(q1q3 - q0q2);
    R->R32 = 2*(q2q3 + q0q1);
    R->R33 = 1 - 2*(q1q1 + q2q2);
}

static void acce_to_abs(float ax, float ay, float az, MIL_RotMat_t* R, float *aax, float *aay, float *aaz)
{
    *aax = R->R11*ax + R->R12*ay + R->R13*az;
    *aay = R->R21*ax + R->R22*ay + R->R23*az;
    *aaz = R->R31*ax + R->R32*ay + R->R33*az;
}

static void acce_to_linear(float ax, float ay, float az, MIL_RotMat_t* R, float *lax, float *lay, float *laz)
{
    float gravity_body_x = R->R13 * GRAV_FACT;
    float gravity_body_y = R->R23 * GRAV_FACT;
    float gravity_body_z = R->R33 * GRAV_FACT;
    if(lax) *lax = ax - gravity_body_x;
    if(lay) *lay = ay - gravity_body_y;
    if(laz) *laz = az - gravity_body_z;
}

static void quat_from_accel(float ax, float ay, float az, MIL_Quat_t* q)
{
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 1e-6f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw = 0.0f;
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    q->w = cr*cp*cy + sr*sp*sy;
    q->x = sr*cp*cy - cr*sp*sy;
    q->y = cr*sp*cy + sr*cp*sy;
    q->z = cr*cp*sy - sr*sp*cy;
    float qnorm = sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
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
    *roll  = atan2(R->R32, R->R33) * 57.3f;
    *pitch = -asin(R->R13) * 57.3f;
    *yaw   = atan2(R->R12, R->R11) * 57.3f;
}
