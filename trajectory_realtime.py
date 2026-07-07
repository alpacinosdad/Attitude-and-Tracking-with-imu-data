import numpy as np
ASSUME_STATIC_AT_START = True
# ============================================================
# 实时轨迹配置
# ============================================================

GRAVITY = 9.8

# 名义采样率，用于一阶高通滤波系数。
# 如果 Qt 端是 50Hz，就保持 50；如果你设置成 25Hz，可改成 25。
FILTER_FS_HZ = 50.0

# 是否启用高通滤波：保留你原离线算法的配置思想
APPLY_ACC_HPF = False
ACC_HPF_CUTOFF_HZ = 0.05

APPLY_VEL_HPF = True
VEL_HPF_CUTOFF_HZ = 0.10

APPLY_DISP_HPF = True
DISP_HPF_CUTOFF_HZ = 0.50

# 静止检测参数：实时版本，单位已换算为 m/s² 和 rad/s
# 原离线代码约等价：gyro std 1 deg/s，gyro mean 3 deg/s，raw accel std 5mg，acc norm tolerance 30mg
GYRO_STD_THRESHOLD_RAD = np.deg2rad(2.0)
GYRO_MEAN_THRESHOLD_RAD = np.deg2rad(5.0)
ACC_STD_THRESHOLD_MS2 = 10 / 1000.0 * GRAVITY
ACC_NORM_TOL_MS2 = 30.0 / 1000.0 * GRAVITY

STATIC_WINDOW_TIME = 0.05
MIN_STATIC_TIME = 0.05

# 静止约束动作
CORRECT_LINEAR_ACCEL_BIAS_BY_STATIC = True
ZERO_LINEAR_ACCEL_WHEN_STATIC = True
ZERO_VELOCITY_WHEN_STATIC = True
HOLD_POSITION_WHEN_STATIC = True

# 静止时 bias 慢更新比例
BIAS_ALPHA = 0.1

# 是否打印少量调试信息
DEBUG_TRAJECTORY = False
DEBUG_PRINT_EVERY = 50


def make_first_order_highpass_coeffs(fs, cutoff_hz):
    if cutoff_hz <= 0:
        raise ValueError("cutoff_hz must be > 0")
    if cutoff_hz >= fs / 2.0:
        raise ValueError("cutoff_hz must be < fs/2")

    K = np.tan(np.pi * cutoff_hz / fs)
    b0 = 1.0 / (1.0 + K)
    b1 = -b0
    a1 = (K - 1.0) / (1.0 + K)
    return b0, b1, a1


class FirstOrderHighPass:
    """实时一阶高通滤波器，支持三轴向量。"""

    def __init__(self, fs, cutoff_hz, dim=3, enabled=True):
        self.enabled = enabled
        self.dim = dim

        self.x_prev = np.zeros(dim, dtype=float)
        self.y_prev = np.zeros(dim, dtype=float)
        self.initialized = False

        if enabled:
            self.b0, self.b1, self.a1 = make_first_order_highpass_coeffs(fs, cutoff_hz)
        else:
            self.b0, self.b1, self.a1 = 1.0, 0.0, 0.0

    def reset(self):
        self.x_prev[:] = 0.0
        self.y_prev[:] = 0.0
        self.initialized = False

    def update(self, x):
        x = np.asarray(x, dtype=float)

        if not self.enabled:
            return x.copy()

        if not self.initialized:
            self.x_prev = x.copy()
            self.y_prev[:] = 0.0
            self.initialized = True
            return np.zeros_like(x)

        y = self.b0 * x + self.b1 * self.x_prev - self.a1 * self.y_prev

        self.x_prev = x.copy()
        self.y_prev = y.copy()

        return y


class CausalStaticDetector:
    """
    实时静止检测。

    和离线 rolling std 不同，这里只使用过去窗口，不看未来数据。
    """

    def __init__(self, fs_hint=FILTER_FS_HZ):
        self.fs_hint = fs_hint
        self.window_samples = max(3, int(STATIC_WINDOW_TIME * fs_hint))
        self.min_static_samples = max(1, int(MIN_STATIC_TIME * fs_hint))

        self.gyro_hist = []
        self.acc_hist = []
        self.static_count = 0
        self.is_stationary = False

    def update(self, acc_body, gyro_body):
        acc_body = np.asarray(acc_body, dtype=float)
        gyro_body = np.asarray(gyro_body, dtype=float)

        self.acc_hist.append(acc_body.copy())
        self.gyro_hist.append(gyro_body.copy())

        if len(self.acc_hist) > self.window_samples:
            self.acc_hist = self.acc_hist[-self.window_samples:]
            self.gyro_hist = self.gyro_hist[-self.window_samples:]

        acc_arr = np.asarray(self.acc_hist, dtype=float)
        gyro_arr = np.asarray(self.gyro_hist, dtype=float)


        if len(acc_arr) < 1:
            self.is_stationary = False
            return False

        gyro_std_norm = np.linalg.norm(np.std(gyro_arr, axis=0, ddof=0))
        acc_std_norm = np.linalg.norm(np.std(acc_arr, axis=0, ddof=0))

        gyro_norm_mean = np.mean(np.linalg.norm(gyro_arr, axis=1))
        acc_norm_mean = np.mean(np.linalg.norm(acc_arr, axis=1))

        sample_static = (
            gyro_std_norm < GYRO_STD_THRESHOLD_RAD and
            acc_std_norm < ACC_STD_THRESHOLD_MS2 and
            gyro_norm_mean < GYRO_MEAN_THRESHOLD_RAD and
            abs(acc_norm_mean - GRAVITY) < ACC_NORM_TOL_MS2
        )

        if sample_static:
            self.static_count += 1
        else:
            self.static_count = 0

        self.is_stationary = self.static_count >= self.min_static_samples
        return self.is_stationary


class RealtimeTrajectoryEstimator:
    """
    实时重心轨迹估计器。

    输入：
        time_s
        acc_body: body系总加速度，单位 m/s²
        gyro_body: body系角速度，单位 rad/s
        R_physical: body -> world 的旋转矩阵

    输出：
        position, velocity, linear_acc_used, is_stationary
    """

    def __init__(self):
        self.prev_time = None

        self.position_raw = np.zeros(3, dtype=float)
        self.position_used = np.zeros(3, dtype=float)
        self.velocity_raw = np.zeros(3, dtype=float)
        self.velocity_used = np.zeros(3, dtype=float)

        self.prev_acc_used = np.zeros(3, dtype=float)
        self.prev_velocity_used = np.zeros(3, dtype=float)

        self.acc_bias = np.zeros(3, dtype=float)

        self.static_detector = CausalStaticDetector(fs_hint=FILTER_FS_HZ)

        self.acc_hpf = FirstOrderHighPass(
            FILTER_FS_HZ,
            ACC_HPF_CUTOFF_HZ,
            dim=3,
            enabled=APPLY_ACC_HPF
        )
        self.vel_hpf = FirstOrderHighPass(
            FILTER_FS_HZ,
            VEL_HPF_CUTOFF_HZ,
            dim=3,
            enabled=APPLY_VEL_HPF
        )
        self.disp_hpf = FirstOrderHighPass(
            FILTER_FS_HZ,
            DISP_HPF_CUTOFF_HZ,
            dim=3,
            enabled=APPLY_DISP_HPF
        )

        self.time_history = []
        self.acc_history = []
        self.velocity_history = []
        self.displacement_history = []
        self.static_history = []

        self.frame_count = 0

    def update(self, time_s, acc_body, gyro_body, R_physical, aaccel_world=None):
        acc_body = np.asarray(acc_body, dtype=float)
        gyro_body = np.asarray(gyro_body, dtype=float)
        R_physical = np.asarray(R_physical, dtype=float)

        if self.prev_time is None:
            self.prev_time = time_s

            if ASSUME_STATIC_AT_START:
                is_stationary = True
                self.static_detector.static_count = self.static_detector.min_static_samples
                self.static_detector.is_stationary = True
            else:
                is_stationary = self.static_detector.update(acc_body, gyro_body)

            self.velocity_raw[:] = 0.0
            self.velocity_used[:] = 0.0
            self.position_raw[:] = 0.0
            self.position_used[:] = 0.0

            self._append_history(
                time_s,
                np.zeros(3),
                self.velocity_used,
                self.position_used,
                is_stationary
            )

            return (
                self.position_used.copy(),
                self.velocity_used.copy(),
                np.zeros(3),
                is_stationary
            )

        dt = time_s - self.prev_time
        self.prev_time = time_s

        if dt <= 0 or dt > 0.5:
            self._append_history(time_s, np.zeros(3), self.velocity_used, self.position_used, False)
            return self.position_used.copy(), self.velocity_used.copy(), np.zeros(3), False

        # 1. 静止检测使用 body 原始总加速度和 gyro
        is_stationary = self.static_detector.update(acc_body, gyro_body)

        # 2. 获取 world 系总加速度
        # 优先使用 Qt 端已经算好的 aax/aay/aaz
        if aaccel_world is not None:
            acc_world_total = np.asarray(aaccel_world, dtype=float)
        #else:
            # 备用方案：如果没有 aax/aay/aaz，才用 Python 自己算
            # acc_world_total = R_physical @ acc_body



        # 3. 去重力，得到 world 系线性加速度
        linear_world = acc_world_total - np.array([0.0, 0.0, GRAVITY], dtype=float)

        # 4. 静止时更新 residual bias
        if is_stationary and CORRECT_LINEAR_ACCEL_BIAS_BY_STATIC:
            self.acc_bias = (1.0 - BIAS_ALPHA) * self.acc_bias + BIAS_ALPHA * linear_world

        linear_corr = linear_world - self.acc_bias

        # 5. 静止时线性加速度置零
        if is_stationary and ZERO_LINEAR_ACCEL_WHEN_STATIC:
            linear_corr = np.zeros(3, dtype=float)

        # 6. 可选加速度高通
        acc_used = self.acc_hpf.update(linear_corr)

        if is_stationary and ZERO_LINEAR_ACCEL_WHEN_STATIC:
            acc_used[:] = 0.0

        # 7. 加速度梯形积分速度
        if is_stationary and ZERO_VELOCITY_WHEN_STATIC:
            self.velocity_raw[:] = 0.0
            self.velocity_used[:] = 0.0
            self.prev_velocity_used[:] = 0.0
        else:
            self.velocity_raw = self.velocity_raw + 0.5 * (self.prev_acc_used + acc_used) * dt
            self.velocity_used = self.vel_hpf.update(self.velocity_raw)

        if is_stationary and ZERO_VELOCITY_WHEN_STATIC:
            self.velocity_used[:] = 0.0

        # 8. 速度梯形积分位移；静止时位置保持
        if is_stationary and HOLD_POSITION_WHEN_STATIC:
            pass
        else:
            self.position_raw = self.position_raw + 0.5 * (self.prev_velocity_used + self.velocity_used) * dt

        # 9. 可选位移高通
        if APPLY_DISP_HPF:
            disp_filtered = self.disp_hpf.update(self.position_raw)
            if is_stationary and HOLD_POSITION_WHEN_STATIC:
                # 保持上一帧显示位置
                disp_filtered = self.position_used.copy()
            self.position_used = disp_filtered
        else:
            self.position_used = self.position_raw.copy()

        self.prev_acc_used = acc_used.copy()
        self.prev_velocity_used = self.velocity_used.copy()

        self._append_history(time_s, acc_used, self.velocity_used, self.position_used, is_stationary)

        self.frame_count += 1
        if DEBUG_TRAJECTORY and self.frame_count % DEBUG_PRINT_EVERY == 0:
            print("traj pos:", self.position_used, "vel:", self.velocity_used, "static:", is_stationary)

        return self.position_used.copy(), self.velocity_used.copy(), acc_used.copy(), is_stationary

    def _append_history(self, time_s, acc, vel, disp, is_static):
        self.time_history.append(float(time_s))
        self.acc_history.append(np.asarray(acc, dtype=float).copy())
        self.velocity_history.append(np.asarray(vel, dtype=float).copy())
        self.displacement_history.append(np.asarray(disp, dtype=float).copy())
        self.static_history.append(bool(is_static))

    def get_histories(self):
        if len(self.time_history) == 0:
            return (
                np.zeros(0),
                np.zeros((0, 3)),
                np.zeros((0, 3)),
                np.zeros((0, 3)),
                np.zeros(0, dtype=bool),
            )

        return (
            np.asarray(self.time_history, dtype=float),
            np.asarray(self.acc_history, dtype=float),
            np.asarray(self.velocity_history, dtype=float),
            np.asarray(self.displacement_history, dtype=float),
            np.asarray(self.static_history, dtype=bool),
        )