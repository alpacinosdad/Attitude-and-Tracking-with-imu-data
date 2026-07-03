from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# 从上一份姿态长方体显示代码中复用基础函数
from gesture_rectangle_show import (
    load_attitude_df,              # 读取 CSV，并统一列名格式
    sensor_to_world,               # 传感器坐标系到世界坐标系转换
    get_rotation_matrix_from_row,   # 从 CSV 某一行中读取旋转矩阵或四元数
    plot_attitude_on_axes,          # 绘制姿态长方体
)


# ============================================================
# 集中配置区
# ============================================================

# ----------------------------
# 文件配置
# ----------------------------

# imu_result CSV，应该包含：
# time_s / aax / aay / aaz / 姿态矩阵或四元数
RESULT_IMU_CSV_PATH = "BAT_Heat_Log_Data_2026_06_26_17_10_31_imu_result.csv"

# 原始 IMU CSV，包含原始加速度和陀螺仪
RAW_IMU_CSV_PATH = "BAT_Heat_Log_Data_2026_06_26_17_10_31.csv"

# 时间列名
TIME_COL = "time_s"

# imu_result CSV 中的世界系总加速度列，包含重力
AACCEL_COLS = ("aax", "aay", "aaz")

# 如果你还想临时切回旧方法，可用这个，但默认不用
BODY_LINEAR_ACCEL_COLS = ("lax", "lay", "laz")

# raw IMU CSV 中陀螺仪列名，单位 deg/s
GYRO_COLS = ("gyro_x_h", "gyro_y_h", "gyro_z_h")

# raw IMU CSV 中原始加速度列名，单位 mg
RAW_ACCEL_COLS = ("accel_x_h", "accel_y_h", "accel_z_h")

# raw/result 按 time_s 对齐时允许的最大时间差，单位秒
MERGE_TOLERANCE_S = 0.02

# 重力加速度，与你 C 代码里的 GRAV_FACT 保持一致
GRAVITY = 9.8


# ----------------------------
# 加速度来源配置
# ----------------------------

# 正确轨迹积分建议 True：
# 使用世界系线性加速度：
#   [aax, aay, aaz - GRAVITY]
USE_AACCEL_AS_WORLD_LINEAR = False


# ----------------------------
# 线性加速度 / 速度 / 位移滤波配置
# ----------------------------

# 调试阶段建议全部 False。
# 高通滤波会抑制漂移，但也可能破坏真实低频位移。
APPLY_ACC_HPF = False
ACC_HPF_CUTOFF_HZ = 0.05

APPLY_VEL_HPF = False
VEL_HPF_CUTOFF_HZ = 0.05

APPLY_DISP_HPF = False
DISP_HPF_CUTOFF_HZ = 0.02


# ----------------------------
# 最终轨迹坐标映射
# ----------------------------

# 使用 aax/aay/aaz - g 后，displacement_used 已经是世界系。
# 所以默认 False。
# 只有你为了显示做固定轴交换时才改 True。
APPLY_SENSOR_TO_WORLD_FOR_TRAJ = True


# ----------------------------
# gyro + raw accel 静止检测配置 steady
# ----------------------------

# 是否启用基于陀螺仪和原始加速度的静止检测
APPLY_GYRO_ACCEL_STATIC_DETECTION = True

# rolling std 条件
GYRO_STD_THRESHOLD = 1.0          # deg/s
RAW_ACCEL_STD_THRESHOLD = 5.0     # mg

# rolling mean 条件，防止稳定运动/稳定转动被误判静止
GYRO_MEAN_THRESHOLD = 3.0         # deg/s
RAW_ACCEL_NORM_TOL = 30.0        # mg，|acc_norm_mean - 1000| < 30

# rolling std / mean 窗口长度，单位秒
STATIC_WINDOW_TIME = 0.16

# 至少连续静止多久才算静止，单位秒
MIN_STATIC_TIME = 0.18


# ----------------------------
# 静止约束动作
# ----------------------------

# 默认关闭。
# 这个会用静止段均值修正线性加速度 residual bias。
# 如果 static_mask 不准，会影响真实运动段，所以先关闭。
CORRECT_LINEAR_ACCEL_BIAS_BY_STATIC = True

# 静止时线性加速度置零
ZERO_LINEAR_ACCEL_WHEN_STATIC = True

# 静止时速度置零
ZERO_VELOCITY_WHEN_STATIC = True

# 相邻静止段之间速度线性漂移修正
APPLY_VELOCITY_DRIFT_CORRECTION = True

# 静止时位置保持
HOLD_POSITION_WHEN_STATIC = True


# ============================================================
# 基础工具函数
# ============================================================

def find_true_segments(mask):
    """
    找 bool mask 中连续 True 的区间。

    返回:
        [(start, end), ...]
        start / end 都是闭区间索引。
    """
    mask = np.asarray(mask, dtype=bool)

    segments = []
    start = None

    # 遍历 bool 数组，记录连续 True 的起止位置
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif (not v) and start is not None:
            segments.append((start, i - 1))
            start = None

    # 如果数组最后仍处于 True 区间，需要补上最后一段
    if start is not None:
        segments.append((start, len(mask) - 1))

    return segments


def make_first_order_highpass_coeffs(fs, cutoff_hz):
    """
    一阶高通滤波器系数。

    K = tan(pi * fc / fs)

    b0 = 1 / (1 + K)
    b1 = -b0
    a1 = (K - 1) / (1 + K)

    差分方程：
        y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
    """
    if cutoff_hz <= 0:
        raise ValueError("cutoff_hz 必须大于 0")

    if cutoff_hz >= fs / 2.0:
        raise ValueError(
            f"cutoff_hz={cutoff_hz} 太高，必须小于 Nyquist 频率 fs/2={fs / 2.0}"
        )

    # 根据采样率和截止频率计算滤波器参数
    K = np.tan(np.pi * cutoff_hz / fs)

    b0 = 1.0 / (1.0 + K)
    b1 = -b0
    a1 = (K - 1.0) / (1.0 + K)

    return b0, b1, a1


def highpass_filter_1d(signal, fs, cutoff_hz):
    """
    一维一阶高通滤波。
    不调用 scipy。
    """
    signal = np.asarray(signal, dtype=float)

    if len(signal) == 0:
        return signal.copy()

    b0, b1, a1 = make_first_order_highpass_coeffs(fs, cutoff_hz)

    y = np.zeros_like(signal, dtype=float)
    y[0] = 0.0

    # 按差分方程逐点滤波
    for n in range(1, len(signal)):
        y[n] = b0 * signal[n] + b1 * signal[n - 1] - a1 * y[n - 1]

    return y


def highpass_filter(data, fs, cutoff_hz):
    """
    对 1D 或 NxM 数据逐列做一阶高通滤波。
    """
    data = np.asarray(data, dtype=float)

    input_was_1d = False

    # 如果输入是一维数据，临时转换成 Nx1，方便统一处理
    if data.ndim == 1:
        data = data[:, None]
        input_was_1d = True

    filtered = np.zeros_like(data, dtype=float)

    # 对每一列分别做高通滤波
    for k in range(data.shape[1]):
        filtered[:, k] = highpass_filter_1d(
            data[:, k],
            fs=fs,
            cutoff_hz=cutoff_hz,
        )

    if input_was_1d:
        return filtered[:, 0]

    return filtered


def maybe_highpass(data, fs, cutoff_hz, enabled, name="signal"):
    """
    根据 enabled 决定是否高通。
    """
    data = np.asarray(data, dtype=float)

    # 如果未开启高通滤波，直接返回数据副本
    if not enabled:
        print(f"{name}: HPF disabled, pass-through")
        return data.copy()

    print(f"{name}: HPF enabled, cutoff = {cutoff_hz} Hz")
    return highpass_filter(data, fs=fs, cutoff_hz=cutoff_hz)


def integrate_trapezoidal(values, time_s):
    """
    普通梯形积分。
    当前脚本保留此函数备用。
    主轨迹计算使用带 static reset 的积分函数。
    """
    values = np.asarray(values, dtype=float)
    time_s = np.asarray(time_s, dtype=float)

    if values.ndim == 1:
        values = values[:, None]

    if len(values) != len(time_s):
        raise ValueError(
            f"values 长度 {len(values)} 与 time_s 长度 {len(time_s)} 不一致"
        )

    dt = np.diff(time_s)

    integrated = np.zeros_like(values)

    # 梯形积分：increment = 0.5 * (前一帧值 + 当前帧值) * dt
    if dt.size > 0:
        increments = 0.5 * (values[:-1] + values[1:]) * dt[:, None]
        integrated[1:] = np.cumsum(increments, axis=0)

    return integrated[:, 0] if integrated.shape[1] == 1 else integrated


def integrate_accel_to_velocity_with_static_reset(accel, time_s, static_mask):
    """
    加速度积分得到速度，但遇到 static_mask=True 时重置速度。

    逻辑：
        static=True:
            velocity = 0

        static=False:
            从上一帧速度继续积分

        如果上一帧是 static、当前帧刚离开 static：
            上一帧速度视为 0
            上一帧加速度视为 0
            从当前位置重新开始积分
    """
    accel = np.asarray(accel, dtype=float)
    time_s = np.asarray(time_s, dtype=float)
    static_mask = np.asarray(static_mask, dtype=bool)

    if accel.ndim == 1:
        accel = accel[:, None]

    N, M = accel.shape

    velocity = np.zeros_like(accel, dtype=float)

    for n in range(1, N):
        dt = time_s[n] - time_s[n - 1]

        # 如果时间异常，不积分，沿用上一帧速度
        if dt <= 0:
            velocity[n, :] = velocity[n - 1, :]
            continue

        # 当前帧静止，速度强制为 0
        if static_mask[n]:
            velocity[n, :] = 0.0
            continue

        # 如果上一帧是静止，当前刚开始运动，则从 0 速度和 0 加速度开始积分
        if static_mask[n - 1]:
            v_prev = np.zeros(M, dtype=float)
            a_prev = np.zeros(M, dtype=float)
        else:
            v_prev = velocity[n - 1, :]
            a_prev = accel[n - 1, :]

        # 梯形积分加速度得到速度
        velocity[n, :] = v_prev + 0.5 * (a_prev + accel[n, :]) * dt

    return velocity


def integrate_velocity_to_displacement_with_static_hold(velocity, time_s, static_mask):
    """
    速度积分得到位移，但遇到 static_mask=True 时保持位置。
    """
    velocity = np.asarray(velocity, dtype=float)
    time_s = np.asarray(time_s, dtype=float)
    static_mask = np.asarray(static_mask, dtype=bool)

    if velocity.ndim == 1:
        velocity = velocity[:, None]

    N, M = velocity.shape

    displacement = np.zeros_like(velocity, dtype=float)

    for n in range(1, N):
        dt = time_s[n] - time_s[n - 1]

        # 如果时间异常，不积分，位置保持
        if dt <= 0:
            displacement[n, :] = displacement[n - 1, :]
            continue

        # 当前帧静止，位置保持不变
        if static_mask[n]:
            displacement[n, :] = displacement[n - 1, :]
            continue

        # 梯形积分速度得到位移
        displacement[n, :] = (
            displacement[n - 1, :]
            + 0.5 * (velocity[n - 1, :] + velocity[n, :]) * dt
        )

    return displacement


def rolling_std_nd(data, window):
    """
    纯 NumPy / for 循环实现的滑动窗口标准差。
    不使用 pandas rolling，不使用 scipy。
    """
    data = np.asarray(data, dtype=float)

    if data.ndim == 1:
        data = data[:, None]

    N, M = data.shape
    std_out = np.zeros((N, M), dtype=float)

    if N == 0:
        return std_out

    window = int(window)

    if window <= 1:
        return std_out

    half = window // 2

    # 对每个采样点取前后半个窗口，计算局部标准差
    for i in range(N):
        start = max(0, i - half)
        end = min(N, i + half + 1)

        segment = data[start:end, :]

        std_out[i, :] = np.std(segment, axis=0, ddof=0)

    return std_out


def rolling_mean_1d(data, window):
    """
    纯 NumPy / for 循环实现一维滑动均值。
    """
    data = np.asarray(data, dtype=float)

    N = len(data)
    out = np.zeros(N, dtype=float)

    if N == 0:
        return out

    window = int(window)

    if window <= 1:
        return data.copy()

    half = window // 2

    # 对每个点计算局部均值
    for i in range(N):
        start = max(0, i - half)
        end = min(N, i + half + 1)

        out[i] = np.mean(data[start:end])

    return out


def get_existing_columns(df, cols):
    """
    在 merge 后查找列名。

    如果原始列存在，优先用原始列。
    如果不存在，尝试使用 col_raw。
    """
    resolved = []

    for c in cols:
        if c in df.columns:
            resolved.append(c)
        elif f"{c}_raw" in df.columns:
            resolved.append(f"{c}_raw")
        else:
            raise ValueError(f"找不到列 {c} 或 {c}_raw")

    return tuple(resolved)


def get_raw_columns_after_merge(df, cols):
    """
    专门用于 raw IMU 列。

    merge 后如果 result_df 和 raw_df 有重名列，raw 列通常会变成 xxx_raw。
    这里优先找 xxx_raw，找不到再找 xxx。
    """
    resolved = []

    for c in cols:
        if f"{c}_raw" in df.columns:
            resolved.append(f"{c}_raw")
        elif c in df.columns:
            resolved.append(c)
        else:
            raise ValueError(f"找不到 raw 列 {c}_raw 或 {c}")

    return tuple(resolved)


def fill_nan_by_nearest(arr):
    """
    用纯 NumPy 做 NaN 填充：
        1. 前向填充
        2. 后向填充
    """
    arr = np.asarray(arr, dtype=float).copy()

    if arr.ndim == 1:
        arr = arr[:, None]
        input_was_1d = True
    else:
        input_was_1d = False

    N, M = arr.shape

    for j in range(M):
        col = arr[:, j]

        valid = np.isfinite(col)

        # 如果整列都无有效值，则整列填 0
        if not np.any(valid):
            col[:] = 0.0
            arr[:, j] = col
            continue

        # 前向填充
        last = np.nan
        for i in range(N):
            if np.isfinite(col[i]):
                last = col[i]
            else:
                col[i] = last

        # 处理开头仍然为 NaN 的部分
        first_valid_idx = np.where(np.isfinite(col))[0][0]
        first_valid_val = col[first_valid_idx]

        for i in range(first_valid_idx):
            col[i] = first_valid_val

        # 后向填充
        last = np.nan
        for i in range(N - 1, -1, -1):
            if np.isfinite(col[i]):
                last = col[i]
            else:
                col[i] = last

        arr[:, j] = col

    if input_was_1d:
        return arr[:, 0]

    return arr


def get_numeric_array_with_fill(df, cols):
    """
    取出数值列，并处理 NaN。
    """
    arr = df[list(cols)].to_numpy(dtype=float)
    return fill_nan_by_nearest(arr)


# ============================================================
# 数据对齐
# ============================================================

def load_and_merge_raw_with_result(result_df, raw_csv_path, time_col=TIME_COL):
    """
    将 imu_result 数据和 raw IMU 数据合并。

    支持两种模式：

    1. raw IMU CSV 有 time_s:
        使用 merge_asof 按时间对齐。

    2. raw IMU CSV 没有 time_s:
        认为 raw IMU 与 imu_result 是逐行严格对齐的，
        直接按 index 拼接。
    """
    raw_path = Path(raw_csv_path)

    # 如果 raw CSV 不存在，直接返回 result 数据
    if not raw_path.exists():
        print(f"Raw IMU CSV not found: {raw_path}")
        print("Static detection using gyro/raw accel will be disabled.")
        return result_df.copy()

    print(f"Loading raw IMU CSV for static detection: {raw_path}")

    raw_df = load_attitude_df(str(raw_path))

    if time_col not in result_df.columns:
        raise ValueError("imu_result CSV 缺少 time_s")

    # 如果 raw CSV 中有 time_s，则按时间最近邻合并
    if time_col in raw_df.columns:
        print("Raw IMU CSV has time_s, using time-based merge_asof.")

        result_sorted = result_df.sort_values(time_col).reset_index(drop=True)
        raw_sorted = raw_df.sort_values(time_col).reset_index(drop=True)

        merged = pd.merge_asof(
            result_sorted,
            raw_sorted,
            on=time_col,
            direction="nearest",
            tolerance=MERGE_TOLERANCE_S,
            suffixes=("", "_raw"),
        )

        return merged

    # 如果 raw CSV 没有 time_s，则按行号对齐
    print("Raw IMU CSV has no time_s, using row-index alignment.")

    result_aligned = result_df.reset_index(drop=True).copy()
    raw_aligned = raw_df.reset_index(drop=True).copy()

    n_result = len(result_aligned)
    n_raw = len(raw_aligned)

    print(f"imu_result rows = {n_result}")
    print(f"raw imu rows    = {n_raw}")

    # 如果行数不一致，取较短长度进行对齐
    if n_result != n_raw:
        n = min(n_result, n_raw)
        print(
            f"WARNING: row count mismatch. "
            f"Using first {n} rows for index alignment."
        )

        result_aligned = result_aligned.iloc[:n].reset_index(drop=True)
        raw_aligned = raw_aligned.iloc[:n].reset_index(drop=True)

    renamed_raw_cols = {}

    # 如果 raw 列名和 result 列名冲突，则 raw 列加 _raw 后缀
    for col in raw_aligned.columns:
        if col in result_aligned.columns:
            renamed_raw_cols[col] = f"{col}_raw"

    raw_aligned = raw_aligned.rename(columns=renamed_raw_cols)

    merged = pd.concat(
        [result_aligned, raw_aligned],
        axis=1,
    )

    return merged


# ============================================================
# 静止检测
# ============================================================

def detect_static_by_gyro_and_raw_accel(
    df,
    time_col=TIME_COL,
    gyro_cols=GYRO_COLS,
    raw_accel_cols=RAW_ACCEL_COLS,
):
    """
    使用 gyro 稳定性 + raw accel 稳定性联合判断静止。

    静止条件：
        gyro rolling std 小
        raw accel rolling std 小
        gyro 模长 rolling mean 小
        raw accel 模长 rolling mean 接近 1000 mg
        且连续时间超过 MIN_STATIC_TIME
    """
    time_s = df[time_col].to_numpy(dtype=float)

    dt = np.diff(time_s)
    if dt.size == 0:
        raise ValueError("time_s has too few samples")

    if np.any(dt <= 0):
        raise ValueError("time_s 必须严格递增")

    # 根据时间戳估算采样率
    fs = 1.0 / np.median(dt)

    gyro_cols_resolved = get_raw_columns_after_merge(df, gyro_cols)
    raw_accel_cols_resolved = get_raw_columns_after_merge(df, raw_accel_cols)

    gyro_raw = get_numeric_array_with_fill(df, gyro_cols_resolved)
    raw_accel = get_numeric_array_with_fill(df, raw_accel_cols_resolved)

    window = max(3, int(STATIC_WINDOW_TIME * fs))
    min_samples = max(1, int(MIN_STATIC_TIME * fs))

    # 1. rolling std 条件
    gyro_std = rolling_std_nd(gyro_raw, window)
    gyro_std_norm = np.linalg.norm(gyro_std, axis=1)

    raw_accel_std = rolling_std_nd(raw_accel, window)
    raw_accel_std_norm = np.linalg.norm(raw_accel_std, axis=1)

    gyro_std_condition = gyro_std_norm < GYRO_STD_THRESHOLD
    accel_std_condition = raw_accel_std_norm < RAW_ACCEL_STD_THRESHOLD

    # 2. rolling mean 条件
    gyro_norm = np.linalg.norm(gyro_raw, axis=1)
    raw_accel_norm = np.linalg.norm(raw_accel, axis=1)

    gyro_norm_mean = rolling_mean_1d(gyro_norm, window)
    raw_accel_norm_mean = rolling_mean_1d(raw_accel_norm, window)

    gyro_mean_condition = gyro_norm_mean < GYRO_MEAN_THRESHOLD
    accel_norm_condition = np.abs(raw_accel_norm_mean - 1000.0) < RAW_ACCEL_NORM_TOL

    # 四个条件同时成立时，初步认为该点静止
    raw_static = (
        gyro_std_condition
        & accel_std_condition
        & gyro_mean_condition
        & accel_norm_condition
    )

    static_mask = np.zeros_like(raw_static, dtype=bool)

    # 只有连续静止样本数达到 min_samples，才最终标记为静止
    for s, e in find_true_segments(raw_static):
        if e - s + 1 >= min_samples:
            static_mask[s:e + 1] = True

    print("========== Static Detection ==========")
    print(f"gyro columns               = {gyro_cols_resolved}")
    print(f"raw accel columns          = {raw_accel_cols_resolved}")
    print(f"gyro std threshold         = {GYRO_STD_THRESHOLD}")
    print(f"raw accel std threshold    = {RAW_ACCEL_STD_THRESHOLD}")
    print(f"gyro mean threshold        = {GYRO_MEAN_THRESHOLD}")
    print(f"raw accel norm tolerance   = {RAW_ACCEL_NORM_TOL}")
    print(f"window samples             = {window}")
    print(f"min static samples         = {min_samples}")
    print(f"static samples             = {np.sum(static_mask)} / {len(static_mask)}")
    print("static segments            =", find_true_segments(static_mask))
    print("======================================")

    print("========== Static Feature Debug ==========")
    print("gyro std ok count:",
            np.sum(gyro_std_condition), "/", len(gyro_std_condition))

    print("accel std ok count:",
            np.sum(accel_std_condition), "/", len(accel_std_condition))

    print("gyro mean ok count:",
            np.sum(gyro_mean_condition), "/", len(gyro_mean_condition))

    print("accel norm ok count:",
            np.sum(accel_norm_condition), "/", len(accel_norm_condition))

    print("all ok count:",
            np.sum(raw_static), "/", len(raw_static))

    print("gyro_std_norm percentiles:",
            np.percentile(gyro_std_norm, [5, 25, 50, 75, 95]))

    print("raw_accel_std_norm percentiles:",
            np.percentile(raw_accel_std_norm, [5, 25, 50, 75, 95]))

    print("gyro_norm_mean percentiles:",
            np.percentile(gyro_norm_mean, [5, 25, 50, 75, 95]))

    print("raw_accel_norm_mean percentiles:",
            np.percentile(raw_accel_norm_mean, [5, 25, 50, 75, 95]))
    print("==========================================")

    return static_mask, gyro_std_norm, raw_accel_std_norm


# ============================================================
# 静止约束修正
# ============================================================

def correct_linear_accel_bias_by_static_segments(linear_accel, static_mask):
    """
    用静止段估计并扣除线性加速度 residual bias。

    当前建议默认关闭，因为错误 bias 插值可能影响运动段。
    """
    linear_accel = np.asarray(linear_accel, dtype=float)
    static_mask = np.asarray(static_mask, dtype=bool)

    corrected = linear_accel.copy()
    segments = find_true_segments(static_mask)

    if len(segments) == 0:
        print("No static segment found, skip linear accel bias correction.")
        return corrected

    N = len(linear_accel)
    idx_all = np.arange(N)

    centers = []
    biases = []

    # 每个静止段中，线性加速度理论上应为 0；
    # 其均值被作为 residual bias 估计
    for s, e in segments:
        center = (s + e) // 2
        bias = np.mean(linear_accel[s:e + 1], axis=0)

        centers.append(center)
        biases.append(bias)

    centers = np.asarray(centers, dtype=int)
    biases = np.asarray(biases, dtype=float)

    bias_est = np.zeros_like(linear_accel)

    # 对每个轴的 bias 做时间插值
    for axis in range(linear_accel.shape[1]):
        bias_est[:, axis] = np.interp(
            idx_all,
            centers,
            biases[:, axis],
        )

    corrected = linear_accel - bias_est

    # 静止段线性加速度强制置零
    if ZERO_LINEAR_ACCEL_WHEN_STATIC:
        corrected[static_mask, :] = 0.0

    print("========== Static Linear Accel Bias ==========")
    for i, ((s, e), bias) in enumerate(zip(segments, biases)):
        print(f"segment {i}: {s}-{e}, bias={bias}")
    print("=============================================")

    return corrected


def remove_velocity_drift_between_static(velocity, static_mask):
    """
    相邻静止段之间的运动段速度线性漂移修正。

    假设：
        静止段 A 后速度应接近 0
        静止段 B 前速度应接近 0
    """
    velocity = np.asarray(velocity, dtype=float).copy()
    static_mask = np.asarray(static_mask, dtype=bool)

    segments = find_true_segments(static_mask)

    if len(segments) < 2:
        if np.any(static_mask):
            velocity[static_mask, :] = 0.0
        return velocity

    # 对每两个相邻静止段之间的运动区间做速度漂移线性扣除
    for i in range(len(segments) - 1):
        left = segments[i]
        right = segments[i + 1]

        move_start = left[1] + 1
        move_end = right[0] - 1

        if move_end <= move_start:
            continue

        v_start = velocity[move_start].copy()
        v_end = velocity[move_end].copy()

        length = move_end - move_start + 1

        for k in range(length):
            alpha = k / max(1, length - 1)
            drift = (1.0 - alpha) * v_start + alpha * v_end
            velocity[move_start + k] -= drift

    velocity[static_mask, :] = 0.0

    return velocity


def hold_position_during_static(displacement, static_mask):
    """
    静止段位置保持不变。
    """
    displacement = np.asarray(displacement, dtype=float).copy()

    if not HOLD_POSITION_WHEN_STATIC:
        return displacement

    # 每个静止段内的位置固定为该段起点的位置
    for s, e in find_true_segments(static_mask):
        displacement[s:e + 1, :] = displacement[s, :]

    return displacement


# ============================================================
# 加速度来源
# ============================================================

def get_world_linear_accel_from_aaccel(df):
    """
    使用 aax/aay/aaz 计算世界系线性加速度。

    C 代码里的 acce_to_abs：
        a_world_total = R * a_body

    静止时理论上：
        a_world_total = [0, 0, GRAVITY]

    所以用于积分的世界系线性加速度：
        linear_world = [aax, aay, aaz - GRAVITY]
    """
    aaccel_cols = get_existing_columns(df, AACCEL_COLS)
    aaccel = get_numeric_array_with_fill(df, aaccel_cols)

    linear_world = aaccel.copy()
    linear_world[:, 2] -= GRAVITY

    print("Using world-frame linear acceleration from aax/aay/aaz - gravity")
    print(f"aaccel columns = {aaccel_cols}")

    return linear_world


def get_body_linear_accel_from_lax(df):
    """
    读取已有 lax/lay/laz。

    注意：
        按你现在 C 代码，lax/lay/laz 是 body-frame linear acceleration。
        不建议直接用于世界轨迹积分。
    """
    cols = get_existing_columns(df, BODY_LINEAR_ACCEL_COLS)
    linear_body = get_numeric_array_with_fill(df, cols)

    print("Using existing lax/lay/laz directly")
    print(f"linear accel columns = {cols}")

    return linear_body


def print_accel_debug(accel, static_mask, name="accel"):
    """
    打印加速度调试信息。
    """
    accel = np.asarray(accel, dtype=float)
    static_mask = np.asarray(static_mask, dtype=bool)

    finite = np.isfinite(accel).all(axis=1)

    print(f"========== {name} Debug ==========")
    print("shape:", accel.shape)

    if np.any(finite):
        print("overall mean:", np.mean(accel[finite], axis=0))
        print("overall std :", np.std(accel[finite], axis=0))
        print("overall abs percentile 50/90/95/99:")
        print(np.percentile(np.abs(accel[finite]), [50, 90, 95, 99], axis=0))

    if np.any(static_mask):
        m = static_mask & finite
        if np.any(m):
            print("static mean:", np.mean(accel[m], axis=0))
            print("static std :", np.std(accel[m], axis=0))
            print("static abs percentile 50/90/95/99:")
            print(np.percentile(np.abs(accel[m]), [50, 90, 95, 99], axis=0))
    else:
        print("no static samples")

    print("==================================")


# ============================================================
# 轨迹计算
# ============================================================

def compute_trajectory(df, time_col=TIME_COL):
    """
    使用 aax/aay/aaz 重新计算轨迹。

    关键逻辑：
        1. aax/aay/aaz 是世界系总加速度，包含重力
        2. 用 [aax, aay, aaz - GRAVITY] 得到世界系线性加速度
        3. 对世界系线性加速度积分速度
        4. 对世界系速度积分位移
        5. static=True:
            accel = 0
            velocity = 0
            displacement 保持
    """
    df = load_and_merge_raw_with_result(
        result_df=df,
        raw_csv_path=RAW_IMU_CSV_PATH,
        time_col=time_col,
    )

    if time_col not in df.columns:
        raise ValueError(f"CSV must contain {time_col}")

    time_s = df[time_col].to_numpy(dtype=float)

    dt = np.diff(time_s)

    if dt.size == 0:
        raise ValueError("time_s has too few samples")

    if np.any(dt <= 0):
        raise ValueError("time_s 必须严格递增")

    # 根据 time_s 估计采样率
    fs = 1.0 / np.median(dt)

    print(f"Estimated fs = {fs:.3f} Hz")
    print("========== Config ==========")
    print(f"RESULT_IMU_CSV_PATH = {RESULT_IMU_CSV_PATH}")
    print(f"RAW_IMU_CSV_PATH = {RAW_IMU_CSV_PATH}")
    print(f"USE_AACCEL_AS_WORLD_LINEAR = {USE_AACCEL_AS_WORLD_LINEAR}")
    print(f"GRAVITY = {GRAVITY}")
    print(f"APPLY_ACC_HPF = {APPLY_ACC_HPF}")
    print(f"ACC_HPF_CUTOFF_HZ = {ACC_HPF_CUTOFF_HZ}")
    print(f"APPLY_VEL_HPF = {APPLY_VEL_HPF}")
    print(f"VEL_HPF_CUTOFF_HZ = {VEL_HPF_CUTOFF_HZ}")
    print(f"APPLY_DISP_HPF = {APPLY_DISP_HPF}")
    print(f"DISP_HPF_CUTOFF_HZ = {DISP_HPF_CUTOFF_HZ}")
    print(f"APPLY_GYRO_ACCEL_STATIC_DETECTION = {APPLY_GYRO_ACCEL_STATIC_DETECTION}")
    print("============================")

    # 1. 用 aax/aay/aaz 计算世界系线性加速度
    if USE_AACCEL_AS_WORLD_LINEAR:
        accel_raw_linear = get_world_linear_accel_from_aaccel(df)
    else:
        accel_raw_linear = get_body_linear_accel_from_lax(df)

    # 2. gyro + raw accel 静止检测
    if APPLY_GYRO_ACCEL_STATIC_DETECTION:
        try:
            static_mask, gyro_std_norm, raw_accel_std_norm = detect_static_by_gyro_and_raw_accel(
                df,
                time_col=time_col,
                gyro_cols=GYRO_COLS,
                raw_accel_cols=RAW_ACCEL_COLS,
            )
        except Exception as e:
            print("Static detection failed:", e)
            static_mask = np.zeros(len(time_s), dtype=bool)
    else:
        static_mask = np.zeros(len(time_s), dtype=bool)

    print_accel_debug(
        accel_raw_linear,
        static_mask,
        name="World Linear Accel Before Processing",
    )

    # 3. 线性加速度可选高通
    accel_used = maybe_highpass(
        accel_raw_linear,
        fs=fs,
        cutoff_hz=ACC_HPF_CUTOFF_HZ,
        enabled=APPLY_ACC_HPF,
        name="Linear Acceleration",
    )

    # 4. 静止段线性加速度 residual bias 修正，可选
    if CORRECT_LINEAR_ACCEL_BIAS_BY_STATIC and np.any(static_mask):
        accel_used = correct_linear_accel_bias_by_static_segments(
            accel_used,
            static_mask,
        )

    # 5. static=True，加速度置零
    if ZERO_LINEAR_ACCEL_WHEN_STATIC and np.any(static_mask):
        accel_used[static_mask, :] = 0.0

    print_accel_debug(
        accel_used,
        static_mask,
        name="World Linear Accel Used For Integration",
    )

    # 6. 加速度积分速度
    velocity_raw = integrate_accel_to_velocity_with_static_reset(
        accel_used,
        time_s,
        static_mask,
    )

    # 7. 速度可选高通
    velocity_used = maybe_highpass(
        velocity_raw,
        fs=fs,
        cutoff_hz=VEL_HPF_CUTOFF_HZ,
        enabled=APPLY_VEL_HPF,
        name="Velocity",
    )

    # 高通后再次保证 static 段速度为 0
    if ZERO_VELOCITY_WHEN_STATIC and np.any(static_mask):
        velocity_used[static_mask, :] = 0.0

    # 8. 相邻静止段之间速度漂移修正
    if APPLY_VELOCITY_DRIFT_CORRECTION and np.any(static_mask):
        velocity_used = remove_velocity_drift_between_static(
            velocity_used,
            static_mask,
        )

    # 漂移修正后再次保证 static 段速度为 0
    if ZERO_VELOCITY_WHEN_STATIC and np.any(static_mask):
        velocity_used[static_mask, :] = 0.0

    # 9. 速度积分位移
    displacement_raw = integrate_velocity_to_displacement_with_static_hold(
        velocity_used,
        time_s,
        static_mask,
    )

    # 10. 位移可选高通
    displacement_used = maybe_highpass(
        displacement_raw,
        fs=fs,
        cutoff_hz=DISP_HPF_CUTOFF_HZ,
        enabled=APPLY_DISP_HPF,
        name="Displacement",
    )

    # 如果位移高通开启，它可能破坏静止平台；所以这里再次 hold
    if HOLD_POSITION_WHEN_STATIC and np.any(static_mask):
        displacement_used = hold_position_during_static(
            displacement_used,
            static_mask,
        )

    # 11. 最终轨迹
    if APPLY_SENSOR_TO_WORLD_FOR_TRAJ:
        displacement_world = sensor_to_world(displacement_used)
    else:
        displacement_world = displacement_used.copy()

    print("========== Trajectory Result ==========")
    print("final displacement:", displacement_world[-1])
    print("max abs displacement:", np.max(np.abs(displacement_world), axis=0))
    print("=======================================")

    return (
        df,
        time_s,
        accel_used,
        velocity_used,
        displacement_used,
        displacement_world,
        static_mask,
    )


# ============================================================
# 可视化
# ============================================================

def plot_slider(
    df,
    time_s,
    accel,
    velocity,
    displacement,
    displacement_world,
    static_mask,
):
    """
    保持你原本的布局：
        左侧：3D 位移轨迹
        右侧：姿态长方体
        底部三张图：
            加速度
            速度
            位移
    """
    fig = plt.figure(figsize=(18, 13))

    gs = fig.add_gridspec(
        5,
        4,
        height_ratios=[1.1, 1.1, 0.45, 0.45, 0.45],
        hspace=0.55,
        wspace=0.3,
        top=0.95,
        bottom=0.18,
    )

    # 左上：3D 轨迹
    ax_traj = fig.add_subplot(gs[0:2, 0:2], projection="3d")

    # 右上：姿态长方体
    ax_orient = fig.add_subplot(gs[0:2, 2:], projection="3d")

    # 下方三张曲线图：加速度、速度、位移
    ax_acc = fig.add_subplot(gs[2, :])
    ax_vel = fig.add_subplot(gs[3, :], sharex=ax_acc)
    ax_disp = fig.add_subplot(gs[4, :], sharex=ax_acc)

    # 初始化轨迹线
    trajectory_line, = ax_traj.plot(
        [],
        [],
        [],
        color="tab:blue",
        lw=1.5,
        alpha=0.8,
        label="trajectory",
    )

    # 初始化当前点
    current_point, = ax_traj.plot(
        [displacement_world[0, 0]],
        [displacement_world[0, 1]],
        [displacement_world[0, 2]],
        "o",
        color="red",
        markersize=8,
        label="current position",
    )

    ax_traj.set_title("Displacement Trajectory")
    ax_traj.set_xlabel("X")
    ax_traj.set_ylabel("Y")
    ax_traj.set_zlabel("Z")
    ax_traj.legend(loc="upper left")

    def set_equal_3d(ax, data):
        """
        设置 3D 坐标轴等比例显示。
        """
        data = np.asarray(data, dtype=float)

        if data.ndim != 2 or data.shape[1] != 3:
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            return

        finite = np.isfinite(data).all(axis=1)

        if not np.any(finite):
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            return

        data = data[finite]
        xs, ys, zs = data.T

        max_range = np.max([
            xs.max() - xs.min(),
            ys.max() - ys.min(),
            zs.max() - zs.min(),
        ]) / 2.0

        if max_range == 0.0:
            max_range = 0.1

        mid_x = (xs.max() + xs.min()) / 2.0
        mid_y = (ys.max() + ys.min()) / 2.0
        mid_z = (zs.max() + zs.min()) / 2.0

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        try:
            ax.set_box_aspect([1, 1, 1])
        except Exception:
            pass

    set_equal_3d(ax_traj, displacement_world[:1])

    def shade_static_regions(ax):
        """
        在曲线图中用灰色背景标出静止段。
        """
        for s, e in find_true_segments(static_mask):
            ax.axvspan(
                time_s[s],
                time_s[e],
                color="gray",
                alpha=0.18,
            )

    # 注意：
    # 这里的 accel 已经是用于积分的世界系线性加速度：
    # [aax, aay, aaz - GRAVITY]
    # 布局保持原样，只是 label 改成更准确的 world ax/ay/az
    ax_acc.plot(time_s, accel[:, 0], label="ax_world", color="C0", alpha=0.85)
    ax_acc.plot(time_s, accel[:, 1], label="ay_world", color="C1", alpha=0.85)
    ax_acc.plot(time_s, accel[:, 2], label="az_world", color="C2", alpha=0.85)
    shade_static_regions(ax_acc)

    current_line_acc = ax_acc.axvline(time_s[0], color="black", lw=1.2)

    ax_acc.set_title("Linear Acceleration Used")
    ax_acc.set_ylabel("accel")
    ax_acc.legend(loc="upper right", fontsize="small", ncol=3)
    ax_acc.grid(True)

    ax_vel.plot(time_s, velocity[:, 0], label="vx", color="C0", alpha=0.85)
    ax_vel.plot(time_s, velocity[:, 1], label="vy", color="C1", alpha=0.85)
    ax_vel.plot(time_s, velocity[:, 2], label="vz", color="C2", alpha=0.85)
    shade_static_regions(ax_vel)

    current_line_vel = ax_vel.axvline(time_s[0], color="black", lw=1.2)

    ax_vel.set_title("Velocity Used")
    ax_vel.set_ylabel("velocity")
    ax_vel.legend(loc="upper right", fontsize="small", ncol=3)
    ax_vel.grid(True)

    ax_disp.plot(time_s, displacement[:, 0], label="dx", color="C0", alpha=0.85)
    ax_disp.plot(time_s, displacement[:, 1], label="dy", color="C1", alpha=0.85)
    ax_disp.plot(time_s, displacement[:, 2], label="dz", color="C2", alpha=0.85)
    shade_static_regions(ax_disp)

    current_line_disp = ax_disp.axvline(time_s[0], color="black", lw=1.2)

    ax_disp.set_title("Displacement Used")
    ax_disp.set_xlabel("time_s (s)")
    ax_disp.set_ylabel("displacement")
    ax_disp.legend(loc="upper right", fontsize="small", ncol=3)
    ax_disp.grid(True)

    plt.setp(ax_acc.get_xticklabels(), visible=False)
    plt.setp(ax_vel.get_xticklabels(), visible=False)

    ax_orient.set_title("Reference sensor orientation")
    ax_orient.set_axis_off()

    # 显示当前时间、索引、静止状态、位移和轨迹坐标
    displacement_text = fig.text(
        0.15,
        0.105,
        "",
        fontsize=11,
        family="monospace",
        bbox=dict(
            boxstyle="round",
            facecolor="white",
            alpha=0.9,
            edgecolor="gray",
        ),
    )

    # 底部滑块
    slider_ax = fig.add_axes([0.15, 0.045, 0.7, 0.035])

    idx_slider = Slider(
        ax=slider_ax,
        label="sample index",
        valmin=0,
        valmax=len(time_s) - 1,
        valinit=0,
        valfmt="%0.0f",
        valstep=1,
    )

    def update_plot(val):
        """
        滑块回调函数。
        根据当前 index 更新：
            1. 3D 当前点
            2. 已走过的轨迹
            3. 三张曲线上的当前时间竖线
            4. 文本信息
            5. 右侧姿态长方体
        """
        idx = int(val)
        idx = max(0, min(idx, len(time_s) - 1))

        # 更新当前 3D 位置点
        current_point.set_data(
            [displacement_world[idx, 0]],
            [displacement_world[idx, 1]],
        )

        current_point.set_3d_properties(
            [displacement_world[idx, 2]]
        )

        # 更新从起点到当前点的轨迹线
        trajectory_line.set_data(
            displacement_world[: idx + 1, 0],
            displacement_world[: idx + 1, 1],
        )

        trajectory_line.set_3d_properties(
            displacement_world[: idx + 1, 2]
        )

        # 更新三张曲线上的当前时间竖线
        current_line_acc.set_xdata([time_s[idx], time_s[idx]])
        current_line_vel.set_xdata([time_s[idx], time_s[idx]])
        current_line_disp.set_xdata([time_s[idx], time_s[idx]])

        dx, dy, dz = displacement[idx]
        wx, wy, wz = displacement_world[idx]

        is_static = bool(static_mask[idx])

        # 更新底部文本信息
        displacement_text.set_text(
            f"Time: {time_s[idx]:.3f} s | Index: {idx} | Static: {is_static}\n"
            f"Displacement: dx={dx:+.6f}, dy={dy:+.6f}, dz={dz:+.6f}\n"
            f"Trajectory:   X ={wx:+.6f}, Y ={wy:+.6f}, Z ={wz:+.6f}"
        )

        # 更新右侧姿态长方体
        try:
            row = df.iloc[idx]
            R = get_rotation_matrix_from_row(row)
            plot_attitude_on_axes(
                ax_orient,
                R,
                row_index=idx,
                row=row,
            )

        except Exception:
            ax_orient.cla()
            ax_orient.text(
                0.5,
                0.5,
                "No valid orientation row",
                horizontalalignment="center",
                verticalalignment="center",
                transform=ax_orient.transAxes,
            )
            ax_orient.set_axis_off()

        # 根据当前轨迹范围动态调整 3D 坐标比例
        set_equal_3d(ax_traj, displacement_world[: idx + 1])

        fig.canvas.draw_idle()

    idx_slider.on_changed(update_plot)

    # 初始化绘制第 0 个样本
    update_plot(0)

    plt.show()


# ============================================================
# main
# ============================================================

def main():
    # 检查 imu_result CSV 文件是否存在
    result_path = Path(RESULT_IMU_CSV_PATH)

    if not result_path.exists():
        raise FileNotFoundError(f"找不到 imu_result CSV 文件: {result_path}")

    # 读取 imu_result CSV
    df_result = load_attitude_df(str(result_path))

    # 计算轨迹、速度、加速度、静止 mask 等
    (
        merged_df,
        time_s,
        accel,
        velocity,
        displacement,
        displacement_world,
        static_mask,
    ) = compute_trajectory(df_result)

    # 打印主要数组形状，便于检查数据是否正常
    print("accel shape:", accel.shape)
    print("velocity shape:", velocity.shape)
    print("displacement shape:", displacement.shape)
    print("displacement_world shape:", displacement_world.shape)
    print("static_mask shape:", static_mask.shape)

    # 启动交互式可视化
    plot_slider(
        merged_df,
        time_s,
        accel,
        velocity,
        displacement,
        displacement_world,
        static_mask,
    )




if __name__ == "__main__":
    main()