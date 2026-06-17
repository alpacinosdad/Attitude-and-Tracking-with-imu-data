from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from gesture_rectangle_show import (
    load_attitude_df,
    sensor_to_world,
    get_rotation_matrix_from_row,
    plot_attitude_on_axes,
)


# ============================================================
# 集中配置区：以后主要改这里
# ============================================================

# ----------------------------
# 文件配置：只需要在这里改文件名
# ----------------------------

# imu_result CSV，包含 time_s / lax / lay / laz / 姿态矩阵或四元数
RESULT_IMU_CSV_PATH = "low_allmove_imu_result_26.6.11.csv"

# 原始 IMU CSV，包含原始加速度和陀螺仪
RAW_IMU_CSV_PATH = "low_allmove_raw.csv"

# imu_result CSV 中已有的线性加速度列
LINEAR_ACCEL_COLS = ("lax", "lay", "laz")

# raw IMU CSV 中陀螺仪列名
# load_attitude_df 会把列名转成小写
GYRO_COLS = ("gyro_x_h", "gyro_y_h", "gyro_z_h")

# raw IMU CSV 中原始加速度列名
# 如果原始 CSV 里是 Accel_x_h / Accel_y_h / Accel_z_h，
# 读取后会自动变成 accel_x_h / accel_y_h / accel_z_h
RAW_ACCEL_COLS = ("accel_x_h", "accel_y_h", "accel_z_h")

# raw/result 按 time_s 对齐时允许的最大时间差，单位秒
# 如果 raw 没有 time_s，会自动按行号对齐，此参数不生效
MERGE_TOLERANCE_S = 0.02


# ----------------------------
# 线性加速度 / 速度 / 位移滤波配置
# ----------------------------

# 是否对已有 lax/lay/laz 做一阶高通
APPLY_ACC_HPF = False
ACC_HPF_CUTOFF_HZ = 0.5

# 是否对速度做一阶高通
# 调试 static reset 时建议 False
APPLY_VEL_HPF = False
VEL_HPF_CUTOFF_HZ = 0.05

# 是否对位移做一阶高通
# 如果想严格静止段位置保持，建议 False
APPLY_DISP_HPF = False
DISP_HPF_CUTOFF_HZ = 0.02

# 是否对最终轨迹做 sensor_to_world 坐标映射
APPLY_SENSOR_TO_WORLD_FOR_TRAJ = False


# ----------------------------
# gyro + raw accel 静止检测配置
# ----------------------------

# 是否启用 gyro + raw accel 联合静止检测
APPLY_GYRO_ACCEL_STATIC_DETECTION = True

# 是否使用 gyro rolling std 判断静止
USE_GYRO_STD_FOR_STATIC = True

# gyro rolling std 阈值
# 如果 gyro 单位是 deg/s，可以先试 0.5 ~ 5.0
GYRO_STD_THRESHOLD = 1

# 原始加速度 rolling std 阈值
# 你的加速度单位是 mg，可以先试 3 ~ 12
RAW_ACCEL_STD_THRESHOLD = 5

# rolling std 窗口长度，单位秒
STATIC_WINDOW_TIME = 0.16

# 至少连续静止多久才算静止，单位秒
MIN_STATIC_TIME = 0.2


# ----------------------------
# 静止约束动作
# ----------------------------

# 是否用静止段均值修正已有线性加速度 residual bias
# 目前建议 False，避免错误 bias 插值影响运动段
CORRECT_LINEAR_ACCEL_BIAS_BY_STATIC = False

# 静止时线性加速度置零
ZERO_LINEAR_ACCEL_WHEN_STATIC = True

# 静止时速度置零
ZERO_VELOCITY_WHEN_STATIC = True

# 是否做相邻静止段之间的速度线性漂移修正
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

    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif (not v) and start is not None:
            segments.append((start, i - 1))
            start = None

    if start is not None:
        segments.append((start, len(mask) - 1))

    return segments


def make_first_order_highpass_coeffs(fs, cutoff_hz):
    """
    一阶 Butterworth 高通滤波器系数。

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

    K = np.tan(np.pi * cutoff_hz / fs)

    b0 = 1.0 / (1.0 + K)
    b1 = -b0
    a1 = (K - 1.0) / (1.0 + K)

    return b0, b1, a1


def highpass_filter_1d(signal, fs, cutoff_hz):
    """
    一维一阶高通滤波。
    不调用 scipy / butter / filtfilt。
    """
    signal = np.asarray(signal, dtype=float)

    if len(signal) == 0:
        return signal.copy()

    b0, b1, a1 = make_first_order_highpass_coeffs(fs, cutoff_hz)

    y = np.zeros_like(signal, dtype=float)
    y[0] = 0.0

    for n in range(1, len(signal)):
        y[n] = b0 * signal[n] + b1 * signal[n - 1] - a1 * y[n - 1]

    return y


def highpass_filter(data, fs, cutoff_hz):
    """
    对 1D 或 NxM 数据逐列做一阶高通滤波。
    """
    data = np.asarray(data, dtype=float)

    input_was_1d = False

    if data.ndim == 1:
        data = data[:, None]
        input_was_1d = True

    filtered = np.zeros_like(data, dtype=float)

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

        if static_mask[n]:
            velocity[n, :] = 0.0
            continue

        if static_mask[n - 1]:
            v_prev = np.zeros(M, dtype=float)
            a_prev = np.zeros(M, dtype=float)
        else:
            v_prev = velocity[n - 1, :]
            a_prev = accel[n - 1, :]

        velocity[n, :] = v_prev + 0.5 * (a_prev + accel[n, :]) * dt

    return velocity


def integrate_velocity_to_displacement_with_static_hold(velocity, time_s, static_mask):
    """
    速度积分得到位移，但遇到 static_mask=True 时保持位置。

    逻辑：
        static=True:
            displacement[n] = displacement[n-1]

        static=False:
            正常由 velocity 积分得到 displacement
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

        if static_mask[n]:
            displacement[n, :] = displacement[n - 1, :]
            continue

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

    for i in range(N):
        start = max(0, i - half)
        end = min(N, i + half + 1)

        segment = data[start:end, :]

        std_out[i, :] = np.std(segment, axis=0, ddof=0)

    return std_out


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

        if not np.any(valid):
            col[:] = 0.0
            arr[:, j] = col
            continue

        last = np.nan
        for i in range(N):
            if np.isfinite(col[i]):
                last = col[i]
            else:
                col[i] = last

        first_valid_idx = np.where(np.isfinite(col))[0][0]
        first_valid_val = col[first_valid_idx]

        for i in range(first_valid_idx):
            col[i] = first_valid_val

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

def load_and_merge_raw_with_result(result_df, raw_csv_path, time_col="time_s"):
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

    if not raw_path.exists():
        print(f"Raw IMU CSV not found: {raw_path}")
        print("Static detection using gyro/raw accel will be disabled.")
        return result_df.copy()

    print(f"Loading raw IMU CSV for static detection: {raw_path}")

    raw_df = load_attitude_df(str(raw_path))

    if time_col not in result_df.columns:
        raise ValueError("imu_result CSV 缺少 time_s")

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

    print("Raw IMU CSV has no time_s, using row-index alignment.")

    result_aligned = result_df.reset_index(drop=True).copy()
    raw_aligned = raw_df.reset_index(drop=True).copy()

    n_result = len(result_aligned)
    n_raw = len(raw_aligned)

    print(f"imu_result rows = {n_result}")
    print(f"raw imu rows    = {n_raw}")

    if n_result != n_raw:
        n = min(n_result, n_raw)
        print(
            f"WARNING: row count mismatch. "
            f"Using first {n} rows for index alignment."
        )

        result_aligned = result_aligned.iloc[:n].reset_index(drop=True)
        raw_aligned = raw_aligned.iloc[:n].reset_index(drop=True)

    renamed_raw_cols = {}

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
    time_col="time_s",
    gyro_cols=GYRO_COLS,
    raw_accel_cols=RAW_ACCEL_COLS,
):
    """
    使用 gyro 稳定性 + raw accel 稳定性联合判断静止。

    静止条件：
        gyro_std_norm < GYRO_STD_THRESHOLD
        且 raw_accel_std_norm < RAW_ACCEL_STD_THRESHOLD
        且连续时间超过 MIN_STATIC_TIME
    """
    time_s = df[time_col].to_numpy(dtype=float)

    dt = np.diff(time_s)
    if dt.size == 0:
        raise ValueError("time_s has too few samples")

    fs = 1.0 / np.median(dt)

    gyro_cols_resolved = get_existing_columns(df, gyro_cols)
    raw_accel_cols_resolved = get_existing_columns(df, raw_accel_cols)

    gyro_raw = get_numeric_array_with_fill(df, gyro_cols_resolved)
    raw_accel = get_numeric_array_with_fill(df, raw_accel_cols_resolved)

    window = max(3, int(STATIC_WINDOW_TIME * fs))
    min_samples = max(1, int(MIN_STATIC_TIME * fs))

    gyro_std = rolling_std_nd(gyro_raw, window)
    gyro_std_norm = np.linalg.norm(gyro_std, axis=1)

    raw_accel_std = rolling_std_nd(raw_accel, window)
    raw_accel_std_norm = np.linalg.norm(raw_accel_std, axis=1)

    gyro_condition = gyro_std_norm < GYRO_STD_THRESHOLD
    accel_condition = raw_accel_std_norm < RAW_ACCEL_STD_THRESHOLD

    raw_static = gyro_condition & accel_condition

    static_mask = np.zeros_like(raw_static, dtype=bool)

    for s, e in find_true_segments(raw_static):
        if e - s + 1 >= min_samples:
            static_mask[s:e + 1] = True

    print("========== Static Detection ==========")
    print(f"gyro columns         = {gyro_cols_resolved}")
    print(f"raw accel columns    = {raw_accel_cols_resolved}")
    print(f"gyro std threshold   = {GYRO_STD_THRESHOLD}")
    print(f"raw accel std thres  = {RAW_ACCEL_STD_THRESHOLD}")
    print(f"window samples       = {window}")
    print(f"min static samples   = {min_samples}")
    print(f"static samples       = {np.sum(static_mask)} / {len(static_mask)}")
    print("static segments      =", find_true_segments(static_mask))
    print("=====================================")

    print("========== Static Feature Debug ==========")
    print("gyro std ok count:",
            np.sum(gyro_condition), "/", len(gyro_condition))

    print("accel std ok count:",
            np.sum(accel_condition), "/", len(accel_condition))

    print("both ok count:",
            np.sum(raw_static), "/", len(raw_static))

    print("gyro_std_norm min/max/mean:",
            np.min(gyro_std_norm),
            np.max(gyro_std_norm),
            np.mean(gyro_std_norm))

    print("gyro_std_norm percentiles:",
            np.percentile(gyro_std_norm, [5, 25, 50, 75, 95]))

    print("raw_accel_std_norm min/max/mean:",
            np.min(raw_accel_std_norm),
            np.max(raw_accel_std_norm),
            np.mean(raw_accel_std_norm))

    print("raw_accel_std_norm percentiles:",
            np.percentile(raw_accel_std_norm, [5, 25, 50, 75, 95]))

    print("last 100 gyro_std_norm percentiles:",
            np.percentile(gyro_std_norm[-100:], [5, 25, 50, 75, 95]))

    print("last 100 raw_accel_std_norm percentiles:",
            np.percentile(raw_accel_std_norm[-100:], [5, 25, 50, 75, 95]))
    print("========================================")

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

    for s, e in segments:
        center = (s + e) // 2
        bias = np.mean(linear_accel[s:e + 1], axis=0)

        centers.append(center)
        biases.append(bias)

    centers = np.asarray(centers, dtype=int)
    biases = np.asarray(biases, dtype=float)

    bias_est = np.zeros_like(linear_accel)

    for axis in range(linear_accel.shape[1]):
        bias_est[:, axis] = np.interp(
            idx_all,
            centers,
            biases[:, axis],
        )

    corrected = linear_accel - bias_est

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

    for s, e in find_true_segments(static_mask):
        displacement[s:e + 1, :] = displacement[s, :]

    return displacement


# ============================================================
# 轨迹计算
# ============================================================

def compute_trajectory(df, accel_cols=LINEAR_ACCEL_COLS, time_col="time_s"):
    """
    使用 imu_result 中已有的 lax/lay/laz，
    同时读取 raw IMU CSV 中的 gyro 和 raw accel 做静止检测。

    关键逻辑：
        static=True:
            accel = 0
            velocity = 0
            displacement 保持

        离开 static：
            从 velocity=0 重新积分
    """
    df = load_and_merge_raw_with_result(
        result_df=df,
        raw_csv_path=RAW_IMU_CSV_PATH,
        time_col=time_col,
    )

    required = {time_col, *accel_cols}

    if not required.issubset(df.columns):
        raise ValueError(
            f"CSV must contain columns {required}, actual columns: {list(df.columns)}"
        )

    time_s = df[time_col].to_numpy(dtype=float)
    accel_raw_linear = get_numeric_array_with_fill(df, accel_cols)

    dt = np.diff(time_s)

    if dt.size == 0:
        raise ValueError("time_s has too few samples")

    if np.any(dt <= 0):
        raise ValueError("time_s 必须严格递增")

    fs = 1.0 / np.median(dt)

    print(f"Estimated fs = {fs:.3f} Hz")
    print("========== Config ==========")
    print(f"RESULT_IMU_CSV_PATH = {RESULT_IMU_CSV_PATH}")
    print(f"RAW_IMU_CSV_PATH = {RAW_IMU_CSV_PATH}")
    print(f"APPLY_ACC_HPF = {APPLY_ACC_HPF}")
    print(f"ACC_HPF_CUTOFF_HZ = {ACC_HPF_CUTOFF_HZ}")
    print(f"APPLY_VEL_HPF = {APPLY_VEL_HPF}")
    print(f"VEL_HPF_CUTOFF_HZ = {VEL_HPF_CUTOFF_HZ}")
    print(f"APPLY_DISP_HPF = {APPLY_DISP_HPF}")
    print(f"DISP_HPF_CUTOFF_HZ = {DISP_HPF_CUTOFF_HZ}")
    print(f"APPLY_GYRO_ACCEL_STATIC_DETECTION = {APPLY_GYRO_ACCEL_STATIC_DETECTION}")
    print("============================")

    # 1. 线性加速度可选高通
    accel_used = maybe_highpass(
        accel_raw_linear,
        fs=fs,
        cutoff_hz=ACC_HPF_CUTOFF_HZ,
        enabled=APPLY_ACC_HPF,
        name="Linear Acceleration",
    )

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

    # 3. 静止段线性加速度处理
    if CORRECT_LINEAR_ACCEL_BIAS_BY_STATIC and np.any(static_mask):
        accel_used = correct_linear_accel_bias_by_static_segments(
            accel_used,
            static_mask,
        )

    # 只要 static=True，加速度最终必须为 0
    if ZERO_LINEAR_ACCEL_WHEN_STATIC and np.any(static_mask):
        accel_used[static_mask, :] = 0.0

    # 4. 带静止重置的加速度积分
    velocity_raw = integrate_accel_to_velocity_with_static_reset(
        accel_used,
        time_s,
        static_mask,
    )

    # 5. 速度可选高通
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

    # 6. 相邻静止段之间速度漂移修正
    if APPLY_VELOCITY_DRIFT_CORRECTION and np.any(static_mask):
        velocity_used = remove_velocity_drift_between_static(
            velocity_used,
            static_mask,
        )

    # 漂移修正后再次保证 static 段速度为 0
    if ZERO_VELOCITY_WHEN_STATIC and np.any(static_mask):
        velocity_used[static_mask, :] = 0.0

    # 7. 带静止位置保持的速度积分
    displacement_raw = integrate_velocity_to_displacement_with_static_hold(
        velocity_used,
        time_s,
        static_mask,
    )

    # 8. 位移可选高通
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

    # 9. 最终轨迹
    if APPLY_SENSOR_TO_WORLD_FOR_TRAJ:
        displacement_world = sensor_to_world(displacement_used)
    else:
        displacement_world = displacement_used

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
    交互式显示：
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

    ax_traj = fig.add_subplot(gs[0:2, 0:2], projection="3d")
    ax_orient = fig.add_subplot(gs[0:2, 2:], projection="3d")

    ax_acc = fig.add_subplot(gs[2, :])
    ax_vel = fig.add_subplot(gs[3, :], sharex=ax_acc)
    ax_disp = fig.add_subplot(gs[4, :], sharex=ax_acc)

    trajectory_line, = ax_traj.plot(
        [],
        [],
        [],
        color="tab:blue",
        lw=1.5,
        alpha=0.8,
        label="trajectory",
    )

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
        for s, e in find_true_segments(static_mask):
            ax.axvspan(
                time_s[s],
                time_s[e],
                color="gray",
                alpha=0.18,
            )

    ax_acc.plot(time_s, accel[:, 0], label="lax", color="C0", alpha=0.85)
    ax_acc.plot(time_s, accel[:, 1], label="lay", color="C1", alpha=0.85)
    ax_acc.plot(time_s, accel[:, 2], label="laz", color="C2", alpha=0.85)
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
        idx = int(val)
        idx = max(0, min(idx, len(time_s) - 1))

        current_point.set_data(
            [displacement_world[idx, 0]],
            [displacement_world[idx, 1]],
        )

        current_point.set_3d_properties(
            [displacement_world[idx, 2]]
        )

        trajectory_line.set_data(
            displacement_world[: idx + 1, 0],
            displacement_world[: idx + 1, 1],
        )

        trajectory_line.set_3d_properties(
            displacement_world[: idx + 1, 2]
        )

        current_line_acc.set_xdata([time_s[idx], time_s[idx]])
        current_line_vel.set_xdata([time_s[idx], time_s[idx]])
        current_line_disp.set_xdata([time_s[idx], time_s[idx]])

        dx, dy, dz = displacement[idx]
        wx, wy, wz = displacement_world[idx]

        is_static = bool(static_mask[idx])

        displacement_text.set_text(
            f"Time: {time_s[idx]:.3f} s | Index: {idx} | Static: {is_static}\n"
            f"Displacement: dx={dx:+.6f}, dy={dy:+.6f}, dz={dz:+.6f}\n"
            f"Trajectory:   X ={wx:+.6f}, Y ={wy:+.6f}, Z ={wz:+.6f}"
        )

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

        set_equal_3d(ax_traj, displacement_world[: idx + 1])

        fig.canvas.draw_idle()

    idx_slider.on_changed(update_plot)

    update_plot(0)

    plt.show()


# ============================================================
# main
# ============================================================

def main():
    result_path = Path(RESULT_IMU_CSV_PATH)

    if not result_path.exists():
        raise FileNotFoundError(f"找不到 imu_result CSV 文件: {result_path}")

    df_result = load_attitude_df(str(result_path))

    (
        merged_df,
        time_s,
        accel,
        velocity,
        displacement,
        displacement_world,
        static_mask,
    ) = compute_trajectory(df_result)

    print("accel shape:", accel.shape)
    print("velocity shape:", velocity.shape)
    print("displacement shape:", displacement.shape)
    print("displacement_world shape:", displacement_world.shape)
    print("static_mask shape:", static_mask.shape)

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