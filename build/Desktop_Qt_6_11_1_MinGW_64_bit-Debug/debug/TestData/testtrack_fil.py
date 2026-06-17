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

RESULT_IMU_CSV_PATH = "BAT_Heat_Log_Data_2026_06_15_20_39_39_imu_result.csv"
RAW_IMU_CSV_PATH = "BAT_Heat_Log_Data_2026_06_15_20_39_39.csv"

LINEAR_ACCEL_COLS = ("lax", "lay", "laz")

GYRO_COLS = ("gyro_x_h", "gyro_y_h", "gyro_z_h")
RAW_ACCEL_COLS = ("accel_x_h", "accel_y_h", "accel_z_h")

MERGE_TOLERANCE_S = 0.02


# ----------------------------
# 最终轨迹配置
# ----------------------------

APPLY_SENSOR_TO_WORLD_FOR_TRAJ = False


# ----------------------------
# 线性加速度 / 速度 / 位移纯因果高通配置
# ----------------------------

# 是否对 lax/lay/laz 做纯因果一阶高通
APPLY_ACC_HPF = False

# 加速度高通截止频率 Hz
ACC_HPF_CUTOFF_HZ = 0.01

# 选择对哪些加速度轴做高通：
# (x, y, z) = (lax, lay, laz)
ACC_HPF_AXES = (False, False, False)


# 是否对速度做纯因果一阶高通
APPLY_VEL_HPF = False

# 速度高通截止频率 Hz
VEL_HPF_CUTOFF_HZ = 1.0

# 选择对哪些速度轴做高通：
# (x, y, z) = (vx, vy, vz)
VEL_HPF_AXES = (False, True, False)


# 是否对位移做纯因果一阶高通
APPLY_DISP_HPF = False

# 位移高通截止频率 Hz
DISP_HPF_CUTOFF_HZ = 0.3

# 选择对哪些位移轴做高通：
# 例如只对 dz 高通： (False, False, True)
DISP_HPF_AXES = (False, False, False)


# ----------------------------
# gyro + raw accel 静止检测配置
# ----------------------------

APPLY_GYRO_ACCEL_STATIC_DETECTION = True

GYRO_STD_THRESHOLD = 3
RAW_ACCEL_STD_THRESHOLD = 10

STATIC_WINDOW_TIME = 0.16
MIN_STATIC_TIME = 0.16


# ----------------------------
# 静止约束
# ----------------------------

ZERO_LINEAR_ACCEL_WHEN_STATIC = True
ZERO_VELOCITY_WHEN_STATIC = True
HOLD_POSITION_WHEN_STATIC = True


# ----------------------------
# 3D 轨迹显示范围配置
# ----------------------------

TRAJ_VIEW_SIZE_M = 10.0
TRAJ_VIEW_CENTER = (0.0, 0.0, 0.0)


# ============================================================
# 基础工具函数
# ============================================================

def find_true_segments(mask):
    """
    找 bool mask 中连续 True 的区间。
    返回 [(start, end), ...]，start/end 都是闭区间索引。
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


def rolling_std_nd_causal(data, window):
    """
    因果滑动窗口标准差。
    每个点 i 只使用当前点和过去数据：[i-window+1, i]。
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

    for i in range(N):
        start = max(0, i - window + 1)
        end = i + 1
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
# 纯因果一阶高通滤波
# ============================================================

def make_first_order_highpass_coeffs(fs, cutoff_hz):
    """
    一阶因果高通滤波器系数。

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


def causal_highpass_filter_1d(signal, fs, cutoff_hz, static_mask=None):
    """
    一维纯因果一阶高通滤波。

    如果传入 static_mask：
        static=True 时输出强制为 0；
        但是 x_prev 更新为当前输入 x，而不是 0。
        这样离开静止段时，高通相当于围绕最近静止基线变化。
    """
    signal = np.asarray(signal, dtype=float)

    N = len(signal)

    if N == 0:
        return signal.copy()

    if static_mask is None:
        static_mask = np.zeros(N, dtype=bool)
    else:
        static_mask = np.asarray(static_mask, dtype=bool)

    b0, b1, a1 = make_first_order_highpass_coeffs(fs, cutoff_hz)

    y = np.zeros_like(signal, dtype=float)

    x_prev = signal[0]
    y_prev = 0.0

    for n in range(N):
        x = signal[n]

        if static_mask[n]:
            y[n] = 0.0
            x_prev = x
            y_prev = 0.0
            continue

        y_curr = b0 * x + b1 * x_prev - a1 * y_prev

        y[n] = y_curr

        x_prev = x
        y_prev = y_curr

    return y


def causal_highpass_filter_axes(
    data,
    fs,
    cutoff_hz,
    axes,
    static_mask=None,
):
    """
    对 1D 或 NxM 数据中的指定轴做纯因果一阶高通。

    axes:
        tuple/list/bool array，长度应等于数据列数。
        例如：
            (True, True, True)      三轴都高通
            (False, False, True)    只对 z 轴高通
            (True, False, False)    只对 x 轴高通
    """
    data = np.asarray(data, dtype=float)

    input_was_1d = False

    if data.ndim == 1:
        data = data[:, None]
        input_was_1d = True

    N, M = data.shape

    axes = tuple(bool(v) for v in axes)

    if len(axes) != M:
        raise ValueError(
            f"axes 长度 {len(axes)} 与数据轴数 {M} 不一致，"
            f"data shape = {data.shape}"
        )

    filtered = data.copy()

    for k in range(M):
        if axes[k]:
            filtered[:, k] = causal_highpass_filter_1d(
                data[:, k],
                fs=fs,
                cutoff_hz=cutoff_hz,
                static_mask=static_mask,
            )

    if input_was_1d:
        return filtered[:, 0]

    return filtered


def maybe_causal_highpass_axes(
    data,
    fs,
    cutoff_hz,
    enabled,
    axes,
    name="signal",
    static_mask=None,
):
    """
    根据 enabled 决定是否对指定轴做纯因果高通。
    """
    data = np.asarray(data, dtype=float)

    if data.ndim == 1:
        axis_count = 1
    else:
        axis_count = data.shape[1]

    axes = tuple(bool(v) for v in axes)

    if len(axes) != axis_count:
        raise ValueError(
            f"{name}: axes 长度 {len(axes)} 与数据轴数 {axis_count} 不一致"
        )

    if not enabled:
        print(f"{name}: causal HPF disabled, pass-through")
        return data.copy()

    if not any(axes):
        print(f"{name}: causal HPF enabled but no axes selected, pass-through")
        return data.copy()

    print(
        f"{name}: causal HPF enabled, "
        f"cutoff={cutoff_hz} Hz, axes={axes}"
    )

    return causal_highpass_filter_axes(
        data,
        fs=fs,
        cutoff_hz=cutoff_hz,
        axes=axes,
        static_mask=static_mask,
    )


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

    注意：
        使用因果窗口，只看当前和过去数据。
        静止确认也是因果确认：
            只有当前点已经连续满足 min_samples 后，当前点才判静止；
            不会回头修改前面的点。
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

    gyro_std = rolling_std_nd_causal(gyro_raw, window)
    gyro_std_norm = np.linalg.norm(gyro_std, axis=1)

    raw_accel_std = rolling_std_nd_causal(raw_accel, window)
    raw_accel_std_norm = np.linalg.norm(raw_accel_std, axis=1)

    gyro_condition = gyro_std_norm < GYRO_STD_THRESHOLD
    accel_condition = raw_accel_std_norm < RAW_ACCEL_STD_THRESHOLD

    raw_static = gyro_condition & accel_condition

    static_mask = np.zeros_like(raw_static, dtype=bool)
    count = 0

    for i, is_static_raw in enumerate(raw_static):
        if is_static_raw:
            count += 1
            if count >= min_samples:
                static_mask[i] = True
        else:
            count = 0

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
# 状态机积分
# ============================================================

def integrate_motion_state_machine(accel_input, time_s, static_mask):
    """
    逐点状态机积分。

    accel_input:
        已经经过可选因果高通的 lax/lay/laz。

    逻辑：

        static=True:
            accel_used = 0
            velocity = 0
            displacement 保持

        static=False:
            accel_used = accel_input[n]
            从上一帧 velocity 继续积分；
            如果上一帧是 static，则从 velocity=0 重新开始积分。
    """
    accel_input = np.asarray(accel_input, dtype=float)
    time_s = np.asarray(time_s, dtype=float)
    static_mask = np.asarray(static_mask, dtype=bool)

    if accel_input.ndim == 1:
        accel_input = accel_input[:, None]

    N, M = accel_input.shape

    accel_used = accel_input.copy()
    velocity = np.zeros_like(accel_input, dtype=float)
    displacement = np.zeros_like(accel_input, dtype=float)

    for n in range(N):
        if n == 0:
            dt = 0.0
        else:
            dt = time_s[n] - time_s[n - 1]

        if static_mask[n]:
            if ZERO_LINEAR_ACCEL_WHEN_STATIC:
                accel_used[n, :] = 0.0

            if ZERO_VELOCITY_WHEN_STATIC:
                velocity[n, :] = 0.0

            if n == 0:
                displacement[n, :] = 0.0
            else:
                if HOLD_POSITION_WHEN_STATIC:
                    displacement[n, :] = displacement[n - 1, :]
                else:
                    displacement[n, :] = displacement[n - 1, :]

            continue

        accel_used[n, :] = accel_input[n, :]

        if n == 0:
            velocity[n, :] = 0.0
            displacement[n, :] = 0.0
            continue

        if static_mask[n - 1]:
            v_prev = np.zeros(M, dtype=float)
            a_prev = np.zeros(M, dtype=float)
        else:
            v_prev = velocity[n - 1, :]
            a_prev = accel_used[n - 1, :]

        a_curr = accel_used[n, :]

        velocity[n, :] = v_prev + 0.5 * (a_prev + a_curr) * dt

        displacement[n, :] = (
            displacement[n - 1, :]
            + 0.5 * (v_prev + velocity[n, :]) * dt
        )

    return accel_used, velocity, displacement


def integrate_velocity_to_displacement_with_static_hold(velocity, time_s, static_mask):
    """
    速度积分得到位移，static=True 时保持位置。

    该函数主要用于速度被高通后，重新积分 displacement。
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

    新逻辑：
        1. 因果静止检测
        2. 对 lax/lay/laz 指定轴做纯因果高通
        3. 状态机积分速度和位移
        4. 速度/位移也可以指定轴做纯因果高通
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
    print(f"APPLY_SENSOR_TO_WORLD_FOR_TRAJ = {APPLY_SENSOR_TO_WORLD_FOR_TRAJ}")
    print(f"APPLY_GYRO_ACCEL_STATIC_DETECTION = {APPLY_GYRO_ACCEL_STATIC_DETECTION}")
    print(f"APPLY_ACC_HPF = {APPLY_ACC_HPF}")
    print(f"ACC_HPF_CUTOFF_HZ = {ACC_HPF_CUTOFF_HZ}")
    print(f"ACC_HPF_AXES = {ACC_HPF_AXES}")
    print(f"APPLY_VEL_HPF = {APPLY_VEL_HPF}")
    print(f"VEL_HPF_CUTOFF_HZ = {VEL_HPF_CUTOFF_HZ}")
    print(f"VEL_HPF_AXES = {VEL_HPF_AXES}")
    print(f"APPLY_DISP_HPF = {APPLY_DISP_HPF}")
    print(f"DISP_HPF_CUTOFF_HZ = {DISP_HPF_CUTOFF_HZ}")
    print(f"DISP_HPF_AXES = {DISP_HPF_AXES}")
    print("============================")

    # ============================================================
    # 1. 因果静止检测
    # ============================================================
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

    # ============================================================
    # 2. 对 lax/lay/laz 指定轴做纯因果高通
    # ============================================================
    accel_filtered = maybe_causal_highpass_axes(
        accel_raw_linear,
        fs=fs,
        cutoff_hz=ACC_HPF_CUTOFF_HZ,
        enabled=APPLY_ACC_HPF,
        axes=ACC_HPF_AXES,
        name="Linear Acceleration",
        static_mask=static_mask,
    )

    # ============================================================
    # 3. 状态机积分
    # ============================================================
    accel_used, velocity_used, displacement_raw = integrate_motion_state_machine(
        accel_filtered,
        time_s,
        static_mask,
    )

    # ============================================================
    # 4. 速度指定轴可选纯因果高通
    # ============================================================
    velocity_used = maybe_causal_highpass_axes(
        velocity_used,
        fs=fs,
        cutoff_hz=VEL_HPF_CUTOFF_HZ,
        enabled=APPLY_VEL_HPF,
        axes=VEL_HPF_AXES,
        name="Velocity",
        static_mask=static_mask,
    )

    if ZERO_VELOCITY_WHEN_STATIC and np.any(static_mask):
        velocity_used[static_mask, :] = 0.0

    # 如果速度被高通改过，需要重新积分位移
    if APPLY_VEL_HPF:
        displacement_raw = integrate_velocity_to_displacement_with_static_hold(
            velocity_used,
            time_s,
            static_mask,
        )

    # ============================================================
    # 5. 位移指定轴可选纯因果高通
    # ============================================================
    displacement_used = maybe_causal_highpass_axes(
        displacement_raw,
        fs=fs,
        cutoff_hz=DISP_HPF_CUTOFF_HZ,
        enabled=APPLY_DISP_HPF,
        axes=DISP_HPF_AXES,
        name="Displacement",
        static_mask=None,
    )

    if HOLD_POSITION_WHEN_STATIC and np.any(static_mask):
        displacement_used = hold_position_during_static(
            displacement_used,
            static_mask,
        )

    # ============================================================
    # 6. 最终轨迹
    # ============================================================
    if APPLY_SENSOR_TO_WORLD_FOR_TRAJ:
        try:
            displacement_world = sensor_to_world(displacement_used)
        except Exception as e:
            print("sensor_to_world failed, fallback to displacement_used:", e)
            displacement_world = displacement_used
    else:
        displacement_world = displacement_used

    # ============================================================
    # 7. 调试检查
    # ============================================================
    if np.any(static_mask):
        print("========== Final Static Check ==========")
        print("max |accel_used| in static:",
                np.max(np.abs(accel_used[static_mask])))
        print("max |velocity_used| in static:",
                np.max(np.abs(velocity_used[static_mask])))
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

    # ============================================================
    # 3D 轨迹
    # ============================================================

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

    def set_fixed_3d_view(ax):
        cx, cy, cz = TRAJ_VIEW_CENTER
        half = TRAJ_VIEW_SIZE_M / 2.0

        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        ax.set_zlim(cz - half, cz + half)

        try:
            ax.set_box_aspect([1, 1, 1])
        except Exception:
            pass

    set_fixed_3d_view(ax_traj)

    # ============================================================
    # 静止段背景显示
    # ============================================================

    def shade_static_regions(ax):
        for s, e in find_true_segments(static_mask):
            ax.axvspan(
                time_s[s],
                time_s[e],
                color="gray",
                alpha=0.18,
            )

    # ============================================================
    # 加速度图
    # ============================================================

    ax_acc.plot(
        time_s,
        accel[:, 0],
        label="lax used",
        color="C0",
        alpha=0.85,
        drawstyle="steps-post",
    )

    ax_acc.plot(
        time_s,
        accel[:, 1],
        label="lay used",
        color="C1",
        alpha=0.85,
        drawstyle="steps-post",
    )

    ax_acc.plot(
        time_s,
        accel[:, 2],
        label="laz used",
        color="C2",
        alpha=0.85,
        drawstyle="steps-post",
    )

    shade_static_regions(ax_acc)

    current_line_acc = ax_acc.axvline(time_s[0], color="black", lw=1.2)

    ax_acc.set_title("Linear Acceleration Used")
    ax_acc.set_ylabel("accel")
    ax_acc.legend(loc="upper right", fontsize="small", ncol=3)
    ax_acc.grid(True)

    # ============================================================
    # 速度图
    # ============================================================

    ax_vel.plot(time_s, velocity[:, 0], label="vx", color="C0", alpha=0.85)
    ax_vel.plot(time_s, velocity[:, 1], label="vy", color="C1", alpha=0.85)
    ax_vel.plot(time_s, velocity[:, 2], label="vz", color="C2", alpha=0.85)

    shade_static_regions(ax_vel)

    current_line_vel = ax_vel.axvline(time_s[0], color="black", lw=1.2)

    ax_vel.set_title("Velocity Used")
    ax_vel.set_ylabel("velocity")
    ax_vel.legend(loc="upper right", fontsize="small", ncol=3)
    ax_vel.grid(True)

    # ============================================================
    # 位移图
    # ============================================================

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

    # ============================================================
    # 姿态图
    # ============================================================

    ax_orient.set_title("Reference sensor orientation")
    ax_orient.set_axis_off()

    # ============================================================
    # 当前数值文本
    # ============================================================

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

    # ============================================================
    # Slider
    # ============================================================

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

        set_fixed_3d_view(ax_traj)

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

