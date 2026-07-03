from pathlib import Path

import sys
import io
import numpy as np
import pandas as pd

"""
0.静止点判定
├─ load_imu_csv
│
├─ load_world_linear_accel_from_result_for_raw_timeline
│   ├─ 查找 aax/aay/aaz
│   └─ 输出 linear_world_ms2 = [aax, aay, aaz - GRAV_FACT]
│
├─ simulate_c_bias_state_machine
│   ├─ compute_static_sample / detect_static_sample
│   │   ├─ 指标1: gyro rolling std 小
│   │   │   └─ gyro_std_norm < GYRO_STD_THRESHOLD
│   │   │
│   │   ├─ 指标2: accel rolling std 小
│   │   │   └─ accel_std_norm < RAW_ACCEL_STD_THRESHOLD
│   │   │
│   │   ├─ 指标3: gyro rolling mean 小
│   │   │   └─ gyro_norm_mean < GYRO_MEAN_THRESHOLD
│   │   │
│   │   ├─ 指标4: raw accel norm rolling mean 接近 1g
│   │   │   └─ abs(raw_accel_norm_mean - 1000mg) < RAW_ACCEL_NORM_TOL
│   │   │
│   │   └─ 指标5: 连续静止时间足够
│   │       └─ static segment length >= MIN_STATIC_TIME
│   │
│   ├─ Phase 0: 连续静止确认
│   │   └─ stationary_count >= STATIONARY_SAMPLE_COUNT 后进入 Phase 1
│   │
│   ├─ Phase 1: bias 采样
│   │   └─ bias_sample_count >= STATIONARY_SAMPLE_COUNT 后计算 new_bias
│   │
│   ├─ bias 更新
│   │   ├─ 第一次: gyro_bias = new_bias
│   │   └─ 后续: gyro_bias += clamp(alpha * (new_bias - old_bias))
│   │
│   └─ 记录 update_events
│
└─ print_summary
    └─ 打印最终理论 bias


尚没有写成库
尚没有线性因果

"""





# ============================================================
# Windows / VSCode Code Runner 中文输出兼容
# ============================================================

def force_utf8_stdout():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:
        sys.stdout = io.TextIOWrapper(
            sys.stdout.buffer,
            encoding="utf-8",
            errors="replace",
        )
        sys.stderr = io.TextIOWrapper(
            sys.stderr.buffer,
            encoding="utf-8",
            errors="replace",
        )


force_utf8_stdout()


# ============================================================
# 文件配置
# ============================================================

CSV_PATH = "BAT_Heat_Log_Data_2026_06_26_17_09_30.csv"

ACC_COLS = ("Accel_x_h", "Accel_y_h", "Accel_z_h")
GYRO_COLS = ("gyro_x_h", "gyro_y_h", "gyro_z_h")


# C 输出的 imu_result CSV，里面需要包含 aax/aay/aaz
# 用于检查静止窗口内世界系线性加速度 residual：
#   [aax, aay, aaz - GRAV_FACT]
RESULT_IMU_CSV_PATH = "BAT_Heat_Log_Data_2026_06_26_17_09_30_imu_result.csv"

AACCEL_COLS = ("aax", "aay", "aaz")

# 是否检查 aax/aay/aaz 静止 residual
ENABLE_AACCEL_STATIC_RESIDUAL_CHECK = True



# ============================================================
# 输入数据单位配置
# ============================================================
# 你的 C 代码输入要求：
#   ax/ay/az: m/s^2
#   gx/gy/gz: rad/s
#
# 但很多日志文件里常见是：
#   加速度: mg
#   陀螺仪: deg/s
#
# 如果你的 CSV 已经是滤波后传入 C 的原始单位：
#   acc 已经是 m/s^2，则改成 "m/s2"
#   gyro 已经是 rad/s，则改成 "rad/s"

ACC_INPUT_UNIT = "mg"       # 可选: "mg" 或 "m/s2"
GYRO_INPUT_UNIT = "deg/s"   # 可选: "deg/s" 或 "rad/s"


# ============================================================
# 对应当前 C 代码的参数
# ============================================================

GRAV_FACT = 9.8

# 当前 C 里的静止判定阈值
GYRO_STATIONARY_THRESH = 0.05     # rad/s
ACC_STATIONARY_TOL = 0.5          # m/s^2

STATIONARY_SAMPLE_COUNT = 30

# 当前 C 里的后续 bias 慢更新逻辑
BIAS_UPDATE_ALPHA = 0.15
BIAS_MAX_STEP_RAD_S = 0.005

MG_TO_MS2 = GRAV_FACT / 1000.0
DEG_TO_RAD = np.pi / 180.0
RAD_TO_DEG = 180.0 / np.pi


# ============================================================
# 工具函数
# ============================================================

def normalize_col_name(name):
    s = str(name).strip()
    s = s.replace('<strong data-lexical-text="true">', "")
    s = s.replace("</strong>", "")
    s = s.replace("<strong>", "")
    s = s.strip()
    return s.lower()


def find_column(df, target_name):
    target_norm = normalize_col_name(target_name)

    for col in df.columns:
        if normalize_col_name(col) == target_norm:
            return col

    raise ValueError(
        f"找不到列 {target_name}。当前 CSV 列名为：\n{list(df.columns)}"
    )


def finite_or_zero(arr):
    arr = np.asarray(arr, dtype=float).copy()
    arr[~np.isfinite(arr)] = 0.0
    return arr


def get_time_array(df, n):
    for c in ("time_s", "Time_s", "time", "Time", "timestamp", "Timestamp"):
        if c in df.columns:
            t = finite_or_zero(df[c].to_numpy(dtype=float))
            return t, c

    return np.arange(n, dtype=float), "index"


def vector_norm(v):
    v = np.asarray(v, dtype=float)
    return float(np.sqrt(np.sum(v * v)))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ============================================================
# 数据读取
# ============================================================

def load_imu_csv(csv_path):
    path = Path(csv_path)

    if not path.exists():
        raise FileNotFoundError(f"找不到 CSV 文件: {path.resolve()}")

    df = pd.read_csv(path)

    acc_cols_real = [find_column(df, c) for c in ACC_COLS]
    gyro_cols_real = [find_column(df, c) for c in GYRO_COLS]

    acc_raw = finite_or_zero(df[acc_cols_real].to_numpy(dtype=float))
    gyro_raw = finite_or_zero(df[gyro_cols_real].to_numpy(dtype=float))

    if ACC_INPUT_UNIT == "mg":
        acc_ms2 = acc_raw * MG_TO_MS2
        acc_mg = acc_raw
    elif ACC_INPUT_UNIT == "m/s2":
        acc_ms2 = acc_raw
        acc_mg = acc_ms2 / GRAV_FACT * 1000.0
    else:
        raise ValueError("ACC_INPUT_UNIT 只能是 'mg' 或 'm/s2'")

    if GYRO_INPUT_UNIT == "deg/s":
        gyro_deg_s = gyro_raw
        gyro_rad_s = gyro_raw * DEG_TO_RAD
    elif GYRO_INPUT_UNIT == "rad/s":
        gyro_rad_s = gyro_raw
        gyro_deg_s = gyro_rad_s * RAD_TO_DEG
    else:
        raise ValueError("GYRO_INPUT_UNIT 只能是 'deg/s' 或 'rad/s'")

    time_arr, time_col = get_time_array(df, len(df))

    return {
        "df": df,
        "acc_cols_real": acc_cols_real,
        "gyro_cols_real": gyro_cols_real,
        "acc_raw": acc_raw,
        "gyro_raw": gyro_raw,
        "acc_mg": acc_mg,
        "gyro_deg_s": gyro_deg_s,
        "acc_ms2": acc_ms2,
        "gyro_rad_s": gyro_rad_s,
        "time_arr": time_arr,
        "time_col": time_col,
    }


def load_world_linear_accel_from_result_for_raw_timeline(raw_data):
    """
    读取 C 输出的 imu_result CSV，并对齐到 raw CSV 的时间轴。

    输出:
        linear_world_ms2: Nx3
            [:,0] = aax
            [:,1] = aay
            [:,2] = aaz - GRAV_FACT

    如果找不到 imu_result 或 aax/aay/aaz，则返回 None。
    """
    if not ENABLE_AACCEL_STATIC_RESIDUAL_CHECK:
        return None

    result_path = Path(RESULT_IMU_CSV_PATH)

    if not result_path.exists():
        print(f"[AACCEL CHECK] 找不到 imu_result CSV: {result_path.resolve()}")
        print("[AACCEL CHECK] 将跳过 aax/aay/aaz 静止 residual 检查。")
        return None

    result_df = pd.read_csv(result_path)

    try:
        aaccel_cols_real = [find_column(result_df, c) for c in AACCEL_COLS]
    except Exception as e:
        print("[AACCEL CHECK] 找不到 aax/aay/aaz 列:", e)
        print("[AACCEL CHECK] 将跳过 aax/aay/aaz 静止 residual 检查。")
        return None

    aaccel = finite_or_zero(result_df[aaccel_cols_real].to_numpy(dtype=float))

    raw_time = raw_data["time_arr"]
    raw_n = len(raw_time)
    raw_time_col = raw_data["time_col"]

    # 情况1：行数相同，直接按 index 对齐
    if len(result_df) == raw_n:
        print("[AACCEL CHECK] imu_result 与 raw 行数一致，使用 index 对齐。")

        linear_world_ms2 = aaccel.copy()
        linear_world_ms2[:, 2] -= GRAV_FACT

        return linear_world_ms2

    # 情况2：两边都有有效时间列，用时间插值对齐
    result_time_col = None
    for c in ("time_s", "Time_s", "time", "Time", "timestamp", "Timestamp"):
        if c in result_df.columns:
            result_time_col = c
            break

    if raw_time_col != "index" and result_time_col is not None:
        print(
            "[AACCEL CHECK] imu_result 与 raw 行数不一致，"
            f"使用时间列对齐: raw={raw_time_col}, result={result_time_col}"
        )

        result_time = finite_or_zero(result_df[result_time_col].to_numpy(dtype=float))

        # 确保时间递增，否则按时间排序
        order = np.argsort(result_time)
        result_time_sorted = result_time[order]
        aaccel_sorted = aaccel[order, :]

        linear_world_ms2 = np.zeros((raw_n, 3), dtype=float)

        for axis in range(3):
            linear_world_ms2[:, axis] = np.interp(
                raw_time,
                result_time_sorted,
                aaccel_sorted[:, axis],
            )

        linear_world_ms2[:, 2] -= GRAV_FACT

        return linear_world_ms2

    print("[AACCEL CHECK] imu_result 与 raw 行数不一致，且无法按时间对齐。")
    print("[AACCEL CHECK] 将跳过 aax/aay/aaz 静止 residual 检查。")

    return None


# ============================================================
# C 单点静止判定
# 对应 C:
#
# static int is_stationary_sample(...)
# {
#     gyro_norm = sqrt(gx^2 + gy^2 + gz^2)
#     acc_norm = sqrt(ax^2 + ay^2 + az^2)
#     return gyro_norm < GYRO_STATIONARY_THRESH &&
#            fabs(acc_norm - GRAV_FACT) < ACC_STATIONARY_TOL
# }
# ============================================================

def compute_c_static_sample(acc_ms2, gyro_rad_s):
    ax = acc_ms2[:, 0]
    ay = acc_ms2[:, 1]
    az = acc_ms2[:, 2]

    gx = gyro_rad_s[:, 0]
    gy = gyro_rad_s[:, 1]
    gz = gyro_rad_s[:, 2]

    acc_norm_ms2 = np.sqrt(ax * ax + ay * ay + az * az)
    gyro_norm_rad_s = np.sqrt(gx * gx + gy * gy + gz * gz)

    static_sample = (
        (gyro_norm_rad_s < GYRO_STATIONARY_THRESH)
        & (np.abs(acc_norm_ms2 - GRAV_FACT) < ACC_STATIONARY_TOL)
    )

    return static_sample, gyro_norm_rad_s, acc_norm_ms2


# ============================================================
# 模拟当前 C 代码的两阶段静止窗口 + bias 更新逻辑
# ============================================================

def simulate_c_bias_state_machine(acc_ms2, gyro_rad_s, gyro_deg_s, time_arr,linear_world_ms2,):
    static_sample, gyro_norm_rad_s, acc_norm_ms2 = compute_c_static_sample(
        acc_ms2,
        gyro_rad_s,
    )

    n = len(static_sample)

    # 对应 MIL_Handle_t 里的状态
    stationary_count = 0
    stationary_target = STATIONARY_SAMPLE_COUNT

    stationary_phase = 0      # 0: 静止确认阶段；1: bias采样阶段
    bias_sample_count = 0

    bias_gx_sum = 0.0
    bias_gy_sum = 0.0
    bias_gz_sum = 0.0

    gyro_bias_rad_s = np.zeros(3, dtype=float)
    is_gyro_bias_inited = False

    # 记录 phase 1 的采样起止
    bias_sample_start_index = None

    debug_records = []
    update_events = []
    reset_events = []

    for i in range(n):
        gx = float(gyro_rad_s[i, 0])
        gy = float(gyro_rad_s[i, 1])
        gz = float(gyro_rad_s[i, 2])

        is_static = bool(static_sample[i])

        phase_before = stationary_phase
        stationary_count_before = stationary_count
        bias_sample_count_before = bias_sample_count
        bias_before = gyro_bias_rad_s.copy()

        event = ""

        if is_static:
            if stationary_phase == 0:
                # phase 0: 只做连续静止确认，不累计 bias
                stationary_count += 1

                if stationary_count >= stationary_target:
                    # 当前样本使静止确认完成
                    # 注意：按照 C 逻辑，当前这个样本不进入 bias 求均值
                    stationary_phase = 1
                    bias_sample_count = 0

                    bias_gx_sum = 0.0
                    bias_gy_sum = 0.0
                    bias_gz_sum = 0.0

                    bias_sample_start_index = i + 1
                    event = "static_confirmed_enter_bias_sampling"

            else:
                # phase 1: 已确认静止，重新采样 gyro 求 newBias
                if bias_sample_count == 0:
                    bias_sample_start_index = i

                bias_sample_count += 1

                bias_gx_sum += gx
                bias_gy_sum += gy
                bias_gz_sum += gz

                if bias_sample_count >= stationary_target:
                    inv = 1.0 / float(bias_sample_count)

                    new_bias_rad_s = np.array([
                        bias_gx_sum * inv,
                        bias_gy_sum * inv,
                        bias_gz_sum * inv,
                    ], dtype=float)

                    new_bias_deg_s = new_bias_rad_s * RAD_TO_DEG
                    prev_bias_rad_s = gyro_bias_rad_s.copy()
                    prev_bias_deg_s = prev_bias_rad_s * RAD_TO_DEG

                    if not is_gyro_bias_inited:
                        # 第一次 bias：直接赋值
                        gyro_bias_rad_s = new_bias_rad_s.copy()
                        is_gyro_bias_inited = True

                        update_mode = "init_set_direct"
                        step_rad_s = gyro_bias_rad_s - prev_bias_rad_s
                    else:
                        # 后续 bias：慢更新 + 单次限幅
                        raw_step_rad_s = BIAS_UPDATE_ALPHA * (new_bias_rad_s - gyro_bias_rad_s)

                        step_rad_s = np.array([
                            clamp(float(raw_step_rad_s[0]), -BIAS_MAX_STEP_RAD_S, BIAS_MAX_STEP_RAD_S),
                            clamp(float(raw_step_rad_s[1]), -BIAS_MAX_STEP_RAD_S, BIAS_MAX_STEP_RAD_S),
                            clamp(float(raw_step_rad_s[2]), -BIAS_MAX_STEP_RAD_S, BIAS_MAX_STEP_RAD_S),
                        ], dtype=float)

                        gyro_bias_rad_s = gyro_bias_rad_s + step_rad_s
                        update_mode = "slow_update_alpha_limit"

                    updated_bias_rad_s = gyro_bias_rad_s.copy()
                    updated_bias_deg_s = updated_bias_rad_s * RAD_TO_DEG
                    step_deg_s = step_rad_s * RAD_TO_DEG

                    sidx = int(bias_sample_start_index) if bias_sample_start_index is not None else int(i - bias_sample_count + 1)
                    eidx = int(i)

                    # 统计当前 bias 采样窗口质量，帮助你判断静止段是否靠谱
                    gyro_window_deg_s = gyro_deg_s[sidx:eidx + 1, :]
                    acc_norm_window_ms2 = acc_norm_ms2[sidx:eidx + 1]
                    gyro_std_deg_s = np.std(gyro_window_deg_s, axis=0, ddof=0)
                    gyro_std_norm_deg_s = vector_norm(gyro_std_deg_s)
                    acc_norm_mean_ms2 = float(np.mean(acc_norm_window_ms2))
                    acc_norm_std_ms2 = float(np.std(acc_norm_window_ms2, ddof=0))
                    # 统计当前 bias 采样窗口内的世界系线性加速度 residual
                    # linear_world_ms2 = [aax, aay, aaz - GRAV_FACT]
                    if linear_world_ms2 is not None:
                        lin_window_ms2 = linear_world_ms2[sidx:eidx + 1, :]

                        lin_mean_ms2 = np.mean(lin_window_ms2, axis=0)
                        lin_std_ms2 = np.std(lin_window_ms2, axis=0, ddof=0)

                        lin_mean_norm_ms2 = vector_norm(lin_mean_ms2)
                        lin_std_norm_ms2 = vector_norm(lin_std_ms2)

                        lin_mean_mg = lin_mean_ms2 / GRAV_FACT * 1000.0
                        lin_std_mg = lin_std_ms2 / GRAV_FACT * 1000.0

                        lin_mean_norm_mg = lin_mean_norm_ms2 / GRAV_FACT * 1000.0
                        lin_std_norm_mg = lin_std_norm_ms2 / GRAV_FACT * 1000.0
                    else:
                        lin_mean_ms2 = np.array([np.nan, np.nan, np.nan], dtype=float)
                        lin_std_ms2 = np.array([np.nan, np.nan, np.nan], dtype=float)

                        lin_mean_norm_ms2 = np.nan
                        lin_std_norm_ms2 = np.nan

                        lin_mean_mg = np.array([np.nan, np.nan, np.nan], dtype=float)
                        lin_std_mg = np.array([np.nan, np.nan, np.nan], dtype=float)

                        lin_mean_norm_mg = np.nan
                        lin_std_norm_mg = np.nan

                    update_events.append({
                        "event_index": len(update_events),
                        "bias_sample_start_index": sidx,
                        "bias_sample_end_index": eidx,
                        "bias_sample_length": int(bias_sample_count),

                        "bias_sample_start_time": float(time_arr[sidx]),
                        "bias_sample_end_time": float(time_arr[eidx]),

                        "update_mode": update_mode,

                        "new_bias_x_rad_s": float(new_bias_rad_s[0]),
                        "new_bias_y_rad_s": float(new_bias_rad_s[1]),
                        "new_bias_z_rad_s": float(new_bias_rad_s[2]),

                        "new_bias_x_deg_s": float(new_bias_deg_s[0]),
                        "new_bias_y_deg_s": float(new_bias_deg_s[1]),
                        "new_bias_z_deg_s": float(new_bias_deg_s[2]),

                        "prev_bias_x_rad_s": float(prev_bias_rad_s[0]),
                        "prev_bias_y_rad_s": float(prev_bias_rad_s[1]),
                        "prev_bias_z_rad_s": float(prev_bias_rad_s[2]),

                        "prev_bias_x_deg_s": float(prev_bias_deg_s[0]),
                        "prev_bias_y_deg_s": float(prev_bias_deg_s[1]),
                        "prev_bias_z_deg_s": float(prev_bias_deg_s[2]),

                        "step_x_rad_s": float(step_rad_s[0]),
                        "step_y_rad_s": float(step_rad_s[1]),
                        "step_z_rad_s": float(step_rad_s[2]),

                        "step_x_deg_s": float(step_deg_s[0]),
                        "step_y_deg_s": float(step_deg_s[1]),
                        "step_z_deg_s": float(step_deg_s[2]),

                        "updated_bias_x_rad_s": float(updated_bias_rad_s[0]),
                        "updated_bias_y_rad_s": float(updated_bias_rad_s[1]),
                        "updated_bias_z_rad_s": float(updated_bias_rad_s[2]),

                        "updated_bias_x_deg_s": float(updated_bias_deg_s[0]),
                        "updated_bias_y_deg_s": float(updated_bias_deg_s[1]),
                        "updated_bias_z_deg_s": float(updated_bias_deg_s[2]),

                        "gyro_std_x_deg_s": float(gyro_std_deg_s[0]),
                        "gyro_std_y_deg_s": float(gyro_std_deg_s[1]),
                        "gyro_std_z_deg_s": float(gyro_std_deg_s[2]),
                        "gyro_std_norm_deg_s": float(gyro_std_norm_deg_s),

                        "acc_norm_mean_ms2": acc_norm_mean_ms2,
                        "acc_norm_std_ms2": acc_norm_std_ms2,
                        "acc_norm_mean_mg": float(acc_norm_mean_ms2 / GRAV_FACT * 1000.0),
                        "acc_norm_std_mg": float(acc_norm_std_ms2 / GRAV_FACT * 1000.0),

                        # 世界系线性加速度 residual:
                        #   x = aax
                        #   y = aay
                        #   z = aaz - GRAV_FACT
                        "lin_res_mean_x_ms2": float(lin_mean_ms2[0]),
                        "lin_res_mean_y_ms2": float(lin_mean_ms2[1]),
                        "lin_res_mean_z_ms2": float(lin_mean_ms2[2]),
                        "lin_res_mean_norm_ms2": float(lin_mean_norm_ms2),

                        "lin_res_std_x_ms2": float(lin_std_ms2[0]),
                        "lin_res_std_y_ms2": float(lin_std_ms2[1]),
                        "lin_res_std_z_ms2": float(lin_std_ms2[2]),
                        "lin_res_std_norm_ms2": float(lin_std_norm_ms2),

                        "lin_res_mean_x_mg": float(lin_mean_mg[0]),
                        "lin_res_mean_y_mg": float(lin_mean_mg[1]),
                        "lin_res_mean_z_mg": float(lin_mean_mg[2]),
                        "lin_res_mean_norm_mg": float(lin_mean_norm_mg),

                        "lin_res_std_x_mg": float(lin_std_mg[0]),
                        "lin_res_std_y_mg": float(lin_std_mg[1]),
                        "lin_res_std_z_mg": float(lin_std_mg[2]),
                        "lin_res_std_norm_mg": float(lin_std_norm_mg),





                        "bias_update_alpha": float(BIAS_UPDATE_ALPHA),
                        "bias_max_step_rad_s": float(BIAS_MAX_STEP_RAD_S),
                        "bias_max_step_deg_s": float(BIAS_MAX_STEP_RAD_S * RAD_TO_DEG),
                    })

                    event = "bias_updated"

                    # 对应 C 里的 reset_stationary_window(p)
                    stationary_count = 0
                    stationary_phase = 0
                    bias_sample_count = 0

                    bias_gx_sum = 0.0
                    bias_gy_sum = 0.0
                    bias_gz_sum = 0.0

                    bias_sample_start_index = None

        else:
            # 对应 C:
            # else { reset_stationary_window(p); }
            if stationary_count > 0 or stationary_phase != 0 or bias_sample_count > 0:
                reset_events.append({
                    "reset_index": int(i),
                    "reset_time": float(time_arr[i]),
                    "phase_before_reset": int(stationary_phase),
                    "stationary_count_before_reset": int(stationary_count),
                    "bias_sample_count_before_reset": int(bias_sample_count),
                    "gyro_norm_rad_s": float(gyro_norm_rad_s[i]),
                    "gyro_norm_deg_s": float(gyro_norm_rad_s[i] * RAD_TO_DEG),
                    "acc_norm_ms2": float(acc_norm_ms2[i]),
                    "acc_norm_mg": float(acc_norm_ms2[i] / GRAV_FACT * 1000.0),
                })

            stationary_count = 0
            stationary_phase = 0
            bias_sample_count = 0

            bias_gx_sum = 0.0
            bias_gy_sum = 0.0
            bias_gz_sum = 0.0

            bias_sample_start_index = None

            event = "reset_not_static"

        bias_after = gyro_bias_rad_s.copy()

        debug_records.append({
            "index": int(i),
            "time": float(time_arr[i]),

            "ax_ms2": float(acc_ms2[i, 0]),
            "ay_ms2": float(acc_ms2[i, 1]),
            "az_ms2": float(acc_ms2[i, 2]),

            "gx_rad_s": float(gyro_rad_s[i, 0]),
            "gy_rad_s": float(gyro_rad_s[i, 1]),
            "gz_rad_s": float(gyro_rad_s[i, 2]),

            "gx_deg_s": float(gyro_deg_s[i, 0]),
            "gy_deg_s": float(gyro_deg_s[i, 1]),
            "gz_deg_s": float(gyro_deg_s[i, 2]),

            "gyro_norm_rad_s": float(gyro_norm_rad_s[i]),
            "gyro_norm_deg_s": float(gyro_norm_rad_s[i] * RAD_TO_DEG),
            "acc_norm_ms2": float(acc_norm_ms2[i]),
            "acc_norm_mg": float(acc_norm_ms2[i] / GRAV_FACT * 1000.0),

            "is_static_raw_c_logic": int(is_static),

            "phase_before": int(phase_before),
            "phase_after": int(stationary_phase),

            "stationary_count_before": int(stationary_count_before),
            "stationary_count_after": int(stationary_count),

            "bias_sample_count_before": int(bias_sample_count_before),
            "bias_sample_count_after": int(bias_sample_count),

            "gyro_bias_x_rad_s_before": float(bias_before[0]),
            "gyro_bias_y_rad_s_before": float(bias_before[1]),
            "gyro_bias_z_rad_s_before": float(bias_before[2]),

            "gyro_bias_x_rad_s_after": float(bias_after[0]),
            "gyro_bias_y_rad_s_after": float(bias_after[1]),
            "gyro_bias_z_rad_s_after": float(bias_after[2]),

            "gyro_bias_x_deg_s_after": float(bias_after[0] * RAD_TO_DEG),
            "gyro_bias_y_deg_s_after": float(bias_after[1] * RAD_TO_DEG),
            "gyro_bias_z_deg_s_after": float(bias_after[2] * RAD_TO_DEG),

            "is_gyro_bias_inited": int(is_gyro_bias_inited),
            "event": event,
        })

    debug_df = pd.DataFrame(debug_records)
    update_df = pd.DataFrame(update_events)
    reset_df = pd.DataFrame(reset_events)

    final_bias_rad_s = gyro_bias_rad_s.copy()
    final_bias_deg_s = final_bias_rad_s * RAD_TO_DEG

    return debug_df, update_df, reset_df, final_bias_deg_s, final_bias_rad_s


# ============================================================
# 打印
# ============================================================

def print_summary(data, debug_df, update_df, reset_df, final_bias_deg_s, final_bias_rad_s):
    print("========== CSV Info ==========")
    print(f"file: {CSV_PATH}")
    print(f"rows: {len(debug_df)}")
    print(f"time column: {data['time_col']}")
    print(f"acc columns: {data['acc_cols_real']}")
    print(f"gyro columns: {data['gyro_cols_real']}")
    print(f"ACC_INPUT_UNIT: {ACC_INPUT_UNIT}")
    print(f"GYRO_INPUT_UNIT: {GYRO_INPUT_UNIT}")
    print("==============================")

    print("\n========== C Static Detection Config ==========")
    print(
        f"GYRO_STATIONARY_THRESH = {GYRO_STATIONARY_THRESH:.9f} rad/s "
        f"= {GYRO_STATIONARY_THRESH * RAD_TO_DEG:.9f} deg/s"
    )
    print(
        f"ACC_STATIONARY_TOL = {ACC_STATIONARY_TOL:.9f} m/s^2 "
        f"= {ACC_STATIONARY_TOL / GRAV_FACT * 1000.0:.6f} mg"
    )
    print(f"STATIONARY_SAMPLE_COUNT = {STATIONARY_SAMPLE_COUNT}")
    print(f"BIAS_UPDATE_ALPHA = {BIAS_UPDATE_ALPHA}")
    print(
        f"BIAS_MAX_STEP_RAD_S = {BIAS_MAX_STEP_RAD_S:.9f} rad/s "
        f"= {BIAS_MAX_STEP_RAD_S * RAD_TO_DEG:.9f} deg/s"
    )
    print("===============================================")

    raw_static_count = int(debug_df["is_static_raw_c_logic"].sum())
    print("\n========== Static Sample Summary ==========")
    print(f"raw static samples = {raw_static_count} / {len(debug_df)}")
    print(f"bias update events = {len(update_df)}")
    print(f"reset events during partial window = {len(reset_df)}")
    print("===========================================")

    print("\n========== Bias Update Events ==========")
    if update_df.empty:
        print("没有完成任何一次 bias 更新。")
        print("原因通常是：没有连续完成 50 点静止确认 + 50 点bias采样。")
    else:
        pd.set_option("display.max_rows", None)
        pd.set_option("display.max_columns", None)
        pd.set_option("display.width", 260)
        pd.set_option("display.float_format", lambda x: f"{x:.9f}")

        display_cols = [
            "event_index",
            "bias_sample_start_index",
            "bias_sample_end_index",
            "bias_sample_length",
            "bias_sample_start_time",
            "bias_sample_end_time",
            "update_mode",

            "new_bias_x_deg_s",
            "new_bias_y_deg_s",
            "new_bias_z_deg_s",

            "prev_bias_x_deg_s",
            "prev_bias_y_deg_s",
            "prev_bias_z_deg_s",

            "step_x_deg_s",
            "step_y_deg_s",
            "step_z_deg_s",

            "updated_bias_x_deg_s",
            "updated_bias_y_deg_s",
            "updated_bias_z_deg_s",

            "gyro_std_norm_deg_s",
            "acc_norm_mean_mg",
            "acc_norm_std_mg",

            # 静止窗口内世界系线性加速度 residual
            # 也就是 [aax, aay, aaz - 9.8]
            "lin_res_mean_x_ms2",
            "lin_res_mean_y_ms2",
            "lin_res_mean_z_ms2",
            "lin_res_mean_norm_ms2",

            "lin_res_mean_x_mg",
            "lin_res_mean_y_mg",
            "lin_res_mean_z_mg",
            "lin_res_mean_norm_mg",

            "lin_res_std_norm_mg",
        ]

        display_cols = [c for c in display_cols if c in update_df.columns]

        print(update_df[display_cols])

    print("=======================================")

    print("\n========== Final Theoretical Bias ==========")
    print(
        "final bias deg/s = "
        f"({final_bias_deg_s[0]:+.9f}, "
        f"{final_bias_deg_s[1]:+.9f}, "
        f"{final_bias_deg_s[2]:+.9f})"
    )
    print(
        "final bias rad/s = "
        f"({final_bias_rad_s[0]:+.9f}, "
        f"{final_bias_rad_s[1]:+.9f}, "
        f"{final_bias_rad_s[2]:+.9f})"
    )
    print("============================================")


# ============================================================
# 主函数
# ============================================================

def main():
    data = load_imu_csv(CSV_PATH)

    linear_world_ms2 = load_world_linear_accel_from_result_for_raw_timeline(data)

    debug_df, update_df, reset_df, final_bias_deg_s, final_bias_rad_s = (
        simulate_c_bias_state_machine(
            acc_ms2=data["acc_ms2"],
            gyro_rad_s=data["gyro_rad_s"],
            gyro_deg_s=data["gyro_deg_s"],
            time_arr=data["time_arr"],
            linear_world_ms2=linear_world_ms2,
        )
    )

    print_summary(
        data=data,
        debug_df=debug_df,
        update_df=update_df,
        reset_df=reset_df,
        final_bias_deg_s=final_bias_deg_s,
        final_bias_rad_s=final_bias_rad_s,
    )


if __name__ == "__main__":
    main()