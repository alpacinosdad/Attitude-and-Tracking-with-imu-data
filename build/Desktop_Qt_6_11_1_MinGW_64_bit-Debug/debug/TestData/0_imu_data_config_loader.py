"""
imu_data_config_loader.py

功能：
1. 集中配置文件路径、列名、单位和常用参数。
2. 读取 imu_result CSV：
   UTC_Time, Time_s, q0~q3, R11~R33, roll/pitch/yaw, aax/aay/aaz, lax/lay/laz
3. 读取 raw CSV：
   UTC_Time, Vin, ..., Accel_x/y/z, gyro_x/y/z, Accel_x_h/y_h/z_h, gyro_x_h/y_h/z_h
4. 统一列名访问方式，输出一个 IMUDataBundle，供后续算法文件 import 后直接使用。

推荐用法：
    from imu_data_config_loader import load_imu_data, DEFAULT_CONFIG

    data = load_imu_data(DEFAULT_CONFIG)
    result_df = data.result_df
    raw_df = data.raw_df
    time_s = data.time_s
    quat = data.quaternion          # Nx4, columns = q0,q1,q2,q3
    rotation_matrices = data.rotation_matrices  # Nx3x3
    world_accel = data.world_accel_with_gravity # Nx3, aax,aay,aaz
    body_linear_accel = data.body_linear_accel  # Nx3, lax,lay,laz
    raw_accel = data.raw_accel                 # Nx3, Accel_x_h/y_h/z_h by default
    raw_gyro = data.raw_gyro                   # Nx3, gyro_x_h/y_h/z_h by default
"""







from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple, Dict, Any

import numpy as np
import pandas as pd


# ============================================================
# 1. 集中参数配置区
# ============================================================

@dataclass
class IMUDataConfig:
    """IMU 数据读取与后续算法共用配置。"""

    # ----------------------------
    # 文件路径
    # ----------------------------
    result_imu_csv_path: str = "BAT_Heat_Log_Data_2026_06_26_18_23_36_imu_result.csv"
    raw_imu_csv_path: str = "BAT_Heat_Log_Data_2026_06_26_18_23_36.csv"

    # ----------------------------
    # 基础列名
    # 注意：程序内部会把 CSV 列名统一转换成小写，所以这里也建议用小写
    # ----------------------------
    utc_time_col: str = "utc_time"
    time_col: str = "time_s"

    # ----------------------------
    # imu_result CSV 列名
    # ----------------------------
    quat_cols: Tuple[str, str, str, str] = ("q0", "q1", "q2", "q3")

    rot_cols: Tuple[str, str, str, str, str, str, str, str, str] = (
        "r11", "r12", "r13",
        "r21", "r22", "r23",
        "r31", "r32", "r33",
    )

    euler_cols: Tuple[str, str, str] = ("roll", "pitch", "yaw")

    # 世界系总加速度，通常包含重力
    world_accel_cols: Tuple[str, str, str] = ("aax", "aay", "aaz")

    # 机体系线性加速度，如果 C 输出中已经提供 lax/lay/laz
    body_linear_accel_cols: Tuple[str, str, str] = ("lax", "lay", "laz")

    # ----------------------------
    # raw CSV 列名
    # ----------------------------
    # 原始加速度列：可能是 Accel_x/y/z，也可能滤波后用 Accel_x_h/y_h/z_h
    raw_accel_cols: Tuple[str, str, str] = ("accel_x_h", "accel_y_h", "accel_z_h")

    # 原始角速度列：可能是 gyro_x/y/z，也可能滤波后用 gyro_x_h/y_h/z_h
    raw_gyro_cols: Tuple[str, str, str] = ("gyro_x_h", "gyro_y_h", "gyro_z_h")

    # 如需读取未滤波原始列，可在外部改成：
    # raw_accel_cols=("accel_x", "accel_y", "accel_z")
    # raw_gyro_cols=("gyro_x", "gyro_y", "gyro_z")

    # ----------------------------
    # 单位与物理参数
    # ----------------------------
    gravity: float = 9.8

    # raw CSV 中加速度单位，常见："mg" 或 "m/s2"
    raw_accel_unit: str = "mg"

    # raw CSV 中角速度单位，常见："deg/s" 或 "rad/s"
    raw_gyro_unit: str = "deg/s"

    # ----------------------------
    # 是否严格检查列完整性
    # True：缺少关键列立即报错
    # False：缺少列时对应 array 返回 None
    # ----------------------------
    strict: bool = True

    # ----------------------------
    # 列名标准化
    # True：读取 CSV 后将列名全部 strip + lower
    # ----------------------------
    normalize_columns: bool = True


DEFAULT_CONFIG = IMUDataConfig()


# ============================================================
# 2. 数据容器：算法文件 import 后主要使用这个对象
# ============================================================

@dataclass
class IMUDataBundle:
    """统一保存 raw CSV 和 imu_result CSV 读取后的关键数据。"""

    config: IMUDataConfig

    # 原始 DataFrame
    result_df: pd.DataFrame
    raw_df: pd.DataFrame

    # 时间
    result_utc_time: Optional[pd.Series]
    raw_utc_time: Optional[pd.Series]
    time_s: Optional[np.ndarray]

    # imu_result 中姿态数据
    quaternion: Optional[np.ndarray]              # shape = Nx4, q0,q1,q2,q3
    rotation_matrices: Optional[np.ndarray]       # shape = Nx3x3
    euler_deg: Optional[np.ndarray]               # shape = Nx3, roll,pitch,yaw

    # imu_result 中加速度数据
    world_accel_with_gravity: Optional[np.ndarray] # shape = Nx3, aax,aay,aaz
    body_linear_accel: Optional[np.ndarray]        # shape = Nx3, lax,lay,laz

    # raw 中传感器数据
    raw_accel: Optional[np.ndarray]               # shape = Mx3
    raw_gyro: Optional[np.ndarray]                # shape = Mx3

    # 单位换算后的 raw 数据，方便后续算法直接用
    raw_accel_ms2: Optional[np.ndarray]           # m/s^2
    raw_gyro_rad_s: Optional[np.ndarray]          # rad/s


# ============================================================
# 3. 基础工具函数
# ============================================================

def _normalize_col_name(name: Any) -> str:
    """统一列名：去掉前后空格并转小写。"""
    return str(name).strip().lower()


def _normalize_df_columns(df: pd.DataFrame) -> pd.DataFrame:
    """返回列名标准化后的 DataFrame。"""
    df = df.copy()
    df.columns = [_normalize_col_name(c) for c in df.columns]
    return df


def _read_csv_with_normalized_columns(path: str | Path, normalize_columns: bool = True) -> pd.DataFrame:
    """读取 CSV，并按配置决定是否标准化列名。"""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"CSV 文件不存在: {path}")

    df = pd.read_csv(path, encoding="utf-8-sig")
    if normalize_columns:
        df = _normalize_df_columns(df)
    return df


def _existing_columns(df: pd.DataFrame, cols: Sequence[str]) -> list[str]:
    """返回 cols 中实际存在于 df 的列名。"""
    return [c for c in cols if c in df.columns]


def _check_required_columns(
    df: pd.DataFrame,
    cols: Sequence[str],
    df_name: str,
    strict: bool = True,
) -> bool:
    """检查必需列是否存在。strict=True 时缺列直接报错。"""
    missing = [c for c in cols if c not in df.columns]
    if missing and strict:
        raise KeyError(
            f"{df_name} 缺少必要列: {missing}\n"
            f"当前已有列: {list(df.columns)}"
        )
    return len(missing) == 0


def _get_numeric_array(
    df: pd.DataFrame,
    cols: Sequence[str],
    df_name: str,
    strict: bool = True,
) -> Optional[np.ndarray]:
    """从 DataFrame 中读取多列数值数组。缺列且 strict=False 时返回 None。"""
    ok = _check_required_columns(df, cols, df_name=df_name, strict=strict)
    if not ok:
        return None

    arr = df[list(cols)].apply(pd.to_numeric, errors="coerce").to_numpy(dtype=float)
    return arr


def _get_optional_series(
    df: pd.DataFrame,
    col: str,
) -> Optional[pd.Series]:
    """读取可选列。不存在则返回 None。"""
    if col not in df.columns:
        return None
    return df[col]


def _to_ms2(accel: Optional[np.ndarray], unit: str, gravity: float) -> Optional[np.ndarray]:
    """将 raw 加速度转换为 m/s^2。"""
    if accel is None:
        return None

    unit_norm = unit.strip().lower().replace(" ", "")
    if unit_norm in ("m/s2", "m/s^2", "ms2"):
        return accel.astype(float)
    if unit_norm == "mg":
        return accel.astype(float) * gravity / 1000.0
    if unit_norm == "g":
        return accel.astype(float) * gravity

    raise ValueError(f"不支持的 raw_accel_unit: {unit}. 可选: 'mg', 'g', 'm/s2'")


def _to_rad_s(gyro: Optional[np.ndarray], unit: str) -> Optional[np.ndarray]:
    """将 raw 角速度转换为 rad/s。"""
    if gyro is None:
        return None

    unit_norm = unit.strip().lower().replace(" ", "")
    if unit_norm in ("rad/s", "rads"):
        return gyro.astype(float)
    if unit_norm in ("deg/s", "dps", "degs"):
        return gyro.astype(float) * np.pi / 180.0

    raise ValueError(f"不支持的 raw_gyro_unit: {unit}. 可选: 'deg/s', 'rad/s'")


def _rotation_columns_to_matrices(rot_arr: Optional[np.ndarray]) -> Optional[np.ndarray]:
    """
    将 Nx9 的 R11~R33 数组转换成 Nx3x3。
    输入列顺序必须是：
    R11,R12,R13,R21,R22,R23,R31,R32,R33
    """
    if rot_arr is None:
        return None
    if rot_arr.ndim != 2 or rot_arr.shape[1] != 9:
        raise ValueError(f"rot_arr 形状应为 Nx9，当前为 {rot_arr.shape}")
    return rot_arr.reshape(-1, 3, 3)


# ============================================================
# 4. 主读取函数：后续算法文件主要 import 这个函数
# ============================================================

def load_imu_data(config: IMUDataConfig = DEFAULT_CONFIG) -> IMUDataBundle:
    """
    读取 imu_result CSV 和 raw IMU CSV，并返回统一的数据包。

    后续算法文件推荐这样使用：
        from imu_data_config_loader import load_imu_data, DEFAULT_CONFIG
        data = load_imu_data(DEFAULT_CONFIG)
    """

    # 1. 读取两个 CSV
    result_df = _read_csv_with_normalized_columns(
        config.result_imu_csv_path,
        normalize_columns=config.normalize_columns,
    )
    raw_df = _read_csv_with_normalized_columns(
        config.raw_imu_csv_path,
        normalize_columns=config.normalize_columns,
    )

    # 2. 读取时间
    result_utc_time = _get_optional_series(result_df, config.utc_time_col)
    raw_utc_time = _get_optional_series(raw_df, config.utc_time_col)

    if config.time_col in result_df.columns:
        time_s = pd.to_numeric(result_df[config.time_col], errors="coerce").to_numpy(dtype=float)
    else:
        if config.strict:
            raise KeyError(
                f"imu_result CSV 缺少时间列: {config.time_col}\n"
                f"当前已有列: {list(result_df.columns)}"
            )
        time_s = None

    # 3. 读取 imu_result 姿态数据
    quaternion = _get_numeric_array(
        result_df,
        config.quat_cols,
        df_name="imu_result CSV quaternion",
        strict=config.strict,
    )

    rot_arr = _get_numeric_array(
        result_df,
        config.rot_cols,
        df_name="imu_result CSV rotation matrix",
        strict=config.strict,
    )
    rotation_matrices = _rotation_columns_to_matrices(rot_arr)

    euler_deg = _get_numeric_array(
        result_df,
        config.euler_cols,
        df_name="imu_result CSV euler angles",
        strict=False,
    )

    # 4. 读取 imu_result 加速度数据
    world_accel_with_gravity = _get_numeric_array(
        result_df,
        config.world_accel_cols,
        df_name="imu_result CSV world acceleration aax/aay/aaz",
        strict=config.strict,
    )

    body_linear_accel = _get_numeric_array(
        result_df,
        config.body_linear_accel_cols,
        df_name="imu_result CSV body linear accel lax/lay/laz",
        strict=False,
    )

    # 5. 读取 raw IMU 传感器数据
    raw_accel = _get_numeric_array(
        raw_df,
        config.raw_accel_cols,
        df_name="raw CSV acceleration",
        strict=config.strict,
    )

    raw_gyro = _get_numeric_array(
        raw_df,
        config.raw_gyro_cols,
        df_name="raw CSV gyro",
        strict=config.strict,
    )

    # 6. 单位转换，方便算法文件直接使用
    raw_accel_ms2 = _to_ms2(raw_accel, config.raw_accel_unit, config.gravity)
    raw_gyro_rad_s = _to_rad_s(raw_gyro, config.raw_gyro_unit)

    return IMUDataBundle(
        config=config,
        result_df=result_df,
        raw_df=raw_df,
        result_utc_time=result_utc_time,
        raw_utc_time=raw_utc_time,
        time_s=time_s,
        quaternion=quaternion,
        rotation_matrices=rotation_matrices,
        euler_deg=euler_deg,
        world_accel_with_gravity=world_accel_with_gravity,
        body_linear_accel=body_linear_accel,
        raw_accel=raw_accel,
        raw_gyro=raw_gyro,
        raw_accel_ms2=raw_accel_ms2,
        raw_gyro_rad_s=raw_gyro_rad_s,
    )


# ============================================================
# 5. 调试辅助函数：检查读取是否正确
# ============================================================

def summarize_imu_data(data: IMUDataBundle) -> Dict[str, Any]:
    """返回一个简短 summary，方便调试打印。"""
    summary = {
        "result_csv": data.config.result_imu_csv_path,
        "raw_csv": data.config.raw_imu_csv_path,
        "result_rows": len(data.result_df),
        "raw_rows": len(data.raw_df),
        "result_columns": list(data.result_df.columns),
        "raw_columns": list(data.raw_df.columns),
        "time_s_shape": None if data.time_s is None else data.time_s.shape,
        "quaternion_shape": None if data.quaternion is None else data.quaternion.shape,
        "rotation_matrices_shape": None if data.rotation_matrices is None else data.rotation_matrices.shape,
        "euler_deg_shape": None if data.euler_deg is None else data.euler_deg.shape,
        "world_accel_with_gravity_shape": None if data.world_accel_with_gravity is None else data.world_accel_with_gravity.shape,
        "body_linear_accel_shape": None if data.body_linear_accel is None else data.body_linear_accel.shape,
        "raw_accel_shape": None if data.raw_accel is None else data.raw_accel.shape,
        "raw_gyro_shape": None if data.raw_gyro is None else data.raw_gyro.shape,
        "raw_accel_ms2_shape": None if data.raw_accel_ms2 is None else data.raw_accel_ms2.shape,
        "raw_gyro_rad_s_shape": None if data.raw_gyro_rad_s is None else data.raw_gyro_rad_s.shape,
    }
    return summary


def print_imu_data_summary(data: IMUDataBundle) -> None:
    """打印读取结果摘要。"""
    summary = summarize_imu_data(data)
    print("========== IMU Data Summary ==========")
    for k, v in summary.items():
        print(f"{k}: {v}")
    print("======================================")


# ============================================================
# 6. 可直接运行本文件做读取检查
# ============================================================

if __name__ == "__main__":
    data = load_imu_data(DEFAULT_CONFIG)
    print_imu_data_summary(data)
