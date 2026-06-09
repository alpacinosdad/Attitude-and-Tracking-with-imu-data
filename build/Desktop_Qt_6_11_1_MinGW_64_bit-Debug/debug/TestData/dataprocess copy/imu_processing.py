"""
清晰版 IMU 数据处理工具

目标
- 文件读取：只负责把原始 txt/csv 读成 df_raw
- 滤波设计：只提供 低通 / 高通 / 带通 三个正向因果滤波函数
- 滤波调用：统一用 scipy.signal 设计，统一用因果 forward 方式滤波
- 绘图：尽量少逻辑，参数简单、函数名字直接表达意义

说明
- 这里不提供双向滤波，不提供 filtfilt。
- notebook 中只要改参数区即可复用。
"""

from __future__ import annotations

from typing import Iterable, Mapping, Sequence, Tuple, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal


# =========================
# 文件读取
# =========================

def read_imu_text_file(
    file_path: str,
    *,
    sample_rate_hz: float,
    imu_column_names: Sequence[str],
    time_column_name: str = "time_sec",
) -> pd.DataFrame:
    """读取 IMU 文本/CSV，生成统一的 df_raw。"""
    df = pd.read_csv(file_path, sep=r"\s+|,", engine="python", skipinitialspace=True)
    df.columns = df.columns.str.strip()
    df = df.loc[:, ~df.columns.str.contains(r"^Unnamed", regex=True)]

    raw = df.loc[:, list(imu_column_names)].copy()
    for column_name in imu_column_names:
        raw[column_name] = pd.to_numeric(raw[column_name], errors="coerce")
    raw = raw.dropna(subset=list(imu_column_names)).reset_index(drop=True)
    raw[time_column_name] = np.arange(len(raw), dtype=np.int64) / float(sample_rate_hz)
    return raw


# =========================
# 滤波设计：三个明确函数
# =========================

def create_lowpass_sos(*, sample_rate_hz: float, filter_order: int, cutoff_hz: float) -> np.ndarray:
    """设计正向低通滤波器 SOS。"""
    return signal.butter(int(filter_order), float(cutoff_hz), btype="lowpass", fs=float(sample_rate_hz), output="sos")


def create_highpass_sos(*, sample_rate_hz: float, filter_order: int, cutoff_hz: float) -> np.ndarray:
    """设计正向高通滤波器 SOS。"""
    return signal.butter(int(filter_order), float(cutoff_hz), btype="highpass", fs=float(sample_rate_hz), output="sos")


def create_bandpass_sos(
    *,
    sample_rate_hz: float,
    filter_order: int,
    lowcut_hz: float,
    highcut_hz: float,
) -> np.ndarray:
    """设计正向带通滤波器 SOS。"""
    return signal.butter(
        int(filter_order),
        [float(lowcut_hz), float(highcut_hz)],
        btype="bandpass",
        fs=float(sample_rate_hz),
        output="sos",
    )


# =========================
# 因果滤波调用：统一 forward 方式
# =========================

def apply_forward_sos_filter(
    dataframe: pd.DataFrame,
    *,
    input_columns: Sequence[str],
    sos: np.ndarray,
    output_suffix: str,
    initial_padding_iter: int = 0,
) -> pd.DataFrame:
    """对指定列做正向因果 SOS 滤波，返回新 dataframe，不改原数据。"""
    filtered = dataframe.copy()
    for column_name in input_columns:
        values = filtered[column_name].to_numpy(dtype=float)
        if len(values) == 0:
            filtered[f"{column_name}{output_suffix}"] = values
            continue

        channel = signal.sosfilt_zi(sos) * values[0]
        if initial_padding_iter > 0:
            for _ in range(int(initial_padding_iter)):
                _, channel = signal.sosfilt(sos, [values[0]], zi=channel)

        result, _ = signal.sosfilt(sos, values, zi=channel)
        filtered[f"{column_name}{output_suffix}"] = result
    return filtered


# =========================
# 去重力：两个常用方法 + IIR 公式法
# =========================

def remove_gravity_by_lowpass_subtraction(
    dataframe: pd.DataFrame,
    *,
    acceleration_columns: Sequence[str],
    sample_rate_hz: float,
    filter_order: int,
    cutoff_hz: float,
    output_gravity_suffix: str = "_gravity_lowpass",
    output_linear_suffix: str = "_linear_lowpass",
    initial_padding_iter: int = 5000,
) -> pd.DataFrame:
    """先低通估计重力，再用 raw - gravity 得到线性加速度。"""
    sos = create_lowpass_sos(
        sample_rate_hz=sample_rate_hz,
        filter_order=filter_order,
        cutoff_hz=cutoff_hz,
    )
    gravity = apply_forward_sos_filter(
        dataframe,
        input_columns=acceleration_columns,
        sos=sos,
        output_suffix=output_gravity_suffix,
        initial_padding_iter=initial_padding_iter,
    )
    for column_name in acceleration_columns:
        gravity[f"{column_name}{output_linear_suffix}"] = gravity[column_name] - gravity[f"{column_name}{output_gravity_suffix}"]
    return gravity


def remove_gravity_by_highpass_filter(
    dataframe: pd.DataFrame,
    *,
    acceleration_columns: Sequence[str],
    sample_rate_hz: float,
    filter_order: int,
    cutoff_hz: float,
    output_suffix: str = "_highpass",
    initial_padding_iter: int = 5000,
) -> pd.DataFrame:
    """直接高通去重力。"""
    sos = create_highpass_sos(
        sample_rate_hz=sample_rate_hz,
        filter_order=filter_order,
        cutoff_hz=cutoff_hz,
    )
    return apply_forward_sos_filter(
        dataframe,
        input_columns=acceleration_columns,
        sos=sos,
        output_suffix=output_suffix,
        initial_padding_iter=initial_padding_iter,
    )


def remove_gravity_by_first_order_iir_formula(
    dataframe: pd.DataFrame,
    *,
    acceleration_columns: Sequence[str],
    sample_rate_hz: float,
    cutoff_hz: float,
    output_gravity_suffix: str = "_gravity_iir",
    output_linear_suffix: str = "_linear_iir",
    init_mode: str = "mean",
    init_mean_seconds: float = 1.0,
) -> Tuple[pd.DataFrame, float]:
    """一阶 IIR 公式法去重力，正向递推。"""
    result = dataframe.copy()
    dt = 1.0 / float(sample_rate_hz)
    alpha = float(np.exp(-2.0 * np.pi * float(cutoff_hz) * dt))

    acc = result.loc[:, list(acceleration_columns)].to_numpy(dtype=float)
    if acc.shape[1] != 3:
        raise ValueError("acceleration_columns 必须是三个轴")

    gravity = np.zeros_like(acc)
    if init_mode == "first":
        gravity[0, :] = acc[0, :]
    else:
        n0 = max(1, int(round(float(init_mean_seconds) * float(sample_rate_hz))))
        gravity[0, :] = acc[:n0, :].mean(axis=0)

    for index in range(1, len(acc)):
        gravity[index, :] = alpha * gravity[index - 1, :] + (1.0 - alpha) * acc[index, :]

    linear = acc - gravity
    for axis_index, column_name in enumerate(acceleration_columns):
        result[f"{column_name}{output_gravity_suffix}"] = gravity[:, axis_index]
        result[f"{column_name}{output_linear_suffix}"] = linear[:, axis_index]
    return result, alpha


# =========================
# 绘图：简单、可控、少逻辑
# =========================

def plot_acceleration_gravity_removal_accx(
    *,
    dataframe_raw: pd.DataFrame,
    dataframe_processed: pd.DataFrame,
    time_column_name: str,
    raw_accx_column: str,
    gravity_column_name: str,
    linear_column_name: str,
    title: str,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    raw_alpha: float = 0.35,
) -> None:
    """只画 AccX：raw / gravity / linear。"""
    time_values = dataframe_raw[time_column_name]
    plt.figure(figsize=(14, 6))
    plt.plot(time_values, dataframe_raw[raw_accx_column], label=f"{raw_accx_column} raw", color="blue", alpha=raw_alpha, lw=0.8)
    plt.plot(time_values, dataframe_processed[gravity_column_name], label=gravity_column_name, color="tab:orange", lw=1.8)
    plt.plot(time_values, dataframe_processed[linear_column_name], label=linear_column_name, color="black", lw=1.8)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Accel")
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.show()


def plot_acceleration_methods_comparison(
    *,
    dataframe_raw: pd.DataFrame,
    time_column_name: str,
    raw_accx_column: str,
    named_series: Sequence[Tuple[str, pd.Series]],
    title: str,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    raw_alpha: float = 0.35,
) -> None:
    """比较多个 AccX 结果。"""
    time_values = dataframe_raw[time_column_name]
    colors = ["black", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown"]
    plt.figure(figsize=(14, 6))
    plt.plot(time_values, dataframe_raw[raw_accx_column], label=f"{raw_accx_column} raw", color="blue", alpha=raw_alpha, lw=0.8)
    for index, (label, series) in enumerate(named_series):
        plt.plot(time_values, series, label=label, color=colors[index % len(colors)], lw=1.8)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Accel")
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.show()


def plot_triplet_series(
    *,
    dataframe: pd.DataFrame,
    time_column_name: str,
    columns: Sequence[str],
    title: str,
    ylabel: str,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    colors: Optional[Mapping[str, str]] = None,
    line_width: float = 1.6,
) -> None:
    """画三轴/三列曲线，直接可复用。"""
    default_colors = {"x": "blue", "y": "green", "z": "red"}
    if colors is not None:
        default_colors.update(colors)

    time_values = dataframe[time_column_name]
    plt.figure(figsize=(14, 6))
    for column_name in columns:
        axis_key = column_name.split("_")[-1].lower()
        plt.plot(time_values, dataframe[column_name], label=column_name, color=default_colors.get(axis_key, "blue"), lw=line_width)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    plt.legend(ncol=2)
    plt.show()
