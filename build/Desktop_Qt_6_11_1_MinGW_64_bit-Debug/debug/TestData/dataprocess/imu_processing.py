"""清晰版 IMU 数据处理函数文件

目标
- 只保留清晰、短小、意义明确的函数
- 提供文件读取、三种正向因果滤波、基础绘图
- 不使用双向滤波，不使用 filtfilt
- notebook 里只需要改参数区即可复用

说明
- 去重力不单独封装成独立业务函数，直接在 notebook 中调用滤波函数完成
- 函数只做最基本、最清晰的事情
"""

from __future__ import annotations

from typing import Iterable, Sequence, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal


# =========================
# 文件读取
# =========================

def load_imu_text_file(
    file_path: str,
    imu_column_names: Sequence[str],
    *,
    sample_rate_hz: float,
    time_column_name: str = "time_sec",
) -> pd.DataFrame:
    """读取空格或逗号分隔的 IMU 文本/CSV，返回统一格式 DataFrame。"""
    df = pd.read_csv(file_path, sep=r"\s+|,", engine="python", skipinitialspace=True)
    df.columns = df.columns.str.strip()
    df = df.loc[:, ~df.columns.str.contains(r"^Unnamed", regex=True)]

    missing = [name for name in imu_column_names if name not in df.columns]
    if missing:
        raise ValueError(f"缺少列: {missing}")

    df = df.loc[:, list(imu_column_names)].copy()
    df.insert(0, time_column_name, np.arange(len(df), dtype=float) / float(sample_rate_hz))
    return df


# =========================
# 滤波设计
# =========================

def design_causal_sos_lowpass(
    *,
    sample_rate_hz: float,
    cutoff_hz: float,
    order: int = 4,
    design_name: str = "butter",
) -> np.ndarray:
    """设计正向因果低通 SOS 滤波器。"""
    normalized = cutoff_hz / (sample_rate_hz / 2.0)
    return signal.iirfilter(order, normalized, btype="lowpass", ftype=design_name, output="sos")


def design_causal_sos_highpass(
    *,
    sample_rate_hz: float,
    cutoff_hz: float,
    order: int = 4,
    design_name: str = "butter",
) -> np.ndarray:
    """设计正向因果高通 SOS 滤波器。"""
    normalized = cutoff_hz / (sample_rate_hz / 2.0)
    return signal.iirfilter(order, normalized, btype="highpass", ftype=design_name, output="sos")


def design_causal_sos_bandpass(
    *,
    sample_rate_hz: float,
    lowcut_hz: float,
    highcut_hz: float,
    order: int = 4,
    design_name: str = "butter",
) -> np.ndarray:
    """设计正向因果带通 SOS 滤波器。"""
    band = [lowcut_hz / (sample_rate_hz / 2.0), highcut_hz / (sample_rate_hz / 2.0)]
    return signal.iirfilter(order, band, btype="bandpass", ftype=design_name, output="sos")


# =========================
# 滤波应用
# =========================

def apply_causal_sos_filter(
    data: Sequence[float],
    sos: np.ndarray,
    *,
    initial_value: Optional[float] = None,
    initial_iterations: int = 0,
) -> np.ndarray:
    """对一维数据执行正向因果 SOS 滤波。"""
    values = np.asarray(data, dtype=float)
    if len(values) == 0:
        return values.copy()

    if initial_value is None:
        initial_value = float(values[0])

    if initial_iterations > 0:
        values = values.copy()
        values[: min(initial_iterations, len(values))] = initial_value

    zf = signal.sosfilt_zi(sos) * initial_value
    filtered, _ = signal.sosfilt(sos, values, zi=zf)
    return filtered


# =========================
# 基础绘图
# =========================

def plot_time_series_lines(
    df: pd.DataFrame,
    columns: Sequence[str],
    *,
    title: str,
    xlabel: str = "Time (s)",
    ylabel: str = "Value",
    xlim: Optional[tuple[float, float]] = None,
    ylim: Optional[tuple[float, float]] = None,
    alpha: float = 1.0,
    line_width: float = 1.5,
    legend: bool = True,
) -> None:
    """画多条时间序列曲线。"""
    plt.figure(figsize=(14, 6))
    time_values = df.iloc[:, 0]
    default_colors = ["blue", "green", "red", "orange", "purple", "brown"]

    for index, column_name in enumerate(columns):
        plt.plot(
            time_values,
            df[column_name],
            label=column_name,
            color=default_colors[index % len(default_colors)],
            alpha=alpha,
            linewidth=line_width,
        )

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    if legend:
        plt.legend()
    plt.tight_layout()
    plt.show()
