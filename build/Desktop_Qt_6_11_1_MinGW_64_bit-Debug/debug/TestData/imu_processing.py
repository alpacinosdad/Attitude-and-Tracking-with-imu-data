"""
IMU 数据处理与作图工具（重构版）
--------------------------------
目标：
1) 统一滤波器设计接口：不同目的（低通/高通/带通/带阻）只改参数即可复用。
2) 统一处理流水线：所有处理函数输入都以 raw data 为起点，避免 df_proc 被反复覆盖导致“到底是不是 raw”混乱。
3) 统一作图函数：减少大片重复绘图代码。

说明：
- 这里提供两类滤波：
  A) “因果/正向”IIR：逐点 SOS 级联 + 可选稳态初始化（复现你 notebook 里的 C++ 逻辑）
  B) “双向/零相位”IIR：scipy.signal.sosfiltfilt（会产生 |H|^2 的幅频，不只是相位校正）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import pandas as pd
from scipy import signal
import matplotlib.pyplot as plt


# -----------------------------
# 1) 滤波器设计（统一接口）
# -----------------------------

FilterSpec = Mapping[str, object]


def design_sos(
    *,
    fs: float,
    filter_spec: FilterSpec,
) -> np.ndarray:
    """
    统一滤波器设计入口（目前实现 Butterworth）。

    filter_spec 支持字段：
      - design: "butter"（默认）
      - filter_type: "lowpass" | "highpass" | "bandpass" | "bandstop"
      - order: int
      - cutoff_hz: float（用于 low/high pass）
      - lowcut_hz/highcut_hz: float（用于 bandpass/bandstop；low/high pass 也可兼容）

    返回：SOS (second-order sections)，形状 (n_sections, 6)
    """
    fs = float(fs)
    nyq = fs / 2.0

    design = str(filter_spec.get("design", "butter")).lower()
    ftype = str(filter_spec.get("filter_type", "lowpass")).lower()
    order = int(filter_spec.get("order", 2))

    cutoff = filter_spec.get("cutoff_hz", None)
    lowcut = filter_spec.get("lowcut_hz", None)
    highcut = filter_spec.get("highcut_hz", None)

    if ftype in ("bandpass", "bandstop"):
        if lowcut is None or highcut is None:
            raise ValueError(f"{ftype} 需要同时提供 lowcut_hz 和 highcut_hz")
        low = float(lowcut)
        high = float(highcut)
        if not (0.0 < low < high < nyq):
            raise ValueError(
                f"{ftype} 截止频率必须满足 0 < lowcut < highcut < fs/2({nyq}). "
                f"当前: lowcut={low}, highcut={high}, fs={fs}"
            )
        wn = [low, high]
    elif ftype in ("lowpass", "highpass"):
        # 低通/高通：优先 cutoff_hz；否则兼容 lowcut_hz/highcut_hz
        if cutoff is None:
            cutoff = lowcut if lowcut is not None else highcut
        if cutoff is None:
            raise ValueError(f"{ftype} 需要提供 cutoff_hz（推荐）或 lowcut_hz/highcut_hz 之一")
        cut = float(cutoff)
        if not (0.0 < cut < nyq):
            raise ValueError(
                f"{ftype} 截止频率必须满足 0 < cutoff < fs/2({nyq}). 当前 cutoff={cut}, fs={fs}"
            )
        wn = cut
    else:
        raise ValueError(f"未知 filter_type: {ftype}")

    if design != "butter":
        raise NotImplementedError("当前只实现 butter（后续可扩展 cheby/ellip）")

    return signal.butter(order, wn, btype=ftype, fs=fs, output="sos")


# ----------------------------------------
# 2) 因果 SOS 滤波（复现 C++ 的逐点实现）
# ----------------------------------------


@dataclass
class SosState:
    z1: float = 0.0
    z2: float = 0.0


class FilterChannel:
    def __init__(self, sos_coeffs: np.ndarray):
        self.sos_coeffs = np.asarray(sos_coeffs, dtype=np.float64)
        if self.sos_coeffs.ndim != 2 or self.sos_coeffs.shape[1] != 6:
            raise ValueError("sos_coeffs 必须是形状 (n_sections, 6) 的数组")
        self.num_stages = self.sos_coeffs.shape[0]
        self.states: List[SosState] = [SosState() for _ in range(self.num_stages)]

    def reset(self) -> None:
        for s in self.states:
            s.z1 = 0.0
            s.z2 = 0.0


def _sos_filter_single(sec: np.ndarray, x: float, state: SosState) -> float:
    b0, b1, b2, a0, a1, a2 = sec
    # scipy 的 SOS 约定 a0=1；这里仍然保留 a0 字段以兼容通用格式
    out = b0 * x + state.z1
    state.z1 = b1 * x - a1 * out + state.z2
    state.z2 = b2 * x - a2 * out
    return out


def _sos_filter_cascade(channel: FilterChannel, x: float) -> float:
    y = float(x)
    for k in range(channel.num_stages):
        y = _sos_filter_single(channel.sos_coeffs[k], y, channel.states[k])
    return y


def _init_filter_steady_state(channel: FilterChannel, dc_offset: float, n_iter: int = 5000) -> None:
    """
    用常量 dc_offset 反复“空跑”滤波器，逼近稳态初值，减少起始瞬态。
    这对应你 notebook 里的 initFilterSteadyState / steady_init_iter。
    """
    channel.reset()
    for _ in range(int(n_iter)):
        _sos_filter_cascade(channel, float(dc_offset))


def apply_causal_sos_filter(
    df_raw: pd.DataFrame,
    *,
    cols: Sequence[str],
    sos: np.ndarray,
    steady_init_iter: int = 0,
    suffix: str = "_fwd",
) -> pd.DataFrame:
    """
    对 df_raw 的指定列做“因果/正向”SOS IIR 滤波，输出新列 col+suffix。
    注意：该函数不修改输入 df_raw（返回一个 copy）。
    """
    out = df_raw.copy()
    channels = {col: FilterChannel(sos) for col in cols}

    if len(out) == 0:
        for col in cols:
            out[col + suffix] = np.array([], dtype=np.float64)
        return out

    if steady_init_iter and steady_init_iter > 0:
        for col, ch in channels.items():
            _init_filter_steady_state(ch, float(out[col].iloc[0]), n_iter=int(steady_init_iter))

    for col, ch in channels.items():
        x = out[col].to_numpy(np.float64)
        y = np.empty_like(x)
        for i in range(len(x)):
            y[i] = _sos_filter_cascade(ch, float(x[i]))
        out[col + suffix] = y

    return out


def apply_zero_phase_filter(
    df_raw: pd.DataFrame,
    *,
    cols: Sequence[str],
    sos: np.ndarray,
    suffix: str = "_bi",
) -> pd.DataFrame:
    """
    双向/零相位滤波（sosfiltfilt）。结果不是“同一个滤波器的相位校正”，
    幅频会变成 |H|^2（等价于更陡的滤波）。
    """
    out = df_raw.copy()
    for col in cols:
        x = out[col].to_numpy(np.float64)
        out[col + suffix] = signal.sosfiltfilt(sos, x)
    return out


# -----------------------------
# 3) 重力估计 / 去重力
# -----------------------------


def gravity_lowpass_subtract(
    df_raw: pd.DataFrame,
    *,
    acc_cols: Sequence[str] = ("Accel_x", "Accel_y", "Accel_z"),
    fs: float,
    fc: float = 0.4,
    order: int = 1,
    steady_init_iter: int = 5000,
    causal_suffix: str = "_g",
    linear_suffix: str = "_lin",
) -> pd.DataFrame:
    """
    方法：先对加速度做低通得到重力分量 g，再做 linear = raw - g。
    注意：这里的“低通”用的是因果 SOS（支持稳态初始化），更接近你之前的写法。
    """
    sos_g = design_sos(fs=fs, filter_spec={"filter_type": "lowpass", "order": order, "cutoff_hz": fc})
    df_g = apply_causal_sos_filter(
        df_raw,
        cols=acc_cols,
        sos=sos_g,
        steady_init_iter=steady_init_iter,
        suffix=causal_suffix,
    )
    for col in acc_cols:
        df_g[col + linear_suffix] = df_g[col] - df_g[col + causal_suffix]
    return df_g


def gravity_remove_iir_first_order(
    df_raw: pd.DataFrame,
    *,
    acc_cols: Sequence[str] = ("Accel_x", "Accel_y", "Accel_z"),
    fs: float = 50.0,
    fc: float = 0.4,
    init_mode: str = "mean",  # "first" | "mean"
    init_mean_sec: float = 1.0,
    suffix_g: str = "_g",
    suffix_lin: str = "_lin",
) -> Tuple[pd.DataFrame, float]:
    """
    公式法去重力：1 阶 IIR 低通估计重力，再相减。
    与你 notebook 中 gravity_remove_iir_df 同源，但这里保证输入 df_raw 不被覆盖。
    """
    df_out = df_raw.copy()
    dt = 1.0 / float(fs)
    alpha = float(np.exp(-2.0 * np.pi * float(fc) * dt))

    A = df_out.loc[:, acc_cols].to_numpy(dtype=float)
    n, c = A.shape
    if c != 3:
        raise ValueError(f"acc_cols 必须为 3 个轴列名，当前={acc_cols}")

    G = np.zeros_like(A)
    if init_mode == "first":
        G[0, :] = A[0, :]
    elif init_mode == "mean":
        n0 = int(max(1, round(init_mean_sec * fs)))
        G[0, :] = A[:n0, :].mean(axis=0)
    else:
        raise ValueError('init_mode 只能是 "first" 或 "mean"')

    for i in range(1, n):
        G[i, :] = alpha * G[i - 1, :] + (1.0 - alpha) * A[i, :]

    LIN = A - G
    for k, col in enumerate(acc_cols):
        df_out[col + suffix_g] = G[:, k]
        df_out[col + suffix_lin] = LIN[:, k]

    return df_out, alpha


# -----------------------------
# 4) 读取 raw data（统一入口）
# -----------------------------


def load_raw_imu_txt(
    file_path: str,
    *,
    fs: float,
    imu_cols: Sequence[str],
    time_col: str = "time_sec",
) -> pd.DataFrame:
    """
    读取 IMU txt/csv（空格/Tab/逗号都兼容），并生成 time_sec。
    输出 df_raw：只包含 imu_cols + time_col（确保后续“所有处理从 raw 开始”）。
    """
    df = pd.read_csv(file_path, sep=r"\s+|,", engine="python", skipinitialspace=True)
    df.columns = df.columns.str.strip()
    df = df.loc[:, ~df.columns.str.contains(r"^Unnamed", regex=True)]
    df = df.replace(r"^\s*$", np.nan, regex=True)

    df_raw = df[list(imu_cols)].copy()
    for c in imu_cols:
        df_raw[c] = pd.to_numeric(df_raw[c], errors="coerce")
    df_raw = df_raw.dropna(subset=list(imu_cols)).reset_index(drop=True)
    df_raw[time_col] = np.arange(len(df_raw), dtype=np.int64) / float(fs)
    return df_raw


# -----------------------------
# 5) 统一作图（减少重复）
# -----------------------------


def plot_triplet_raw_vs_processed(
    *,
    df_raw: pd.DataFrame,
    df_proc: pd.DataFrame,
    time_col: str,
    cols: Sequence[str],
    proc_suffix: str,
    colors: Mapping[str, str],
    title: str,
    ylabel: str,
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    alpha_raw: float = 0.35,
    lw_proc: float = 1.8,
) -> None:
    """
    画三轴对比：raw(半透明) vs processed(实线)。
    cols 例：("Accel_x","Accel_y","Accel_z") 或 ("gyro_x","gyro_y","gyro_z")
    """
    t = df_raw[time_col]
    plt.figure(figsize=(14, 7))
    for col in cols:
        axis_key = col.split("_")[-1].lower()  # x/y/z
        color = colors.get(axis_key, "blue")
        plt.plot(t, df_raw[col], label=f"{col} (raw)", color=color, lw=0.8, alpha=alpha_raw)
        plt.plot(t, df_proc[col + proc_suffix], label=f"{col}{proc_suffix} (processed)", color=color, lw=lw_proc)

    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    plt.legend(ncol=2)
    plt.show()


def plot_accx_gravity_only(
    *,
    df_raw: pd.DataFrame,
    df_with_gravity: pd.DataFrame,
    time_col: str = "time_sec",
    accx_col: str = "Accel_x",
    gravity_suffix: str = "_g",
    linear_suffix: str = "_lin",
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    alpha_raw: float = 0.35,
) -> None:
    """
    只画 AccX：raw vs gravity vs linear（符合你的要求：重力那部分只看 AccX）。
    """
    t = df_raw[time_col]
    plt.figure(figsize=(14, 7))
    plt.plot(t, df_raw[accx_col], alpha=alpha_raw, lw=0.8, label=f"{accx_col} raw", color="blue")
    plt.plot(t, df_with_gravity[accx_col + gravity_suffix], lw=2.0, label=f"{accx_col}{gravity_suffix} gravity", color="blue")
    plt.plot(t, df_with_gravity[accx_col + linear_suffix], lw=1.6, label=f"{accx_col}{linear_suffix} linear", color="black")
    plt.xlabel("Time (s)")
    plt.ylabel("Accel")
    plt.title("AccX: Raw vs Gravity vs Linear")
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.show()


def plot_accx_methods_comparison(
    *,
    df_raw: pd.DataFrame,
    time_col: str,
    accx_col: str,
    series: Sequence[Tuple[str, pd.Series]],
    title: str = "AccX: Raw vs Methods",
    xlim: Optional[Tuple[float, float]] = None,
    ylim: Optional[Tuple[float, float]] = None,
    alpha_raw: float = 0.35,
) -> None:
    """
    在一张图里对比多个处理结果（默认包含 raw 作为参考）。

    Parameters
    ----------
    df_raw : DataFrame
        raw 数据（用于画 raw）
    time_col : str
        时间列名
    accx_col : str
        raw 的 AccX 列名
    series : list[(label, y_series)]
        多条曲线；y_series 应与 df_raw 行数一致
    """
    t = df_raw[time_col]
    plt.figure(figsize=(14, 7))
    plt.plot(t, df_raw[accx_col], alpha=alpha_raw, lw=0.8, label=f"{accx_col} raw", color="blue")

    # 给不同方法分配可读颜色（不强制用户传）
    default_colors = ["black", "tab:orange", "tab:green", "tab:red", "tab:purple", "tab:brown"]
    for i, (label, y) in enumerate(series):
        c = default_colors[i % len(default_colors)]
        plt.plot(t, y, lw=1.8, label=label, color=c)

    plt.xlabel("Time (s)")
    plt.ylabel("Accel")
    plt.title(title)
    if xlim is not None:
        plt.xlim(*xlim)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.show()
