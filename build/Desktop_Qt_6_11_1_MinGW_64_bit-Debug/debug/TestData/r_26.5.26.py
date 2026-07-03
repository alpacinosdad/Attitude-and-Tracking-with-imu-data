# -*- coding: utf-8 -*-
"""IMU CSV pipeline (Qt/C++ reproduction)

功能：
1) 只读取原始 IMU 六轴 (Accel_x/y/z, gyro_x/y/z)，不读取 UTC_Time，也不读取任何 *_h 列。
2) 横轴完全由采样率 fs 决定：time_sec = sample_index / fs
3) 使用二阶节 (SOS) 级联滤波，并复现 C++ 的“稳态初始化”(initFilterSteadyState)：
   在第一帧的直流值上循环 n_iter 次，把滤波器状态充到稳态。
4) 输出 df_repro：包含 raw + 计算得到的 *_h_py（以及可选复制到 *_h）。
5) 画图：Accel 原始 vs 滤波后；Gyro 原始 vs 滤波后。颜色固定：x蓝 y绿 z红。
6) 可选：画滤波器频响并标注截止频率。

使用：
- 修改 CONFIG 区域即可。
- 运行本脚本：python imu_pipeline.py

依赖：pandas, numpy, matplotlib
可选依赖：scipy (用于自动设计 Butterworth SOS 与频响图)
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =========================
# 0) 参数集中区（你只改这里）
# =========================
CONFIG = {
    # 输入 CSV
    "raw_file_path": "accelero and gyro data.txt",

    # 采样率
    "fs": 50,

    # 稳态初始化迭代次数（复现 C++ initFilterSteadyState）
    "steady_init_iter": 5000,

    # 是否把 _h_py 复制为 _h（方便你后续统一用 *_h）
    "also_write_h": False,

    # 输出（可选）
    "save_processed_csv": False,
    "processed_csv_path": "imu_processed.csv",

    # 滤波器设计（核心旋钮）
    # filter_type: "lowpass" | "highpass" | "bandpass" | "bandstop"
    "filter": {
        "design": "butter",
        "filter_type": "bandpass",
        "order": 4,
        "lowcut_hz": 0.02,     # bandpass/bandstop 下用
        "highcut_hz": 0.2,   # bandpass/bandstop 下用
        # 如果你用 lowpass/highpass，则使用 cutoff_hz
        # "cutoff_hz": 5.0,
    },

    # 绘图设置
    "plot": {
        "xlim": (0, 250),
        "acc_ylim": (-1500, 1500),
        "gyro_ylim": (-1000, 1000),
        "alpha_raw": 0.35,
        "lw_raw": 0.8,
        "lw_filt": 1.8,
        "show_filter_response": True,
        "filter_response_ylim_db": (-80, 5),
    }
}

# =========================
# 1) 统一颜色规则（x蓝 y绿 z红）
# =========================
axis_colors = {
    'x': 'blue',
    'y': 'green',
    'z': 'red'
}

ACC_COLORS = {
    'Accel_x': axis_colors['x'],
    'Accel_y': axis_colors['y'],
    'Accel_z': axis_colors['z'],
}

GYRO_COLORS = {
    'gyro_x': axis_colors['x'],
    'gyro_y': axis_colors['y'],
    'gyro_z': axis_colors['z'],
}

RAW_IMU_COLS = ['Accel_x', 'Accel_y', 'Accel_z', 'gyro_x', 'gyro_y', 'gyro_z']


# =========================
# 2) SOS 设计（优先用 scipy 自动设计）
# =========================

def design_sos_from_config(cfg):
    """根据 CONFIG["filter"] 生成 SOS。

    需要 scipy.signal。
    """
    try:
        from scipy import signal
    except Exception as e:
        raise RuntimeError(
            "未检测到 scipy，无法自动设计 SOS。\n"
            "解决：pip install scipy 或 conda install scipy，或在代码里手动提供 SOS_COEFFS。"
        ) from e

    fs = float(cfg['fs'])
    fcfg = cfg['filter']
    ftype = fcfg['filter_type'].lower()
    order = int(fcfg['order'])
    nyq = fs / 2.0

    if ftype in ('bandpass', 'bandstop'):
        low = float(fcfg['lowcut_hz'])
        high = float(fcfg['highcut_hz'])
        if not (0 < low < high < nyq):
            raise ValueError(
                f"band* 截止频率必须满足 0 < lowcut < highcut < fs/2({nyq}). 当前: low={low}, high={high}"
            )
        wn = [low, high]
    else:
        cutoff = float(fcfg.get('cutoff_hz', fcfg.get('highcut_hz', 5.0)))
        if not (0 < cutoff < nyq):
            raise ValueError(
                f"{ftype} 截止频率必须满足 0 < cutoff < fs/2({nyq}). 当前: cutoff={cutoff}"
            )
        wn = cutoff

    if fcfg['design'].lower() != 'butter':
        raise NotImplementedError('当前仅实现 butter。你要 cheby/ellip 我可以再帮你加。')

    sos = signal.butter(order, wn, btype=ftype, fs=fs, output='sos')
    return sos


# =========================
# 3) 读取 CSV（只读原始，不读 UTC_Time，也不读 *_h）
# =========================

def load_raw_imu(cfg):
    path = cfg['raw_file_path']
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到文件：{path}")

    df = pd.read_csv(
    CONFIG["raw_file_path"],
    sep="\t",          # ✅ 关键
    engine="python"    # ✅ 防止异常格式
    )

    # 去掉 CSV 尾逗号可能产生的 Unnamed 空列
    df = df.loc[:, ~df.columns.str.contains(r'^Unnamed', regex=True)]

    # 空白/空格 -> NaN
    df = df.replace(r'^\s*$', np.nan, regex=True)

    # 只保留原始六轴
    missing = [c for c in RAW_IMU_COLS if c not in df.columns]
    if missing:
        raise KeyError(f"CSV 缺少必要列：{missing}\n当前列名：{list(df.columns)}")

    df_imu = df[RAW_IMU_COLS].copy()

    # 数值化
    for c in RAW_IMU_COLS:
        df_imu[c] = pd.to_numeric(df_imu[c], errors='coerce')

    df_imu = df_imu.dropna(subset=RAW_IMU_COLS).reset_index(drop=True)

    fs = float(cfg['fs'])
    df_imu['time_sec'] = np.arange(len(df_imu), dtype=np.int64) / fs

    return df_imu


# =========================
# 4) SOS 级联滤波（复现 C++）
# =========================

class SosState:
    def __init__(self):
        self.z1 = 0.0
        self.z2 = 0.0


class FilterChannel:
    def __init__(self, sos_coeffs):
        self.sos_coeffs = np.array(sos_coeffs, dtype=np.float64)
        self.num_stages = self.sos_coeffs.shape[0]
        self.states = [SosState() for _ in range(self.num_stages)]

    def reset(self):
        for s in self.states:
            s.z1 = 0.0
            s.z2 = 0.0


def sos_filter_single(sec, x, state):
    b0, b1, b2, a0, a1, a2 = sec

    out = b0 * x + state.z1
    state.z1 = b1 * x - a1 * out + state.z2
    state.z2 = b2 * x - a2 * out

    return out


def sos_filter_cascade(channel, x):
    y = x
    for stage_idx in range(channel.num_stages):
        y = sos_filter_single(channel.sos_coeffs[stage_idx], y, channel.states[stage_idx])
    return y


def init_filter_steady_state(channel, dc_offset, n_iter=5000):
    channel.reset()
    for _ in range(int(n_iter)):
        sos_filter_cascade(channel, dc_offset)


def process_imu_like_cpp(df_imu, sos_coeffs, steady_iter=5000):
    """对六轴 raw 逐点滤波，输出 *_h_py（可选写 *_h）。"""
    out = df_imu.copy()

    channels = {col: FilterChannel(sos_coeffs) for col in RAW_IMU_COLS}

    # 第一帧稳态初始化
    for col, ch in channels.items():
        first_value = out[col].iloc[0]
        init_filter_steady_state(ch, dc_offset=first_value, n_iter=steady_iter)

    # 正式逐点滤波
    for col, ch in channels.items():
        filt = []
        for x in out[col].to_numpy(dtype=np.float64):
            filt.append(sos_filter_cascade(ch, x))
        out[col + '_h_py'] = np.array(filt, dtype=np.float64)

    # 可选：复制为 *_h，方便后续统一引用
    if CONFIG.get('also_write_h', False):
        for col in RAW_IMU_COLS:
            out[col + '_h'] = out[col + '_h_py']

    return out


# =========================
# 5) 画滤波器频响（可选）
# =========================

def plot_filter_response(sos, cfg):
    try:
        from scipy import signal
    except Exception:
        print("[WARN] 没有 scipy，跳过滤波器频响图。")
        return

    fs = float(cfg['fs'])
    fcfg = cfg['filter']
    ylim_db = cfg['plot'].get('filter_response_ylim_db', (-80, 5))

    w, h = signal.sosfreqz(sos, worN=4096, fs=fs)
    mag_db = 20 * np.log10(np.maximum(np.abs(h), 1e-12))

    nyq = fs / 2.0

    plt.figure(figsize=(10, 4))
    plt.plot(w, mag_db)
    plt.title(f"{fcfg['design'].upper()} {fcfg['filter_type']} frequency response (fs={fs} Hz)")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (dB)")
    plt.xlim(0, nyq)
    plt.ylim(*ylim_db)
    plt.grid(True, which='both', linestyle='--', alpha=0.4)

    ftype = fcfg['filter_type'].lower()
    if ftype in ('bandpass', 'bandstop'):
        low = float(fcfg['lowcut_hz'])
        high = float(fcfg['highcut_hz'])
        for f, lab in [(low, 'lowcut'), (high, 'highcut')]:
            plt.axvline(f, color='red', linestyle=':', linewidth=1.5)
            plt.text(f, ylim_db[1], f"{lab}={f}Hz", rotation=90, va='top', ha='right', color='red')
    else:
        cut = float(fcfg.get('cutoff_hz', fcfg.get('highcut_hz', 5.0)))
        plt.axvline(cut, color='red', linestyle=':', linewidth=1.5)
        plt.text(cut, ylim_db[1], f"cutoff={cut}Hz", rotation=90, va='top', ha='right', color='red')

    plt.tight_layout()
    plt.show()


# =========================
# 6) 画图：原始 vs 滤波后（Accel + Gyro）
# =========================

def plot_accel_raw_vs_filtered(df_repro, cfg):
    xlim = cfg['plot']['xlim']
    ylim = cfg['plot']['acc_ylim']
    alpha_raw = cfg['plot']['alpha_raw']
    lw_raw = cfg['plot']['lw_raw']
    lw_filt = cfg['plot']['lw_filt']

    plt.figure(figsize=(14, 7))

    # 原始（半透明）
    for col in ['Accel_x', 'Accel_y', 'Accel_z']:
        plt.plot(df_repro['time_sec'], df_repro[col],
                 label=f"{col} (raw)", color=ACC_COLORS[col], linewidth=lw_raw, alpha=alpha_raw)

    # 滤波后（加粗）
    for col in ['Accel_x', 'Accel_y', 'Accel_z']:
        plt.plot(df_repro['time_sec'], df_repro[col + '_h_py'],
                 label=f"{col}_h_py (filtered)", color=ACC_COLORS[col], linewidth=lw_filt)

    plt.xlabel("Time (s)")
    plt.ylabel("Accel")
    plt.title("Accelerometer: Raw vs Filtered")
    plt.xlim(*xlim)
    plt.ylim(*ylim)
    plt.legend(ncol=2)
    plt.grid(True)
    plt.show()


def plot_gyro_raw_vs_filtered(df_repro, cfg):
    xlim = cfg['plot']['xlim']
    ylim = cfg['plot']['gyro_ylim']
    alpha_raw = cfg['plot']['alpha_raw']
    lw_raw = cfg['plot']['lw_raw']
    lw_filt = cfg['plot']['lw_filt']

    plt.figure(figsize=(14, 7))

    # 原始（半透明）
    for col in ['gyro_x', 'gyro_y', 'gyro_z']:
        plt.plot(df_repro['time_sec'], df_repro[col],
                 label=f"{col} (raw)", color=GYRO_COLORS[col], linewidth=lw_raw, alpha=alpha_raw)

    # 滤波后（加粗）
    for col in ['gyro_x', 'gyro_y', 'gyro_z']:
        plt.plot(df_repro['time_sec'], df_repro[col + '_h_py'],
                 label=f"{col}_h_py (filtered)", color=GYRO_COLORS[col], linewidth=lw_filt)

    plt.xlabel("Time (s)")
    plt.ylabel("Gyro")
    plt.title("Gyroscope: Raw vs Filtered")
    plt.xlim(*xlim)
    plt.ylim(*ylim)
    plt.legend(ncol=2)
    plt.grid(True)
    plt.show()


# =========================
# 7) main
# =========================

def main():
    print("\n=== IMU pipeline start ===")
    print("file:", CONFIG['raw_file_path'])
    print("fs:", CONFIG['fs'])
    print("filter:", CONFIG['filter'])

    # 读取原始
    df_imu = load_raw_imu(CONFIG)
    print("data length:", len(df_imu), "samples")
    if len(df_imu) == 0:
        raise RuntimeError("读到的 IMU 数据为空（可能全是空值或列名不匹配）。")

    # 设计 SOS
    sos = design_sos_from_config(CONFIG)

    # 可选：画频响
    if CONFIG['plot'].get('show_filter_response', False):
        plot_filter_response(sos, CONFIG)

    # 滤波
    df_repro = process_imu_like_cpp(
        df_imu,
        sos_coeffs=sos,
        steady_iter=CONFIG['steady_init_iter']
    )

    # 输出保存（可选）
    if CONFIG.get('save_processed_csv', False):
        out_path = CONFIG.get('processed_csv_path', 'imu_processed.csv')
        df_repro.to_csv(out_path, index=False)
        print("saved:", out_path)

    # 画图
    plot_accel_raw_vs_filtered(df_repro, CONFIG)
    plot_gyro_raw_vs_filtered(df_repro, CONFIG)

    print("=== IMU pipeline done ===")


# =========================
    # 8) 重力验证实验（核心）
    # =========================

    print("\n=== Gravity consistency test ===")

    # 取低通后的加速度（当作重力）
    gx = df_repro['Accel_x_h_py'].values
    gy = df_repro['Accel_y_h_py'].values
    gz = df_repro['Accel_z_h_py'].values

    # 计算模长
    g_mag = np.sqrt(gx**2 + gy**2 + gz**2)

    # =========================
    # 统计信息（很重要）
    # =========================
    print(f"g magnitude mean: {np.mean(g_mag):.4f}")
    print(f"g magnitude std : {np.std(g_mag):.4f}")
    print(f"g magnitude min : {np.min(g_mag):.4f}")
    print(f"g magnitude max : {np.max(g_mag):.4f}")

    # =========================
    # 图1：三个轴的低频分量
    # =========================
    plt.figure(figsize=(14, 6))

    plt.plot(df_repro['time_sec'], gx, label='g_x', color='blue')
    plt.plot(df_repro['time_sec'], gy, label='g_y', color='green')
    plt.plot(df_repro['time_sec'], gz, label='g_z', color='red')

    plt.xlabel('Time (s)')
    plt.ylabel('Low-frequency acceleration')
    plt.title('Low-frequency components (candidate gravity)')
    plt.grid()
    plt.legend()
    plt.show()

    # =========================
    # 图2：模长（核心图）
    # =========================
    plt.figure(figsize=(14, 6))

    plt.plot(df_repro['time_sec'], g_mag, label='|g|', color='black')

    # 理想重力线（单位取决于你原始数据）
    g_ref = np.mean(g_mag)
    plt.axhline(g_ref, color='red', linestyle='--', label='mean(|g|)')

    plt.xlabel('Time (s)')
    plt.ylabel('|g|')
    plt.title('Gravity magnitude consistency check')
    plt.grid()
    plt.legend()
    plt.show()

    # =========================
    # 图3：误差（离g的偏移）
    # =========================
    plt.figure(figsize=(14, 6))

    plt.plot(df_repro['time_sec'], g_mag - g_ref, color='purple')

    plt.xlabel('Time (s)')
    plt.ylabel('Deviation')
    plt.title('Deviation of gravity magnitude')
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()