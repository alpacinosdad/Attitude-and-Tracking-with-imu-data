import os
from pathlib import Path
import matplotlib
import numpy as np
import pandas as pd
from gesture_rectangle_show import load_attitude_df, sensor_to_world

matplotlib.use("Agg")

import matplotlib.pyplot as plt



def integrate_trapezoidal(values, dt):
    values = np.asarray(values, dtype=float)
    dt = np.asarray(dt, dtype=float)
    if values.ndim == 1:
        values = values[:, None]

    integrated = np.zeros_like(values)
    if dt.size > 0:
        segments = 0.5 * (values[:-1] + values[1:]) * dt[:, None]
        integrated[1:] = np.cumsum(segments, axis=0)

    return integrated[:, 0] if integrated.shape[1] == 1 else integrated


def compute_trajectory(time_s, accel):
    time_s = np.asarray(time_s, dtype=float)
    accel = np.asarray(accel, dtype=float)
    if accel.ndim == 1:
        accel = accel[:, None]

    dt = np.diff(time_s)
    velocity = integrate_trapezoidal(accel, dt)
    displacement = integrate_trapezoidal(velocity, dt)
    return velocity, displacement


def test_integrate_trapezoidal_constant_acceleration():
    time_s = np.array([0.0, 0.02, 0.04, 0.06], dtype=float)
    accel = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
    velocity, displacement = compute_trajectory(time_s, accel)

    expected_velocity = np.array([0.0, 0.02, 0.04, 0.06], dtype=float)
    expected_displacement = np.array([0.0, 0.0002, 0.0008, 0.0018], dtype=float)

    assert np.allclose(velocity, expected_velocity, atol=1e-8)
    assert np.allclose(displacement, expected_displacement, atol=1e-10)


def test_load_attitude_df_has_required_columns(tmp_path):
    sample_csv = tmp_path / "sample.csv"
    sample_csv.write_text(
        "time_s,lax,lay,laz\n"
        "0.00,0.0,0.0,0.0\n"
        "0.02,1.0,0.0,0.0\n"
    )
    df = load_attitude_df(str(sample_csv))
    assert "time_s" in df.columns
    assert "lax" in df.columns
    assert "lay" in df.columns
    assert "laz" in df.columns
    assert df.shape[0] == 2


def test_plot_linear_accel_velocity_displacement(tmp_path):
    csv_path = Path(__file__).resolve().parent / "BAT_Heat_Log_Data_2026_06_11_16_13_23_imu_result.csv"
    if not csv_path.exists():
        pytest.skip("Reference CSV file not available for plot test")

    df = load_attitude_df(str(csv_path))
    required_columns = {"time_s", "lax", "lay", "laz"}
    assert required_columns.issubset(set(df.columns))

    time_s = df["time_s"].to_numpy(dtype=float)
    accel = df[["lax", "lay", "laz"]].to_numpy(dtype=float)

    velocity, displacement = compute_trajectory(time_s, accel)
    displacement_world = sensor_to_world(displacement)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    labels = ["lax", "lay", "laz"]
    for idx, label in enumerate(labels):
        axes[idx].plot(time_s, accel[:, idx], label=f"accel {label}", color="C0")
        axes[idx].plot(time_s, velocity[:, idx], label=f"vel {label}", color="C1")
        axes[idx].plot(time_s, displacement[:, idx], label=f"disp {label}", color="C2")
        axes[idx].set_ylabel(label)
        axes[idx].legend(loc="upper left", fontsize="small")
        axes[idx].grid(True)

    axes[-1].set_xlabel("time_s")
    fig.tight_layout()
    fig_path = tmp_path / "linear_accel_velocity_displacement.png"
    fig.savefig(fig_path)
    plt.close(fig)
    assert fig_path.exists()

    fig2 = plt.figure(figsize=(8, 6))
    ax3d = fig2.add_subplot(111, projection="3d")
    ax3d.plot(
        displacement_world[:, 0],
        displacement_world[:, 1],
        displacement_world[:, 2],
        color="tab:orange",
        linewidth=2,
    )
    ax3d.set_title("Displacement in reference world coordinate")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.grid(True)
    fig2.tight_layout()
    world_fig_path = tmp_path / "displacement_world.png"
    fig2.savefig(world_fig_path)
    plt.close(fig2)
    assert world_fig_path.exists()



import argparse
# ...existing code...

def main():
    parser = argparse.ArgumentParser(
        description="读取 CSV 中的线性加速度 lax/lay/laz，按时间积分出速度/位移并保存图像。"
    )
    parser.add_argument(
        "csv_file",
        type=Path,
        nargs="?",
        default=Path(__file__).resolve().parent
        / "BAT_Heat_Log_Data_2026_06_11_16_13_23_imu_result.csv",
        help="输入 CSV 文件，必须包含 time_s、lax、lay、laz 列。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.cwd(),
        help="输出图片目录，默认当前目录。",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="如果支持显示，则显示图像。",
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    df = load_attitude_df(str(args.csv_file))
    required_columns = {"time_s", "lax", "lay", "laz"}
    if not required_columns.issubset(df.columns):
        raise ValueError(
            f"CSV 文件必须包含列 {required_columns}，实际列: {list(df.columns)}"
        )

    time_s = df["time_s"].to_numpy(dtype=float)
    accel = df[["lax", "lay", "laz"]].to_numpy(dtype=float)

    velocity, displacement = compute_trajectory(time_s, accel)
    displacement_world = sensor_to_world(displacement)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    labels = ["lax", "lay", "laz"]
    for idx, label in enumerate(labels):
        axes[idx].plot(time_s, accel[:, idx], label=f"accel {label}", color="C0")
        axes[idx].plot(time_s, velocity[:, idx], label=f"vel {label}", color="C1")
        axes[idx].plot(time_s, displacement[:, idx], label=f"disp {label}", color="C2")
        axes[idx].set_ylabel(label)
        axes[idx].legend(loc="upper left", fontsize="small")
        axes[idx].grid(True)

    axes[-1].set_xlabel("time_s (s)")
    fig.tight_layout()
    fig_path = output_dir / "linear_accel_velocity_displacement.png"
    fig.savefig(fig_path, dpi=200)
    plt.close(fig)
    print(f"Saved {fig_path}")

    fig2 = plt.figure(figsize=(8, 6))
    ax3d = fig2.add_subplot(111, projection="3d")
    ax3d.plot(
        displacement_world[:, 0],
        displacement_world[:, 1],
        displacement_world[:, 2],
        color="tab:orange",
        linewidth=2,
    )
    ax3d.set_title("Displacement in reference world coordinate")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.grid(True)
    fig2.tight_layout()
    world_fig_path = output_dir / "displacement_world.png"
    fig2.savefig(world_fig_path, dpi=200)
    plt.close(fig2)
    print(f"Saved {world_fig_path}")

    
    print("cwd:", Path.cwd())
    print("output_dir:", output_dir.resolve())


    if args.show:
        plt.show()


if __name__ == "__main__":
    main()