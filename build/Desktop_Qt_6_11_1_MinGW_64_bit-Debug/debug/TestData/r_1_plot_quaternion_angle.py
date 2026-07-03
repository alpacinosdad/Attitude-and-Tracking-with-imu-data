from __future__ import annotations

from math import sin, cos, tan, pi
from datetime import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==========================================================
# File paths
# ==========================================================
RAW_CSV = "BAT_Heat_Log_Data_2026_06_26_18_17_00.csv"
QUAT_CSV = "BAT_Heat_Log_Data_2026_06_26_18_17_00_imu_result.csv"

# ==========================================================
# Settings
# ==========================================================
N = 100000000
BIAS_SAMPLES = 20

# If the two curves have opposite direction, flip one of them:
THETA_SIGN = 1.0
QUAT_SIGN = 1.0

# If you want to start gyro Euler from zero:
INIT_THETA_DEG = 0.0
INIT_PHI_DEG = 0.0
INIT_PSI_DEG = 0.0

# ==========================================================
# Time parsing
# ==========================================================
def parse_time_series(series: pd.Series) -> np.ndarray:
    s = series.astype(str).str.strip()

    # Format like: 2026_06_25_17_56_11.338
    try:
        vals = [datetime.strptime(x, "%Y_%m_%d_%H_%M_%S.%f").timestamp() for x in s]
        vals = np.asarray(vals, dtype=float)
        vals = vals - vals[0]
        return vals
    except Exception:
        pass

    # Numeric time
    numeric = pd.to_numeric(s, errors="coerce")
    if numeric.notna().all():
        vals = numeric.to_numpy(dtype=float).copy()
        vals = vals - vals[0]
        return vals

    # Generic datetime
    dt = pd.to_datetime(s, errors="coerce")
    if dt.notna().all():
        vals = (dt - dt.iloc[0]).dt.total_seconds().to_numpy(dtype=float).copy()
        vals = vals - vals[0]
        return vals

    raise ValueError(f"Cannot parse time column. First samples: {s.head(5).tolist()}")


def find_col(df: pd.DataFrame, candidates: list[str], required: bool = True):
    cmap = {str(c).strip().lower(): c for c in df.columns}
    for c in candidates:
        if c.lower() in cmap:
            return cmap[c.lower()]
    if required:
        raise KeyError(f"Cannot find columns: {candidates}. Existing columns: {list(df.columns)}")
    return None


# ==========================================================
# Part 1: Gyro-only Euler integration -> theta
# ==========================================================
def load_raw_gyro_csv(path: str):
    df = pd.read_csv(path, encoding="utf-8-sig")

    utc_col = find_col(df, ["utc_time"])
    gx_col = find_col(df, ["gyro_x", "gx", "Gx"])
    gy_col = find_col(df, ["gyro_y", "gy", "Gy"])
    gz_col = find_col(df, ["gyro_z", "gz", "Gz"])

    t = parse_time_series(df[utc_col])

    # According to your previous IMUCSV logic:
    # new CSV gyro values are in deg/s -> convert to rad/s
    gx = np.deg2rad(pd.to_numeric(df[gx_col], errors="coerce").to_numpy(dtype=float))
    gy = np.deg2rad(pd.to_numeric(df[gy_col], errors="coerce").to_numpy(dtype=float))
    gz = np.deg2rad(pd.to_numeric(df[gz_col], errors="coerce").to_numpy(dtype=float))

    valid = np.isfinite(t) & np.isfinite(gx) & np.isfinite(gy) & np.isfinite(gz)

    return {
        "time_s": t[valid],
        "gx": gx[valid],
        "gy": gy[valid],
        "gz": gz[valid],
    }


def integrate_theta_from_gyro(raw: dict,
                                init_phi_deg: float = 0.0,
                                init_theta_deg: float = 0.0,
                                init_psi_deg: float = 0.0):
    t = raw["time_s"]
    gx = raw["gx"]
    gy = raw["gy"]
    gz = raw["gz"]

    n = min(len(t), N)

    t = t[:n]
    gx = gx[:n]
    gy = gy[:n]
    gz = gz[:n]

    # Gyro bias
    bn = min(BIAS_SAMPLES, n)
    bx = np.mean(gx[:bn])
    by = np.mean(gy[:bn])
    bz = np.mean(gz[:bn])

    phi = np.radians(init_phi_deg)
    theta = np.radians(init_theta_deg)
    psi = np.radians(init_psi_deg)

    time_list = [t[0]]
    theta_list = [np.degrees(theta)]

    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if dt <= 0 or not np.isfinite(dt):
            continue

        p = gx[i] - bx
        q = gy[i] - by
        r = gz[i] - bz

        # Euler kinematics (keep singular behavior intentionally)
        phi_dot = p + sin(phi) * tan(theta) * q + cos(phi) * tan(theta) * r
        theta_dot = cos(phi) * q - sin(phi) * r
        psi_dot = (sin(phi) / cos(theta)) * q + (cos(phi) / cos(theta)) * r

        phi += dt * phi_dot
        theta += dt * theta_dot
        psi += dt * psi_dot

        time_list.append(t[i])
        theta_list.append(np.degrees(theta))

    return np.asarray(time_list), THETA_SIGN * np.asarray(theta_list)


# ==========================================================
# Part 2: Quaternion -> continuous y-axis angle (red line)
# ==========================================================
def normalize_quaternions(q0, q1, q2, q3):
    n = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    n[n < 1e-12] = 1.0
    return q0 / n, q1 / n, q2 / n, q3 / n


def enforce_sign_continuity(q0, q1, q2, q3):
    """
    q and -q represent the same attitude.
    Enforce frame-to-frame sign continuity to avoid jumps.
    """
    q = np.column_stack([q0, q1, q2, q3]).copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] = -q[i]
    return q[:, 0], q[:, 1], q[:, 2], q[:, 3]


def load_quaternion_angle_y(path: str):
    df = pd.read_csv(path, encoding="utf-8-sig")

    # Time
    time_col = None
    for c in ["time_s", "time", "timestamp", "t", "utc_time"]:
        if c.lower() in [str(x).strip().lower() for x in df.columns]:
            time_col = find_col(df, [c])
            break

    if time_col is None:
        t = np.arange(len(df), dtype=float)
    else:
        t = parse_time_series(df[time_col])

    # Quaternion columns
    # Assume q0=w, q1=x, q2=y, q3=z
    try:
        q0_col = find_col(df, ["q0"])
        q1_col = find_col(df, ["q1"])
        q2_col = find_col(df, ["q2"])
        q3_col = find_col(df, ["q3"])
    except Exception:
        q0_col = find_col(df, ["qw"])
        q1_col = find_col(df, ["qx"])
        q2_col = find_col(df, ["qy"])
        q3_col = find_col(df, ["qz"])

    q0 = pd.to_numeric(df[q0_col], errors="coerce").to_numpy(dtype=float)
    q1 = pd.to_numeric(df[q1_col], errors="coerce").to_numpy(dtype=float)
    q2 = pd.to_numeric(df[q2_col], errors="coerce").to_numpy(dtype=float)
    q3 = pd.to_numeric(df[q3_col], errors="coerce").to_numpy(dtype=float)

    valid = np.isfinite(t) & np.isfinite(q0) & np.isfinite(q1) & np.isfinite(q2) & np.isfinite(q3)

    t = t[valid]
    q0 = q0[valid]
    q1 = q1[valid]
    q2 = q2[valid]
    q3 = q3[valid]

    q0, q1, q2, q3 = normalize_quaternions(q0, q1, q2, q3)
    q0, q1, q2, q3 = enforce_sign_continuity(q0, q1, q2, q3)

    # Equivalent rotation angle around y-axis
    angle_y_deg = np.degrees(2.0 * np.arctan2(q2, q0))
    angle_y_unwrap_deg = np.degrees(np.unwrap(np.radians(angle_y_deg)))

    return t, QUAT_SIGN * angle_y_unwrap_deg


# ==========================================================
# Main
# ==========================================================
def main():
    print("Loading raw gyro CSV...")
    raw = load_raw_gyro_csv(RAW_CSV)

    print("Integrating gyro-only Euler theta...")
    time_theta, theta_deg = integrate_theta_from_gyro(
        raw,
        init_phi_deg=INIT_PHI_DEG,
        init_theta_deg=INIT_THETA_DEG,
        init_psi_deg=INIT_PSI_DEG,
    )

    print("Loading quaternion CSV...")
    time_quat, quat_y_deg = load_quaternion_angle_y(QUAT_CSV)

    # Save merged result
    out_df = pd.DataFrame({
        "time_theta_s": pd.Series(time_theta),
        "theta_gyro_only_deg": pd.Series(theta_deg),
    })
    out_df["time_quat_s"] = pd.Series(time_quat)
    out_df["quat_y_unwrap_deg"] = pd.Series(quat_y_deg)
    out_df.to_csv("theta_vs_quaternion_y.csv", index=False, encoding="utf-8-sig")
    print("Saved: theta_vs_quaternion_y.csv")

    # Plot only the two required curves
    plt.figure(figsize=(15, 7))

    plt.plot(time_theta, theta_deg,
                color="tab:orange",
                linewidth=2.2,
                label="Theta (Gyro-only Euler Integration)")

    plt.plot(time_quat, quat_y_deg,
                color="red",
                linewidth=2.2,
                label="Quaternion Y Angle (Continuous)")

    plt.axhline(90, color="gray", linestyle="--", linewidth=1)
    plt.axhline(-90, color="gray", linestyle="--", linewidth=1)

    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Theta vs Quaternion Y-Axis Angle 60° to start")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()